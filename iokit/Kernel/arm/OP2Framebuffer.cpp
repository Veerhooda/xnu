/*
 * Built-in IOMobileFramebuffer for OnePlus 2 (MSM8994):
 * The continuous-splash framebuffer (1080x1920 BGRA at 0x03401000 in RAM,
 * scanned out by MDP5 hardware) as the main display, driven by CoreAnimation's
 * CPU renderer.
 *
 * IOMobileFramebuffer.framework counts /arm-io children with
 * device_type "display-subsystem", waits for services matching
 * IOServiceMatching("IOMobileFramebuffer"), opens each with type 0 and
 * reads canvas-width/canvas-height from the service's provider.
 * AppleCLCD matches the /arm-io/disp0 nub.
 *
 * User client selectors used by CoreAnimation:
 *   3  GetLayerDefaultSurface [layer, ?] -> [IOSurface port name]
 *   4  SwapBegin -> [swap id]
 *   5  SwapEnd, struct 0x46c: +0x50 swap id, +0x54 IOSurface id per layer
 *      (4 layers), +0x64 rect a / +0xc4 rect b per layer (int x,y,w,h),
 *      +0x104 enabled layer mask
 *   6  SwapWait, 7 GetID, 8 GetDisplaySize -> [w, h],
 *   12 RequestPowerChange [state]: 0 = display off, else on
 *   18 IsMainDisplay, 28 GetDotPitch,
 *   72 Enable/DisableNotifications [func, refcon, type, 0]
 *
 * SwapEnd records the swap; vsync timer presents frames into panel memory
 * at a throttled rate (boot-arg op2fb_hz, default 15 Hz) to prevent single-core
 * CPU starvation.
 */
#include "OP2Surface.h"
#include "OP2GPU.h"
#include "op2_rpm.h"

#if defined(ARM64_BOARD_CONFIG_MSM8994)
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/system.h>
#include <pexpert/pexpert.h>
#include <kern/clock.h>
#include <kern/thread.h>
#include <sys/proc.h>
#include "apple_logo.h"

/*
 * G5b: with CoreAnimation on the GPU (metal=2 or more), a surface's pages are written by the GPU, which does not see
 * the CPU's caches, and read by the CPU here through a cached mapping. Clean+invalidate the range first, so the CPU
 * reads what the GPU wrote (round g5b-11: after the first frame the panel kept showing stale cached pixels, and the
 * phone looked frozen while backboardd drew new frames), and no dirty line from the CPU lands on top of it later.
 */
static inline void
op2_civac(vm_offset_t va, vm_size_t size)
{
	vm_offset_t addr = va & ~63ULL;
	vm_offset_t end = (va + size + 63ULL) & ~63ULL;
	__asm__ volatile("dsb sy" ::: "memory");
	while (addr + 256 <= end) {
		__asm__ volatile(
			"dc civac, %0\n"
			"dc civac, %1\n"
			"dc civac, %2\n"
			"dc civac, %3\n"
			: : "r"(addr), "r"(addr + 64), "r"(addr + 128), "r"(addr + 192) : "memory");
		addr += 256;
	}
	while (addr < end) {
		__asm__ volatile("dc civac, %0" : : "r"(addr) : "memory");
		addr += 64;
	}
	__asm__ volatile("dsb sy" ::: "memory");
}

static bool
op2_gpu_renders(void)
{
	static int cached = -1;
	if (cached < 0) {
		uint32_t metal_level = 0;
		cached = PE_parse_boot_argn("metal", &metal_level, sizeof(metal_level)) && metal_level >= 2;
	}
	return cached != 0;
}

#ifndef IOMallocData
#define IOMallocData(sz) IOMalloc(sz)
#endif
#ifndef IOFreeData
#define IOFreeData(p, sz) IOFree((p), (sz))
#endif

extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);

static uint64_t op2_fb_last_kick = 0;
static uint64_t op2_fb_kick_count = 0;
static uint64_t op2_fb_present_count; /* moved up: vsyncFired timeline reads it */
class AppleCLCD; /* fwd: full type below, before use */
static AppleCLCD *op2_fb_instance = NULL; /* set in start(), for deadline dump */
static uint32_t op2_fb_last_shown = 0;    /* mirror of present()'s sLastShown */
static uint64_t op2_fb_last_direct_phys = 0; /* physical address of presented surface */
static uint8_t op2_fb_last_rec[0x278];    /* last swap record, for deadline dump */
static uint32_t op2_fb_last_nz = 0;       /* its sampled content */
static uint64_t op2_fb_last_pc = 0;

static void
op2_fb_kick_panel(volatile uint32_t *ctl)
{
	if (ctl == NULL) {
		return;
	}
	uint64_t now = mach_absolute_time();
	/* Spacing to ensure previous DSI transfer finished (60 Hz = ~16.6 ms = 320,000 ticks) */
	if (op2_fb_last_kick != 0) {
		uint64_t diff = now - op2_fb_last_kick;
		if (diff < 280000ULL) {
			uint64_t wait_ticks = 280000ULL - diff;
			uint32_t us = (uint32_t)((wait_ticks * 10) / 192);
			if (us > 15000) {
				us = 15000;
			}
			if (us > 0) {
				IODelay(us);
			}
		}
	}
	op2_fb_last_kick = mach_absolute_time();
	op2_fb_kick_count++;

	__asm__ volatile("dsb sy" ::: "memory");
	ctl[0x18 / 4] = 0x00020041;              /* CTL0_FLUSH: VIG0 (bit 0) | LM0 (bit 6) | CTL (bit 17) */
	__asm__ volatile("dsb sy" ::: "memory");
	ctl[0x1c / 4] = 1;                       /* CTL0_START: kick DSI command-mode transfer */
	__asm__ volatile("dsb sy" ::: "memory");
}

enum {
	kOP2FBGetLayerDefaultSurface  = 3,
	kOP2FBSwapBegin               = 4,
	kOP2FBSwapEnd                 = 5,
	kOP2FBSwapWait                = 6,
	kOP2FBGetID                   = 7,
	kOP2FBGetDisplaySize          = 8,
	kOP2FBRequestPowerChange      = 12,
	kOP2FBIsMainDisplay           = 18,
	kOP2FBGetDotPitch             = 28,
	kOP2FBSurfaceIsReplaceable    = 49,
	kOP2FBNotifications           = 72,
	kOP2FBChangeFrameInfo         = 73,
	kOP2FBSupportedFrameInfo      = 74,
};

#define OP2_FB_SWAP_REC_SIZE          0x46c
#define OP2_FB_LAYERS                 4
#define OP2_FB_NOTIFY_TYPES           8
#define OP2_FB_NOTIFY_VSYNC           5
#define OP2_FB_VSYNC_US               16667   /* 60 Hz */
#define OP2_FB_DOT_PITCH              401     /* ppi, 5.5" 1080x1920 */
#define OP2_FB_FORMAT_BGRA            0x42475241
#define OP2_FB_DEFAULT_PHYS           0x03401000ULL
#define OP2_FB_DEFAULT_WIDTH          1080
#define OP2_FB_DEFAULT_HEIGHT         1920
#define OP2_FB_DEFAULT_ROWBYTES       4352    /* 1088 * 4 */

class IOAcceleratorES : public IOService {
	OSDeclareDefaultStructors(IOAcceleratorES);
};
OSDefineMetaClassAndStructors(IOAcceleratorES, IOService);

class AGXAccelerator : public IOAcceleratorES {
	OSDeclareDefaultStructors(AGXAccelerator);
public:
	virtual IOReturn newUserClient(task_t owningTask, void *securityID,
	    UInt32 type, OSDictionary *properties,
	    IOUserClient **handler) APPLE_KEXT_OVERRIDE;
	virtual IOReturn newUserClient(task_t owningTask, void *securityID,
	    UInt32 type, IOUserClient **handler) APPLE_KEXT_OVERRIDE;
};
OSDefineMetaClassAndStructors(AGXAccelerator, IOAcceleratorES);

IOReturn
AGXAccelerator::newUserClient(task_t owningTask, void *securityID,
    UInt32 type, OSDictionary *properties,
    IOUserClient **handler)
{
	OP2GPU *gpu = OP2GPU::getInstance();
	if (gpu) {
		return gpu->newUserClient(owningTask, securityID, type, properties, handler);
	}
	return kIOReturnNotReady;
}

IOReturn
AGXAccelerator::newUserClient(task_t owningTask, void *securityID,
    UInt32 type, IOUserClient **handler)
{
	return newUserClient(owningTask, securityID, type, NULL, handler);
}

class IOMobileFramebuffer : public IOService {
	OSDeclareDefaultStructors(IOMobileFramebuffer);
};

OSDefineMetaClassAndStructors(IOMobileFramebuffer, IOService);

class IOMobileFramebufferUserClient;

class AppleCLCD : public IOMobileFramebuffer {
	OSDeclareDefaultStructors(AppleCLCD);

	IOWorkLoop         *fWorkLoop;
	IOTimerEventSource *fVsync;
	uint64_t            fVsyncCount;

	static void vsyncFired(OSObject *owner, IOTimerEventSource *sender);

public:
	uint64_t      fPhys;
	uint32_t      fWidth, fHeight, fRowBytes;
	uintptr_t     fPixels;          /* kernel mapping of the panel memory */
	IOMemoryMap  *fMap;
	OP2Surface   *fSurface;         /* the panel memory as an IOSurface */
	IOLock       *fLock;            /* swaps and client list */
	OSArray      *fClients;
	uint32_t      fSwapID;
	bool          fConsolePaused;
	uint8_t       fPending[OP2_FB_SWAP_REC_SIZE];     /* latest swap, not yet shown */
	bool          fDirty;
	uint32_t      fFlushTicks;      /* vsync ticks between panel copies */
	bool          fPowerOff;        /* RequestPowerChange(0) */
	uint32_t      fFadeStep;        /* fade-out steps done while off */
	volatile uint32_t *fMdpCtl;
	volatile uint32_t *fMdpPP;
	volatile uint32_t *fDsi0;
	volatile uint32_t *fActivePipe;
	uint32_t           fPipeFlushBit;
	bool               fPipeHasScaler;   /* VIG pipes can downscale in hardware (MDP5 QSEED phase-step) */

	uint64_t      fBootStartMachTime;
	uint32_t      fLastProgressPct;
	bool          fUserlandTookOver;
	bool          fClientUI;             /* op2fb_client_ui: show client surfaces when server is blank */

	/*
	 * Double-buffered panel scanout. Buffer 0 is the LK splash memory
	 * (fPhys), buffer 1 a carveout slot. present() always writes the
	 * buffer the panel is NOT reading, then flips VIG0 SRC0 and kicks,
	 * so a DSI transfer never reads a half-written frame (the old
	 * "half the screen is stale/blank" tearing).
	 */
	uint64_t            fBufPhys[2];
	uintptr_t           fBufPixels[2];
	IOMemoryDescriptor *fBufMem;
	IOMemoryMap        *fBufMap;
	uint32_t            fFrontBuf;
	bool                fDoubleBuffer;

	/* Swap completion: SwapEnd arms an immediate present; SwapWait
	 * blocks the render server until its frame has left its buffer. */
	IOTimerEventSource *fPresentNow;
	uint32_t            fLastSwapEndID;
	uint32_t            fCompletedSwapID;
	bool                fSwapWaitEnabled;

	/* 2026-11-17 flash fix: copy at swap time, flip+kick when the DSI
	 * spacing allows. fEarlyCopy is set by presentPending for one present()
	 * call; that call either copies into the back buffer and leaves the
	 * flip pending (fFlipPending) or refuses (fEarlyRejected). */
	bool                fEarlyCopy;
	bool                fEarlyRejected;
	bool                fFlipPending;
	uint32_t            fFlipIdx;
	uintptr_t           fFlipPixels;
	uint64_t            fFlipPhys;
	void flipAndKick(void);

	static void presentNowFired(OSObject *owner, IOTimerEventSource *sender);
	void presentPending(void);

	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	void addClient(IOMobileFramebufferUserClient *client);
	void removeClient(IOMobileFramebufferUserClient *client);
	void swap(const uint8_t *rec);
	void present(const uint8_t *rec);
	void drawProgressBar(uint32_t percent);
};

OSDefineMetaClassAndStructors(AppleCLCD, IOMobileFramebuffer);

class IOMobileFramebufferUserClient : public IOUserClient {
	OSDeclareDefaultStructors(IOMobileFramebufferUserClient);

public:
	AppleCLCD          *fFB;
	task_t              fTask;
	mach_port_t         fPort;
	io_user_reference_t fFunc[OP2_FB_NOTIFY_TYPES];
	io_user_reference_t fRefcon[OP2_FB_NOTIFY_TYPES];

	bool initWithTask(task_t owningTask, void *securityID, UInt32 type) APPLE_KEXT_OVERRIDE;
	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	IOReturn registerNotificationPort(mach_port_t port, UInt32 type,
	    io_user_reference_t refCon) APPLE_KEXT_OVERRIDE;
	IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target,
	    void *reference) APPLE_KEXT_OVERRIDE;
	void notify(uint32_t type, const uint64_t *values, uint32_t count);
};

OSDefineMetaClassAndStructors(IOMobileFramebufferUserClient, IOUserClient);

void
AppleCLCD::drawProgressBar(uint32_t percent)
{
	if (fPixels == 0) return;
	uint8_t *fb = (uint8_t *)fPixels;

	int32_t bar_w = 440;
	int32_t bar_h = 12;
	int32_t bx0 = (int32_t)(fWidth / 2) - (bar_w / 2); /* 540 - 220 = 320 */
	int32_t by0 = 1030;
	int32_t bx1 = bx0 + bar_w;                          /* 760 */
	int32_t by1 = by0 + bar_h;                          /* 1042 */
	if (percent > 100) percent = 100;
	int32_t fill_x = bx0 + (bar_w * (int32_t)percent) / 100;

	int32_t cr = bar_h / 2; /* 6 px radius */
	int32_t cy = by0 + cr;
	int32_t lcx = bx0 + cr;
	int32_t rcx = bx1 - cr;

	for (int32_t y = by0; y < by1; y++) {
		if (y < 0 || y >= (int32_t)fHeight) continue;
		uint32_t *row = (uint32_t *)(void *)(fb + (uint32_t)y * fRowBytes);
		for (int32_t x = bx0; x < bx1; x++) {
			if (x < 0 || x >= (int32_t)fWidth) continue;
			bool inside = true;
			if (x < lcx) {
				int32_t dx = x - lcx;
				int32_t dy = y - cy;
				if (dx * dx + dy * dy > cr * cr) inside = false;
			} else if (x > rcx) {
				int32_t dx = x - rcx;
				int32_t dy = y - cy;
				if (dx * dx + dy * dy > cr * cr) inside = false;
			}
			if (inside) {
				if (x <= fill_x) {
					row[x] = 0xFFFFFFFFu;
				} else {
					row[x] = 0xFF3A3A3Cu;
				}
			}
		}
	}
	__asm__ volatile("dsb sy" ::: "memory");
	op2_fb_kick_panel(fMdpCtl);
}

extern "C" void
op2_fb_kick_panel_external(void)
{
	if (op2_fb_instance != NULL && op2_fb_instance->fMdpCtl != NULL) {
		__asm__ volatile("dsb sy" ::: "memory");
		op2_fb_kick_panel(op2_fb_instance->fMdpCtl);
	}
}

static void op2_cpu_boost(void);
static void
op2_cpu_boost_thread(__unused void *param, __unused wait_result_t wr)
{
	IOSleep(20000);
	op2_cpu_boost();
	/* returning from a kernel thread's entry terminates it */
}

extern "C" void op2_threaddump_start(void);

/*
 * Effective CPU clock: count core cycles (PMCCNTR_EL0) across 10 ms of the
 * 19.2 MHz generic timer. Nothing in this port programs the A53 PLL, so the
 * cores run at whatever LK left them at; this tells us what that is.
 * Disable with boot-arg op2_cpumhz=0.
 */
extern "C" uint32_t op2_measure_cpu_mhz(void);
extern "C" uint32_t op2_cpu_mhz_now;
uint32_t op2_cpu_mhz_now = 0;   /* last measurement, shown on every OP2 TOP line */
extern "C" uint32_t
op2_measure_cpu_mhz(void)
{
	uint32_t enable = 1;
	PE_parse_boot_argn("op2_cpumhz", &enable, sizeof(enable));
	if (enable == 0) {
		return 0;
	}
	boolean_t istate = ml_set_interrupts_enabled(FALSE);
	uint64_t pmcr = 0;
	__asm__ volatile("mrs %0, PMCR_EL0" : "=r"(pmcr));
	__asm__ volatile("msr PMCR_EL0, %0" : : "r"(pmcr | 1ULL | (1ULL << 2)));
	__asm__ volatile("msr PMCNTENSET_EL0, %0" : : "r"(1ULL << 31));
	__asm__ volatile("isb" ::: "memory");
	uint64_t c0 = 0, c1 = 0;
	uint64_t t0 = mach_absolute_time();
	__asm__ volatile("mrs %0, PMCCNTR_EL0" : "=r"(c0));
	while (mach_absolute_time() - t0 < 192000ULL) {
		/* spin 10 ms */
	}
	uint64_t t1 = mach_absolute_time();
	__asm__ volatile("mrs %0, PMCCNTR_EL0" : "=r"(c1));
	ml_set_interrupts_enabled(istate);
	if (t1 <= t0) {
		return 0;
	}
	op2_cpu_mhz_now = (uint32_t)(((c1 - c0) * 192ULL) / ((t1 - t0) * 10ULL));
	return op2_cpu_mhz_now;
}

/*
 * A53 cluster 768 -> 1536 MHz, the way stock Linux does it:
 *   LK leaves PLL0 at 1536 MHz (L=0x50) feeding the CPUs through the /2
 *   post-divider (LF mux sel 2), with APC0 (PM8994 S8, FTS2 range0) at the
 *   NOMINAL-corner 1.0 V. 1536 MHz is fuse corner 4 (TURBO):
 *   1. vote VDD_MX (smpa2) to SUPER_TURBO via RPM (cpr vdd-mx-corner-map),
 *   2. APC0 -> 1.145 V (stock open-loop TURBO PVS voltage) via the
 *      system-cci SAW2 VCTL; must be confirmed by PMIC_STS,
 *   3. HF mux sel 3 = PLL0 undivided.
 * Every precondition is checked; any surprise aborts before step 3.
 * boot-arg op2_cpuboost=0 disables.
 */
/*
 * Thermal throttle (op2_snapshot.c op2_thermal_thread): back to the LK clock path (PLL0 /2, 768 MHz) and up again.
 * Lowering the clock at the TURBO voltage is always safe; the voltage stays where the boost put it.
 */
static volatile uint32_t *op2_boost_glb, *op2_boost_saw;
static uint32_t op2_boost_mux_fast, op2_boost_mux_slow, op2_vlevel_fast, op2_vlevel_slow;

static bool
op2_apc0_set_vlevel(uint32_t v)
{
	op2_boost_saw[0x1c / 4] = v;
	__asm__ volatile("dsb sy" ::: "memory");
	for (int i = 0; i < 500; i++) {
		if ((op2_boost_saw[0x14 / 4] & 0xff) == v) {
			return true;
		}
		IODelay(1);
	}
	return false;
}

/*
 * Thermal throttle (op2_snapshot.c op2_thermal_thread). On: clock down to the LK path (PLL0 /2, 768 MHz) FIRST, then
 * APC0 back to LK's voltage -- round fable-45 showed the A53 cores (TSENS 7..10) still climbing at idle at 768 MHz
 * with the TURBO 1.145 V left on: leakage, not load. Off: voltage up and confirmed FIRST, then the clock; if the
 * PMIC does not confirm, the clock stays down.
 */
extern "C" int op2_cpu_set_throttle(int on);
extern "C" int
op2_cpu_set_throttle(int on)
{
	if (op2_boost_glb == NULL) {
		return 0;   /* never boosted: already at 768 MHz and LK's voltage */
	}
	if (on) {
		op2_boost_glb[0x54 / 4] = op2_boost_mux_slow;
		__asm__ volatile("dsb sy; isb" ::: "memory");
		IODelay(10);
		if (op2_vlevel_slow && !op2_apc0_set_vlevel(op2_vlevel_slow)) {
			IOLog("OP2 THERM: APC0 did not confirm vlevel 0x%x\n", op2_vlevel_slow);
		}
	} else {
		if (!op2_apc0_set_vlevel(op2_vlevel_fast)) {
			IOLog("OP2 THERM: APC0 did not confirm TURBO vlevel 0x%x: staying at 768 MHz\n", op2_vlevel_fast);
			return (int)op2_measure_cpu_mhz();
		}
		IODelay(300);   /* slew up, as the boost does */
		op2_boost_glb[0x54 / 4] = op2_boost_mux_fast;
		__asm__ volatile("dsb sy; isb" ::: "memory");
		IODelay(10);
	}
	return (int)op2_measure_cpu_mhz();
}

extern "C" uint32_t op2_measure_cpu_mhz(void);
static void
op2_cpu_boost(void)
{
	uint32_t en = 1;
	PE_parse_boot_argn("op2_cpuboost", &en, sizeof(en));
	if (en == 0) {
		IOLog("OP2 BOOST: disabled by boot-arg\n");
		return;
	}
	volatile uint32_t *pll = (volatile uint32_t *)ml_io_map(0xf9015000, 0x1000);
	volatile uint32_t *glb = (volatile uint32_t *)ml_io_map(0xf900d000, 0x1000);
	volatile uint32_t *saw = (volatile uint32_t *)ml_io_map(0xf9065000, 0x1000);
	if (pll == NULL || glb == NULL || saw == NULL) {
		IOLog("OP2 BOOST: abort: map failed\n");
		return;
	}
	uint32_t mode = pll[0x00 / 4], l = pll[0x04 / 4], user = pll[0x10 / 4];
	uint32_t mux = glb[0x54 / 4];
	uint32_t acc0 = glb[0x84 / 4], acc1 = glb[0x88 / 4], acc2 = glb[0x8c / 4], acc3 = glb[0x90 / 4];
	uint32_t sawid = saw[0x04 / 4], sawver = saw[0xfd0 / 4];
	IOLog("OP2 BOOST: pre: PLL0 MODE=%08x L=%08x USER=%08x MUX=%08x ACC=%x/%x/%x/%x SAW ID=%08x VER=%08x\n",
	    mode, l, user, mux, acc0, acc1, acc2, acc3, sawid, sawver);
	if (!(mode & 0x80000000u) || l != 0x50 || !(user & 0x8u) ||
	    ((mux >> 3) & 3) != 0 || ((mux >> 1) & 3) != 2) {
		IOLog("OP2 BOOST: abort: clock tree not in the expected LK state\n");
		return;
	}
	if ((acc0 | acc1 | acc2 | acc3) != 0) {
		IOLog("OP2 BOOST: abort: mem-acc not at nominal/turbo setting\n");
		return;
	}
	if (!((sawid >> 2) & 1) || (sawver >> 28) != 3) {
		IOLog("OP2 BOOST: abort: SAW2 has no PMIC arbiter / unknown version\n");
		return;
	}

	/* 1. VDD_MX up first (RPM wire value = corner - 1; SUPER_TURBO = 7). */
	if (!op2_rpm_set_corner(OP2_RPM_RSC_SMPA, 2, 6)) {
		IOLog("OP2 BOOST: abort: RPM did not ack VDD_MX super-turbo vote\n");
		return;
	}

	/* 2. APC0 voltage: FTS2 range0, 5 mV/step from 0 V. */
	const uint32_t vlevel = 1145000 / 5000;   /* 0xE5 = 1.145 V */
	uint32_t vlevel_lk = saw[0x14 / 4] & 0xff;   /* what LK left (NOMINAL, ~1.0 V): the throttle returns to it */
	if (saw[0x08 / 4] == 0) {
		saw[0x08 / 4] = 0x01;                 /* SAW2_CFG per DT (qcom,saw2-cfg) */
	}
	saw[0x18 / 4] = 1;                        /* RST: state machine to idle */
	__asm__ volatile("dsb sy" ::: "memory");
	saw[0x1c / 4] = vlevel;                   /* VCTL, port 0 */
	__asm__ volatile("dsb sy" ::: "memory");
	uint32_t sts = 0, waited = 0;
	for (waited = 0; waited < 500; waited++) {
		sts = saw[0x14 / 4];
		if ((sts & 0xff) == vlevel) {
			break;
		}
		IODelay(1);
	}
	if ((sts & 0xff) != vlevel) {
		IOLog("OP2 BOOST: abort: PMIC_STS=%08x never confirmed vlevel 0x%x; clock unchanged\n",
		    sts, vlevel);
		return;
	}
	IODelay(300);                             /* slew 1.0 -> 1.145 V at ~2.4 mV/us, with margin */

	/* 3. HF mux -> PLL0 full rate. */
	uint32_t newmux = (mux & ~(3u << 3)) | (3u << 3);
	glb[0x54 / 4] = newmux;
	op2_boost_glb = glb;
	op2_boost_saw = saw;
	op2_boost_mux_fast = newmux;
	op2_boost_mux_slow = mux;
	op2_vlevel_fast = vlevel;
	/* PMIC_STS reads 0 before the first VCTL write (round fable-46): then LK's documented NOMINAL 1.0 V (0xC8),
	 * the stock voltage LK itself runs 768 MHz at */
	op2_vlevel_slow = (vlevel_lk >= 0xb4 && vlevel_lk < vlevel) ? vlevel_lk : 1000000 / 5000;
	IOLog("OP2 BOOST: throttle returns to vlevel 0x%x (LK's)\n", op2_vlevel_slow);
	extern int op2_temp_throttled;   /* op2_snapshot.c: already too hot when the boost ran -> back down at once */
	if (op2_temp_throttled) {
		IOLog("OP2 BOOST: thermal throttle active: back to 768 MHz\n");
		op2_cpu_set_throttle(1);
	}
	__asm__ volatile("dsb sy; isb" ::: "memory");
	IODelay(10);
	uint32_t mhz = op2_measure_cpu_mhz();
	IOLog("OP2 BOOST: done: APC0 vlevel 0x%x (PMIC_STS %08x after %uus), MUX %08x -> %08x, cpu now ~%u MHz\n",
	    vlevel, sts, waited, mux, glb[0x54 / 4], mhz);
}

bool
AppleCLCD::start(IOService *provider)
{
	PE_Video video;

	if (!IOService::start(provider)) {
		return false;
	}
	op2_threaddump_start(); /* no-op unless op2_deadline is armed */
	{
		uint32_t mhz = op2_measure_cpu_mhz();
		IOLog("OP2 CPU: effective clock ~%u MHz (A53 Fmax 1555; 0 = not measured)\n", mhz);
	}
	{
		/* READ-ONLY dump of the A53 clock tree and APC0 voltage controller,
		 * to plan the 767 -> 1555 MHz bring-up from real register values. */
		volatile uint32_t *pll = (volatile uint32_t *)ml_io_map(0xf9015000, 0x1000); /* c0_pll */
		volatile uint32_t *mux = (volatile uint32_t *)ml_io_map(0xf900d000, 0x1000); /* c0_mux (ALIAS0_GLB) */
		volatile uint32_t *saw = (volatile uint32_t *)ml_io_map(0xf9065000, 0x1000); /* APC0 SAW2 */
		if (pll != NULL) {
			IOLog("OP2 CLK: PLL0 MODE=%08x L=%08x ALPHA=%08x USER=%08x CFG=%08x STAT=%08x TLO=%08x THI=%08x\n",
			    pll[0x00 / 4], pll[0x04 / 4], pll[0x08 / 4], pll[0x10 / 4],
			    pll[0x14 / 4], pll[0x1c / 4], pll[0x20 / 4], pll[0x24 / 4]);
			IOLog("OP2 CLK: PLL1 MODE=%08x L=%08x ALPHA=%08x USER=%08x CFG=%08x STAT=%08x TLO=%08x THI=%08x\n",
			    pll[0x40 / 4], pll[0x44 / 4], pll[0x48 / 4], pll[0x50 / 4],
			    pll[0x54 / 4], pll[0x5c / 4], pll[0x60 / 4], pll[0x64 / 4]);
		}
		if (mux != NULL) {
			IOLog("OP2 CLK: MUX 0x50=%08x 0x54=%08x 0x58=%08x DIAG=%08x\n",
			    mux[0x50 / 4], mux[0x54 / 4], mux[0x58 / 4], mux[0x1c / 4]);
		}
		if (saw != NULL) {
			IOLog("OP2 CLK: SAW2 ID=%08x CFG=%08x SPM_STS=%08x AVS_STS=%08x PMIC_STS=%08x VCTL=%08x AVS_CTL=%08x AVS_LIM=%08x\n",
			    saw[0x04 / 4], saw[0x08 / 4], saw[0x0c / 4], saw[0x10 / 4],
			    saw[0x14 / 4], saw[0x1c / 4], saw[0x20 / 4], saw[0x24 / 4]);
			IOLog("OP2 CLK: SAW2 DATA0..7=%08x %08x %08x %08x %08x %08x %08x %08x VER=%08x\n",
			    saw[0x40 / 4], saw[0x44 / 4], saw[0x48 / 4], saw[0x4c / 4],
			    saw[0x50 / 4], saw[0x54 / 4], saw[0x58 / 4], saw[0x5c / 4], saw[0xfd0 / 4]);
		}
	}
	{
		/* Defer the boost: RPM SMD is still being brought up by OP2GPU at
		 * this point (round 3 got "RPM did not ack" from that race), and
		 * op2_rpm_request is not re-entrant. 20 s later it is idle. */
		thread_t bth;
		if (kernel_thread_start(op2_cpu_boost_thread, NULL, &bth) == KERN_SUCCESS) {
			thread_deallocate(bth);
		}
	}
	PE_current_console(&video);
	fPhys = video.v_baseAddr;
	fWidth = (uint32_t)video.v_width;
	fHeight = (uint32_t)video.v_height;
	fRowBytes = (uint32_t)video.v_rowBytes;
	if (fPhys == 0 || fWidth == 0 || fHeight == 0 || fRowBytes < fWidth * 4) {
		fPhys = OP2_FB_DEFAULT_PHYS;
		fWidth = OP2_FB_DEFAULT_WIDTH;
		fHeight = OP2_FB_DEFAULT_HEIGHT;
		fRowBytes = OP2_FB_DEFAULT_ROWBYTES;
	}

	volatile uint32_t *mmcc = (volatile uint32_t *)ml_io_map(0xfd8c0000, 0x6000);
	fMdpCtl                 = (volatile uint32_t *)ml_io_map(0xfd902000, 0x1000);
	volatile uint32_t *vig0 = (volatile uint32_t *)ml_io_map(0xfd905000, 0x1000);
	volatile uint32_t *rgb0 = (volatile uint32_t *)ml_io_map(0xfd915000, 0x1000);
	volatile uint32_t *dma0 = (volatile uint32_t *)ml_io_map(0xfd925000, 0x1000);
	volatile uint32_t *lm0  = (volatile uint32_t *)ml_io_map(0xfd945000, 0x1000);
	fMdpPP                  = (volatile uint32_t *)ml_io_map(0xfd971000, 0x1000);
	fDsi0                   = (volatile uint32_t *)ml_io_map(0xfd998000, 0x1000);

	if (mmcc != NULL) {
		IOLog("OP2 FB: MMCC MDSS AHB=0x%08x AXI=0x%08x MDP=0x%08x PCLK0=0x%08x BYTE0=0x%08x GDSC=0x%08x\n",
		    mmcc[0x2308 / 4], mmcc[0x2314 / 4], mmcc[0x231c / 4],
		    mmcc[0x2334 / 4], mmcc[0x233c / 4], mmcc[0x2304 / 4]);
	}
	if (fMdpCtl != NULL) {
		IOLog("OP2 FB: CTL0 LAYER0=0x%08x LAYER1=0x%08x TOP=0x%08x FLUSH=0x%08x START=0x%08x\n",
		    fMdpCtl[0x00 / 4], fMdpCtl[0x04 / 4], fMdpCtl[0x14 / 4], fMdpCtl[0x18 / 4], fMdpCtl[0x1c / 4]);
	}
	if (lm0 != NULL) {
		IOLog("OP2 FB: LM0 OP=0x%08x OUT=0x%08x BORDER=0x%08x\n",
		    lm0[0x00 / 4], lm0[0x04 / 4], lm0[0x08 / 4]);
	}
	if (vig0 != NULL) {
		IOLog("OP2 FB: VIG0 SRC0=0x%08x SIZE=0x%08x STRIDE=%u FMT=0x%08x UNPACK=0x%08x\n",
		    vig0[0x14 / 4], vig0[0x00 / 4], vig0[0x24 / 4], vig0[0x30 / 4], vig0[0x34 / 4]);
	}
	if (rgb0 != NULL) {
		IOLog("OP2 FB: RGB0 SRC0=0x%08x SIZE=0x%08x STRIDE=%u FMT=0x%08x UNPACK=0x%08x\n",
		    rgb0[0x14 / 4], rgb0[0x00 / 4], rgb0[0x24 / 4], rgb0[0x30 / 4], rgb0[0x34 / 4]);
	}
	if (dma0 != NULL) {
		IOLog("OP2 FB: DMA0 SRC0=0x%08x SIZE=0x%08x STRIDE=%u FMT=0x%08x UNPACK=0x%08x\n",
		    dma0[0x14 / 4], dma0[0x00 / 4], dma0[0x24 / 4], dma0[0x30 / 4], dma0[0x34 / 4]);
	}
	if (fMdpPP != NULL) {
		IOLog("OP2 FB: PP0 TEAR=0x%08x SYNC_H=0x%08x AUTO=0x%08x LINES=%u\n",
		    fMdpPP[0x00 / 4], fMdpPP[0x08 / 4], fMdpPP[0x38 / 4], fMdpPP[0x28 / 4]);
	}
	if (fDsi0 != NULL) {
		IOLog("OP2 FB: DSI0 CTRL=0x%08x STAT=0x%08x MDP_CTRL=0x%08x TRIG=0x%08x MUX=0x%08x\n",
		    fDsi0[0x00 / 4], fDsi0[0x04 / 4], fDsi0[0x3c / 4], fDsi0[0x80 / 4], fDsi0[0x84 / 4]);
	}

	fActivePipe = NULL;
	fPipeFlushBit = 0;
	fPipeHasScaler = false;
	const char *pipeName = "NONE";

	if (rgb0 != NULL && rgb0[0x14 / 4] != 0) {
		fActivePipe = rgb0;
		fPipeFlushBit = 1u << 3; /* RGB0 bit 3 */
		pipeName = "RGB0";
	} else if (vig0 != NULL && vig0[0x14 / 4] != 0) {
		fActivePipe = vig0;
		fPipeFlushBit = 1u << 0; /* VIG0 bit 0 */
		fPipeHasScaler = true; /* VIG pipes carry the MDP5 phase-step downscaler */
		pipeName = "VIG0";
	} else if (dma0 != NULL && dma0[0x14 / 4] != 0) {
		fActivePipe = dma0;
		fPipeFlushBit = 1u << 6; /* DMA0 bit 6 */
		pipeName = "DMA0";
	}

	if (fActivePipe != NULL) {
		uint64_t lkPhys = fActivePipe[0x14 / 4];
		uint32_t lkStride = fActivePipe[0x24 / 4];
		uint32_t lkSize = fActivePipe[0x00 / 4];
		IOLog("OP2 FB: active bootloader pipe %s at 0x%llx (size=0x%08x stride=%u)\n",
		    pipeName, lkPhys, lkSize, lkStride);
		if (lkPhys != 0) {
			fPhys = lkPhys;
		}
		if (lkStride >= 1080 * 4) {
			fRowBytes = lkStride;
		}
		if ((lkSize & 0xffff) >= 1080) {
			fWidth = lkSize & 0xffff;
			fHeight = (lkSize >> 16) & 0xffff;
		}

		/*
		 * Phase 8 MDP5 Direct Scanout: Reconfigure active pipe to 32bpp
		 * XRGB/BGRA at fPhys with 4352-byte row stride.
		 */
		fActivePipe[0x00 / 4] = (fHeight << 16) | fWidth;
		fActivePipe[0x04 / 4] = (fHeight << 16) | fWidth;
		fActivePipe[0x08 / 4] = 0;
		fActivePipe[0x0c / 4] = (fHeight << 16) | fWidth;
		fActivePipe[0x10 / 4] = 0;
		fActivePipe[0x14 / 4] = (uint32_t)fPhys;
		fActivePipe[0x24 / 4] = fRowBytes;
		fActivePipe[0x30 / 4] = 0x000236ffu;
		fActivePipe[0x34 / 4] = 0x03020001u;
		if (fPipeHasScaler) {
			fActivePipe[0x204 / 4] = 0;
		}
		__asm__ volatile("dsb sy" ::: "memory");
		IOLog("OP2 FB: reconfigured active pipe %s to 32bpp at 0x%llx (%ux%u stride=%u)\n",
		    pipeName, fPhys, fWidth, fHeight, fRowBytes);
	} else {
		IOLog("OP2 FB: no active pipe found, configuring RGB0 on LM0\n");
		fActivePipe = rgb0;
		fPipeFlushBit = 1u << 3;
		if (rgb0 != NULL) {
			rgb0[0x00 / 4] = (fHeight << 16) | fWidth;
			rgb0[0x04 / 4] = (fHeight << 16) | fWidth;
			rgb0[0x08 / 4] = 0;
			rgb0[0x0c / 4] = (fHeight << 16) | fWidth;
			rgb0[0x10 / 4] = 0;
			rgb0[0x14 / 4] = (uint32_t)fPhys;
			rgb0[0x24 / 4] = fRowBytes;
			rgb0[0x30 / 4] = 0x000236ffu;
			rgb0[0x34 / 4] = 0x03020001u;
		}
		if (fMdpCtl != NULL) {
			fMdpCtl[0x00 / 4] = 1 << 9; /* RGB0 stage BASE on LM0 */
		}
	}

	fLock = IOLockAlloc();
	fClients = OSArray::withCapacity(4);
	fSurface = op2_surface_create_physical(fPhys, fWidth, fHeight, fRowBytes, OP2_FB_FORMAT_BGRA);
	fPixels = fSurface != NULL ? fSurface->kernelAddress() : 0;
	fWorkLoop = IOWorkLoop::workLoop();
	if (fLock == NULL || fClients == NULL || fPixels == 0 || fWorkLoop == NULL) {
		IOLog("OP2 FB: setup failed\n");
		OSSafeReleaseNULL(fSurface);    /* stays published: also held by the table */
		OSSafeReleaseNULL(fWorkLoop);
		return false;
	}
	fVsync = IOTimerEventSource::timerEventSource(this, &AppleCLCD::vsyncFired);
	if (fVsync == NULL || fWorkLoop->addEventSource(fVsync) != kIOReturnSuccess) {
		IOLog("OP2 FB: no vsync timer\n");
		return false;
	}
	uint32_t hz = 60;
	PE_parse_boot_argn("op2fb_hz", &hz, sizeof(hz));
	fFlushTicks = (hz == 0 || hz >= 60) ? 1 : 60 / hz;
	int cui = 1;
	PE_parse_boot_argn("op2fb_client_ui", &cui, sizeof(cui));
	fClientUI = (cui != 0);
	fVsync->setTimeoutUS(OP2_FB_VSYNC_US);

	IOLog("OP2 FB: fPixels=0x%lx fPhys=0x%llx %ux%u stride=%u\n",
	    (unsigned long)fPixels, fPhys, fWidth, fHeight, fRowBytes);

	/* Render authentic Apple boot splash: pitch-black background with white Apple logo & progress bar */
	if (fPixels != 0) {
		uint8_t *fb = (uint8_t *)fPixels;

		/* 1. Deep pitch-black background */
		bzero(fb, (size_t)fHeight * fRowBytes);

		/* 2. Official Apple Retina Boot Logo from ProgressUI (194x237 @3x) */
		int32_t lx0 = (int32_t)(fWidth / 2) - (APPLE_LOGO_WIDTH / 2);   /* 540 - 97 = 443 */
		int32_t ly0 = 850 - (APPLE_LOGO_HEIGHT / 2);                    /* 850 - 118 = 732 */

		for (int32_t ly = 0; ly < APPLE_LOGO_HEIGHT; ly++) {
			int32_t y = ly0 + ly;
			if (y < 0 || y >= (int32_t)fHeight) continue;
			uint32_t *row = (uint32_t *)(void *)(fb + (uint32_t)y * fRowBytes);
			const uint8_t *alpha_row = &apple_logo_alpha[ly * APPLE_LOGO_WIDTH];
			for (int32_t lx = 0; lx < APPLE_LOGO_WIDTH; lx++) {
				uint8_t a = alpha_row[lx];
				if (a != 0) {
					int32_t x = lx0 + lx;
					if (x >= 0 && x < (int32_t)fWidth) {
						row[x] = (0xFFu << 24) | ((uint32_t)a << 16) | ((uint32_t)a << 8) | (uint32_t)a;
					}
				}
			}
		}

		/* 3. Authentic iOS Progress Bar: initialize animator at 0% */
		fBootStartMachTime = mach_absolute_time();
		fLastProgressPct = 0;
		fUserlandTookOver = false;
		drawProgressBar(0);
		__asm__ volatile("dsb sy" ::: "memory");
		IOLog("OP2 FB: authentic Apple boot splash rendered to framebuffer (0x%llx)\n", fPhys);
	}

	/* Kick command-mode panel to replace the LK splash logo with iOS boot screen */
	op2_fb_kick_panel(fMdpCtl);
	delay(20000); /* 20 ms to allow initial frame transfer to finish cleanly */
	IOLog("OP2 FB: initial panel kickoff complete (kicks=%llu)\n", op2_fb_kick_count);

	/* Immediate-present timer + swap completion (see presentPending). */
	fPresentNow = IOTimerEventSource::timerEventSource(this, &AppleCLCD::presentNowFired);
	if (fPresentNow != NULL && fWorkLoop->addEventSource(fPresentNow) != kIOReturnSuccess) {
		fPresentNow->release();
		fPresentNow = NULL;
	}
	{
		uint32_t sw = 1;
		PE_parse_boot_argn("op2fb_swapwait", &sw, sizeof(sw));
		fSwapWaitEnabled = (sw != 0);
	}

	/* Second panel buffer from the LK-reserved carveout (slot 1). */
	fBufPhys[0] = fPhys;
	fBufPixels[0] = fPixels;
	fFrontBuf = 0;
	fDoubleBuffer = false;
	fEarlyCopy = false;
	fEarlyRejected = false;
	fFlipPending = false;
	{
		uint32_t db = 1;   /* 2026-11-17: back on. Only panel-sized surfaces use the carveout now (OP2Surface), so slot 1 is free; single-buffered copies write the buffer the DSI is reading (flashes). op2fb_double=0 turns it off. */
		PE_parse_boot_argn("op2fb_double", &db, sizeof(db));
		uint64_t bytes = ((uint64_t)fHeight * fRowBytes + 0xfffULL) & ~0xfffULL;
		uint64_t back = (db != 0) ? op2_carveout_alloc_slot((uint32_t)bytes) : 0;
		if (back != 0) {
			fBufMem = IOMemoryDescriptor::withPhysicalAddress(back, bytes, kIODirectionInOut);
			if (fBufMem != NULL) {
				fBufMap = fBufMem->map(kIOMapWriteCombineCache);
			}
			if (fBufMap != NULL && fBufMap->getVirtualAddress() != 0) {
				fBufPhys[1] = back;
				fBufPixels[1] = (uintptr_t)fBufMap->getVirtualAddress();
				memcpy((void *)fBufPixels[1], (const void *)fPixels, (size_t)fHeight * fRowBytes);
				__asm__ volatile("dsb sy" ::: "memory");
				fDoubleBuffer = true;
			}
		}
		IOLog("OP2 FB: double buffering %s (buf0 0x%llx, buf1 0x%llx), swapwait %s\n",
		    fDoubleBuffer ? "ON" : "off", fBufPhys[0], fBufPhys[1],
		    fSwapWaitEnabled ? "on" : "off");
	}

	setProperty(kIOUserClientClassKey, "IOMobileFramebufferUserClient");
	setProperty("external", false);
	setProperty("DisplayPipePlaneBaseAlignment", 64ULL, 32);
	registerService();
	IOLog("OP2 FB: AppleCLCD %ux%u stride %u at 0x%llx, default surface %u, flush every %u vsyncs, client-ui %s\n",
	    fWidth, fHeight, fRowBytes, fPhys, fSurface->id, fFlushTicks, fClientUI ? "on" : "off");
	op2_fb_instance = this;

	/*
	 * Phase 1 Metal Bring-Up: Publish IOAcceleratorES service.
	 * CoreAnimation and Metal.framework discover graphics accelerators via
	 * IOServiceMatching("IOAcceleratorES") and IOServiceMatching("AGXAccelerator").
	 * When boot-arg metal=N is present (e.g. metal=1, metal=2), we set
	 * MetalPluginName and MetalPluginClassName so userland loads OP2Metal.bundle.
	 * By default (metal=0), no accelerator is published so CoreAnimation uses
	 * its native, fully-functional CPU software rasterizer.
	 */
	uint32_t metal = 0;
	PE_parse_boot_argn("metal", &metal, sizeof(metal));
	if (metal > 0) {
		AGXAccelerator *accelerator = OSTypeAlloc(AGXAccelerator);
		if (accelerator != NULL) {
			if (accelerator->init() && accelerator->attach(this)) {
				accelerator->setName("sgx");
				accelerator->setProperty("MetalPluginName", "OP2Metal");
				accelerator->setProperty("MetalPluginClassName", "OP2MetalDevice");
				accelerator->setProperty("IOGLBundleName", "OP2Metal");
				accelerator->setProperty("IOGLPluginName", "OP2Metal");
				accelerator->setProperty("IOGLESBundleName", "AGXGLDriver");
				accelerator->setProperty("IOGLESMetalBundleName", "AGXGLDriver");
				IOLog("OP2 FB: metal=%u: published IOAcceleratorES/AGXAccelerator naming OP2Metal\n", metal);
				accelerator->setProperty(kIOUserClientClassKey, "OP2GPUUserClient");
				accelerator->registerService();
			}
			accelerator->release();
		}
	} else {
		IOLog("OP2 FB: metal disabled; using native CPU software rasterization\n");
	}

	return true;
}

void
AppleCLCD::vsyncFired(OSObject *owner, IOTimerEventSource *sender)
{
	AppleCLCD *fb = (AppleCLCD *)owner;
	uint64_t now = mach_absolute_time();
	uint64_t count = ++fb->fVsyncCount;
	uint64_t values[6] = {
		(uint32_t)count, (uint32_t)(count >> 32),
		(uint32_t)now, (uint32_t)(now >> 32),
		0, 0,
	};

	/* 60s display timeline: WHEN does the panel go black (timer vs event).
	 * ~20 lines/run; survives the ring. */
	if ((count % 3600) == 0 && op2_fb_instance != NULL &&
	    op2_fb_instance->fPixels != 0) {
		uint32_t tl_nz = 0, tl_n = 0;
		const uint32_t *px = (const uint32_t *)op2_fb_instance->fPixels;
		uint64_t total = (uint64_t)op2_fb_instance->fWidth *
		    (uint64_t)op2_fb_instance->fHeight;
		for (uint64_t i = 0; i < total && tl_n < 2000; i += 997) {
			if ((px[i] & 0x00FFFFFFu) != 0) {
				tl_nz++;
			}
			tl_n++;
		}
		IOLog("OP2 FB TL min%llu: presents=%llu kicks=%llu shown=%u panel_nz=%u\n",
		    count / 3600, op2_fb_present_count, op2_fb_kick_count,
		    op2_fb_last_shown, tl_nz);
	}

	/* Animate authentic Apple boot progress bar smoothly until real userland UI takes over */
	if (!fb->fUserlandTookOver && fb->fPixels != 0) {
		/* MSM8994 counter runs at 19.2 MHz (19200 ticks per ms) */
		uint64_t elapsed_ticks = now - fb->fBootStartMachTime;
		uint32_t elapsed_ms = (uint32_t)(elapsed_ticks / 19200);
		uint32_t target_pct = 0;
		if (elapsed_ms < 3000) {
			target_pct = (elapsed_ms * 35) / 3000;
		} else if (elapsed_ms < 8000) {
			target_pct = 35 + ((elapsed_ms - 3000) * 35) / 5000;
		} else if (elapsed_ms < 15000) {
			target_pct = 70 + ((elapsed_ms - 8000) * 25) / 7000;
		} else if (elapsed_ms < 20000) {
			target_pct = 95 + ((elapsed_ms - 15000) * 5) / 5000;
		} else {
			target_pct = 100;
		}

		if (target_pct > fb->fLastProgressPct) {
			fb->fLastProgressPct = target_pct;
			fb->drawProgressBar(target_pct);
		}
	}

	IOLockLock(fb->fLock);
	for (unsigned int i = 0; i < fb->fClients->getCount(); i++) {
		IOMobileFramebufferUserClient *client =
		    (IOMobileFramebufferUserClient *)fb->fClients->getObject(i);
		client->notify(OP2_FB_NOTIFY_VSYNC, values, 6);
	}
	bool flush = fb->fDirty && (count % fb->fFlushTicks) == 0;
	IOLockUnlock(fb->fLock);
	sender->setTimeoutUS(OP2_FB_VSYNC_US);
	if (flush) {
		fb->presentPending();   /* safety net; SwapEnd normally presents at once */
	}
}

/* Minimum spacing between DSI kicks: a command-mode transfer must finish
 * before the next START, or the START is lost and the panel keeps a stale
 * frame. 15.6 ms at 19.2 MHz. Frames are deferred, never dropped. */
#define OP2_FB_MIN_KICK_TICKS 300000ULL

static uint64_t op2_fb_perf_n, op2_fb_perf_sum_us, op2_fb_perf_max_us;
static uint64_t op2_fb_perf_deferred, op2_fb_perf_early, op2_fb_swapwait_timeouts, op2_fb_swapwait_waits;
static bool op2_fb_hwscale_active = false;  /* VIG scaler currently programmed (must be cleared for copy path) */
static uint64_t op2_fb_hwscale_count = 0;
static void op2_fb_thumb(OP2Surface *s, uint64_t pc);

void
AppleCLCD::presentNowFired(OSObject *owner, __unused IOTimerEventSource *sender)
{
	((AppleCLCD *)owner)->presentPending();
}

/* Workloop context only (vsync timer or present-now timer). */
void
AppleCLCD::presentPending(void)
{
	uint64_t now = mach_absolute_time();
	bool early = false;
	if (op2_fb_last_kick != 0 && (now - op2_fb_last_kick) < OP2_FB_MIN_KICK_TICKS) {
		uint64_t wait_ticks = OP2_FB_MIN_KICK_TICKS - (now - op2_fb_last_kick);
		op2_fb_perf_deferred++;
		if (fPresentNow != NULL) {
			fPresentNow->setTimeoutUS((uint32_t)(wait_ticks * 10 / 192) + 50);
		}
		/* Too soon to kick, but copy the frame now (into the back buffer,
		 * which the panel is not reading) so the client can reuse its
		 * buffer; a deferred copy read it mid-render: the flashes. */
		if (!fDoubleBuffer) {
			return;
		}
		early = true;
	} else if (fFlipPending) {
		IOLockLock(fLock);
		bool dirty = fDirty;
		IOLockUnlock(fLock);
		if (!dirty) {
			flipAndKick();
			return;
		}
		/* A newer frame is waiting: it overwrites the back buffer and is
		 * kicked straight away below. */
		fFlipPending = false;
	}

	uint8_t *rec = NULL;
	IOLockLock(fLock);
	if (fDirty) {
		rec = (uint8_t *)IOMallocData(OP2_FB_SWAP_REC_SIZE);
		if (rec != NULL) {
			memcpy(rec, fPending, OP2_FB_SWAP_REC_SIZE);
			fDirty = false;
		}
	}
	IOLockUnlock(fLock);
	if (rec == NULL) {
		return;
	}

	uint32_t swapID = 0;
	memcpy(&swapID, rec + 0x40, 4);
	uint64_t t0 = mach_absolute_time();
	fEarlyCopy = early;
	fEarlyRejected = false;
	present(rec);
	fEarlyCopy = false;
	uint64_t us = ((mach_absolute_time() - t0) * 10) / 192;
	if (early && fEarlyRejected) {
		/* Not a back-buffer frame: put it back for the timer to present. */
		IOLockLock(fLock);
		if (!fDirty) {
			memcpy(fPending, rec, OP2_FB_SWAP_REC_SIZE);
			fDirty = true;
		}
		IOLockUnlock(fLock);
		IOFreeData(rec, OP2_FB_SWAP_REC_SIZE);
		return;
	}
	if (early) {
		op2_fb_perf_early++;
	}
	IOFreeData(rec, OP2_FB_SWAP_REC_SIZE);

	IOLockLock(fLock);
	if ((int32_t)(swapID - fCompletedSwapID) > 0) {
		fCompletedSwapID = swapID;
	}
	IOLockWakeup(fLock, &fCompletedSwapID, false);
	IOLockUnlock(fLock);

	op2_fb_perf_n++;
	op2_fb_perf_sum_us += us;
	if (us > op2_fb_perf_max_us) {
		op2_fb_perf_max_us = us;
	}
	if ((op2_fb_perf_n % 600) == 0) {
		IOLog("OP2 FB PERF: presents=%llu avg=%lluus max=%lluus deferred=%llu early=%llu swapwait waits=%llu timeouts=%llu dbl=%d\n",
		    op2_fb_perf_n, op2_fb_perf_sum_us / 600, op2_fb_perf_max_us,
		    op2_fb_perf_deferred, op2_fb_perf_early, op2_fb_swapwait_waits, op2_fb_swapwait_timeouts,
		    (int)fDoubleBuffer);
		op2_fb_perf_sum_us = 0;
		op2_fb_perf_max_us = 0;
	}
}

void
AppleCLCD::addClient(IOMobileFramebufferUserClient *client)
{
	IOLockLock(fLock);
	fClients->setObject(client);
	IOLockUnlock(fLock);
}

void
AppleCLCD::removeClient(IOMobileFramebufferUserClient *client)
{
	IOLockLock(fLock);
	unsigned int i = fClients->getNextIndexOfObject(client, 0);
	if (i != (unsigned int)-1) {
		fClients->removeObject(i);
	}
	IOLockUnlock(fLock);
}

static uint32_t
get32(const uint8_t *p, uint32_t off)
{
	uint32_t v;
	memcpy(&v, p + off, 4);
	return v;
}

/*
 * Strided non-zero sample of a surface's pixels (every 8th pixel, capped).
 * Tells "userland renders blanks" apart from "kernel copies the wrong
 * memory". Fully bounds-checked; safe for any userland-created surface.
 */
static uint32_t
op2_fb_sample_nz(OP2Surface *s, uint32_t cap)
{
	uint32_t nz = 0;
	if (s == NULL) {
		return 0;
	}
	uintptr_t psrc = s->kernelAddress();
	uint32_t pbpe = s->bytesPerElement ? s->bytesPerElement : 4;
	if (psrc == 0 || pbpe != 4 || s->memory == NULL ||
	    s->width == 0 || s->height == 0 ||
	    (uint64_t)s->bytesPerRow < (uint64_t)s->width * 4 ||
	    (uint64_t)s->offset + (uint64_t)s->height * s->bytesPerRow > s->memory->getLength()) {
		return 0;
	}
	for (uint32_t y = 0; y < s->height && nz <= cap; y += 8) {
		const uint32_t *row = (const uint32_t *)(psrc + s->offset +
		    (uint64_t)y * s->bytesPerRow);
		for (uint32_t x = 0; x < s->width; x += 8) {
			if ((row[x] & 0x00ffffff) != 0 && ++nz > cap) {
				break;
			}
		}
	}
	return nz;
}

/*
 * Bottom-half non-zero sample (rows height/2..height, every 8th pixel).
 * Catches the "bottom half flashes black" frame: top half rendered, bottom
 * half still zeros because the GPU had not finished when we scanned out.
 * Same bounds discipline as op2_fb_sample_nz.
 */
static uint32_t
op2_fb_sample_nz_bottom(OP2Surface *s, uint32_t cap)
{
	uint32_t nz = 0;
	if (s == NULL) {
		return 0;
	}
	uintptr_t psrc = s->kernelAddress();
	uint32_t pbpe = s->bytesPerElement ? s->bytesPerElement : 4;
	if (psrc == 0 || pbpe != 4 || s->memory == NULL ||
	    s->width == 0 || s->height == 0 ||
	    (uint64_t)s->bytesPerRow < (uint64_t)s->width * 4 ||
	    (uint64_t)s->offset + (uint64_t)s->height * s->bytesPerRow > s->memory->getLength()) {
		return 0;
	}
	for (uint32_t y = s->height / 2; y < s->height && nz <= cap; y += 8) {
		const uint32_t *row = (const uint32_t *)(psrc + s->offset +
		    (uint64_t)y * s->bytesPerRow);
		for (uint32_t x = 0; x < s->width; x += 8) {
			if ((row[x] & 0x00ffffff) != 0 && ++nz > cap) {
				break;
			}
		}
	}
	return nz;
}

static uint64_t op2_fb_swap_count;
static uint64_t op2_fb_present_no_target;
static uint64_t op2_fb_direct_cur;   /* pipe SRC0 set by direct scanout, 0 = copy mode */

/*
 * Swap records the latest frame submitted by userland (IOMobileFramebufferSwapEnd).
 */
void
AppleCLCD::swap(const uint8_t *rec)
{
	op2_fb_swap_count++;
	if (op2_fb_swap_count <= 8 || (op2_fb_swap_count % 100) == 0) {
		IOLog("OP2 FB: swap %llu (id40=%u, s44=%u, maskC8=0x%x)\n",
		    op2_fb_swap_count, get32(rec, 0x40),
		    get32(rec, 0x44), get32(rec, 0xc8));
	}
	if (op2_fb_swap_count <= 10 || (op2_fb_swap_count % 30) == 0) {
		setProperty("OP2SwapCount", op2_fb_swap_count, 64);
	}

	IOLockLock(fLock);
	memcpy(fPending, rec, OP2_FB_SWAP_REC_SIZE);
	fDirty = true;
	fLastSwapEndID = get32(rec, 0x40);
	IOLockUnlock(fLock);
	if (fPresentNow != NULL && fFlushTicks <= 1) {
		fPresentNow->setTimeoutUS(1);
	}
}

/*
 * Present active swap layers to the physical panel memory.
 *
 * In iOS 12 (iPhone 6 Plus, n56ap, 16H81), the swap record structure passed to
 * IOMobileFramebufferSwapEnd (selector 5, size 0x278) layout is:
 *   rec + 0x40: swap ID
 *   rec + 0x44: layer 0 surface ID (fallback 0x54)
 *   rec + 0x48: layer 1 surface ID (fallback 0x58)
 *   rec + 0x4c: layer 2 surface ID (fallback 0x5c)
 *   rec + 0x50: layer 0 source rect (int sx, sy, w, h)
 *   rec + 0xc8: layer enable bitmask (fallback 0x104)
 */
/*
 * VIG0 hardware downscale (MDP5 phase-step scaler): fetch a full 1242x2208
 * Retina surface and scale it to the 1080x1920 panel in the display pipe.
 * Zero CPU copy. Returns true when the pipe was programmed (caller skips the
 * CPU copy/flip); false selects the legacy CPU path (and switches the scaler
 * back off if a previous frame had turned it on).
 * Preconditions are verified at runtime every frame: boot-arg on, VIG pipe,
 * physically contiguous surface covering the whole frame, pipe OUT_SIZE
 * already the panel size. Register map: MDP5 VIG pipe +0x00 SRC_SIZE,
 * +0x04 SRC_IMG_SIZE, +0x0C OUT_SIZE, +0x14 SRC0, +0x24 STRIDE, +0x204
 * SCALE_CONFIG (X_EN|Y_EN + bilinear all components), +0x210/0x214
 * PHASE_STEP = 2^21 * src/dst per axis (Linux mdp5 calc_phase_step).
 */
/*
 * High-Quality Bilinear Subpixel Downsampler (1242x2208 -> 1080x1920):
 * 1080 = 54 blocks of 20 pixels; 1242 = 54 blocks of 23 pixels.
 * Uses exact fixed-point subpixel weights (w0 + w1 = 256) across color channels.
 * Bilinear area filtering eliminates aliasing, shimmering, and jagged edges.
 */
static inline __unused uint32_t   /* kept for reference; the per-frame CPU path must not blend */
op2_blend_pixel(uint32_t p0, uint32_t p1, uint32_t w0, uint32_t w1)
{
	uint32_t rb0 = p0 & 0x00FF00FFu;
	uint32_t ag0 = (p0 >> 8) & 0x00FF00FFu;
	uint32_t rb1 = p1 & 0x00FF00FFu;
	uint32_t ag1 = (p1 >> 8) & 0x00FF00FFu;
	uint32_t rb = ((rb0 * w0 + rb1 * w1) >> 8) & 0x00FF00FFu;
	uint32_t ag = (((ag0 * w0 + ag1 * w1) >> 8) & 0x00FF00FFu) << 8;
	return rb | ag;
}

static bool
op2_fb_try_hwscale(AppleCLCD *fb, OP2Surface *s, uint64_t direct_phys, uint64_t segLen)
{
	static int32_t sHwScale = -1;   /* -1 = boot-arg not yet parsed */
	if (sHwScale < 0) {
		uint32_t hw = 0;   /* Default 0: hardware scaler requires source inside LK carveout */
		PE_parse_boot_argn("op2fb_hwscale", &hw, sizeof(hw));
		sHwScale = (hw != 0) ? 1 : 0;
	}
	if (sHwScale <= 0) {
		return false;
	}
	bool want = (sHwScale != 0 && fb->fPipeHasScaler && s != NULL &&
	    s->width == 1242 && s->height == 2208 && fb->fActivePipe != NULL);
	uint32_t outSize = 0;
	if (want) {
		outSize = fb->fActivePipe[0x0C / 4];
		if (outSize != (uint32_t)(((uint64_t)fb->fHeight << 16) | fb->fWidth))
			want = false;
	}
	uint64_t frameLen = 0;
	if (want) {
		if (s->bytesPerRow < (uint64_t)s->width * 4)
			want = false;
		else {
			frameLen = (uint64_t)s->bytesPerRow * s->height;
			if (direct_phys == 0 || s->offset + frameLen > segLen)
				want = false;
			/*
			 * Qualcomm MSM8994 MDP5 hardware pipe DMA can ONLY access memory within
			 * the Little Kernel continuous splash carveout window (0x03400000 to 0x05600000).
			 * Fetching from general DRAM causes MDP5 AXI bus faults / underflows,
			 * wedging DSI command transfers and leaving the physical panel frozen.
			 */
			if (direct_phys < OP2_CARVEOUT_BASE || (direct_phys + s->offset + frameLen) > (OP2_CARVEOUT_BASE + OP2_CARVEOUT_SIZE))
				want = false;
		}
	}
	if (!want) {
		if (op2_fb_hwscale_active) {
			fb->fActivePipe[0x204 / 4] = 0;
			op2_fb_hwscale_active = false;
		}
		return false;
	}
	/* Cache-clean the GPU-written source so the pipe fetches fresh pixels. */
	if (s->memory != NULL && s->offset < s->memory->getLength()) {
		uint64_t len = frameLen;
		if (s->offset + len > s->memory->getLength())
			len = s->memory->getLength() - s->offset;
		uintptr_t src = s->kernelAddress();
		if (src != 0 && op2_gpu_renders())
			op2_civac((vm_offset_t)(src + s->offset), (vm_size_t)len);
	}
	/*
	 * Isolation mode (op2fb_hwscale=2): 1:1 direct fetch of the surface's
	 * top-left 1080x1920 with the scaler OFF. If the panel refreshes with a
	 * cropped image, the pipe reads GPU memory fine and the scaler block is
	 * at fault; if it stays stale, the pipe cannot read this memory at all.
	 */
	{
		static int32_t sHwScaleMode = 0;    /* 1 = downscale, 2 = 1:1 fetch test */
		if (sHwScaleMode == 0) {
			uint32_t m = 1;
			PE_parse_boot_argn("op2fb_hwscale", &m, sizeof(m));
			sHwScaleMode = (m == 2) ? 2 : 1;
		}
		if (sHwScaleMode == 2) {
			fb->fActivePipe[0x204 / 4] = 0;   /* scaler off */
			fb->fActivePipe[0x00 / 4] = (uint32_t)(((uint64_t)fb->fHeight << 16) | fb->fWidth);
			fb->fActivePipe[0x04 / 4] = (uint32_t)(((uint64_t)fb->fHeight << 16) | fb->fWidth);
			fb->fActivePipe[0x14 / 4] = (uint32_t)(direct_phys + s->offset);
			fb->fActivePipe[0x24 / 4] = (uint32_t)s->bytesPerRow;
			op2_fb_hwscale_active = false;
			op2_fb_direct_cur = direct_phys + s->offset;
			static uint32_t sFetchLogged = 0;
			if (++sFetchLogged <= 5 || (sFetchLogged % 60) == 0) {
				IOLog("OP2 FB: VIG0 1:1 direct fetch surf %u at 0x%llx (%ux%u stride %u, switch #%u)\n",
				    s->id, direct_phys + s->offset, s->width, s->height, s->bytesPerRow, sFetchLogged);
			}
			return true;
		}
	}
	fb->fActivePipe[0x00 / 4] = (uint32_t)(((uint64_t)s->height << 16) | s->width);
	fb->fActivePipe[0x04 / 4] = (uint32_t)(((uint64_t)s->height << 16) | s->width);
	fb->fActivePipe[0x14 / 4] = (uint32_t)(direct_phys + s->offset);
	fb->fActivePipe[0x24 / 4] = (uint32_t)s->bytesPerRow;
	{
		uint32_t step = (uint32_t)(((uint64_t)(1u << 21) * 1242) / 1080);
		fb->fActivePipe[0x210 / 4] = step;
		fb->fActivePipe[0x214 / 4] = step;
		/*
		 * NOTE (evening): programming the C12 steps (0x218/0x21C) and the
		 * init phases (0x220-0x22C) wedged the pipe on MSM8994 (no frames,
		 * userland parked). The C03 steps alone drive the panel correctly,
		 * verified live. Leave the rest at reset/LK values.
		 */
	}
	fb->fActivePipe[0x204 / 4] = 0x00055153u;   /* X_EN|Y_EN, bilinear all components */
	op2_fb_hwscale_active = true;
	op2_fb_hwscale_count++;
	op2_fb_direct_cur = direct_phys + s->offset;
	static uint32_t sHwScaleLogged = 0;
	if (++sHwScaleLogged <= 5 || (sHwScaleLogged % 60) == 0) {
		IOLog("OP2 FB: VIG0 hardware downscale surf %u at 0x%llx (%ux%u stride %u, switch #%u)\n",
		    s->id, direct_phys + s->offset, s->width, s->height, s->bytesPerRow, sHwScaleLogged);
	}
	return true;
}

void
AppleCLCD::present(const uint8_t *rec)
{
	uint64_t pc = ++op2_fb_present_count;
	memcpy(op2_fb_last_rec, rec, sizeof(op2_fb_last_rec));
	op2_fb_last_pc = pc;
	if (!fConsolePaused) {
		int quiet = 0;
		fConsolePaused = true;
		if (PE_parse_boot_argn("op2fb_noconsole", &quiet, sizeof(quiet)) && quiet != 0) {
			PE_Video video;
			PE_current_console(&video);
			PE_initialize_console(&video, kPEDisableScreen);
		}
	}

	uint32_t mask = get32(rec, 0xc8);
	if (mask == 0) {
		mask = get32(rec, 0x104);
	}

	uint32_t target_id = 0;
	/* Check layers 0..2 for active surface */
	for (uint32_t l = 0; l < 3; l++) {
		if (mask & (1u << l)) {
			uint32_t id = get32(rec, 0x44 + 4 * l);
			if (id == 0) {
				id = get32(rec, 0x54 + 4 * l);
			}
			if (id >= 2 && id < OP2_SURFACE_MAX_ID) {
				target_id = id;
				break;
			}
		}
	}

	static uint32_t sLastPresentedID = 0;

	/* Fallback: if swap record had no surface ID specified, maintain previous active surface */
	if (target_id == 0 && sLastPresentedID != 0) {
		target_id = sLastPresentedID;
	}

	if (target_id == 0 && fClientUI) {
		/*
		 * GPU-less or early swap policy: if swap record had no target
		 * surface, scan for an active client surface with non-zero pixels.
		 */
		OP2Surface *best = NULL;
		uint32_t best_nz = 100;
		uint32_t highest_fullscreen_id = 0;
		for (uint32_t sid = 2; sid < 256; sid++) {
			OP2Surface *cand = op2_surface_copy(sid);
			if (cand == NULL) {
				continue;
			}
			if (cand->width >= 1000 && cand->height >= 1000) {
				highest_fullscreen_id = cand->id;
				uint32_t nz = op2_fb_sample_nz(cand, 5000);
				if (nz > best_nz) {
					best_nz = nz;
					if (best != NULL) {
						best->release();
					}
					best = cand;
					cand = NULL;
				}
			}
			if (cand != NULL) {
				cand->release();
			}
		}
		if (best != NULL) {
			target_id = best->id;
			best->release();
		} else if (highest_fullscreen_id != 0) {
			target_id = highest_fullscreen_id;
		}
	}

	if (target_id == 0) {
		/* No layer resolved: mask parsing may be wrong for this userland. */
		if (++op2_fb_present_no_target <= 4 || (op2_fb_present_no_target % 60) == 0) {
			IOLog("OP2 FB: present #%llu no target (mask 0x%x id44=%u id48=%u id4c=%u)\n",
			    pc, mask, get32(rec, 0x44), get32(rec, 0x48), get32(rec, 0x4c));
		}
		return;
	}

	OP2Surface *s = (target_id == (fSurface ? fSurface->id : 1)) ? fSurface : op2_surface_copy(target_id);
	if (s == NULL) {
		return;
	}

	if (target_id != sLastPresentedID) {
		/* The render server double-buffers, so this flips every frame:
		 * log only the first few (IOLog per frame cost real CPU). */
		static uint32_t sSwitchLogged = 0;
		sLastPresentedID = target_id;
		if (++sSwitchLogged <= 6) {
			IOLog("OP2 FB: *** DISPLAY SWITCHED TO SURFACE %u (%ux%u) ***\n",
			    target_id, s->width, s->height);
		}
	}

	if (!fUserlandTookOver && target_id >= 2) {
		fUserlandTookOver = true;
		IOLog("OP2 FB: *** DISPLAY HAND-OFF TO USERLAND (surface %u, %ux%u) ***\n",
		    target_id, s->width, s->height);
	}

	uint32_t shown_id = target_id;
	if (fClientUI && s != fSurface && (op2_fb_sample_nz(s, 5000) == 0 || (s->height >= fHeight && op2_fb_sample_nz_bottom(s, 64) == 0))) {
		/*
		 * GPU-less policy (boot-arg op2fb_client_ui=1): the render
		 * server cannot composite without a GPU, so its target stays
		 * blank or half-black (status bar only). Clients still rasterize
		 * into their own full-screen surfaces on CPU; show the fullest
		 * one instead of a black or half-black frame.
		 */
		OP2Surface *best = NULL;
		uint32_t best_nz = 100; /* minimum content threshold */
		for (uint32_t sid = 2; sid < 256; sid++) {
			if (sid == target_id) {
				continue;
			}
			OP2Surface *cand = op2_surface_copy(sid);
			if (cand == NULL) {
				continue;
			}
			if (cand->width >= 1000 && cand->height >= 1000) {
				uint32_t nz = op2_fb_sample_nz(cand, 5000);
				if (nz > best_nz) {
					best_nz = nz;
					if (best != NULL) {
						best->release();
					}
					best = cand;
					cand = NULL;
				}
			}
			if (cand != NULL) {
				cand->release();
			}
		}
		if (best != NULL) {
			s->release(); /* s is a copy here, never fSurface */
			s = best;
			shown_id = best->id;
		}
	}
	static uint32_t sLastShown = 0;
	if (shown_id != sLastShown) {
		static uint32_t sShownLogged = 0;
		sLastShown = shown_id;
		op2_fb_last_shown = shown_id;
		setProperty("OP2LastShown", (uint64_t)shown_id, 32);
		if (++sShownLogged <= 6 || shown_id != target_id) {
			IOLog("OP2 FB: now showing surface %u (swap target %u)\n",
			    shown_id, target_id);
		}
	}

	uint32_t shown_nz = op2_fb_sample_nz(s, 5000);
	op2_fb_last_nz = shown_nz;

	if (pc <= 10 || (pc % 30) == 0) {
		setProperty("OP2PresentCount", pc, 64);
		setProperty("OP2LastShown", (uint64_t)shown_id, 32);
		setProperty("OP2LastNZ", (uint64_t)shown_nz, 32);
		setProperty("OP2UserlandTookOver", fUserlandTookOver);
		setProperty("OP2HwScaleActive", op2_fb_hwscale_active);
		setProperty("OP2HwScaleCount", op2_fb_hwscale_count, 64);
	}

	/*
	 * Half-black flash diagnostic: a full-screen frame with visible content
	 * up top but a completely black bottom half means we scanned out a surface
	 * the GPU had not finished rendering (top rows done, bottom rows zeros).
	 * Legit frames are almost never empty across the entire bottom half, so
	 * log every occurrence (rate-limited) for correlation with on-screen flashes.
	 */
	if (shown_nz > 0 && s->width >= fWidth && s->height >= fHeight) {
		uint32_t bottom_nz = op2_fb_sample_nz_bottom(s, 64);
		if (bottom_nz == 0) {
			static uint32_t sHalfBlackLog = 0;
			if (++sHalfBlackLog <= 8 || (sHalfBlackLog % 30) == 0) {
				IOLog("OP2 FB: HALF-BLACK? present #%llu surf %u (%ux%u) top_nz~%u bottom_nz=0\n",
				    pc, target_id, s->width, s->height, shown_nz);
			}
		}
	}

	if (pc <= 4 || (pc % 60) == 0) {
		/* Rate-limited to protect the 256 KB ramoops window. */
		IOLog("OP2 FB: present #%llu surf %u (%ux%u bpr %u) mask 0x%x shown %u nz~%u\n",
		    pc, target_id, s->width, s->height, s->bytesPerRow, mask, shown_id, shown_nz);
	}

	static bool sHasNonZeroUI = false;
	if (shown_nz > 0) {
		sHasNonZeroUI = true;
	} else {
		/*
		 * Incoming frame has no visible RGB pixels.
		 * If we haven't seen a non-zero frame yet, preserve the Apple boot splash!
		 * If we've already seen a non-zero frame, preserve the active UI!
		 * Under NO circumstances should an all-zero buffer be scanned out or copied to the panel.
		 */
		static uint32_t sPreserveLog = 0;
		if (++sPreserveLog <= 5 || (sPreserveLog % 60) == 0) {
			IOLog("OP2 FB: preserving display (surface %u has no RGB pixels, non_zero=%d)\n",
			    shown_id, (int)sHasNonZeroUI);
		}
		if (s != fSurface) {
			s->release();
		}
		return;
	}

	uint64_t direct_phys = 0;
	IOPhysicalLength segLen = 0;
	if (s->memory != NULL) {
		direct_phys = (uint64_t)s->memory->getPhysicalSegment(0, &segLen);
	}
	op2_fb_last_direct_phys = direct_phys;

	bool directOK = direct_phys >= OP2_CARVEOUT_BASE &&
	    (direct_phys + s->offset + (uint64_t)s->bytesPerRow * s->height) <= (OP2_CARVEOUT_BASE + OP2_CARVEOUT_SIZE) &&
	    segLen >= (uint64_t)s->bytesPerRow * s->height &&
	    s->width == fWidth && s->height == fHeight && fActivePipe != NULL;
	if (fEarlyCopy) {
		/* Early copy is only safe into the back buffer: full frames with
		 * double buffering, never direct scanout (the panel would read the
		 * client's buffer) or partial updates (they write the front buffer).
		 * The hardware scaler is skipped here; such frames take the CPU copy. */
		bool ff = (s->width == 1242 && s->height == 2208) ||
		    (s->width >= fWidth && s->height >= fHeight);
		if (directOK || !fDoubleBuffer || !ff) {
			fEarlyRejected = true;
			if (s != fSurface) {
				s->release();
			}
			return;
		}
	}

	if (directOK) {
		/*
		 * Phase 8 ZERO-COPY HARDWARE DIRECT SCANOUT:
		 * Surface is physically contiguous in hardware DMA memory.
		 * Point MDP5 hardware pipe directly to the surface's physical address.
		 */
		if (direct_phys != op2_fb_direct_cur) {
			op2_fb_direct_cur = direct_phys;
			fActivePipe[0x14 / 4] = (uint32_t)direct_phys;
			fActivePipe[0x24 / 4] = s->bytesPerRow;
			fActivePipe[0x00 / 4] = (s->height << 16) | s->width;
			fActivePipe[0x04 / 4] = (s->height << 16) | s->width;
			fActivePipe[0x30 / 4] = 0x000236ffu;
			fActivePipe[0x34 / 4] = 0x03020001u;
			static uint32_t sDirectScanoutLogged = 0;
			if (++sDirectScanoutLogged <= 5 || (sDirectScanoutLogged % 60) == 0) {
				IOLog("OP2 FB: direct hardware scanout surf %u at 0x%llx (%ux%u stride %u, switch #%u)\n",
				    shown_id, direct_phys, s->width, s->height, s->bytesPerRow, sDirectScanoutLogged);
			}
		}
	} else if (!fEarlyCopy && op2_fb_try_hwscale(this, s, direct_phys, segLen)) {
		/* VIG0 scales the surface straight to the panel: no CPU copy,
		 * no back-buffer flip. Shared tail below releases, barriers, kicks. */
	} else {
		/*
		 * Copy / downsample path. Full-screen frames go to the back
		 * buffer (the one the panel is not reading) and are flipped in
		 * after the copy; partial frames update the front buffer as before.
		 */
		bool fullFrame = (s->width == 1242 && s->height == 2208) ||
		    (s->width >= fWidth && s->height >= fHeight);
		uint32_t dstIdx = (fDoubleBuffer && fullFrame) ? (fFrontBuf ^ 1) : fFrontBuf;
		uintptr_t dstPixels = fBufPixels[dstIdx] != 0 ? fBufPixels[dstIdx] : fPixels;
		uint64_t dstPhys = fBufPhys[dstIdx] != 0 ? fBufPhys[dstIdx] : fPhys;

		uintptr_t src = s->kernelAddress();
		uint32_t bpe = s->bytesPerElement ? s->bytesPerElement : 4;
		if (src != 0 && op2_gpu_renders() && s->memory != NULL && s->offset < s->memory->getLength()) {
			uint64_t len = (uint64_t)s->bytesPerRow * s->height;
			if (s->offset + len > s->memory->getLength())
				len = s->memory->getLength() - s->offset;
			op2_civac((vm_offset_t)(src + s->offset), (vm_size_t)len);
		}
		if (src != 0 && bpe == 4 && dstPixels != 0) {
			if (s->width == 1242 && s->height == 2208) {
				static uint32_t sy_map[1920];
				static bool maps_ready = false;
				if (!maps_ready) {
					for (int32_t y = 0; y < 1920; y++) {
						sy_map[y] = (uint32_t)(((uint64_t)y * 2208 + 1104) / 1920);
					}
					maps_ready = true;
				}
				uint64_t last_retina = (uint64_t)s->offset + (uint64_t)2207 * s->bytesPerRow + (uint64_t)1242 * 4;
				if (s->memory != NULL && last_retina <= s->memory->getLength()) {
					uint64_t t0 = mach_absolute_time();
					for (int32_t y = 0; y < (int32_t)fHeight; y++) {
						uint32_t sy_coord = sy_map[y];
						const uint32_t *src_row = (const uint32_t *)(src + s->offset + (uint64_t)sy_coord * s->bytesPerRow);
						uint32_t *dst_row = (uint32_t *)(dstPixels + (uint64_t)y * fRowBytes);

						/*
						 * High-Speed 20-pixel Block Downsampler (1242 -> 1080):
						 * 1080 = 54 blocks of 20 pixels; 1242 = 54 blocks of 23 pixels.
						 * Block pattern: 7 contiguous pixels, skip 1, 7 contiguous pixels, skip 1, 6 contiguous pixels, skip 2.
						 * Aligned 64-bit pair loads/stores with zero lookup table overhead.
						 * (2026-11-16: restored. A per-pixel bilinear blend replaced it on 2026-11-15 and took ~335 ms a
						 * frame instead of ~6 ms -- the "everything is slow" of that week. Better quality belongs in
						 * the MDP scaler (op2fb_hwscale), not in a CPU loop.)
						 */
						for (int32_t b = 0; b < 54; b++) {
							const uint32_t *sp = src_row + b * 23;
							uint32_t *dp = dst_row + b * 20;

							const uint64_t *sp64 = (const uint64_t *)(uintptr_t)sp;
							uint64_t *dp64 = (uint64_t *)(uintptr_t)dp;

							/* Pixels 0..5 (3x 64-bit) + pixel 6 */
							dp64[0] = sp64[0];
							dp64[1] = sp64[1];
							dp64[2] = sp64[2];
							dp[6]   = sp[6];

							/* Pixel 7 (skip sp[7]) */
							dp[7]   = sp[8];

							/* Pixels 8..13 (3x 64-bit) from sp[9..14] */
							const uint64_t *sp_mid64 = (const uint64_t *)(uintptr_t)(sp + 9);
							uint64_t *dp_mid64 = (uint64_t *)(uintptr_t)(dp + 8);
							dp_mid64[0] = sp_mid64[0];
							dp_mid64[1] = sp_mid64[1];
							dp_mid64[2] = sp_mid64[2];

							/* Pixels 14..19 (3x 64-bit) from sp[16..21] (skip sp[15]) */
							const uint64_t *sp_end64 = (const uint64_t *)(uintptr_t)(sp + 16);
							uint64_t *dp_end64 = (uint64_t *)(uintptr_t)(dp + 14);
							dp_end64[0] = sp_end64[0];
							dp_end64[1] = sp_end64[1];
							dp_end64[2] = sp_end64[2];
							/* sp[22] is skipped */
						}
					}
					uint64_t dt = mach_absolute_time() - t0;
					uint32_t us = (uint32_t)((dt * 10) / 192);
					static uint32_t sDownsampleLogged = 0;
					if (++sDownsampleLogged <= 5 || (sDownsampleLogged % 60) == 0) {
						IOLog("OP2 FB: downsample 1242x2208->1080x1920 took %u us (frame #%u)\n", us, sDownsampleLogged);
					}
				}
			} else {
				int32_t w = (int32_t)s->width;
				int32_t h = (int32_t)s->height;
				if (w > (int32_t)fWidth) w = (int32_t)fWidth;
				if (h > (int32_t)fHeight) h = (int32_t)fHeight;
				uint64_t last = (uint64_t)s->offset + (uint64_t)(h - 1) * s->bytesPerRow + (uint64_t)w * 4;
				if (w > 0 && h > 0 && s->memory != NULL && last <= s->memory->getLength()) {
					uint32_t line_bytes = (uint32_t)w * 4;
					for (int32_t y = 0; y < h; y++) {
						const void *src_line = (const void *)(src + s->offset + (uint64_t)y * s->bytesPerRow);
						void *dst_line = (void *)(dstPixels + (uint64_t)y * fRowBytes);
						memcpy(dst_line, src_line, line_bytes);
					}
				}
			}
		}

		/* Flip: make the freshly written buffer the scanout source. The
		 * FLUSH in op2_fb_kick_panel latches SRC0 before START. In early-copy
		 * mode the flip and kick wait for flipAndKick(). */
		__asm__ volatile("dsb sy" ::: "memory");
		if (fEarlyCopy) {
			fFlipPending = true;
			fFlipIdx = dstIdx;
			fFlipPixels = dstPixels;
			fFlipPhys = dstPhys;
		} else if (fActivePipe != NULL) {
			fActivePipe[0x00 / 4] = (fHeight << 16) | fWidth;
			fActivePipe[0x04 / 4] = (fHeight << 16) | fWidth;
			fActivePipe[0x08 / 4] = 0;
			fActivePipe[0x0c / 4] = (fHeight << 16) | fWidth;
			fActivePipe[0x10 / 4] = 0;
			fActivePipe[0x14 / 4] = (uint32_t)dstPhys;
			fActivePipe[0x24 / 4] = fRowBytes;
			fActivePipe[0x30 / 4] = 0x000236ffu;
			fActivePipe[0x34 / 4] = 0x03020001u;
			if (fPipeHasScaler) {
				fActivePipe[0x204 / 4] = 0;
			}
		}
		if (!fEarlyCopy) {
			op2_fb_direct_cur = 0;
			fFrontBuf = dstIdx;
			fPixels = dstPixels;
		}
	}

	if ((pc % 240) == 0)
		op2_fb_thumb(s, pc);

	if (s != fSurface) {
		s->release();
	}

	/* Memory barrier to ensure all write-combined stores drain to DRAM */
	__asm__ volatile("dsb sy" ::: "memory");

	/* Kick command-mode panel to send newly swapped frame to display GRAM */
	if (!fEarlyCopy) {
		op2_fb_kick_panel(fMdpCtl);
	}
}

/* Workloop context. Finish an early-copied frame: point the pipe at the
 * back buffer, flip, kick. Caller guarantees the kick spacing has elapsed. */
void
AppleCLCD::flipAndKick(void)
{
	if (!fFlipPending) {
		return;
	}
	fFlipPending = false;
	__asm__ volatile("dsb sy" ::: "memory");
	if (fActivePipe != NULL) {
		fActivePipe[0x00 / 4] = (fHeight << 16) | fWidth;
		fActivePipe[0x04 / 4] = (fHeight << 16) | fWidth;
		fActivePipe[0x08 / 4] = 0;
		fActivePipe[0x0c / 4] = (fHeight << 16) | fWidth;
		fActivePipe[0x10 / 4] = 0;
		fActivePipe[0x14 / 4] = (uint32_t)fFlipPhys;
		fActivePipe[0x24 / 4] = fRowBytes;
		fActivePipe[0x30 / 4] = 0x000236ffu;
		fActivePipe[0x34 / 4] = 0x03020001u;
		if (fPipeHasScaler) {
			fActivePipe[0x204 / 4] = 0;
		}
	}
	op2_fb_direct_cur = 0;
	fFrontBuf = fFlipIdx;
	fPixels = fFlipPixels;
	__asm__ volatile("dsb sy" ::: "memory");
	op2_fb_kick_panel(fMdpCtl);
}

/*
 * Log a tiny grayscale thumbnail of the presented frame (48x27, one hex
 * digit per pixel) so the screen content is verifiable from the log alone:
 * UI vs black vs garbage, without eyes on the panel. Every 240th present.
 */
static void
op2_fb_thumb(OP2Surface *s, uint64_t pc)
{
	if (s == NULL || s->width < 48 || s->height < 27 || s->memory == NULL)
		return;
	uintptr_t psrc = s->kernelAddress();
	if (psrc == 0)
		return;
	uint64_t need = s->offset + (uint64_t)s->height * s->bytesPerRow;
	if (need > s->memory->getLength())
		return;
	uint32_t xs = s->width / 48, ys = s->height / 27;
	if (xs < 1)
		xs = 1;
	if (ys < 1)
		ys = 1;
	char line[128];
	for (uint32_t r = 0; r < 27; r++) {
		const uint32_t *row = (const uint32_t *)(psrc + s->offset +
		    (uint64_t)(r * ys) * s->bytesPerRow);
		for (uint32_t c = 0; c < 48; c++) {
			uint32_t px = row[c * xs];
			uint32_t lum = ((px & 0xff) + ((px >> 8) & 0xff) + ((px >> 16) & 0xff)) / 3;
			line[c] = "0123456789abcdef"[lum >> 4];
		}
		line[48] = '\0';
		IOLog("OP2 FB THUMB #%llu r%u: %s\n", pc, r, line);
	}
}

/*
 * Display getters for input/digitizer synchronization:
 */
extern "C" uint32_t op2_fb_get_last_shown(void);
extern "C" uint32_t op2_fb_get_last_nz(void);

uint32_t
op2_fb_get_last_shown(void)
{
	return op2_fb_last_shown;
}

uint32_t
op2_fb_get_last_nz(void)
{
	return op2_fb_last_nz;
}

extern "C" bool op2_fb_is_power_off(void);
extern "C" uint64_t op2_fb_get_last_kick(void);

bool
op2_fb_is_power_off(void)
{
	return (op2_fb_instance != NULL && op2_fb_instance->fPowerOff);
}

uint64_t
op2_fb_get_last_kick(void)
{
	return op2_fb_last_kick;
}

/*
 * Deadline-dump display verdict: proves lit-UI vs black pixels from the log
 * alone (no eyes needed). Samples panel memory directly; lock-free MMIO read.
 */
extern "C" void op2_fb_dump_stats(void);
extern "C" void
op2_fb_dump_stats(void)
{
	/* kprintf-only in here: the deadline dump runs with interrupts
	 * disabled and IOLog asserts there (run177). */
	uint32_t panel_nz = 0, samples = 0;
	uint64_t presents = op2_fb_present_count, kicks = op2_fb_kick_count;
	uint32_t shown = op2_fb_last_shown, w = 0, h = 0;
	uint32_t active_phys = (op2_fb_instance != NULL && op2_fb_instance->fActivePipe != NULL) ?
	    op2_fb_instance->fActivePipe[0x14 / 4] : 0;

	if (op2_fb_instance != NULL && op2_fb_instance->fPixels != 0) {
		w = op2_fb_instance->fWidth;
		h = op2_fb_instance->fHeight;
		const uint32_t *px = (const uint32_t *)op2_fb_instance->fPixels;
		uint64_t total = (uint64_t)w * h;
		for (uint64_t i = 0; i < total && samples < 2000; i += 997) {
			if ((px[i] & 0x00FFFFFFu) != 0) {
				panel_nz++;
			}
			samples++;
		}
	}
	printf("OP2 FB STATS: presents=%llu kicks=%llu shown=%u active_phys=0x%08x direct_phys=0x%08llx panel=%ux%u panel_nz=%u/%u lastnz=%u hwscale=%d hwscale_count=%llu\n",
	    presents, kicks, shown, active_phys, op2_fb_last_direct_phys, w, h, panel_nz, samples, op2_fb_last_nz, (int)op2_fb_hwscale_active, op2_fb_hwscale_count);
	/* Output directly via printf to be captured in persistent ramoops buffer */
	printf("OP2 FB LAST begin rec=%llu size=%u\n",
	    op2_fb_last_pc, (uint32_t)sizeof(op2_fb_last_rec));
	for (uint32_t off = 0; off + 16 <= (uint32_t)sizeof(op2_fb_last_rec); off += 16) {
		printf("OP2 FB LAST %u: %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u\n",
		    off,
		    op2_fb_last_rec[off+0], op2_fb_last_rec[off+1],
		    op2_fb_last_rec[off+2], op2_fb_last_rec[off+3],
		    op2_fb_last_rec[off+4], op2_fb_last_rec[off+5],
		    op2_fb_last_rec[off+6], op2_fb_last_rec[off+7],
		    op2_fb_last_rec[off+8], op2_fb_last_rec[off+9],
		    op2_fb_last_rec[off+10], op2_fb_last_rec[off+11],
		    op2_fb_last_rec[off+12], op2_fb_last_rec[off+13],
		    op2_fb_last_rec[off+14], op2_fb_last_rec[off+15]);
	}
	printf("OP2 FB LAST end\n");
}

bool
IOMobileFramebufferUserClient::initWithTask(task_t owningTask, void *securityID, UInt32 type)
{
	if (!IOUserClient::initWithTask(owningTask, securityID, type)) {
		return false;
	}
	fTask = owningTask;
	return true;
}

bool
IOMobileFramebufferUserClient::start(IOService *provider)
{
	fFB = OSDynamicCast(AppleCLCD, provider);
	if (fFB == NULL || !IOUserClient::start(provider)) {
		return false;
	}
	fFB->addClient(this);
	return true;
}

IOReturn
IOMobileFramebufferUserClient::clientClose(void)
{
	if (fFB != NULL) {
		fFB->removeClient(this);
		IOLockLock(fFB->fLock);
		mach_port_t old = fPort;
		fPort = MACH_PORT_NULL;
		IOLockUnlock(fFB->fLock);
		if (old != MACH_PORT_NULL) {
			releaseNotificationPort(old);
		}
	}
	terminate();
	return kIOReturnSuccess;
}

IOReturn
IOMobileFramebufferUserClient::registerNotificationPort(mach_port_t port, __unused UInt32 type,
    __unused io_user_reference_t refCon)
{
	mach_port_t old;
	IOLockLock(fFB->fLock);
	old = fPort;
	fPort = port;
	IOLockUnlock(fFB->fLock);
	if (old != MACH_PORT_NULL) {
		releaseNotificationPort(old);
	}
	return kIOReturnSuccess;
}

/* Called with the framebuffer lock held. */
void
IOMobileFramebufferUserClient::notify(uint32_t type, const uint64_t *values, uint32_t count)
{
	OSAsyncReference64 ref;

	if (type >= OP2_FB_NOTIFY_TYPES || fFunc[type] == 0 || fPort == MACH_PORT_NULL) {
		return;
	}
	bzero(ref, sizeof(ref));
	setAsyncReference64(ref, fPort, (mach_vm_address_t)fFunc[type], fRefcon[type], fTask);
	sendAsyncResult64WithOptions(ref, kIOReturnSuccess, (io_user_reference_t *)values, count,
	    kIOUserNotifyOptionCanDrop);
}

static uint64_t op2_fb_logged;        /* selectors already logged (< 64) */

IOReturn
IOMobileFramebufferUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    __unused IOExternalMethodDispatch *dispatch, __unused OSObject *target,
    __unused void *reference)
{
	IOReturn ret = kIOReturnSuccess;
	uint64_t out[2] = { 0, 0 };
	uint32_t outCount = 0;

	if (args->structureOutputDescriptor == NULL && args->structureOutput != NULL) {
		bzero(args->structureOutput, args->structureOutputSize);
	}

	switch (selector) {
	case kOP2FBGetLayerDefaultSurface:
		out[0] = op2_surface_port_in_task(fFB->fSurface, fTask);
		outCount = 1;
		break;
	case kOP2FBSwapBegin:
		IOLockLock(fFB->fLock);
		out[0] = ++fFB->fSwapID;
		IOLockUnlock(fFB->fLock);
		outCount = 1;
		break;
	case kOP2FBSwapEnd: {
		uint8_t rec[OP2_FB_SWAP_REC_SIZE];
		bzero(rec, sizeof(rec));
		if (args->structureInputDescriptor != NULL ||
		    args->structureInputSize < 0x278) {
			ret = kIOReturnBadArgument;
			break;
		}
		uint32_t cplen = args->structureInputSize < sizeof(rec) ? (uint32_t)args->structureInputSize : (uint32_t)sizeof(rec);
		memcpy(rec, args->structureInput, cplen);
		fFB->swap(rec);
		break;
	}
	case kOP2FBGetID:
		out[0] = 1;
		outCount = 1;
		break;
	case kOP2FBGetDisplaySize:
		/*
		 * On iPhone 6 Plus (n56ap), UIKit renders at 3x Retina (1242x2208).
		 * Reporting 1242x2208 matches native n56ap display geometry and
		 * prevents CoreAnimation from dropping [1242x2208] render surfaces.
		 */
		out[0] = 1242;
		out[1] = 2208;
		outCount = 2;
		break;
	case kOP2FBRequestPowerChange:
		if (args->scalarInputCount >= 1) {
			bool off = args->scalarInput[0] == 0;
			static int sKeepOn = -1;
			if (sKeepOn == -1) {
				sKeepOn = 1;
				PE_parse_boot_argn("op2fb_keepon", &sKeepOn, sizeof(sKeepOn));
			}
			if (sKeepOn != 0 && off) {
				IOLog("OP2 FB: userland display sleep requested (op2fb_keepon active, panel stays on)\n");
				break;
			}
			IOLockLock(fFB->fLock);
			if (off != fFB->fPowerOff) {
				fFB->fPowerOff = off;
				fFB->fFadeStep = 0;
				fFB->fDirty = true;     /* on: redraw the latest frame */
			}
			IOLockUnlock(fFB->fLock);
			IOLog("OP2 FB: display %s\n", off ? "off" : "on");
		}
		break;
	case kOP2FBIsMainDisplay:
		out[0] = 1;
		outCount = 1;
		break;
	case kOP2FBGetDotPitch:
		out[0] = OP2_FB_DOT_PITCH;
		outCount = 1;
		break;
	case kOP2FBSurfaceIsReplaceable:
		out[0] = 1;
		outCount = 1;
		break;
	case kOP2FBSupportedFrameInfo:
		if (args->scalarInputCount >= 1 && args->scalarInput[0] > 0) {
			ret = kIOReturnBadArgument;
			break;
		}
		out[0] = 1; /* 1 mode supported */
		outCount = 1;
		if (args->structureOutput != NULL && args->structureOutputSize >= 64) {
			uint32_t *fi = (uint32_t *)args->structureOutput;
			fi[0] = 1242;                 /* Retina logical display width */
			fi[1] = 2208;                 /* Retina logical display height */
			fi[2] = 60;                   /* refresh rate 60 Hz */
			fi[3] = 0;                    /* flags */
			fi[4] = OP2_FB_FORMAT_BGRA;   /* format */
			fi[5] = 1242 * 4;             /* 4968 */
		}
		break;
	case kOP2FBChangeFrameInfo:
		ret = kIOReturnSuccess;
		break;
	case kOP2FBNotifications:
		if (args->scalarInputCount >= 3 && args->scalarInput[2] < OP2_FB_NOTIFY_TYPES) {
			uint32_t type = (uint32_t)args->scalarInput[2];
			bool changed = (fFunc[type] != args->scalarInput[0]);
			IOLockLock(fFB->fLock);
			fFunc[type] = args->scalarInput[0];
			fRefcon[type] = args->scalarInput[1];
			IOLockUnlock(fFB->fLock);
			if (changed) {
				/* backboardd flaps these thousands of times per
				 * run (type 5 on/off per commit); log the first
				 * 8 so the 260KB ring keeps the TL history. */
				static int op2_fb_note_logged = 0;
				if (op2_fb_note_logged < 8) {
					char name[32];
					op2_fb_note_logged++;
					proc_selfname(name, sizeof(name));
					IOLog("OP2 FB: %s notifications type %u %s\n",
					    name, type,
					    args->scalarInput[0] ? "on" : "off");
				}
			}
		}
		break;
	case kOP2FBSwapWait: {
		/* (swap token, timeout, flags): block until the frame for this
		 * token has been copied out of the render server's buffer, so it
		 * cannot start drawing into memory we are still reading. */
		uint64_t want = args->scalarInputCount >= 1 ? args->scalarInput[0] : 0;
		static uint32_t sWaitLogged = 0;
		if (sWaitLogged < 6) {
			sWaitLogged++;
			IOLog("OP2 FB: SwapWait(%llu, %llu, %llu) ended=%u done=%u\n", want,
			    args->scalarInputCount >= 2 ? args->scalarInput[1] : 0,
			    args->scalarInputCount >= 3 ? args->scalarInput[2] : 0,
			    fFB->fLastSwapEndID, fFB->fCompletedSwapID);
		}
		if (!fFB->fSwapWaitEnabled || want == 0) {
			break;
		}
		uint32_t w = (uint32_t)want;
		IOLockLock(fFB->fLock);
		int32_t ahead = (int32_t)(fFB->fLastSwapEndID - w);
		if (ahead >= 0 && ahead < 16 && (int32_t)(w - fFB->fCompletedSwapID) > 0) {
			uint64_t deadline = 0;
			clock_interval_to_deadline(50, kMillisecondScale, &deadline);
			op2_fb_swapwait_waits++;
			while ((int32_t)(w - fFB->fCompletedSwapID) > 0) {
				if (IOLockSleepDeadline(fFB->fLock, &fFB->fCompletedSwapID,
				    deadline, THREAD_UNINT) == THREAD_TIMED_OUT) {
					op2_fb_swapwait_timeouts++;
					break;
				}
			}
		}
		IOLockUnlock(fFB->fLock);
		break;
	}
	default:
		break;
	}

	for (uint32_t i = 0; i < args->scalarOutputCount; i++) {
		args->scalarOutput[i] = i < outCount ? out[i] : 0;
	}
	static uint64_t op2_fb_logged_high = 0;
	bool should_log = false;
	if (selector < 64) {
		if (!(op2_fb_logged & (1ULL << selector))) {
			op2_fb_logged |= 1ULL << selector;
			should_log = true;
		}
	} else if (selector < 128) {
		uint32_t bit = selector - 64;
		if (!(op2_fb_logged_high & (1ULL << bit))) {
			op2_fb_logged_high |= 1ULL << bit;
			should_log = true;
		}
	}
	if (should_log) {
		char name[32];
		proc_selfname(name, sizeof(name));
		IOLog("OP2 FB: %s sel %u in %u/%u out %u/%u -> 0x%x\n", name, selector,
		    args->scalarInputCount, args->structureInputSize,
		    args->scalarOutputCount, args->structureOutputSize, ret);
	}
	return ret;
}
#endif /* ARM64_BOARD_CONFIG_MSM8994 */
