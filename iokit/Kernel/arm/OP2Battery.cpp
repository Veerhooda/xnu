/*
 * OP2Battery: OnePlus 2 (MSM8994 / PMI8994) Battery & Fuel-Gauge Power Source.
 *
 * Implements Apple IOPMPowerSource for iOS powerd and system status bar:
 * - Talks to PMI8994 (SID 2) over Qualcomm SPMI PMIC Arbiter v1 (APPS channel 0).
 * - SPMI core is at physical 0xFC4CF000; channel 0 is at offset 0x800 (0xFC4CF800).
 * - Reads Monotonic SOC from FG_SOC (0x4009 / 0x400A shadow register).
 * - Reads Charger Status from SMBCHG (0x1310/0x130D USB path, 0x1010/0x100E charger).
 * - Reads real Voltage and Current from FG MEM_IF SRAM (0x5CC) when available.
 * - Publishes battery state to iOS power management (IOPMPowerSource) and polls every 30s.
 */

#include <pexpert/arm64/board_config.h>

#if defined(ARM64_BOARD_CONFIG_MSM8994)
#include <device/device_types.h>
#include <libkern/c++/OSObject.h>
#include <libkern/c++/OSDictionary.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSBoolean.h>
#include <libkern/c++/OSSymbol.h>
#include <libkern/c++/OSCollectionIterator.h>
#include <IOKit/IOService.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOTimerEventSource.h>
#include <IOKit/pwr_mgt/IOPMPowerSource.h>
#include <IOKit/pwr_mgt/IOPM.h>
#include <pexpert/pexpert.h>

extern "C" vm_offset_t ml_io_map(vm_offset_t phys_addr, vm_size_t size);
extern "C" int op2_temp_max;

#define OP2_BATTERY_NOMINAL_MAH     3300u   /* OnePlus 2 battery nominal capacity */

/* Qualcomm SPMI Arbiter v1 Hardware (MSM8994) */
#define OP2_SPMI_CORE_PHYS          0xFC4CF000ULL
#define OP2_SPMI_MAP_SIZE           0x1000

#define OP2_SPMI_CH0_OFFSET         0x0800
#define OP2_SPMI_CH_CMD             0x00
#define OP2_SPMI_CH_CONFIG          0x04
#define OP2_SPMI_CH_STATUS          0x08
#define OP2_SPMI_CH_WDATA0          0x10
#define OP2_SPMI_CH_WDATA1          0x14
#define OP2_SPMI_CH_RDATA0          0x18
#define OP2_SPMI_CH_RDATA1          0x1C

#define OP2_SPMI_ST_DONE            (1u << 0)
#define OP2_SPMI_ST_FAILURE         (1u << 1)
#define OP2_SPMI_ST_DENIED          (1u << 2)
#define OP2_SPMI_ST_DROPPED         (1u << 3)

#define OP2_SPMI_OP_EXT_WRITEL      0u
#define OP2_SPMI_OP_EXT_READL       1u

/* PMI8994 Slave ID and Peripherals */
#define OP2_PMI8994_SID             2u

#define OP2_FG_SOC_BASE             0x4000u
#define OP2_FG_SOC_MONOTONIC        0x4009u
#define OP2_FG_BATT_BASE            0x4100u
#define OP2_FG_MEMIF_BASE           0x4400u
#define OP2_FG_MEMIF_INT_RT_STS     0x4410u
#define OP2_FG_MEMIF_CFG            0x4440u
#define OP2_FG_MEMIF_CTL            0x4441u
#define OP2_FG_MEMIF_ADDR           0x4442u
#define OP2_FG_MEMIF_RD_DATA        0x444Cu

#define OP2_CHGR_BASE               0x1000u
#define OP2_CHGR_STS                0x100Eu
#define OP2_CHGR_RT_STS             0x1010u

#define OP2_USB_CHGPTH_BASE         0x1300u
#define OP2_USB_INPUT_STS           0x130Du
#define OP2_USB_RT_STS              0x1310u

class OP2Battery : public IOPMPowerSource {
	OSDeclareDefaultStructors(OP2Battery);

public:
	virtual bool init(OSDictionary *dictionary = NULL) APPLE_KEXT_OVERRIDE;
	virtual bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void stop(IOService *provider) APPLE_KEXT_OVERRIDE;
	virtual void free(void) APPLE_KEXT_OVERRIDE;

private:
	volatile uint32_t  *fCore;
	IOLock             *fSpmiLock;
	IOWorkLoop         *fLoop;
	IOTimerEventSource *fTimer;
	bool                fTimerAdded;
	bool                fRunning;
	bool                fReal;
	int                 fLastMode;
	uint8_t             fLastRegs[8];

	void putNumber(const char *key, UInt32 value);
	void putBool(const char *key, bool value);
	void publish(unsigned percent, unsigned mv, int ma, int temp_hundredths,
	             bool external, bool capable, bool charging, bool full,
	             bool measured, bool currentValid, bool sramTempOk);
	void poll(void);

	static void timerFired(OSObject *owner, IOTimerEventSource *sender);
	static IOReturn stopPolling(OSObject *owner, void *, void *, void *, void *);

	bool spmiRead(uint8_t sid, uint16_t addr, uint8_t *buf, uint32_t len);
	bool spmiWrite(uint8_t sid, uint16_t addr, const uint8_t *buf, uint32_t len);
	bool readSoc(uint8_t *outSoc);
	bool readCharger(bool *outExternal, bool *outCharging, bool *outFull);
	bool acquireSram(void);
	void releaseSram(void);
	bool readSramData(uint16_t sramAddr, uint8_t *buf, uint32_t len);
};

OSDefineMetaClassAndStructors(OP2Battery, IOPMPowerSource);

bool
OP2Battery::init(OSDictionary *dictionary)
{
	if (!IOPMPowerSource::init()) {
		return false;
	}

	fCore = NULL;
	fSpmiLock = IOLockAlloc();
	fLoop = NULL;
	fTimer = NULL;
	fTimerAdded = false;
	fRunning = false;
	fReal = false;
	fLastMode = -1;
	memset(fLastRegs, 0xFF, sizeof(fLastRegs));

	if (dictionary) {
		OSCollectionIterator *it = OSCollectionIterator::withCollection(dictionary);
		if (it) {
			OSObject *object;
			while ((object = it->getNextObject())) {
				const OSSymbol *key = OSDynamicCast(OSSymbol, object);
				if (key) {
					setProperty(key, dictionary->getObject(key));
				}
			}
			it->release();
		}
	}

	return true;
}

void
OP2Battery::free(void)
{
	OSSafeReleaseNULL(fTimer);
	OSSafeReleaseNULL(fLoop);
	if (fSpmiLock) {
		IOLockFree(fSpmiLock);
		fSpmiLock = NULL;
	}
	IOPMPowerSource::free();
}

bool
OP2Battery::spmiRead(uint8_t sid, uint16_t addr, uint8_t *buf, uint32_t len)
{
	if (len == 0 || len > 8 || fCore == NULL || buf == NULL)
		return false;

	IOLockLock(fSpmiLock);

	volatile uint32_t *ch = (volatile uint32_t *)((uintptr_t)fCore + OP2_SPMI_CH0_OFFSET);
	uint32_t bc = (len - 1) & 0x7;
	uint32_t cmd = (OP2_SPMI_OP_EXT_READL << 27) |
	               (((uint32_t)sid & 0xF) << 20) |
	               (((uint32_t)addr & 0xFFFF) << 4) |
	               bc;

	ch[OP2_SPMI_CH_CMD / 4] = cmd;

	bool ok = false;
	for (uint32_t t = 0; t < 10000; t++) {
		uint32_t st = ch[OP2_SPMI_CH_STATUS / 4];
		if (st & OP2_SPMI_ST_DONE) {
			if ((st & (OP2_SPMI_ST_FAILURE | OP2_SPMI_ST_DENIED | OP2_SPMI_ST_DROPPED)) == 0) {
				uint32_t r0 = ch[OP2_SPMI_CH_RDATA0 / 4];
				uint32_t r1 = (len > 4) ? ch[OP2_SPMI_CH_RDATA1 / 4] : 0;
				for (uint32_t i = 0; i < len; i++) {
					buf[i] = (uint8_t)((i < 4 ? r0 : r1) >> (8 * (i % 4)));
				}
				ok = true;
			}
			break;
		}
		IODelay(1);
	}

	IOLockUnlock(fSpmiLock);
	return ok;
}

bool
OP2Battery::spmiWrite(uint8_t sid, uint16_t addr, const uint8_t *buf, uint32_t len)
{
	if (len == 0 || len > 8 || fCore == NULL || buf == NULL)
		return false;

	IOLockLock(fSpmiLock);

	volatile uint32_t *ch = (volatile uint32_t *)((uintptr_t)fCore + OP2_SPMI_CH0_OFFSET);
	uint32_t bc = (len - 1) & 0x7;
	uint32_t cmd = (OP2_SPMI_OP_EXT_WRITEL << 27) |
	               (((uint32_t)sid & 0xF) << 20) |
	               (((uint32_t)addr & 0xFFFF) << 4) |
	               bc;

	uint32_t w0 = 0, w1 = 0;
	for (uint32_t i = 0; i < len; i++) {
		if (i < 4)
			w0 |= ((uint32_t)buf[i]) << (8 * i);
		else
			w1 |= ((uint32_t)buf[i]) << (8 * (i - 4));
	}
	ch[OP2_SPMI_CH_WDATA0 / 4] = w0;
	if (len > 4)
		ch[OP2_SPMI_CH_WDATA1 / 4] = w1;

	ch[OP2_SPMI_CH_CMD / 4] = cmd;

	bool ok = false;
	for (uint32_t t = 0; t < 10000; t++) {
		uint32_t st = ch[OP2_SPMI_CH_STATUS / 4];
		if (st & OP2_SPMI_ST_DONE) {
			if ((st & (OP2_SPMI_ST_FAILURE | OP2_SPMI_ST_DENIED | OP2_SPMI_ST_DROPPED)) == 0) {
				ok = true;
			}
			break;
		}
		IODelay(1);
	}

	IOLockUnlock(fSpmiLock);
	return ok;
}

bool
OP2Battery::readSoc(uint8_t *outSoc)
{
	uint8_t cap[2] = {0, 0};
	for (int i = 0; i < 5; i++) {
		if (spmiRead(OP2_PMI8994_SID, OP2_FG_SOC_MONOTONIC, cap, 2)) {
			if (cap[0] == cap[1]) {
				*outSoc = cap[0];
				return true;
			}
		}
		IODelay(50);
	}
	return false;
}

bool
OP2Battery::readCharger(bool *outExternal, bool *outCharging, bool *outFull)
{
	uint8_t usb = 0, input = 0, chgr_rt = 0, chgr = 0;
	if (!spmiRead(OP2_PMI8994_SID, OP2_USB_RT_STS, &usb, 1) ||
	    !spmiRead(OP2_PMI8994_SID, OP2_USB_INPUT_STS, &input, 1) ||
	    !spmiRead(OP2_PMI8994_SID, OP2_CHGR_RT_STS, &chgr_rt, 1) ||
	    !spmiRead(OP2_PMI8994_SID, OP2_CHGR_STS, &chgr, 1)) {
		return false;
	}

	/*
	 * usb & 0x04: USBIN_SRC_DET (VBUS detected)
	 * !(usb & 0x02): not USBIN_OV (no over-voltage)
	 * input & 0x38: valid input path range
	 */
	bool external = (usb & 0x04u) && !(usb & 0x02u) && (input & 0x38u);
	/* chgr_rt & 0x82: TCC (term current condition) or inhibit => full */
	bool full = external && (chgr_rt & 0x82u);
	/* chgr & 0x06: charge type != none; !(chgr & 0x08): no hold-off */
	bool charging = external && !full && !(chgr & 0x08u) && ((chgr >> 1) & 0x03u);

	*outExternal = external;
	*outCharging = charging;
	*outFull = full;
	return true;
}

bool
OP2Battery::acquireSram(void)
{
	uint8_t cfg = 0;
	if (!spmiRead(OP2_PMI8994_SID, OP2_FG_MEMIF_CFG, &cfg, 1))
		return false;

	cfg |= 0x80; /* RIF_MEM_ACCESS_REQ */
	if (!spmiWrite(OP2_PMI8994_SID, OP2_FG_MEMIF_CFG, &cfg, 1))
		return false;

	uint8_t sts = 0;
	for (uint32_t t = 0; t < 500; t++) {
		if (spmiRead(OP2_PMI8994_SID, OP2_FG_MEMIF_INT_RT_STS, &sts, 1) && (sts & 0x01)) {
			return true;
		}
		IODelay(50);
	}

	/* Timed out; release request */
	cfg &= (uint8_t)~0x80;
	spmiWrite(OP2_PMI8994_SID, OP2_FG_MEMIF_CFG, &cfg, 1);
	return false;
}

void
OP2Battery::releaseSram(void)
{
	uint8_t cfg = 0;
	if (spmiRead(OP2_PMI8994_SID, OP2_FG_MEMIF_CFG, &cfg, 1)) {
		cfg &= (uint8_t)~0x80;
		spmiWrite(OP2_PMI8994_SID, OP2_FG_MEMIF_CFG, &cfg, 1);
	}
}

bool
OP2Battery::readSramData(uint16_t sramAddr, uint8_t *buf, uint32_t len)
{
	if (len == 0 || len > 8 || buf == NULL)
		return false;

	uint8_t ctl0 = 0;
	spmiWrite(OP2_PMI8994_SID, OP2_FG_MEMIF_CTL, &ctl0, 1);

	uint8_t adr[2] = { (uint8_t)(sramAddr & 0xFF), (uint8_t)((sramAddr >> 8) & 0xFF) };
	if (spmiWrite(OP2_PMI8994_SID, OP2_FG_MEMIF_ADDR, adr, 2)) {
		if (spmiRead(OP2_PMI8994_SID, OP2_FG_MEMIF_RD_DATA, buf, len)) {
			return true;
		}
	}
	return false;
}

void
OP2Battery::putNumber(const char *key, UInt32 value)
{
	const OSSymbol *symbol = OSSymbol::withCString(key);
	OSNumber *number = OSNumber::withNumber(value, 32);
	if (symbol && number) {
		setPSProperty(symbol, number);
	}
	OSSafeReleaseNULL(number);
	OSSafeReleaseNULL(symbol);
}

void
OP2Battery::putBool(const char *key, bool value)
{
	const OSSymbol *symbol = OSSymbol::withCString(key);
	if (symbol) {
		setPSProperty(symbol, value ? kOSBooleanTrue : kOSBooleanFalse);
		symbol->release();
	}
}

void
OP2Battery::publish(unsigned percent, unsigned mv, int ma, int temp_hundredths,
                    bool external, bool capable, bool charging, bool full,
                    bool measured, bool currentValid, bool sramTempOk)
{
	const unsigned capacity = (OP2_BATTERY_NOMINAL_MAH * percent + 50u) / 100u;
	setBatteryInstalled(true);
	setCurrentCapacity(capacity);
	setMaxCapacity(OP2_BATTERY_NOMINAL_MAH);
	setExternalConnected(external);
	setExternalChargeCapable(capable);
	setIsCharging(charging);
	setAtWarnLevel(!external && percent <= 20);
	setAtCriticalLevel(!external && percent <= 5);
	setVoltage(mv);
	setAmperage(ma);
	setTimeRemaining(-1);
	setCycleCount(0);

	putBool("FullyCharged", full);
	putBool("AppleRawExternalConnected", external);
	putNumber("AppleRawCurrentCapacity", capacity);
	putNumber("AppleRawMaxCapacity", OP2_BATTERY_NOMINAL_MAH);
	putNumber("DesignCapacity", OP2_BATTERY_NOMINAL_MAH);
	putNumber("InstantAmperage", (UInt32)ma);
	putNumber("Temperature", (UInt32)temp_hundredths);

	putBool("OP2BatteryHardwareSampleValid", measured);
	putBool("OP2BatteryCurrentValid", currentValid);
	putBool("OP2BatterySOCEstimated", measured);
	putBool("OP2BatteryTemperatureEstimated", !sramTempOk);
	putBool("OP2BatteryFallback", !measured);
	putNumber("OP2BatteryPercent", percent);

	OSDictionary *legacy = OSDictionary::withCapacity(7);
	if (legacy) {
		const char *keys[] = {
			kIOBatteryCurrentChargeKey, kIOBatteryCapacityKey,
			kIOBatteryVoltageKey, kIOBatteryAmperageKey, kIOBatteryCycleCountKey,
			kIOBatteryFlagsKey
		};
		UInt32 values[] = {
			capacity, OP2_BATTERY_NOMINAL_MAH, mv, (UInt32)ma, 0,
			(UInt32)(kIOBatteryInstalled | (external ? kIOBatteryChargerConnect : 0) |
			         (charging ? kIOBatteryCharge : 0))
		};
		for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
			const OSSymbol *sym = OSSymbol::withCString(keys[i]);
			OSNumber *num = OSNumber::withNumber(values[i], 32);
			if (sym && num) {
				legacy->setObject(sym, num);
			}
			OSSafeReleaseNULL(num);
			OSSafeReleaseNULL(sym);
		}
		setLegacyIOBatteryInfo(legacy);
		legacy->release();
	}

	updateStatus();

	int mode = measured ? 1 : 0;
	if (mode != fLastMode) {
		int t_abs = (temp_hundredths < 0) ? -temp_hundredths : temp_hundredths;
		IOLog("OP2Battery: mode=%s %u%% %umV %dma temp=%s%d.%02dC (external=%d charging=%d full=%d)\n",
		      measured ? "PMI8994 fuel gauge" : "static fallback",
		      percent, mv, ma, (temp_hundredths < 0) ? "-" : "",
		      t_abs / 100, t_abs % 100, external, charging, full);
		fLastMode = mode;
	}
}

void
OP2Battery::poll(void)
{
	uint8_t rawSoc = 0;
	bool external = true, charging = false, full = false;
	bool socOk = false, chgrOk = false, sramOk = false, sramTempOk = false;
	unsigned percent = 100;
	unsigned mv = 3800;
	int ma = 0;
	int temp_hundredths = 2500;

	if (fReal) {
		socOk = readSoc(&rawSoc);
		chgrOk = readCharger(&external, &charging, &full);

		if (socOk) {
			percent = (rawSoc == 0xFF) ? 100 :
			          (rawSoc == 0) ? 0 :
			          (((unsigned)(rawSoc - 1) * 98 + 126) / 253 + 1);
		}

		/* Attempt real SRAM reads (Voltage/Current at 0x5CC and Battery Temp at 0x550) */
		if (acquireSram()) {
			uint8_t vc[8];
			if (readSramData(0x05CC, vc, 8)) {
				uint32_t raw_v = (uint32_t)vc[1] | (((uint32_t)vc[2]) << 8);
				int16_t raw_i = (int16_t)(vc[3] | (((uint32_t)vc[4]) << 8));

				/* uV = raw_v * 152587 / 1000 */
				unsigned measured_mv = (unsigned)(((uint64_t)raw_v * 152587ULL) / 1000000ULL);
				if (measured_mv >= 2800 && measured_mv <= 4600) {
					mv = measured_mv;
					sramOk = true;
				}

				/* uA = raw_i * 152587 / 1000; Linux negative = charging, Apple positive = charging */
				int measured_ma_qcom = (int)(((int64_t)raw_i * 152587LL) / 1000000LL);
				ma = -measured_ma_qcom;
			}

			uint8_t tb[4];
			if (readSramData(0x0550, tb, 4)) {
				uint16_t raw_t = (uint16_t)tb[2] | (((uint16_t)tb[3]) << 8);
				/* deci-Celsius = (raw_t * 625 / 1000) - 2730 */
				int temp_deci_c = (((int)raw_t * 625) / 1000) - 2730;
				if (temp_deci_c >= -200 && temp_deci_c <= 850) {
					temp_hundredths = temp_deci_c * 10;
					sramTempOk = true;
				}
			}

			releaseSram();
		}

		if (!sramOk) {
			/* Fallback plausible voltage curve if SRAM is busy */
			mv = 3600 + (percent * 600) / 100;
			ma = charging ? 500 : (external ? 0 : -350);
		}

		if (!sramTempOk) {
			/* Fallback to TSENS SoC hottest sensor if valid */
			if (op2_temp_max > -40 && op2_temp_max < 150) {
				temp_hundredths = op2_temp_max * 100;
			} else {
				temp_hundredths = 2500;
			}
		}

		uint8_t regs[8] = {
			rawSoc, (uint8_t)external, (uint8_t)charging,
			(uint8_t)full, (uint8_t)(mv >> 8), (uint8_t)(mv & 0xFF),
			(uint8_t)(temp_hundredths >> 8), (uint8_t)(temp_hundredths & 0xFF)
		};
		if (memcmp(regs, fLastRegs, sizeof(regs)) != 0) {
			int t_abs = (temp_hundredths < 0) ? -temp_hundredths : temp_hundredths;
			IOLog("OP2Battery: soc 0x%02x (%u%%) %umV %dma temp=%s%d.%02dC external=%d charging=%d full=%d sram_v=%d sram_t=%d\n",
			      rawSoc, percent, mv, ma, (temp_hundredths < 0) ? "-" : "",
			      t_abs / 100, t_abs % 100, external, charging, full, sramOk, sramTempOk);
			memcpy(fLastRegs, regs, sizeof(regs));
		}
	} else {
		percent = 100;
		mv = 4200;
		ma = 0;
		temp_hundredths = 2500;
		sramTempOk = false;
		external = true;
		charging = false;
		full = true;
	}

	publish(percent, mv, ma, temp_hundredths, external, external, charging, full, socOk, sramOk, sramTempOk);
}

void
OP2Battery::timerFired(OSObject *owner, IOTimerEventSource *sender)
{
	OP2Battery *self = OSDynamicCast(OP2Battery, owner);
	if (self && self->fRunning) {
		self->poll();
		sender->setTimeoutMS(30000);
	}
}

bool
OP2Battery::start(IOService *provider)
{
	IOLog("OP2Battery: starting PMI8994 battery service\n");
	if (!IOPMPowerSource::start(provider)) {
		IOLog("OP2Battery: IOPMPowerSource::start failed\n");
		return false;
	}

	fCore = (volatile uint32_t *)ml_io_map(OP2_SPMI_CORE_PHYS, OP2_SPMI_MAP_SIZE);
	if (fCore == NULL) {
		IOLog("OP2Battery: SPMI core map failed (0x%llx)\n", OP2_SPMI_CORE_PHYS);
		return false;
	}

	uint32_t ver = fCore[0];
	IOLog("OP2Battery: SPMI arbiter VERSION=0x%08x\n", ver);

	int enable = 1;
	(void)PE_parse_boot_argn("op2battery", &enable, sizeof(enable));
	fReal = (enable != 0);

	/* Probe peripheral subtypes to verify SPMI communication */
	if (fReal) {
		uint8_t sub = 0;
		if (spmiRead(OP2_PMI8994_SID, OP2_FG_SOC_BASE + 0x05, &sub, 1))
			IOLog("OP2Battery: FG SOC subtype=0x%02x\n", sub);
		if (spmiRead(OP2_PMI8994_SID, OP2_FG_BATT_BASE + 0x05, &sub, 1))
			IOLog("OP2Battery: FG BATT subtype=0x%02x\n", sub);
		if (spmiRead(OP2_PMI8994_SID, OP2_FG_MEMIF_BASE + 0x05, &sub, 1))
			IOLog("OP2Battery: FG MEMIF subtype=0x%02x\n", sub);
	}

	/* Initial poll before registering wakes powerd */
	poll();

	if (fReal) {
		fLoop = IOWorkLoop::workLoop();
		fTimer = IOTimerEventSource::timerEventSource(this, &OP2Battery::timerFired);
		if (fLoop && fTimer && fLoop->addEventSource(fTimer) == kIOReturnSuccess) {
			fTimerAdded = true;
			fRunning = true;
			fTimer->setTimeoutMS(30000);
		} else {
			IOLog("OP2Battery: workloop/timer setup failed; single-shot only\n");
		}
	}

	registerService();
	IOLog("OP2Battery: registered battery power source successfully\n");
	return true;
}

void
OP2Battery::stop(IOService *provider)
{
	if (fLoop && fTimerAdded) {
		(void)fLoop->runAction(&OP2Battery::stopPolling, this);
		fLoop->removeEventSource(fTimer);
		fTimerAdded = false;
	}
	IOPMPowerSource::stop(provider);
}

IOReturn
OP2Battery::stopPolling(OSObject *owner, void *, void *, void *, void *)
{
	OP2Battery *self = OSDynamicCast(OP2Battery, owner);
	if (!self) {
		return kIOReturnBadArgument;
	}
	self->fRunning = false;
	if (self->fTimer) {
		self->fTimer->cancelTimeout();
		self->fTimer->disable();
	}
	return kIOReturnSuccess;
}

#endif /* ARM64_BOARD_CONFIG_MSM8994 */
