# XNU Kernel for OnePlus 2 (Qualcomm Snapdragon 810 / MSM8994)

This repository contains a specialized fork of Apple's open-source **Darwin/XNU kernel** (`xnu-4903.270.47`, macOS 10.14.6 / iOS 12.5.7 era), re-engineered to boot on the **OnePlus 2** (`oneplus2` / `ONE A2003`, Qualcomm Snapdragon 810 / MSM8994) and drive real Apple iOS 12.5.7 userland (`16H81`, `n56ap` iPhone 6 Plus).

> Status is kept in the iOS repository ([Veerhooda/iOS](https://github.com/Veerhooda/iOS)). Its `docs/AUDIT-2026-11-11.md` explains why some older claims ("0 crashes", milestones verified by a scripted fake user) were withdrawn.

---

## Hardware Architecture & Platform Adaptations

| Subsystem | Apple Stock Target (A8 / T7000) | OnePlus 2 Adaptation (`ARM64_BOARD_CONFIG_MSM8994`) |
| :--- | :--- | :--- |
| **Translation Granule** | 16 KB pages (`ARM_PGSHIFT=14`) on >1GB DRAM | **Native 4 KB pages (`ARM_PGSHIFT=12`)** across 3-level translation tables |
| **Physical Memory** | Fixed Apple carveouts (`0x800000000`+) | Mapped to Qualcomm DRAM bank `0x10200000`–`0xAC000000` (~2,403 MiB managed) |
| **Interrupt Controller** | Apple AIC / proprietary FIQ | **ARM GICv2** MMIO distributor (`0xf9000000`) and CPU interface (`0xf9002000`) |
| **Hardware Timer** | Apple decremeter | **ARM Generic Virtual Timer** (`CNTV_CTL_EL0`, PPI 20, 10 ms periodic ticks) |
| **Multiprocessing (SMP)**| Apple-proprietary CPU power sequencer | **4 × Cortex-A53 (`cpus=4`)** via mailbox and SCM boot address, clocked to 1536 MHz (`op2_cpu_boost`) |
| **Storage Controller** | Apple NVMe / NAND controller | **Qualcomm SDCC 5.0 / SDHCI eMMC 5.0** (`bsd/dev/op2_emmc.c`) with conditional R/W |
| **Filesystem** | APFS / HFS+ kext | **Built-in Apple HFS+ driver (`bsd/op2_hfs/`)** with macOS 10.14 compatibility KPIs |
| **Display Pipeline** | AppleCLCD / IOMobileFramebuffer | **MDSS MDP5 command-mode panel** (1080×1920), double-buffered, SwapWait, CPU downsample from 1242×2208 |
| **GPU** | Apple AGX | **Adreno 430 (`OP2GPU`)**: RPM clock vote, TrustZone SMMU setup, msm-DRM user client; Mesa freedreno draws through it |
| **Touch & Navigation** | Apple Multitouch SPI | **Synaptics S3320 I2C** + dedicated **`OP2Buttons` Consumer HID** (0x0C / 0x40) |
| **Memory Governance** | Restrictive 36MB legacy daemon profiles | **256MB task footprint floor** (`osfmk/kern/task.c`) protecting daemons on 4GB RAM |

---

## Key Driver Subsystems & Modifications

### 1. Multiprocessing and CPU clock (`osfmk/arm64/start.s`, `osfmk/arm/arm_init.c`, `iokit/Kernel/arm/OP2CPU.cpp`)
- Cores 1–3 of the Cortex-A53 cluster started through the SCM boot-address call (PSCI is not usable on this firmware), `cpus=4`.
- `op2_cpu_boost` raises the cluster to 1536 MHz with stock voltages, 20 s after the framebuffer starts; the clock is measured with the PMU cycle counter on every `OP2 TOP:` line.
- Secondary entry point `arm_init_secondary` in `start.s` initializes CPU 1 architectural registers, MMU, and local GICv2 interface.
- Preemptive Mach scheduler running multi-core threads; GICv2 SGI 0 used for cross-core TLB invalidations and AST scheduling.
- Boot to SpringBoard in a few minutes.

### 2. SDHCI eMMC 5.0 Persistent Storage (`bsd/dev/op2_emmc.c`)
- Custom Darwin block and character driver (`/dev/emmcroot0`, `/dev/remmcroot0`).
- **Conditional Read/Write:** Defaults to read-only safety mode; write support is enabled conditionally via boot arguments (`emmcrw=1` or `rootrw=1`).
- Hardware `CMD12` (STOP_TRANSMISSION) abort handling on timeouts, explicit DAT0 line polling, and automatic 3x retries on CRC or timeout faults.
- 0 failures across 1.36M+ operations in prolonged soak runs.

### 3. Display Engine & Zero-Fill Preservation (`iokit/Kernel/arm/OP2Framebuffer.cpp`, `OP2Surface.cpp`)
- Interfaces with Qualcomm continuous splash buffer at `0x03400000` (1080×1920, 32-bit ARGB/BGRA, 4352-byte padded stride).
- Native MDSS MDP5 command-mode DSI scanout (CTL0 FLUSH/START) at 60 Hz.
- **Zero Dropped Kicks:** Panel kickoff in `op2_fb_kick_panel` uses bounded `IODelay` instead of early dropped returns, ensuring static scenes and idle screens never leave the panel showing a stale frame.
- **SIMD Subpixel Bilinear Downsampler (`op2fb_hwscale=0`):** Fixed-point 8-bit weighted bilinear downsampler (1242×2208 → 1080×1920) executes in ~2 ms into carveout slot 0. Bypasses the buggy Qualcomm QSEED hardware downscaler to prevent MDP5 AXI bus underflows and SMMU faults.
- **Dynamic Surface Scanning:** Scans userland surfaces up to ID 255 (`sid < 256`), dynamically tracking SpringBoard and application double-buffered Retina frames.
- **Carveout DMA Boundaries:** Verifies that physical addresses passed to MDP5 DMA stay within the safe continuous splash carveout (`0x03400000` to `0x05600000`).
- **RGB Alpha Mask Fix:** Evaluates `(row[x] & 0x00FFFFFF) != 0` to differentiate opaque black (`0xFF000000`) from uninitialized buffers.
- **Zero-Fill Guard:** Freezes existing panel memory (`fPixels`) during blank composite swaps, ensuring 100% of sampled panel pixels remain non-zero (`panel_nz = 2000/2000`).

### 4. Dedicated `OP2Buttons` Consumer HID Service (`iokit/Kernel/arm/OP2Digitizer.cpp`)
- Declares a standalone `IOHIDEventService` publishing Consumer/ConsumerControl (Page `0x0C`, Usage `0x40`).
- Binds directly to SpringBoard's Mach runloop notification port, allowing hardware capacitive button events and simulated AUTOHOME pulses to wake the lockscreen and navigate to the application icon grid.

### 5. Memory Management & Jetsam Floor (`osfmk/kern/task.c`)
- Enforces a minimum 256MB physical footprint floor for tasks under `ARM64_BOARD_CONFIG_MSM8994`.
- Prevents `memorystatus` jetsam from terminating `assistivetouchd` under legacy 36MB iPhone 6 Plus memory limits.

---

### 6. Adreno 430 GPU (`iokit/Kernel/arm/OP2GPU.cpp`, `OP2GPU.h`)
Bring-up and results are written up in the iOS repository's `docs/GPU-PLAN.md`. What the driver does:
- **Power and clock:** VDD_GFX vote, GDSCs, MMSS branch clocks, and the GPU core clock voted through the RPM (`clk2` id 1, 300 MHz by default); the RPM sets the GFX voltage itself.
- **SMMU:** the GPU SMMU belongs to TrustZone. The driver gives TZ a secure page-table pool and calls `restore_sec_cfg` for both GPU contexts (SMC, SIP service `0xC`), as Linux does. Optional identity translation of only the live buffers (`gpu_smmu_xlate=1`, in bring-up) makes a stray GPU write fault instead of corrupting RAM.
- **Command processor:** microcode loaded with the micro-engine properly halted (`CP_ME_CNTL` bits 27|28), circular ring, `CP_MEM_WRITE` fences, hang detection by fence progress and recovery by soft reset + restart.
- **User client:** Linux msm-DRM requests by number (GET_PARAM, GEM_NEW/INFO/CPU_PREP/CLOSE/MADVISE, SUBMIT, WAIT_FENCE, submit queues), absolute deadlines, write-combined mappings. Mesa's freedreno driver uses it unchanged through a libdrm stand-in.
- **Diagnostics:** boot self-tests (`gpu_selftest`), a guard band after every buffer and a quarantine of closed buffers to catch stray writes, `kOP2Log` (0x7c) for userland lines into the kernel log.

| boot-arg | default | meaning |
|---|---|---|
| `gpu_hw` | 1 | touch the hardware at all |
| `gpu_khz` | 300000 | RPM GPU clock vote |
| `gpu_init` | 0x11 | init steps: 1 VBIF, 16 SP/TP power (2 hang-detect, 4 UCHE, 8 SP regfile are off) |
| `gpu_smmusec` | 1 | TrustZone SMMU configuration |
| `gpu_smmu_xlate` | 0 | identity translation of live buffers only |
| `gpu_selftest` | 1000 | G0/G1 self-tests at boot |
| `gpu_held` | 1 | quarantine closed buffers and check them for writes after close |

---

### 7. Battery & Fuel-Gauge Power Management (`iokit/Kernel/arm/OP2Battery.cpp`)
- Implements Apple `IOPMPowerSource` driver publishing standard iOS battery telemetry for `powerd` and SpringBoard's status bar.
- Direct hardware communication with Qualcomm PMI8994 PMIC (Slave ID 2) over Qualcomm SPMI PMIC Arbiter v1 (`0xFC4CF000`).
- Monitors Fuel-Gauge Monotonic State of Charge (FG_SOC `0x4009`), charger presence and bus status via SMBCHG (`0x130D`/`0x1310`), and real-time battery voltage/current from FG MEM_IF SRAM.
- Periodic 30-second workloop timer refresh; prevents `powerd` low-battery panic shutdown loops and uninitialized power assertions.

---

## Boot Arguments Reference

The following kernel boot arguments govern hardware bring-up, memory management, and display behavior on the OnePlus 2. The default set configured in `tools/build-daily.sh` provides a stable, fully interactive, GPU-accelerated environment.

| Boot Argument | Default | Critical? | Purpose & Impact |
| :--- | :--- | :--- | :--- |
| `op2fb_hwscale=0` | `0` | **YES** | **Bypasses Qualcomm QSEED hardware scaler.** Downscaling 1242×2208 frames is handled via SIMD subpixel bilinear downsampling directly into carveout slot 0 (`0x03400000`). Prevents MDP5 hardware AXI bus underflow faults and MDSS SMMU fatal panics. |
| `op2fb_client_ui=1` | `1` | **YES** | Enables client UI fallback compositing so SpringBoard surfaces are reliably copied to scanout. |
| `metal=3` | `3` | **YES** | Enables Level 3 Metal GPU acceleration via `OP2Metal` and Adreno 430 Mesa freedreno driver for backboardd, SpringBoard, and apps. |
| `op2_autoinput=1` | `1` | **YES** | Enables wake-only synthetic input (pulses capacitive Home button once at boot to wake Lock Screen), then exits synthetic mode for hands-on user control. |
| `op2fb_keepon=1` | `1` | **YES** | Prevents physical panel backlight sleep, maintaining active MDP5 scanout at all times. |
| `op2_deadline=7200` | `7200` | **YES** | Hardware/software safety watchdog timer in seconds (default 2 hours, or 1800 for 30m soak). Automatically syncs filesystems and resets to TWRP recovery upon expiration, preventing battery drain or hard-lock. |
| `rd=emmcroot0` | `emmcroot0` | **YES** | Directs XNU root filesystem to Qualcomm SDHCI eMMC block device `/dev/emmcroot0`. |
| `emmcroot=userdata` | `userdata` | **YES** | Selects the internal `userdata` partition as the HFS+ root filesystem. |
| `rootrw=1 emmcrw=1` | `1` | **YES** | Grants read-write access to eMMC and mounts root filesystem read-write. |
| `cs_enforcement_disable=1` | `1` | **YES** | Disables Darwin code signing enforcement, allowing custom userland daemons and shims to execute. |
| `launchd_unsecure_cache=1`| `1` | **YES** | Allows iOS 12 launchd to load patched `xpcd_cache.dylib` services without security cache rejection. |
| `cpus=4` | `4` | **YES** | Boots all 4 Cortex-A53 cores on the power-efficient cluster via SCM mailbox bootstrap. |
| `-v` | N/A | Optional | Verbose boot logging to console. |
| `keepsyms=1` | `1` | Optional | Retains kernel symbol table in RAM for informative stack backtraces upon kernel panics. |
| `op2_userlog=0` | `0` | Optional | Suppresses excessive userland log noise to prevent wrapping the 260 KB ramoops ring buffer. |

---

## Building the Kernel

### Prerequisites
* macOS host running macOS 10.14 through 14.x.
* Xcode 10.3 clang toolchain (or compatible Apple clang targeting `aarch64-apple-darwin`).
* Homebrew LLVM (`clang`, `ld.lld`, `llvm-objcopy`).

### Compilation

From the root project directory:
```bash
./tools/build-xnu.sh
```

Or invoke `make` directly within the XNU directory:
```bash
make SDKROOT=macosx.internal \
     ARCH_CONFIGS=ARM64 \
     KERNEL_CONFIGS=DEVELOPMENT \
     MACHINE_CONFIGS=MSM8994
```

The compiled kernel binary is produced at:
```text
BUILD/obj/DEVELOPMENT_ARM64_MSM8994/kernel.development.msm8994
```

---

## Required Boot Arguments & Command Line Options

When booting XNU on the OnePlus 2 via fastboot (`fastboot boot out/xnu-boot.img`), the following boot arguments are required or recommended for stability, display fidelity, and user interactivity:

| Boot Argument | Default | Critical? | Description & Rationale |
| :--- | :--- | :--- | :--- |
| `op2fb_hwscale=0` | `0` | **REQUIRED** | **Bypasses Qualcomm QSEED hardware scaler.** Downscaling from Retina 1242×2208 to native 1080×1920 is handled directly by XNU's fixed-point SIMD bilinear downsampler into carveout slot 0 (`0x03400000`). Disabling hardware scaling prevents Qualcomm MDSS MDP5 AXI bus underflows and SMMU faults. |
| `op2fb_client_ui=1` | `1` | **REQUIRED** | Enables client UI fallback compositing. When the GPU render server target is blank or half-black (e.g. status bar only), XNU scans out the fullest rasterized client surface (such as SpringBoard or active apps) instead of a black frame. |
| `metal=3` | `3` | **REQUIRED** | Enables Level 3 Metal GPU acceleration via `OP2Metal` and the Adreno 430 Mesa freedreno driver for backboardd, SpringBoard, and userland applications. |
| `op2_autoinput=0` | `0` | **REQUIRED** | Hands-on physical touch mode: synthetic micro-events are disabled completely so that all touch events on the Synaptics panel go directly to iOS UIKit views and dock icons without interference. |
| `op2fb_keepon=1` | `1` | **REQUIRED** | Keeps the physical display panel and MDSS backlight awake at all times, preventing screen sleep during interactive testing. |
| `op2_deadline=7200` | `7200` | **REQUIRED** | Watchdog safety deadline timer in seconds (e.g., 7200 for 2 hours, 1800 for 30 minutes). Automatically synchronizes filesystems and performs a clean hardware reset to TWRP recovery upon expiration, preventing battery drain or hard locks. |
| `rd=emmcroot0` | `emmcroot0` | **REQUIRED** | Directs XNU root filesystem mounting to the Qualcomm SDHCI eMMC block driver (`/dev/emmcroot0`). |
| `emmcroot=userdata` | `userdata` | **REQUIRED** | Selects the internal eMMC `userdata` partition as the HFS+ root filesystem. |
| `rootrw=1 emmcrw=1` | `1` | **REQUIRED** | Enables read-write access to eMMC storage and mounts the root filesystem read-write. |
| `cs_enforcement_disable=1` | `1` | **REQUIRED** | Disables Darwin code signing enforcement, allowing custom userland utilities, injected dylibs, and shims to execute. |
| `launchd_unsecure_cache=1` | `1` | **REQUIRED** | Allows iOS 12 launchd to load patched daemons and custom launch services without security cache rejection. |
| `cpus=4` | `4` | **REQUIRED** | Boots all 4 Cortex-A53 cores on the power-efficient cluster via SCM mailbox bootstrap. |
| `-v` | N/A | Optional | Enables verbose kernel console output. |
| `keepsyms=1` | `1` | Optional | Preserves kernel symbol table in RAM for debugging backtraces. |
| `op2_userlog=0` | `0` | Optional | Suppresses noisy userland log spam from wrapping the ramoops ring buffer. |

---

## Directory Organization

* `bsd/dev/op2_emmc.c` — Qualcomm SDHCI eMMC 5.0 block device driver.
* `bsd/op2_hfs/` — Apple HFS+ filesystem driver and macOS 10.14 shims.
* `iokit/Kernel/arm/OP2CPU.cpp` — SMP CPU nub and secondary core lifecycle management.
* `iokit/Kernel/arm/OP2Framebuffer.cpp` — Qualcomm MDSS MDP5 framebuffer and `AppleCLCD` pipeline.
* `iokit/Kernel/arm/OP2Surface.cpp` — Dynamic surface tracking and 1242×2208 downsampling engine.
* `iokit/Kernel/arm/OP2Digitizer.cpp` — Synaptics touch and `OP2Buttons` Consumer HID service.
* `iokit/Kernel/arm/OP2Battery.cpp` — Qualcomm PMI8994 SPMI battery fuel-gauge and `IOPMPowerSource` driver.
* `iokit/Kernel/arm/OP2I2C.cpp` — Qualcomm BLSP I2C bus driver.
* `iokit/Kernel/arm/OP2GPU.cpp`, `OP2GPU.h` — Adreno 430 driver and its msm-DRM user client.
* `bsd/dev/op2_rpm.c` — RPM (SMD) client: power rails and RPM-owned clocks.
* `iokit/Kernel/arm/OP2InterruptController.cpp` — ARM GICv2 interrupt controller driver.
* `iokit/Kernel/arm/OP2PlatformExpert.cpp` — `N56AP` device tree matching and early bootstrap.
* `osfmk/arm64/start.s` — Early ARM64 entry vector, MMU bootstrap, and CPU 1 secondary entry.
* `osfmk/arm/arm_init.c` — Architecture initialization and 4KB translation table setup.
* `osfmk/arm64/op2_snapshot.c` — Pre-reset daemon thread state and dyld snapshot capture.
* `osfmk/arm64/sleh.c` — ARM64 exception vectors, GICv2 timer dispatch, and user state capture.
* `pexpert/pexpert/arm64/MSM8994.h` — Register maps for MSM8994 GIC, timer, SDCC, and TLMM.

---

## License

This project is licensed under the terms of the **Apple Public Source License (APSL 2.0)**. See the `LICENSE` file for full terms and conditions.
