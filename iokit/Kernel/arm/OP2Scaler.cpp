/*
 * AppleM2ScalerCSCDriver stand-in (OnePlus 2 / MSM8994), 2026-11-17.
 *
 * IOSurfaceAcceleratorCreate() only matches this class name and IOServiceOpen()s it. WebCore's
 * IOSurface::convertToFormat (Safari's tab snapshots -> YUV) does not check the result and calls
 * CFRunLoopAddSource(IOSurfaceAcceleratorGetRunLoopSource(NULL)), so with no such service MobileSafari
 * crashed (SIGSEGV in CFRunLoopAddSource, x1 = 0). This service opens and accepts a notification port;
 * every transform is refused (kIOReturnUnsupported), so callers keep the untransformed surface. The
 * MDP5 scaler could implement real transforms later.
 */
#include <pexpert/arm64/board_config.h>

#if defined(ARM64_BOARD_CONFIG_MSM8994)
#include <IOKit/IOService.h>
#include <IOKit/IOUserClient.h>
#include <IOKit/IOLib.h>

class AppleM2ScalerCSCDriver : public IOService
{
	OSDeclareDefaultStructors(AppleM2ScalerCSCDriver);
public:
	bool start(IOService *provider) APPLE_KEXT_OVERRIDE;
	IOReturn newUserClient(task_t owningTask, void *securityID, UInt32 type, IOUserClient **handler) APPLE_KEXT_OVERRIDE;
};

class OP2ScalerUserClient : public IOUserClient
{
	OSDeclareDefaultStructors(OP2ScalerUserClient);
public:
	IOReturn clientClose(void) APPLE_KEXT_OVERRIDE;
	IOReturn registerNotificationPort(mach_port_t port, UInt32 type, io_user_reference_t refCon) APPLE_KEXT_OVERRIDE;
	IOReturn externalMethod(uint32_t selector, IOExternalMethodArguments *args,
	    IOExternalMethodDispatch *dispatch, OSObject *target, void *reference) APPLE_KEXT_OVERRIDE;
};

OSDefineMetaClassAndStructors(AppleM2ScalerCSCDriver, IOService);
OSDefineMetaClassAndStructors(OP2ScalerUserClient, IOUserClient);

bool
AppleM2ScalerCSCDriver::start(IOService *provider)
{
	if (!IOService::start(provider)) {
		return false;
	}
	registerService();
	IOLog("OP2 Scaler: AppleM2ScalerCSCDriver stub registered (transforms unsupported)\n");
	return true;
}

IOReturn
AppleM2ScalerCSCDriver::newUserClient(task_t owningTask, void *securityID, UInt32 type, IOUserClient **handler)
{
#pragma unused(securityID, type)
	OP2ScalerUserClient *uc = new OP2ScalerUserClient;
	if (uc == NULL) {
		return kIOReturnNoMemory;
	}
	if (!uc->initWithTask(owningTask, NULL, 0) || !uc->attach(this) || !uc->start(this)) {
		uc->detach(this);
		uc->release();
		return kIOReturnError;
	}
	*handler = uc;
	return kIOReturnSuccess;
}

IOReturn
OP2ScalerUserClient::clientClose(void)
{
	terminate();
	return kIOReturnSuccess;
}

IOReturn
OP2ScalerUserClient::registerNotificationPort(mach_port_t port, UInt32 type, io_user_reference_t refCon)
{
#pragma unused(port, type, refCon)
	return kIOReturnSuccess;
}

IOReturn
OP2ScalerUserClient::externalMethod(uint32_t selector, IOExternalMethodArguments *args,
    IOExternalMethodDispatch *dispatch, OSObject *target, void *reference)
{
#pragma unused(args, dispatch, target, reference)
	static int logged;
	if (logged++ < 10) {
		IOLog("OP2 Scaler: selector %u refused\n", selector);
	}
	return kIOReturnUnsupported;
}

#endif /* ARM64_BOARD_CONFIG_MSM8994 */
