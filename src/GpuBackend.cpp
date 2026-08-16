#include "GpuBackend.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <d3d11.h>
#include <dxgi1_2.h>

#include <cstring>

namespace flexrevive::gpu {

bool IsNvidia(const Capability& cap)
{
    return cap.vendorId == kVendorNvidia;
}

Mode ModeFromSetting(int value)
{
    switch (value) {
    case 1:  return kModeAuto;
    case 2:  return kModeForce;
    default: return kModeOff;
    }
}

const char* ReasonText(Reason reason)
{
    switch (reason) {
    case kReasonDisabled:       return "disabled in the INI";
    case kReasonNoDevice:       return "no graphics device was found";
    case kReasonNoCompute:      return "the device cannot run compute at the level needed";
    case kReasonNotEnoughMemory:return "the device has too little memory for the working set";
    case kReasonUntestedVendor: return "the device is not one this has been tested against";
    case kReasonTooFewPieces:   return "too few pieces to be worth the round trip";
    case kReasonSelected:       return "selected";
    case kReasonForced:         return "forced on in the INI";
    default:                    return "unknown";
    }
}

// Enumerating adapters and creating a throwaway device are both done through the loader
// rather than by linking, so a machine without D3D11 loads the plugin and simply reports
// nothing present instead of failing to start.
Capability Probe()
{
    Capability best;

    HMODULE dxgiDll = LoadLibraryW(L"dxgi.dll");
    HMODULE d3dDll = LoadLibraryW(L"d3d11.dll");
    if (!dxgiDll || !d3dDll)
        return best;

    using PFN_CreateFactory = HRESULT(WINAPI*)(REFIID, void**);
    auto createFactory =
        reinterpret_cast<PFN_CreateFactory>(GetProcAddress(dxgiDll, "CreateDXGIFactory1"));
    auto createDevice =
        reinterpret_cast<PFN_D3D11_CREATE_DEVICE>(GetProcAddress(d3dDll, "D3D11CreateDevice"));
    if (!createFactory || !createDevice)
        return best;

    IDXGIFactory1* factory = nullptr;
    if (FAILED(createFactory(__uuidof(IDXGIFactory1), reinterpret_cast<void**>(&factory))) ||
        !factory)
        return best;

    for (UINT i = 0;; ++i) {
        IDXGIAdapter1* adapter = nullptr;
        if (factory->EnumAdapters1(i, &adapter) == DXGI_ERROR_NOT_FOUND)
            break;
        if (!adapter)
            continue;

        DXGI_ADAPTER_DESC1 desc = {};
        if (SUCCEEDED(adapter->GetDesc1(&desc)) &&
            !(desc.Flags & DXGI_ADAPTER_FLAG_SOFTWARE)) {
            Capability cap;
            cap.present = true;
            cap.vendorId = desc.VendorId;
            cap.deviceId = desc.DeviceId;
            cap.dedicatedMemory = desc.DedicatedVideoMemory;
            WideCharToMultiByte(CP_UTF8, 0, desc.Description, -1, cap.description,
                                sizeof(cap.description) - 1, nullptr, nullptr);

            // Compute at the level the solver needs means feature level 11. A device is made
            // and released purely to ask, so nothing is held open afterwards.
            const D3D_FEATURE_LEVEL wanted[] = {D3D_FEATURE_LEVEL_11_0};
            D3D_FEATURE_LEVEL got = D3D_FEATURE_LEVEL_9_1;
            ID3D11Device* device = nullptr;
            if (SUCCEEDED(createDevice(adapter, D3D_DRIVER_TYPE_UNKNOWN, nullptr, 0, wanted, 1,
                                       D3D11_SDK_VERSION, &device, &got, nullptr))) {
                cap.computeCapable = got >= D3D_FEATURE_LEVEL_11_0;
                if (device)
                    device->Release();
            }

            // Prefer the most capable adapter, then the one with the most memory: a laptop
            // reports its integrated part alongside its discrete one.
            const bool better = !best.present ||
                                (cap.computeCapable && !best.computeCapable) ||
                                (cap.computeCapable == best.computeCapable &&
                                 cap.dedicatedMemory > best.dedicatedMemory);
            if (better)
                best = cap;
        }
        adapter->Release();
    }

    factory->Release();
    return best;
}

Choice Select(const Capability& cap, Mode mode, int pieceCount, int minPiecesForGpu)
{
    Choice choice;

    if (mode == kModeOff) {
        choice.reason = kReasonDisabled;
        return choice;
    }

    // Hardware first, so forcing cannot dispatch into a device that is not there.
    if (!cap.present) {
        choice.reason = kReasonNoDevice;
        return choice;
    }
    if (!cap.computeCapable) {
        choice.reason = kReasonNoCompute;
        return choice;
    }
    if (cap.dedicatedMemory < kMinDeviceMemory) {
        choice.reason = kReasonNotEnoughMemory;
        return choice;
    }

    if (mode == kModeForce) {
        choice.backend = kBackendGpu;
        choice.reason = kReasonForced;
        return choice;
    }

    if (!IsNvidia(cap)) {
        choice.reason = kReasonUntestedVendor;
        return choice;
    }
    if (pieceCount < minPiecesForGpu) {
        choice.reason = kReasonTooFewPieces;
        return choice;
    }

    choice.backend = kBackendGpu;
    choice.reason = kReasonSelected;
    return choice;
}

}
