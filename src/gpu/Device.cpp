// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "gpu/Device.h"

#include "f4kit/Log.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <d3d11.h>
#include <dxgi.h>

#include <mutex>

using namespace f4kit;

namespace flexrevive::gpu::device {

namespace {

std::mutex s_mutex;
bool s_tried = false;
ID3D11Device* s_device = nullptr;
ID3D11DeviceContext* s_context = nullptr;
DeviceInfo s_info;

// d3d11.dll is loaded rather than linked, so a machine without it fails here with a log line
// instead of failing to load the plugin at all. The game itself is a D3D11 title, so in
// practice it is always present; this is about the failure mode, not the likelihood.
using PFN_D3D11CreateDevice = HRESULT(WINAPI*)(IDXGIAdapter*, D3D_DRIVER_TYPE, HMODULE, UINT,
                                               const D3D_FEATURE_LEVEL*, UINT, UINT,
                                               ID3D11Device**, D3D_FEATURE_LEVEL*,
                                               ID3D11DeviceContext**);

void Release()
{
    if (s_context) {
        s_context->Release();
        s_context = nullptr;
    }
    if (s_device) {
        s_device->Release();
        s_device = nullptr;
    }
    s_info = DeviceInfo();
}

// Fills in the adapter description for whichever adapter the device landed on. Asked of the
// device rather than chosen up front, so the answer describes the card actually in use.
void DescribeAdapter(ID3D11Device* device)
{
    IDXGIDevice* dxgi = nullptr;
    if (FAILED(device->QueryInterface(__uuidof(IDXGIDevice), reinterpret_cast<void**>(&dxgi))))
        return;

    IDXGIAdapter* adapter = nullptr;
    if (SUCCEEDED(dxgi->GetAdapter(&adapter)) && adapter) {
        DXGI_ADAPTER_DESC desc{};
        if (SUCCEEDED(adapter->GetDesc(&desc))) {
            WideCharToMultiByte(CP_UTF8, 0, desc.Description, -1, s_info.description,
                                sizeof(s_info.description) - 1, nullptr, nullptr);
            s_info.vendorId = desc.VendorId;
            s_info.dedicatedMB = uint64_t(desc.DedicatedVideoMemory) / (1024 * 1024);
            // Microsoft's Basic Render Driver reports no dedicated memory and would run the
            // whole solver on the CPU through a driver, which is worse than the CPU path in
            // every respect.
            s_info.software = desc.VendorId == 0x1414 && desc.DedicatedVideoMemory == 0;
        }
        adapter->Release();
    }
    dxgi->Release();
}

} // namespace

const char* VendorName(uint32_t vendorId)
{
    switch (vendorId) {
    case 0x10DE: return "NVIDIA";
    case 0x1002: case 0x1022: return "AMD";
    case 0x8086: return "Intel";
    case 0x1414: return "Microsoft";
    default: return "unknown vendor";
    }
}

bool Start()
{
    std::lock_guard<std::mutex> lock(s_mutex);
    if (s_tried)
        return s_device != nullptr;
    s_tried = true;

    HMODULE lib = LoadLibraryW(L"d3d11.dll");
    if (!lib) {
        log::Write("gpu: d3d11.dll is not present, staying on the CPU");
        return false;
    }
    auto create = reinterpret_cast<PFN_D3D11CreateDevice>(
        GetProcAddress(lib, "D3D11CreateDevice"));
    if (!create) {
        log::Write("gpu: d3d11.dll has no D3D11CreateDevice, staying on the CPU");
        return false;
    }

    // 11_0 is the floor, because cs_5_0 is. That covers NVIDIA from Fermi, AMD from GCN and
    // Intel from Haswell, which is every card that can run this game at a playable rate.
    const D3D_FEATURE_LEVEL wanted[] = {D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0};
    D3D_FEATURE_LEVEL got{};

    // Null adapter, so the driver picks the same one the game is rendering on. Choosing the
    // most capable adapter instead would be wrong on a laptop, where copying between two
    // cards costs more than the work.
    const HRESULT hr = create(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, wanted,
                              UINT(sizeof(wanted) / sizeof(wanted[0])), D3D11_SDK_VERSION,
                              &s_device, &got, &s_context);
    if (FAILED(hr) || !s_device || !s_context) {
        log::Write("gpu: no Direct3D 11 device (0x%08X), staying on the CPU", unsigned(hr));
        Release();
        return false;
    }
    s_info.featureLevel = uint32_t(got);
    DescribeAdapter(s_device);

    if (s_info.software) {
        log::Write("gpu: only a software rasteriser is available, staying on the CPU");
        Release();
        return false;
    }

    // Structured buffers are how every buffer here is bound, so a device without them is no
    // use whatever its feature level claims.
    D3D11_FEATURE_DATA_D3D10_X_HARDWARE_OPTIONS opts{};
    if (got < D3D_FEATURE_LEVEL_11_0 &&
        (FAILED(s_device->CheckFeatureSupport(D3D11_FEATURE_D3D10_X_HARDWARE_OPTIONS, &opts,
                                              sizeof(opts))) ||
         !opts.ComputeShaders_Plus_RawAndStructuredBuffers_Via_Shader_4_x)) {
        log::Write("gpu: %s has no compute support, staying on the CPU", s_info.description);
        Release();
        return false;
    }

    log::Write("gpu: %s (%s), %llu MB, feature level %u.%u", s_info.description,
               VendorName(s_info.vendorId), (unsigned long long)s_info.dedicatedMB,
               unsigned(s_info.featureLevel >> 12), unsigned((s_info.featureLevel >> 8) & 0xF));
    return true;
}

bool Available()
{
    std::lock_guard<std::mutex> lock(s_mutex);
    return s_device != nullptr;
}

const DeviceInfo& Info()
{
    return s_info;
}

void* Device()
{
    return s_device;
}

void* Context()
{
    return s_context;
}

void StopForTesting()
{
    std::lock_guard<std::mutex> lock(s_mutex);
    Release();
    s_tried = false;
}

}
