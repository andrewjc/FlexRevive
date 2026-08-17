// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>

// The compute device the GPU backend runs on.
//
// Deliberately a device of this plugin's own, on the same adapter the game chose, rather than
// the game's. Fallout 4's immediate context belongs to its render thread and is not safe to
// touch from another, so sharing it would mean synchronising against the renderer on every
// dispatch. A second device costs a few MB and is fully isolated: nothing here can stall,
// corrupt or crash the game's rendering, which matters more than the memory.
//
// Everything is optional. A machine with no capable adapter, a driver that refuses to create a
// device, or a card that lacks compute simply leaves this inert and the solver stays on the
// CPU. The one thing that must never happen is a half-initialised device that appears usable.
namespace flexrevive::gpu::device {

// What the plugin found, for the log and for deciding whether to proceed.
struct DeviceInfo {
    char description[128] = {};   // adapter name, as the driver reports it
    uint32_t vendorId = 0;        // 0x10DE NVIDIA, 0x1002 AMD, 0x8086 Intel
    uint64_t dedicatedMB = 0;
    uint32_t featureLevel = 0;    // 0xB000 is 11_0, the minimum for cs_5_0
    bool software = false;        // a rasteriser rather than a card, which is not worth using
};

// Vendor-independent: every one of the three is a hardware adapter carrying cs_5_0, and none
// of them needs a runtime this plugin would have to ship. Named only so the log can say which
// one answered, since a bug report about debris is usually really about a driver.
const char* VendorName(uint32_t vendorId);

// Creates the device if it is not already up. Returns whether one is available afterwards, so
// a caller can treat a first and a later call the same way. Safe to call repeatedly and from
// any thread; the first caller does the work.
//
// Failure is normal rather than exceptional and is logged once with the reason.
bool Start();

// Whether a usable device exists. False before Start, and after a Start that failed.
bool Available();

// What Start found. Zeroed when no device is available.
const DeviceInfo& Info();

// The D3D11 device and its immediate context, as void* so this header stays free of
// <d3d11.h> and the solver does not acquire a Windows graphics dependency it cannot test.
// Both are null when no device is available.
void* Device();
void* Context();

// Releases the device. Only for the tests, which create and destroy one repeatedly; the
// plugin itself holds its device for the life of the process, since a driver teardown during
// DLL detach is not something to invite.
void StopForTesting();

}
