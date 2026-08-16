// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>

// F4SE's plugin ABI, declared here rather than included, so a plugin builds with nothing but
// CMake and a compiler. The layout is fixed public API.
//
// A plugin exports F4SEPlugin_Version for 0.7 and later loaders, and F4SEPlugin_Query for the
// 0.6.x ones that predate it. Exporting both means either generation finds the one it
// understands.

struct F4SEPluginVersionData {
    enum { kVersion = 1 };
    enum {
        // Every address found by signature scanning, none hardcoded.
        kAddressIndependence_Signatures = 1 << 0,
        kAddressIndependence_AddressLibrary_1_10_980 = 1 << 1,
        kAddressIndependence_AddressLibrary_1_11_137 = 1 << 2,
    };
    enum {
        // No dependency on game structure layouts.
        kStructureIndependence_NoStructs = 1 << 0,
        kStructureIndependence_1_10_980Layout = 1 << 1,
        kStructureIndependence_1_11_137Layout = 1 << 2,
    };

    uint32_t dataVersion;
    uint32_t pluginVersion;
    char name[256];
    char author[256];
    uint32_t addressIndependence;
    uint32_t structureIndependence;
    uint32_t compatibleVersions[16];
    uint32_t seVersionRequired;
    uint32_t reservedNonBreaking;
    uint32_t reservedBreaking;
    uint8_t reserved[512];
};

struct F4SEInterface {
    uint32_t f4seVersion;
    uint32_t runtimeVersion;
    uint32_t editorVersion;
    uint32_t isEditor;
    void* (*QueryInterface)(uint32_t id);
    uint32_t (*GetPluginHandle)(void);
    uint32_t (*GetReleaseIndex)(void);
    const void* (*GetPluginInfo)(const char* name);
    // Absent from F4SE 0.6.x. Never called, so the shorter old layout is also valid.
    const char* (*GetSaveFolderName)(void);
};

// Filled in by F4SEPlugin_Query for F4SE 0.6.x loaders, which recognise a plugin solely by
// that export. 0.7+ reads F4SEPlugin_Version and ignores Query; both are exported.
struct PluginInfo {
    enum { kInfoVersion = 1 };
    uint32_t infoVersion;
    const char* name;
    uint32_t version;
};

#define MAKE_EXE_VERSION(major, minor, build) \
    ((((major) & 0xFF) << 24) | (((minor) & 0xFF) << 16) | (((build) & 0xFFF) << 4))

