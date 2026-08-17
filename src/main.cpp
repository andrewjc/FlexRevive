// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

// FlexRevive, weapon debris for Fallout 4, simulated on the CPU.
//
// Every engine address is located by scanning the running executable, and the only game
// structure read is the Setting layout, which is validated before use. No Address Library
// and no per-build offsets.

#include "Config.h"
#include "f4kit/CrashLog.h"
#include "f4kit/EngineSetting.h"
#include "gpu/GpuSolver.h"
#include "DebrisSolver.h"
#include "f4kit/Log.h"
#include "f4kit/PeImage.h"

#include <cstdint>

#include "f4kit/F4SEPlugin.h"

extern "C" {

__declspec(dllexport) F4SEPluginVersionData F4SEPlugin_Version = {
    F4SEPluginVersionData::kVersion,
    MAKE_EXE_VERSION(1, 1, 1), // plugin version 1.1.1
    "FlexRevive",
    "AndyR007",
    F4SEPluginVersionData::kAddressIndependence_Signatures,
    F4SEPluginVersionData::kStructureIndependence_NoStructs,
    // Informational: the independence flags above already permit any runtime.
    {
        MAKE_EXE_VERSION(1, 10, 163),
        MAKE_EXE_VERSION(1, 10, 980),
        MAKE_EXE_VERSION(1, 10, 984),
        MAKE_EXE_VERSION(1, 11, 137),
        MAKE_EXE_VERSION(1, 11, 180),
        MAKE_EXE_VERSION(1, 11, 189),
        MAKE_EXE_VERSION(1, 11, 221),
        0,
    },
    0, // no minimum F4SE version beyond one that understands this structure
    0,
    0,
    {},
};

// Entry point for 0.6.x loaders, which expect the plugin to gate on runtime version here.
// Any runtime carrying the debris imports is accepted; only the Creation Kit is refused.
__declspec(dllexport) bool F4SEPlugin_Query(const F4SEInterface* f4se, PluginInfo* info)
{
    if (info) {
        info->infoVersion = PluginInfo::kInfoVersion;
        info->name = "FlexRevive";
        info->version = MAKE_EXE_VERSION(1, 1, 0);
    }
    return f4se && !f4se->isEditor;
}

__declspec(dllexport) bool F4SEPlugin_Load(const F4SEInterface* f4se)
{
    using namespace flexrevive;
    using namespace f4kit;

    log::Open(L"My Games\\Fallout4\\F4SE", L"FlexRevive.log");
    const uint32_t runtime = f4se ? f4se->runtimeVersion : 0;
    log::Write("FlexRevive 1.1.1, runtime %u.%u.%u, F4SE %08X", (runtime >> 24) & 0xFF,
               (runtime >> 16) & 0xFF, (runtime >> 4) & 0xFFF, f4se ? f4se->f4seVersion : 0);

    // Installed first, so a fault anywhere past this point reaches the log.
    crash::Install(config::PluginDir());

    config::Load();
    if (!config::Get().enabled) {
        log::Write("disabled in FlexRevive.ini, the game is left untouched");
        return true;
    }

    if (!pe::Init()) {
        log::Write("could not read the game's PE headers, plugin inert");
        return true;
    }

    // Patches the import table, so it applies whether or not the debris DLLs have loaded.
    if (!solver::Install()) {
        log::Write("no Flex imports found in this executable, nothing to replace. Weapon "
                   "debris is not a feature of this build of the game.");
        return true;
    }

    // Weapon debris, if the user has it switched off.
    //
    // Everything here turns on one fact: the game parses its ini files after F4SE has loaded
    // its plugins, not before. So at this moment every Setting object still holds the value it
    // was compiled with, and bNVFlexEnable reads as off whatever the user has configured.
    //
    // That is what made the old version of this useless. It read the Setting, always saw off,
    // wrote on, and logged that it had enabled weapon debris; the ini parse then ran and put
    // the user's value back. The log said the same thing in every session, and the setting had
    // never been anything but decorative. It was measured rather than argued: in a session
    // where the prefs enabled debris and all thirty-four Flex entry points ran, this code
    // still reported the setting had been off and turned on.
    //
    // The prefs file is the only account of the user's intent that exists yet, so it is both
    // what gets read and what gets written. Writing it costs one restart, once, which is the
    // honest price of a decision the game makes before this plugin can speak.
    if (config::Get().forceEnableWeaponDebris) {
        const int enabled = engine::ReadGameIni(L"Fallout4", L"Fallout4Prefs.ini", L"NVFlex",
                                                L"bNVFlexEnable", -1);
        if (enabled == 1) {
            log::Write("weapon debris is enabled in Fallout4Prefs.ini, so the game will build "
                       "its debris system and this plugin will drive it");
        } else if (enabled < 0) {
            log::Write("Fallout4Prefs.ini has no bNVFlexEnable under [NVFlex] and could not be "
                       "read, so weapon debris cannot be enabled for you. Add it, set it to 1, "
                       "and restart.");
        } else if (engine::WriteGameIni(L"Fallout4", L"Fallout4Prefs.ini", L"NVFlex",
                                        L"bNVFlexEnable", 1)) {
            log::Write("weapon debris was switched off, so the game will not build its debris "
                       "system this run and nothing can spawn. It is now enabled for the next "
                       "launch: RESTART THE GAME and debris will work. This is a one-off; set "
                       "ForceEnableWeaponDebris=0 in FlexRevive.ini to manage it yourself.");
        } else {
            log::Write("weapon debris is switched off and could not be enabled for you. Set "
                       "bNVFlexEnable=1 under [NVFlex] in Fallout4Prefs.ini and restart.");
        }

        // The two rendering-side switches default to on, so they only need writing when the
        // user has turned them off, and the same ini-parse ordering applies to them.
        for (const wchar_t* key : {L"bNVFlexInstanceDebris", L"bNVFlexDrawDebris"}) {
            if (engine::ReadGameIni(L"Fallout4", L"Fallout4Prefs.ini", L"NVFlex", key, 1) == 0 &&
                engine::WriteGameIni(L"Fallout4", L"Fallout4Prefs.ini", L"NVFlex", key, 1))
                log::Write("%ls was off in Fallout4Prefs.ini, enabled for the next launch", key);
        }
    }

    // iQuality:NVFlex in Fallout4Prefs.ini selects one of three debris budgets:
    //
    //   iQuality=0  ->  6000 particles, culled at 2000 units, 32 neighbours
    //   iQuality=1  -> 16000 particles, culled at 3000 units, 48 neighbours
    //   iQuality=2  -> 32768 particles, culled at 4000 units, 64 neighbours
    //
    // The engine copies these during startup, before any plugin loads, and does not consult
    // the Setting objects again. Writing to them here has no effect, so this only records the
    // tier in force and checks the container built later against it.
    if (config::Get().debrisQuality >= 0)
        solver::ExpectParticleBudget(config::Get().debrisQuality);

    // Asking for the GPU brings the device up and reports what it found. Whether the step
    // actually runs there is the backend's decision, not the setting's: it declines while any
    // part of the work would still have to come back to the CPU mid-step, because the transfer
    // costs more than the arithmetic it would save.
    if (config::Get().computeBackend == config::Backend::kGpu) {
        gpu::Start();
        if (gpu::Ready())
            log::Write("compute backend: gpu");
        else
            log::Write("compute backend: cpu. ComputeBackend=gpu was asked for but %s",
                       gpu::NotReadyReason());
    }

    log::Write("ready, weapon debris is simulated by this plugin");
    return true;
}

}
