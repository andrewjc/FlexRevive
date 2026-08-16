// FlexRevive, weapon debris for Fallout 4, simulated on the CPU.
//
// Every engine address is located by scanning the running executable, and the only game
// structure read is the Setting layout, which is validated before use. No Address Library
// and no per-build offsets.

#include "Config.h"
#include "f4kit/CrashLog.h"
#include "f4kit/EngineSetting.h"
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

    // INI settings are parsed before F4SE loads plugins, so these objects already hold the
    // user's values and writes to them take effect.
    if (config::Get().forceEnableWeaponDebris) {
        engine::Binding debrisSettings[] = {
            {"bNVFlexEnable:NVFlex", nullptr, 0},
            {"bNVFlexInstanceDebris:NVFlex", nullptr, 0},
            {"bNVFlexDrawDebris:NVFlex", nullptr, 0},
        };
        constexpr int kCount = int(sizeof(debrisSettings) / sizeof(debrisSettings[0]));

        engine::Resolve(debrisSettings, kCount);
        for (engine::Binding& b : debrisSettings) {
            bool before = false;
            if (!engine::GetBool(b, before)) {
                log::Write("could not locate %s, leaving it alone", b.fullName);
                continue;
            }
            if (before) {
                log::Write("%s already enabled", b.fullName);
                continue;
            }
            if (engine::SetBool(b, true))
                log::Write("%s was off, turned on (this is what makes debris appear; set "
                           "ForceEnableWeaponDebris=0 in FlexRevive.ini to opt out)",
                           b.fullName);
            else
                log::Write("%s is off and could not be changed, weapon debris will not "
                           "spawn until you enable it yourself", b.fullName);
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

    log::Write("ready, weapon debris is simulated by this plugin");
    return true;
}

}
