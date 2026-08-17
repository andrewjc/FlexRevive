// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "Config.h"
#include "f4kit/Ini.h"
#include "DebrisSolver.h"
#include "f4kit/Log.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include <cstdio>
#include <cstdlib>
#include <share.h>

namespace flexrevive::config {

using namespace f4kit;
using namespace f4kit::ini;

namespace {

Values s_values;
wchar_t s_dir[MAX_PATH] = {};
wchar_t s_ini[MAX_PATH] = {};

// Written verbatim on first run. This file is the mod's documentation, and ships with the
// binary that reads it.
const char* kDefaultIni = R"INI(; FlexRevive - weapon debris physics for Fallout 4
;
; Fallout 4's weapon debris - the chunks that fly off surfaces when you shoot them - is
; driven by a GPU solver built against CUDA 7.5, which predates every current GPU
; architecture. On modern hardware that library faults the moment the game asks it for a
; collision mesh, which is why enabling weapon debris has crashed the game for years.
;
; This plugin does not patch around the crash. It replaces the solver: the game's calls are
; redirected into physics implemented here, running on the CPU. So weapon debris works, and
; it works on any GPU.
;
; Everything below can be changed with the game closed; the plugin reads this file at
; startup. Values are documented with their defaults.

[General]
; Master switch. 0 leaves the game entirely untouched.
Enabled=1

; Turn weapon debris on at startup even if your settings have it disabled.
; Leave this at 1 unless you have deliberately set bNVFlexEnable yourself: almost every
; existing Fallout 4 setup has weapon debris turned off, because that was the only way to
; avoid the crash this plugin fixes. With this at 0 and debris disabled in your prefs, the
; plugin will load correctly and do nothing visible.
;
; Turning the setting on also mounts Fallout4 - Nvflex.ba2, which holds every debris mesh the
; game owns. The engine mounts that archive during startup only if debris was already on, well
; before any plugin loads, so with debris off it is skipped and each chunk resolves to a file
; that is not there.
ForceEnableWeaponDebris=1

; Take each chunk's spawn velocity, size and inertia from the engine rather than inferring
; them. Two things happen with this on:
;
;   - The call that creates a debris fragment is watched for the velocity it states, then
;     handed straight on, so the engine gets back exactly what it always did.
;   - Each chunk is voxelized into particles, so the engine receives a fragment that
;     describes itself instead of one that claims to be empty.
;
; With this off, velocity is inferred from how a burst is arranged, every chunk collides at
; one shared size, and chunk descriptions are left empty. That is the fallback if the game
; becomes unstable when debris spawns.
UseEngineSpawnData=1

; Run debris at real-world speed.
;
; The game asks the solver to advance a fixed 10 milliseconds each time it calls it, but it
; only calls once per rendered frame. Below 100 fps that means less physics happens than a
; second of real time contains, and debris falls in slow motion. With this on, the step comes
; from the clock instead. 0 uses the engine's number as-is.
RealTimestep=1

; How much debris you expect the game to produce, as its debris quality tier.
;
; This does NOT set anything. The tier is chosen by iQuality:NVFlex in Fallout4Prefs.ini and is
; read during startup, before any plugin loads, so no plugin can change it. What this does is
; check what the game actually allocated and tell you in the log if it is lower than you wanted.
;
;   iQuality=0   6000 particles   debris culled 2000 units away   32 neighbours
;   iQuality=1  16000 particles   debris culled 3000 units away   48 neighbours
;   iQuality=2  32768 particles   debris culled 4000 units away   64 neighbours
;
; The budget bounds how much debris appears: the game refuses to spawn a chunk once the
; particles already in flight plus that chunk's own would exceed it, and refusing one abandons
; the rest of that impact's debris. So iQuality is the volume dial.
;
; Going past tier 2
; -----------------
; The game has only three tiers, but it reads the numbers out of the tier you select rather
; than hardcoding them, so the top tier can simply be given larger ones. In Fallout4Prefs.ini
; under [NVFlex], keep iQuality=2 and raise its entries:
;
;   iMaxParticles2=65536      more debris on screen at once
;   fKillRadius2=6000.0000    debris survives further from you
;   iMaxNeighbors2=96
;
; The game builds its container at whatever iMaxParticles2 says. Raise MaxPieces below to
; match, or the solver will cap what the game hands it.
;
; -1 disables the check entirely.
DebrisQuality=2

; Hand the engine debris chunks that really contain particles.
;
; Those particles are drawn from the iMaxParticles budget, which is what bounds how much debris
; reaches the screen, so this is what makes the budget above mean anything. It also gets the
; engine to send back each chunk's rest positions.
;
; If the game crashes the moment debris appears, set this to 0 and report it.
EngineParticles=1

; Write per-call solver tracing to FlexRevive.log. Very noisy - for bug reports only.
VerboseLog=0

[Physics]
; Gravity multiplier. 1.0 is true gravity: the engine asks for 686.6 units/s^2, and at
; this game's scale of 69.99 units per metre that works out to 9.81 m/s^2.
; Lower values give a floatier, more cinematic tumble. Range 0.1 - 4.0.
GravityScale=1.0

; Air resistance. Above 1.0 debris settles sooner; below 1.0 it sails further.
; Range 0.0 - 5.0.
DragScale=1.0

; Bounciness on impact. 0.0 lands dead, higher values skip off hard surfaces.
; Range 0.0 - 3.0.
RestitutionScale=1.0

; Grip. Raise if debris slides too far when it lands, lower for more skating.
; Range 0.0 - 3.0.
FrictionScale=1.0

[Rotation]
; Spin given to a freshly spawned chunk, in radians per second. 0 disables spawn spin.
; Range 0.0 - 60.0.
SpawnSpin=12.0

; How fast a chunk flies away from the impact it came from, in units/sec (about 70 units to
; the metre). The engine never tells the solver which way the shot came from, but fragments
; from one hit arrive as a cluster around the point that was struck, so each one is thrown
; outward from the middle of its own cluster. 0 makes debris drop from a standstill instead.
; Range 0.0 - 600.0.
SpawnBurst=150.0

; How hard the gun throws the chunks it just created.
;
; The game states a launch velocity for every fragment it makes, and at full strength that is
; brisk enough to send them sailing away from the surface. This scales it. It is deliberately
; separate from ImpactShock below, which governs how much the same shot disturbs rubble that
; was already lying there: one is the gun creating debris, the other is the gun hitting a
; pile, and they want different amounts.
;
; 1.0 is exactly what the game asks for. Range 0.0 - 2.0.
SpawnVelocityScale=0.3

; How strongly off-centre impacts set pieces tumbling. Worth knowing: an impulse straight
; through a piece's centre cannot rotate it, so this scales the sideways (friction) part
; of a contact, which is what actually makes a chunk cartwheel. Range 0.0 - 4.0.
ImpactTorque=1.0

; Round pieces roll down slopes instead of only sliding. 1 = on.
Rolling=1

[Collision]
; Debris collides with other debris, so chunks pile up instead of passing through each
; other. Costs a little CPU when a lot of debris is in the air at once.
DebrisVsDebris=1

; Scales the collision radius of every chunk together, relative to the radius the engine asks
; for. If debris rests visibly above the ground, lower this; if it sinks into surfaces, raise
; it. Range 0.1 - 5.0.
PieceRadiusScale=1.0

; How quickly a heap of rubble stops jostling itself.
;
; Chunks resting against each other grind fractionally as the pile shifts, and left alone that
; goes on for seconds after everything looks stationary. This bleeds off the relative motion of
; touching pieces, the way friction does in a real pile of stone; their shared motion is
; untouched, so rubble sliding down a slope still slides. Raise it to settle sooner, lower it
; for longer-lived tumbling. 0 turns it off. Range 0.0 - 5.0.
SettleRate=1.5

; How hard a shot landing in a pile scatters the rubble already lying there.
;
; Worth knowing that this is an embellishment rather than accuracy. Debris is not in the
; collision world a bullet is tested against, and nothing but an explosion's force field can
; disturb debris that already exists, so without this a shot into a heap of rubble does nothing
; to it. What the plugin does have is where each burst of fresh chunks appeared and how fast
; the game launched them, which describes the impact faithfully even though the reaction is
; invented.
;
; 0 turns it off. Range 0.0 - 5.0.
ImpactShock=1.0

; How heavy the debris feels.
;
; Each chunk already has its own mass, taken from the number of particles the game gave it, so
; a slab shoulders a splinter aside rather than trading with it evenly, larger pieces cut
; through the air better than small ones, and walking into a heavy chunk shifts it less than a
; light one. This is the global feel on top of that: it divides air resistance, bounce, and how
; far a chunk is carried when something walks through it.
;
; 1.0 is the raw physics the game asks for. Higher is heavier and deader. Range 0.25 - 4.0.
Heft=1.6

[Limits]
; Safety valve on simultaneously simulated pieces. The game decides how much debris to spawn,
; through DebrisQuality above; this only stops the solver being handed an unbounded amount of
; it. The default is set well clear of what the top tier produces, so ordinarily it never
; engages. Lower it if you want a hard cap on solver cost. Range 64 - 65536.
MaxPieces=32768

[Performance]
; How many threads sweep debris against the world, counting the game's own thread.
;
;   0  size it from your CPU: all but two threads from six up, all but one below that
;   1  no extra threads; the solver runs entirely on the game thread
;   N  N threads in total, so N-1 extra
;
; So a quad core runs the solver on three and a six core on four. Less headroom is held back
; on a small machine because it buys less there: this pool only runs inside the call the game
; is already blocked in, so the threads it uses are not competing with rendering.
;
; The default is right for almost everyone. Raising it past the automatic value takes cores
; away from the game itself, which usually costs more frame time than it saves. Range 0 - 16.
;
; Unlike everything above, this is read once when the game starts, so changing it needs a
; restart rather than just a new cell.
SolverThreads=0

; Which processor steps the debris. cpu or gpu.
;
; The default is cpu and suits almost everyone. The solver runs inside a call the game is
; already waiting on, so on a machine with spare cores the CPU work is close to free, while
; the GPU is usually the part already struggling to draw the frame.
;
; gpu exists for the opposite machine: few cores, a capable card. Asking for it is a request
; rather than an instruction. The plugin brings up a compute device of its own on whichever
; card the game is using, reports what it found in the log, and steps on the GPU only for work
; that is genuinely faster there. Anything else stays on the CPU, so this setting can never be
; the reason the physics is wrong. Whatever it decides is written to the log at startup.
;
; As of this version the collision passes are still the CPU's, and moving only the rest would
; mean copying every piece back from the card in the middle of each substep, which costs far
; more than the arithmetic it saves. So gpu currently initialises the device, says so, and
; steps on the CPU. It is not yet a performance setting.
;
; Needs a restart, like SolverThreads.
ComputeBackend=cpu
)INI";

void ResolvePaths()
{
    if (s_ini[0])
        return;

    HMODULE self = nullptr;
    GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                           GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                       reinterpret_cast<LPCWSTR>(&ResolvePaths), &self);
    GetModuleFileNameW(self, s_dir, MAX_PATH);

    wchar_t* slash = wcsrchr(s_dir, L'\\');
    if (slash)
        slash[1] = 0;

    swprintf_s(s_ini, L"%sFlexRevive.ini", s_dir);

    ini::SetFile(s_ini);
}

void WriteDefaultIni()
{
    FILE* f = _wfsopen(s_ini, L"w", _SH_DENYWR);
    if (!f) {
        log::Write("config: could not create %ls (read-only folder?) - using defaults", s_ini);
        return;
    }
    fputs(kDefaultIni, f);
    fclose(f);
    log::Write("config: wrote default FlexRevive.ini");
}

} // namespace

Values& Get()
{
    return s_values;
}

const wchar_t* PluginDir()
{
    ResolvePaths();
    return s_dir;
}

void Load()
{
    ResolvePaths();

    if (GetFileAttributesW(s_ini) == INVALID_FILE_ATTRIBUTES)
        WriteDefaultIni();

    Values& v = s_values;

    v.enabled = ReadBool(L"General", L"Enabled", v.enabled);
    v.forceEnableWeaponDebris =
        ReadBool(L"General", L"ForceEnableWeaponDebris", v.forceEnableWeaponDebris);
    v.useEngineSpawnData =
        ReadBool(L"General", L"UseEngineSpawnData", v.useEngineSpawnData);
    v.realTimestep = ReadBool(L"General", L"RealTimestep", v.realTimestep);
    v.engineParticles = ReadBool(L"General", L"EngineParticles", v.engineParticles);
    v.debrisQuality = ClampI(ReadInt(L"General", L"DebrisQuality", v.debrisQuality), -1, 2);
    v.verboseLog = ReadBool(L"General", L"VerboseLog", v.verboseLog);

    v.gravityScale = Clamp(ReadFloat(L"Physics", L"GravityScale", v.gravityScale), 0.1f, 4.0f);
    v.dragScale = Clamp(ReadFloat(L"Physics", L"DragScale", v.dragScale), 0.0f, 5.0f);
    v.restitutionScale =
        Clamp(ReadFloat(L"Physics", L"RestitutionScale", v.restitutionScale), 0.0f, 3.0f);
    v.frictionScale =
        Clamp(ReadFloat(L"Physics", L"FrictionScale", v.frictionScale), 0.0f, 3.0f);

    v.spawnSpin = Clamp(ReadFloat(L"Rotation", L"SpawnSpin", v.spawnSpin), 0.0f, 60.0f);
    v.spawnBurst = Clamp(ReadFloat(L"Rotation", L"SpawnBurst", v.spawnBurst), 0.0f, 600.0f);
    v.spawnVelocityScale =
        Clamp(ReadFloat(L"Rotation", L"SpawnVelocityScale", v.spawnVelocityScale), 0.0f, 2.0f);
    v.impactTorque = Clamp(ReadFloat(L"Rotation", L"ImpactTorque", v.impactTorque), 0.0f, 4.0f);
    v.rolling = ReadBool(L"Rotation", L"Rolling", v.rolling);

    v.debrisVsDebris = ReadBool(L"Collision", L"DebrisVsDebris", v.debrisVsDebris);
    v.pieceRadiusScale =
        Clamp(ReadFloat(L"Collision", L"PieceRadiusScale", v.pieceRadiusScale), 0.1f, 5.0f);
    v.settleRate = Clamp(ReadFloat(L"Collision", L"SettleRate", v.settleRate), 0.0f, 5.0f);
    v.impactShock = Clamp(ReadFloat(L"Collision", L"ImpactShock", v.impactShock), 0.0f, 5.0f);
    v.heft = Clamp(ReadFloat(L"Collision", L"Heft", v.heft), 0.25f, 4.0f);

    v.maxPieces = ClampI(ReadInt(L"Limits", L"MaxPieces", v.maxPieces), 64, 65536);

    v.solverThreads = ClampI(ReadInt(L"Performance", L"SolverThreads", v.solverThreads), 0, 16);

    {
        static const wchar_t* const kBackends[] = {L"cpu", L"gpu"};
        v.computeBackend = Backend(ReadEnum(L"Performance", L"ComputeBackend", kBackends, 2,
                                            int(v.computeBackend)));
    }

    log::SetVerbose(v.verboseLog);
    Apply();

    log::Write("config: gravity %.2f drag %.2f bounce %.2f grip %.2f spin %.1f burst %.0f torque %.2f "
               "rolling %d d-vs-d %d radius %.2f max %d threads %d backend %s",
               v.gravityScale, v.dragScale, v.restitutionScale, v.frictionScale, v.spawnSpin,
               v.spawnBurst, v.impactTorque, v.rolling ? 1 : 0, v.debrisVsDebris ? 1 : 0,
               v.pieceRadiusScale, v.maxPieces, v.solverThreads,
               v.computeBackend == Backend::kGpu ? "gpu" : "cpu");
}

void Apply()
{
    solver::Tunables& t = solver::g_tune;
    const Values& v = s_values;

    t.gravityScale = v.gravityScale;
    t.dragScale = v.dragScale;
    t.restitutionScale = v.restitutionScale;
    t.frictionScale = v.frictionScale;
    t.spawnSpin = v.spawnSpin;
    t.spawnBurst = v.spawnBurst;
    t.spawnVelocityScale = v.spawnVelocityScale;
    t.impactTorque = v.impactTorque;
    t.rolling = v.rolling;
    t.debrisVsDebris = v.debrisVsDebris;
    t.pieceRadiusScale = v.pieceRadiusScale;
    t.settleRate = v.settleRate;
    t.impactShock = v.impactShock;
    t.heft = v.heft;
    t.maxPieces = v.maxPieces;
    t.solverThreads = v.solverThreads;
    t.useEngineSpawnData = v.useEngineSpawnData;
    t.realTimestep = v.realTimestep;
    t.engineParticles = v.engineParticles;
}

}
