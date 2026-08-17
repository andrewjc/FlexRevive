// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "Config.h"
#include "Fragment.h"
#include "Collision.h"
#include "Contact.h"
#include "Response.h"
#include "Shock.h"
#include "SlotTracker.h"
#include "Math3D.h"
#include "PairContact.h"
#include "PieceGrid.h"
#include "f4kit/ImportHook.h"
#include "ParticleMap.h"
#include "DebrisSolver.h"
#include "f4kit/Log.h"
#include "f4kit/PeImage.h"
#include "f4kit/ThreadPool.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <chrono>
#include <mutex>
#include <unordered_map>
#include <new>
#include <vector>

namespace flexrevive::solver {

Tunables g_tune;

namespace {

using namespace f4kit;
using namespace math;
using namespace collision;
using namespace contact;
using namespace response;
using namespace shock;
using namespace particles;
using namespace grid;

std::atomic<int> s_calls{0};
bool s_installed = false;

// Per-function call counters. The first few calls of each are logged in full, then only the
// running total.
struct CallSite {
    const char* name;
    int count;
};

constexpr int kMaxTracked = 48;
CallSite s_sites[kMaxTracked] = {};
int s_siteCount = 0;

void Note(const char* name)
{
    s_calls.fetch_add(1, std::memory_order_relaxed);

    for (int i = 0; i < s_siteCount; ++i) {
        if (s_sites[i].name == name) {
            ++s_sites[i].count;
            if (s_sites[i].count <= 3)
                log::Trace("%s (call %d)", name, s_sites[i].count);
            else if (s_sites[i].count == 50)
                log::Trace("%s reached 50 calls (further calls not logged)", name);
            return;
        }
    }
    if (s_siteCount < kMaxTracked) {
        s_sites[s_siteCount++] = {name, 1};
        log::Trace("%s (first call)", name);
    }
}

// ---- solver state -----------------------------------------------------------------------
// The engine owns spawning, mesh rasterisation and rendering. The solver keeps the particle
// buffers it is handed, advances them, and hands them back.
//
// Buffer layout is fixed by the engine: particles are float4 (x, y, z, invMass), velocities
// float3. invMass == 0 means pinned.
struct Solver {
    std::vector<float> particles;   // 4 per particle
    std::vector<float> velocities;  // 3 per particle
    int count = 0;
    // Taken from the parameter block the engine publishes, so each debris material keeps the
    // feel it was authored with. A field outside a plausible range falls back to the default.
    float gravity[3] = {0.0f, 0.0f, -9.8f};
    float radius = 3.5f;
    float dynamicFriction = 0.6f;  // matches kFriction below
    float staticFriction = 0.6f;
    float restitution = 0.25f;     // matches kRestitution below
    float damping = 0.0f;
    float dissipation = 0.0f;
    float solidRestDistance = 0.0f;  // how far apart the engine wants solid particles kept
    float particleFriction = 0.0f;   // friction between particles, i.e. between chunks
    float sleepThreshold = 0.0f;     // below this the engine considers a particle stopped
    float maxSpeed = 0.0f; // 0 = unlimited
    bool haveParams = false;
};

// Holds the buffers the engine writes spawn data into and reads results from through
// flexExtGetParticleData. These are the buffers that must advance for anything to be seen.
struct Container {
    std::vector<float> particles;   // 4 per particle (x, y, z, invMass)
    std::vector<float> velocities;  // 3 per particle
    std::vector<int> phases;
    std::vector<float> normals;     // 4 per particle
    // The engine's active particle list, handed out as the sixth output of
    // flexExtGetParticleData. The engine fills it with the indices of every live particle and
    // passes it to flexSetActive.
    std::vector<int> active;
    int maxParticles = 0;
    // Highest slot the engine has put a live particle in, so a substep walks the occupied
    // prefix rather than all 32768 slots of a top-tier container.
    int liveHigh = 0;
    Solver* solver = nullptr;
};

// Debris pieces are rigid bodies rather than loose particles: the engine rasterises a mesh
// into a particle group and draws it at the transform reported back here.
struct Pieces {
    std::vector<float> rotations;    // 4 per rigid (quaternion x,y,z,w)
    std::vector<float> translations; // 3 per rigid
    std::vector<float> velocities;   // 3 per rigid
    std::vector<float> angular;      // 3 per rigid, radians/sec
    std::vector<float> radii;        // per rigid, measured from its own rest particle cloud
    std::vector<float> gyration;     // per rigid, the radius its inertia behaves as
    std::vector<int> hullIndex;     // per rigid, into s_hulls; -1 falls back to a sphere
    // Relative mass, from the particle count the engine gave the chunk.
    std::vector<float> mass;
    std::vector<uint8_t> resting;    // 1 once a piece has settled
    // What was last handed back through flexGetRigidTransforms. The engine round-trips those
    // values into the next flexSetRigids unchanged, so a slot arriving with a different
    // position has been re-seeded with a new fragment. Since the engine recycles a fixed pool,
    // the piece count does not change when that happens, and this is the only signal of it.
    std::vector<float> lastReported; // 3 per rigid
    std::vector<uint8_t> reported;   // 1 once a piece has been handed back at least once
    int count = 0;
};

// How far an incoming position must differ from the last reported one before the slot counts
// as re-seeded rather than round-tripped. Values come back bit-exact, but the engine also
// nudges a settled piece by a unit or so from time to time. 16 units is about 23 cm: well
// above any nudge, well below the spacing between impacts.
constexpr float kRespawnDistance = 16.0f;

// A blast published by the engine. Debris inside the radius is pushed away from its centre.
struct ForceField {
    float pos[3] = {0, 0, 0};
    float radius = 0.0f;
    float strength = 0.0f;
    bool linearFalloff = true;
};
// Stride of the array the engine publishes, which is wider than the fields read from it.
constexpr size_t kForceFieldSize = 44;

std::mutex s_solverMutex;
std::vector<Container*> s_containers;
std::vector<ForceField> s_forceFields;
// How many settled pieces the blasts above woke during the current update. A blast that
// arrives and reaches nothing looks exactly like one that never arrived, and the two want
// different fixes, so the count is reported rather than inferred.
int s_blastWoke = 0;
Pieces s_pieces;
// Which slots the engine re-seeded on the current flexSetRigids call, so the position and
// orientation passes agree on what is new.
//
// s_wasResting remembers which of them had already settled. The engine occasionally shifts a
// piece that is lying still by more than the round-trip threshold, and such a slot is
// indistinguishable from a recycled one by position alone.
std::vector<uint8_t> s_fresh;
std::vector<uint8_t> s_wasResting;

// Which piece holds each container particle, so a settled body's particles hold still with it.
// The engine reads these buffers back, and a particle left integrating on its own accelerates
// for as long as its piece exists, which is not what the piece is doing.

// The reverse: which container slots each piece owns, so the particle buffers can be advanced
// by walking the pieces that are moving rather than the whole container.
std::vector<int> s_pieceSlotStart;
std::vector<int> s_pieceSlots;

// Each moving piece's position before it is stepped, so its particles can be carried by the
// distance it actually travels.
std::vector<float> s_prePos;

// Where wakes come from, counted only when VerboseLog asks for it. Six unrelated mechanisms
// can clear a resting flag and from the outside they look identical: the heap stirs. Nothing
// here is touched unless the tracing is on.
bool s_traceWakes = false;
std::atomic<int> s_wokeCollider{0};
int s_wokeShock = 0;
int s_wokeBlast = 0;
int s_wokeMover = 0;
int s_wokePair = 0;
int s_wokeFresh = 0;
int s_wokeMigrated = 0;

// What the graphics device can do, probed once, and which backend the last step ran on.
// Where a step's time actually goes, against how much debris is in the air.
//
// A GPU backend has to dispatch and then read its results back before the frame can be drawn,
// and that round trip costs the same at any size. Whether it could ever pay for itself is a
// question about this curve: what the CPU solver costs per step as the piece count climbs.
struct StepCost {
    double sweepMs = 0.0;      // integrate and sweep, across the pool
    double pairMs = 0.0;       // piece against piece, serial
    double particleMs = 0.0;   // advancing the engine's own particle buffers
    double listMs = 0.0;       // deciding which pieces need a step at all
    double settleMs = 0.0;     // parking pieces the pile is holding up
    double loopMs = 0.0;       // the substep loop as a whole
    double totalMs = 0.0;
    int steps = 0;
};
StepCost s_cost;
int s_costSamples = 0;
int s_costPeak = 0;

// Neighbourhoods for piece-versus-piece, rebuilt each step.
grid::PieceGrid s_pieceGrid;

// The pieces needing a step this frame, compacted so the settled majority never reaches the
// parallel loop. Reused between frames to keep the step allocation-free.
std::vector<int> s_stepList;

// Which slots are in the step list, so a pair of moving pieces is handled once. Taken with the
// list rather than read from the resting flags, which change as the pass resolves contacts.
std::vector<uint8_t> s_stepping;

// Pieces held up by another piece this step. A chunk resting on other chunks never reaches a
// world contact, which is where the other sleep tests live.
std::vector<uint8_t> s_supported;

// Height a piece has climbed since it appeared, and how it climbed it. A slot is reset when the
// engine seeds a new chunk into it, so a recycled slot does not inherit the last one's history.
std::vector<float> s_riseFrom;
std::vector<float> s_risePrev;
std::vector<uint8_t> s_riseLogged;
constexpr float kRiseReport = 600.0f;
constexpr int kRiseReports = 24;
constexpr float kSankReport = 400.0f;
int s_riseReported = 0;
int s_sankReported = 0;

std::vector<Impact> s_impacts;

// A moving collider reduced to a sphere, used only to decide which settled pieces to wake.
struct Mover {
    float pos[3] = {0, 0, 0};
    float reach = 0.0f;
};
std::vector<Mover> s_movers;

// The slots seeded by the current flexSetRigids call, plus a counter distinguishing one burst
// from the next so a recycled slot does not repeat its predecessor's tumble.
std::vector<int> s_freshList;
// Every reported position, keyed exactly, so a piece the engine moves to a different slot is
// recognised as the same piece rather than taken for a new one.
// Everything that travels with a piece when it changes slots.
struct PieceState {
    float vel[3];
    float ang[3];
    float rot[4];
    float radius;
    float gyration;
    int shape;
    uint8_t resting;
};
std::vector<PieceState> s_migrated;
std::vector<int> s_migrateTo;
// How many times the engine has set rigids without reading them back. Two sets per read would
// leave the second comparison against a position the engine never saw.
int s_setsSinceRead = 0;
// Which of those took their velocity from an engine spawn, so the inferred-direction fallback
// leaves them alone.
std::vector<uint8_t> s_seededFromEngine;
uint32_t s_spawnEpoch = 0;

// How much of a moving collider's motion a chunk responds to when it is walked into.
//
// A contact lasting a step delivers roughly the same impulse whatever it lands on, and an
// impulse changes a heavy chunk's velocity less than a light one's: the force follows the
// contact area while the inertia follows the volume, so what the chunk picks up falls off as
// the cube root of its mass. Heft scales the whole effect.
//
// This is applied to the collider's velocity where it is derived, so the contact is resolved
// in a single consistent frame. Scaling only the return to world space would leave the piece
// holding the unscaled remainder as spurious motion and fling it.
float KickShare(int piece)
{
    const float m = (piece < int(s_pieces.mass.size())) ? s_pieces.mass[size_t(piece)] : 1.0f;
    return CarryShare(m, g_tune.heft);
}

// The fastest a fragment is ever genuinely launched at. Observed spawns run 80 to 270
// units/s, and the inferred fallback is 150.
//
// This is a plausibility bound on the velocity read out of the container, not a clamp on the
// physics. Those particle buffers are integrated every step under gravity and never collide
// with anything, so a particle belonging to a piece that settled long ago keeps accelerating
// and levels out near terminal velocity, over a thousand units/s. When the engine later
// recycles that slot, the value sitting there is accumulated free fall rather than anything
// the engine wrote, and reading it as a launch throws a settled piece across the room.
constexpr float kMaxPlausibleSpawnSpeed = 600.0f;

// How many particles the engine last published through flexSetActive, which is what
// flexGetActiveCount reports back.
std::atomic<int> s_activeCount{0};
// The particle budget requested through DebrisQuality, compared against what the engine
// actually allocates.
int s_expectedBudget = 0;
std::unordered_map<void*, TriMesh> s_meshes;
std::vector<Collider> s_colliders;
std::vector<Collider> s_prevColliders; // last frame's, so a shape without an engine-supplied
                                 // previous transform still has one
// Every convex hull's planes, float4 each, rebuilt with the collider list.
std::vector<float> s_planes;
int s_vertexStride = 0;   // 3 or 4 floats per vertex; determined from the data, not assumed
int s_geometryStride = 0; // bytes per ColliderGeometry entry; likewise determined
bool s_geometryScanned = false;

// The geometry block is an array of unions over the collision primitives, 16 bytes each: the
// triangle-mesh member is a handle plus a scale.
constexpr size_t kGeometryEntrySize = 16;

// Collects candidate handle values, keeping the scan itself free of C++ objects: __try cannot
// appear in a function that needs unwinding.
void* s_meshHandleList[64] = {};
int s_meshHandleCount = 0;

// ---- forwarding to the original imports --------------------------------------------------
// Some entry points are intercepted only to read what the engine passes. Saving the address a
// thunk held before it was rewritten lets such a shim record its arguments and call straight
// through, leaving behaviour unchanged.

// ---- debris chunk geometry ---------------------------------------------------------------
// flexExtCreateRigidFromMesh is handed the mesh of the chunk about to be spawned, which is the
// only per-piece size description the engine ever provides.
struct ModelInfo {
    float radius = 0.0f;      // bounding radius about the mesh's own centre
    float gyration = 0.0f;    // rms radius of the particle cloud, for the inertia term
    float centre[3] = {0, 0, 0};
    int verts = 0;
    int tris = 0;
    int particles = 0;        // 0 if the chunk was too thin to voxelize meaningfully
    int hullIndex = -1;      // into s_hulls
};
std::unordered_map<void*, ModelInfo> s_models;

// Chunk silhouettes, in a pool that only grows. Pieces refer to a hull by index rather than
// pointer, since the engine can destroy a chunk's model while pieces built from it are still
// in the air. There are only ever as many entries as there are distinct debris models.
std::vector<fragment::Hull> s_hulls;
// Silhouettes built from engine-supplied rest positions, keyed by the cloud itself so the
// distinct debris models share entries rather than accumulating one per piece.
std::unordered_map<uint64_t, int> s_restHulls;
// How the engine packs rest positions, 3 or 4 floats each. Settled once, by which reading
// yields a chunk-sized cloud.
int s_restStride = 0;

// A spawn seen through flexExtCreateInstance, waiting to be matched to the slot the engine
// puts it in. Nothing states which slot that is, so the two are correlated by position: a
// freshly seeded slot's translation is the transform this call was given.
struct PendingSpawn {
    float xform[16] = {};
    int xformFloats = 0;
    float vel[3] = {0, 0, 0};
    float radius = 0.0f;
    float gyration = 0.0f;
    int hullIndex = -1;
    int age = 0;
    bool used = false;
};
std::vector<PendingSpawn> s_pendingSpawns;

// Where the position sits inside the transform the engine passes, which arrives as a bare
// float pointer. Settled by which reading lands on a translation the engine later reports;
// -1 until then.
int s_xformPosLayout = -1;
constexpr int kXformLayouts = 3;

// How close a spawn transform must be to a fresh slot's translation to count as the same
// event. Generous: the only competition is other spawns in the same frame, metres away.
constexpr float kSpawnMatchDistance = 8.0f;
constexpr int kSpawnMaxAge = 4; // flexSetRigids calls before an unclaimed spawn is dropped

void CandidatePos(const PendingSpawn& sp, int layout, float* out)
{
    switch (layout) {
    case 0: // column-major 4x4: translation in the last column
        out[0] = sp.xform[12]; out[1] = sp.xform[13]; out[2] = sp.xform[14];
        break;
    case 1: // position first, e.g. a position followed by a quaternion
        out[0] = sp.xform[0]; out[1] = sp.xform[1]; out[2] = sp.xform[2];
        break;
    default: // row-major 4x4: translation in the last column of each row
        out[0] = sp.xform[3]; out[1] = sp.xform[7]; out[2] = sp.xform[11];
        break;
    }
}

// Reads the transform without knowing its true length. Sixteen floats is the largest it could
// be, and a shorter one reads whatever follows: harmless, since the layout probe above trusts
// only values that match a real translation. Free of C++ objects, since __try cannot appear in
// a function that needs unwinding.
int ReadTransform(const float* src, float* dst)
{
    int got = 0;
    __try {
        for (int i = 0; i < 16; ++i) {
            dst[i] = src[i];
            got = i + 1;
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
    }
    return got;
}

// Probes a vertex array under a candidate stride. Returns false if the read faults, ruling out
// a stride that is too large without knowing the buffer's extent.
bool ProbeVertexBounds(const float* verts, int count, int stride, float* lo, float* hi)
{
    __try {
        for (int i = 0; i < count; ++i) {
            const float* v = verts + size_t(i) * stride;
            for (int a = 0; a < 3; ++a) {
                const float f = v[a];
                if (f != f || f > 1e18f || f < -1e18f)
                    return false; // NaN or absurd; not a vertex under this reading
                if (f < lo[a]) lo[a] = f;
                if (f > hi[a]) hi[a] = f;
            }
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return false;
    }
    return true;
}

bool IsKnownMeshHandle(const void* p)
{
    for (int i = 0; i < s_meshHandleCount; ++i)
        if (s_meshHandleList[i] == p)
            return true;
    return false;
}

// Walks the geometry block for handles this module issued, returning how many were seen and
// where the first two sat. Guarded, since the block's extent is not known.
int ScanForMeshHandles(const void* block, size_t bytes, size_t& firstOffset,
                       size_t& secondOffset)
{
    int found = 0;
    auto base = static_cast<const uint8_t*>(block);
    __try {
        for (size_t off = 0; off + sizeof(void*) <= bytes; off += sizeof(void*)) {
            auto p = *reinterpret_cast<void* const*>(base + off);
            if (!IsKnownMeshHandle(p))
                continue;
            if (found == 0)
                firstOffset = off;
            else if (found == 1)
                secondOffset = off;
            if (++found >= 8)
                break;
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return -1; // block shorter than assumed
    }
    return found;
}

// The handle returned from flexCreateSolver is the Solver itself.
Solver* AsSolver(void* handle)
{
    return static_cast<Solver*>(handle);
}

// A particle is live only with a finite position and a non-zero inverse mass. The rest of a
// container's slots are unused capacity and are left alone.
bool IsLiveParticle(const float* p)
{
    return p[3] > 0.0f && std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

// ---- replacement implementations --------------------------------------------------------
// The engine does not check for a failed initialisation and writes into whatever it is given,
// so anything returning memory or a handle must return something real. Entry points whose
// results are consumed only when particles are active may stay no-ops.
//
// Handles are opaque to the engine, so a zeroed block suffices; it is sized generously in case
// a handle is treated as a struct.
constexpr size_t kHandleBytes = 4096;

void* NewHandle()
{
    void* h = calloc(1, kHandleBytes);
    return h;
}

// Returns an error enum, where 0 is success.
int   WINAPI Shim_flexInit(...)            { Note("flexInit");            return 0; }
void  WINAPI Shim_flexShutdown(...)        { Note("flexShutdown");        }
void* WINAPI Shim_flexAcquireContext(...)  { Note("flexAcquireContext");  return NewHandle(); }
void* WINAPI Shim_flexCreateSolver(int maxParticles, int)
{
    Note("flexCreateSolver");
    auto* s = new (std::nothrow) Solver();
    if (s)
        log::Write("solver created (maxParticles=%d)", maxParticles);
    return s;
}

void WINAPI Shim_flexDestroySolver(void* handle)
{
    Note("flexDestroySolver");
    std::lock_guard<std::mutex> lock(s_solverMutex);
    delete AsSolver(handle);
}

// The parameter block begins { int numIterations; float gravity[3]; float radius; ... }.
// Taking gravity from the engine settles both the units and which axis is up.
void WINAPI Shim_flexSetParams(void* handle, const void* params)
{
    Note("flexSetParams");
    Solver* s = AsSolver(handle);
    if (!s || !params)
        return;

    auto* asInt = static_cast<const int*>(params);
    auto* asFloat = reinterpret_cast<const float*>(asInt + 1); // skip mNumIterations

    std::lock_guard<std::mutex> lock(s_solverMutex);
    s->gravity[0] = asFloat[0];
    s->gravity[1] = asFloat[1];
    s->gravity[2] = asFloat[2];
    if (asFloat[3] > 0.0f && asFloat[3] < 1000.0f)
        s->radius = asFloat[3];

    // The block continues: solidRestDistance, fluidRestDistance, dynamicFriction,
    // staticFriction, particleFriction, restitution, adhesion, sleepThreshold, maxSpeed,
    // shockPropagation, dissipation, damping. Nothing past +0x40 is read, since the fields
    // beyond it cannot be located with the same confidence.
    //
    // Every value is range checked, so an unexpected layout degrades to the defaults rather
    // than to nonsense physics.
    auto adopt = [](float value, float lo, float hi, float& target) {
        if (std::isfinite(value) && value >= lo && value <= hi)
            target = value;
    };
    adopt(asFloat[4], 0.0f, 1000.0f, s->solidRestDistance);
    adopt(asFloat[6], 0.0f, 1.0f, s->dynamicFriction);
    adopt(asFloat[7], 0.0f, 1.0f, s->staticFriction);
    adopt(asFloat[8], 0.0f, 1.0f, s->particleFriction);
    adopt(asFloat[9], 0.0f, 1.0f, s->restitution);
    adopt(asFloat[11], 0.0f, 1e6f, s->sleepThreshold);
    adopt(asFloat[12], 0.0f, 1e6f, s->maxSpeed);
    adopt(asFloat[14], 0.0f, 10.0f, s->dissipation);
    adopt(asFloat[15], 0.0f, 10.0f, s->damping);

    if (!s->haveParams) {
        s->haveParams = true;
        log::Write("engine gravity = (%.3f, %.3f, %.3f), radius = %.3f",
                   s->gravity[0], s->gravity[1], s->gravity[2], s->radius);
        log::Write("material params: dynFriction=%.3f statFriction=%.3f partFriction=%.3f "
                   "restitution=%.3f damping=%.3f dissipation=%.3f maxSpeed=%.1f "
                   "solidRest=%.3f sleep=%.3f",
                   s->dynamicFriction, s->staticFriction, s->particleFriction,
                   s->restitution, s->damping, s->dissipation, s->maxSpeed,
                   s->solidRestDistance, s->sleepThreshold);
        log::Write("raw params [4..15] = %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f "
                   "%.3f %.3f %.3f %.3f", asFloat[4], asFloat[5], asFloat[6], asFloat[7],
                   asFloat[8], asFloat[9], asFloat[10], asFloat[11], asFloat[12],
                   asFloat[13], asFloat[14], asFloat[15]);
    }
}

void WINAPI Shim_flexSetParticles(void* handle, const float* p, int n, int mem)
{
    Note("flexSetParticles");
    Solver* s = AsSolver(handle);
    if (!s || !p || n <= 0)
        return;

    // Confirms the argument layout: a sane count and plausible world coordinates mean the
    // signature is right.
    static int logged = 0;
    if (logged < 4) {
        ++logged;
        log::Write("SetParticles n=%d mem=%d first=(%.1f, %.1f, %.1f) invMass=%.3f",
                   n, mem, p[0], p[1], p[2], p[3]);
    }

    std::lock_guard<std::mutex> lock(s_solverMutex);
    s->count = n;
    s->particles.assign(p, p + size_t(n) * 4);
    s->velocities.resize(size_t(n) * 3, 0.0f);
}

void WINAPI Shim_flexGetParticles(void* handle, float* p, int n, int mem)
{
    Note("flexGetParticles");
    Solver* s = AsSolver(handle);
    if (!s || !p || n <= 0)
        return;
    std::lock_guard<std::mutex> lock(s_solverMutex);
    const size_t want = size_t(n) * 4;

    static int logged = 0;
    if (logged < 4) {
        ++logged;
        log::Write("GetParticles n=%d mem=%d have=%zu returning=(%.1f, %.1f, %.1f)",
                   n, mem, s->particles.size() / 4,
                   s->particles.size() >= 4 ? s->particles[0] : 0.0f,
                   s->particles.size() >= 4 ? s->particles[1] : 0.0f,
                   s->particles.size() >= 4 ? s->particles[2] : 0.0f);
    }

    if (s->particles.size() >= want)
        memcpy(p, s->particles.data(), want * sizeof(float));
}

void WINAPI Shim_flexSetVelocities(void* handle, const float* v, int n, int)
{
    Note("flexSetVelocities");
    Solver* s = AsSolver(handle);
    if (!s || !v || n <= 0)
        return;
    std::lock_guard<std::mutex> lock(s_solverMutex);
    s->velocities.assign(v, v + size_t(n) * 3);
}

void WINAPI Shim_flexGetVelocities(void* handle, float* v, int n, int)
{
    Note("flexGetVelocities");
    Solver* s = AsSolver(handle);
    if (!s || !v || n <= 0)
        return;
    std::lock_guard<std::mutex> lock(s_solverMutex);
    const size_t want = size_t(n) * 3;
    if (s->velocities.size() >= want)
        memcpy(v, s->velocities.data(), want * sizeof(float));
}

// flexUpdateSolver(solver, dt, substeps, timers)
//
// Semi-implicit Euler: velocity first, then position. Particles with invMass == 0 are pinned
// and left alone.
void WINAPI Shim_flexUpdateSolver(void* handle, float frameDt, int substeps, void*)
{
    Note("flexUpdateSolver");
    Solver* s = AsSolver(handle);
    if (!s)
        return;

    // Guard against a paused or absurd frame time.
    if (!(frameDt > 0.0f) || frameDt > 0.25f)
        frameDt = 1.0f / 60.0f;

    // The engine asks for a number of substeps and gets it, clamped only against a value that
    // would stall a frame. Substepping is what keeps a fast chunk's contact from being resolved
    // once, late, against a surface it has already travelled most of the way through.
    //
    // The frame time is taken from the clock rather than the fixed 10 ms the engine passes,
    // which it passes once per rendered frame: below 100 fps that advances less physics than a
    // second of real time contains. Nothing downstream depends on matching the engine's number,
    // since it round-trips whatever transform it is given. Clamped hard, so the gap across a
    // loading screen is not integrated as a step.
    {
        using clock = std::chrono::steady_clock;
        static clock::time_point last{};
        static clock::time_point first{};
        static double simTime = 0.0;
        static int calls = 0;
        static int reports = 0;

        const auto now = clock::now();
        if (calls == 0) {
            first = now;
            last = now;
        }
        const double measured = std::chrono::duration<double>(now - last).count();
        last = now;
        ++calls;

        if (g_tune.realTimestep && measured > 0.0005 && measured < 0.1)
            frameDt = float(measured);

        simTime += double(frameDt);
        if (calls % 240 == 0 && reports < 5) {
            ++reports;
            const double wall = std::chrono::duration<double>(now - first).count();
            log::Write("timing: %d updates, simulated %.2fs of physics in %.2fs of real time "
                       "(%.0f%% of real speed, %.1f updates/sec, step source: %s)", calls,
                       simTime, wall, wall > 1e-6 ? 100.0 * simTime / wall : 0.0,
                       wall > 1e-6 ? double(calls) / wall : 0.0,
                       g_tune.realTimestep ? "measured" : "engine");
        }
    }

    const int steps = std::max(1, std::min(substeps > 0 ? substeps : 1, 8));
    const float dt = frameDt / float(steps);

    std::lock_guard<std::mutex> lock(s_solverMutex);

    // The margin a chunk holds off a surface is this solver's own. The engine's solid rest
    // distance describes the spacing between solid particles, not between a whole chunk and a
    // surface, and its collision margin sits past the confirmed part of the parameter block.
    const float contactSkin = kContactSkin;

    // The engine's sleep threshold, used as a floor rather than the answer: it runs around
    // 1.5 cm/s, which suits a solver resolving contacts exactly, and leaves an approximate one
    // creeping and jittering before it parks. A larger value from the engine is honoured.
    const float sleepSpeed = std::max(kSleepSpeed, s->sleepThreshold);
    // Rubble that rebounds off the floor reads as light whatever its mass, so Heft scales the
    // bounce as well as the drag.
    const float heftBounce = 1.0f / std::max(g_tune.heft, 0.05f);

    // How strongly the sideways part of a contact sets a piece turning. Rolling switched off
    // leaves a chunk sliding flat rather than rolling down a slope, so the spin the tangential
    // impulse would impart is dropped entirely.
    const float contactSpin = g_tune.rolling ? g_tune.impactTorque : 0.0f;

    // This engine never calls flexSetParticles, so the solver's own particle buffer stays
    // empty. That must not short-circuit the step: debris lives in the piece list below.
    const int n = (s->particles.size() >= size_t(s->count) * 4) ? s->count : 0;
    if (n > 0 && s->velocities.size() < size_t(n) * 3)
        s->velocities.resize(size_t(n) * 3, 0.0f);

    // Drag per unit mass falls off as a chunk grows, since drag follows surface area while
    // mass follows volume. Heft divides it globally on top of that.
    const float dragBase = s->damping * g_tune.dragScale / std::max(g_tune.heft, 0.05f);
    auto dragFor = [&](float m) { return DragFactor(m, dragBase, dt, 1.0f); };
    const float drag = 1.0f - std::min(dragBase * dt, 1.0f);

    // One particle advanced under gravity.

    int moved = 0;
    int rigidsMoved = 0;
    int pairContacts = 0;
    std::atomic<int> contactCount{0};

    // Deliver shocks recorded since the last step, once each, before anything is integrated.
    // Applied as a direct change in velocity outside the substep loop, so a burst shoves the
    // rubble once regardless of frame rate or substep count.
    if (!s_impacts.empty()) {
        int shoved = 0;
        // Pieces outer, shocks inner, so everything landing on one chunk this step is summed
        // and capped together. Sustained fire into one spot puts impacts inside the pile at
        // full strength many times a second, which applied one at a time would compound.
        for (int i = 0; i < s_pieces.count; ++i) {
            float* pos = &s_pieces.translations[size_t(i) * 3];
            if (!std::isfinite(pos[0]))
                continue;

            const float m = (i < int(s_pieces.mass.size())) ? s_pieces.mass[size_t(i)] : 1.0f;
            float push[3] = {0, 0, 0};
            if (!PushFor(s_impacts, pos, m, push))
                continue;

            float* vel = &s_pieces.velocities[size_t(i) * 3];
            for (int a = 0; a < 3; ++a)
                vel[a] += push[a];
            if (i < int(s_pieces.resting.size())) {
                if (s_traceWakes && s_pieces.resting[size_t(i)])
                    ++s_wokeShock;
                s_pieces.resting[size_t(i)] = 0;
            }
            ++shoved;
        }
        static int loggedShock = 0;
        if (shoved > 0 && loggedShock < 4) {
            ++loggedShock;
            log::Write("impact shock: %zu burst(s) disturbed %d settled pieces",
                       s_impacts.size(), shoved);
        }
        s_impacts.clear();
    }

    // Each substep is a complete pass: integrate, sweep, resolve. Waking and settling are
    // re-evaluated each time, so a piece that stopped in one substep is not stepped again in
    // the next, and one shoved awake is.
    s_traceWakes = log::Verbose();
    int restingBefore = 0;
    if (s_traceWakes) {
        for (uint8_t r : s_pieces.resting)
            restingBefore += r ? 1 : 0;
        s_wokeShock = s_wokeBlast = s_wokeMover = s_wokePair = 0;
        s_wokeCollider.store(0, std::memory_order_relaxed);
    }
    s_blastWoke = 0;

    const auto stepStart = std::chrono::steady_clock::now();
    double sweepMs = 0.0, pairMs = 0.0, particleMs = 0.0, listMs = 0.0, settleMs = 0.0;
    double loopMs = 0.0;   // the substep loop as a whole, to say whether the gap is inside it
    int stepCountForLog = 0;

    const auto loopStart = std::chrono::steady_clock::now();
    for (int step = 0; step < steps; ++step) {
        // The solver's own particle buffer, which this engine never fills.
        for (int k = 0; k < n; ++k) {
            float* pos = &s->particles[size_t(k) * 4];
            float* vel = &s->velocities[size_t(k) * 3];
            if (!IsLiveParticle(pos))
                continue;
            for (int a = 0; a < 3; ++a) {
                vel[a] = (vel[a] + s->gravity[a] * g_tune.gravityScale * dt) * drag;
                pos[a] += vel[a] * dt;
            }
            ++moved;
        }

        // The engine reads debris positions out of the container's buffers, so those must
        // advance for anything to be visible.
        const auto particleStart = std::chrono::steady_clock::now();
        for (Container* c : s_containers) {
            if (!c || c->maxParticles <= 0)
                continue;
        }

        // Each piece is integrated as a point mass under the engine's gravity, then swept
        // against the world.
        //
        // Which pieces need stepping is decided up front in one serial pass. That keeps the
        // settled majority out of the parallel loop, so threads balance real work rather than
        // racing through slots they skip, and it is where a blast or a moving collider wakes
        // settled debris.
        //
        // Colliders that travelled this frame, in practice actors. There are only ever a few,
        // so a settled pile costs a couple of distance tests rather than a walk of the whole
        // collider list.
        particleMs += std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - particleStart).count();

        const auto listStart = std::chrono::steady_clock::now();
        s_movers.clear();
        for (const Collider& sh : s_colliders) {
            if (!sh.moved)
                continue;
            // Hulls count too: actors appear as convex hulls as well as capsules, and are not
            // flagged dynamic, so restricting this to primitives would miss the player.
            float span = std::max(sh.dims[0], std::max(sh.dims[1], sh.dims[2])) +
                         PrimitiveRadius(sh);
            if (sh.planeCount > 0 && sh.haveAabb) {
                float half = 0.0f;
                for (int a = 0; a < 3; ++a)
                    half = std::max(half, (sh.aabbHi[a] - sh.aabbLo[a]) * 0.5f);
                span = std::max(span, half);
            }
            if (!(span > 0.0f))
                continue;
            const float travel[3] = {sh.pos[0] - sh.prevPos[0], sh.pos[1] - sh.prevPos[1],
                                     sh.pos[2] - sh.prevPos[2]};
            Mover m;
            for (int a = 0; a < 3; ++a)
                m.pos[a] = (sh.planeCount > 0 && sh.haveAabb)
                               ? (sh.aabbLo[a] + sh.aabbHi[a]) * 0.5f : sh.pos[a];
            m.reach = span + std::sqrt(travel[0] * travel[0] + travel[1] * travel[1] +
                                       travel[2] * travel[2]) + contactSkin;
            s_movers.push_back(m);
        }

        s_stepList.clear();
        for (int i = 0; i < s_pieces.count; ++i) {
            const float* pos = &s_pieces.translations[size_t(i) * 3];
            if (!std::isfinite(pos[0]) || !std::isfinite(pos[1]) || !std::isfinite(pos[2]))
                continue;

            // A settled piece is left exactly where it is, rather than re-colliding every step
            // as gravity pulls it in and the contact pushes it back out.
            if (i < int(s_pieces.resting.size()) && s_pieces.resting[size_t(i)]) {
                const float r = (i < int(s_pieces.radii.size())) ? s_pieces.radii[size_t(i)] : 0.0f;
                bool blasted = false;
                bool disturbed = false;

                for (const ForceField& f : s_forceFields) {
                    const float d[3] = {pos[0] - f.pos[0], pos[1] - f.pos[1], pos[2] - f.pos[2]};
                    const float dist2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
                    if (dist2 <= f.radius * f.radius) {
                        blasted = true;
                        break;
                    }
                }
                disturbed = blasted;
                for (size_t k = 0; !disturbed && k < s_movers.size(); ++k) {
                    const Mover& m = s_movers[k];
                    const float d[3] = {pos[0] - m.pos[0], pos[1] - m.pos[1], pos[2] - m.pos[2]};
                    const float dist2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
                    const float reach = m.reach + r;
                    if (dist2 <= reach * reach)
                        disturbed = true;
                }

                if (!disturbed)
                    continue;
                if (s_traceWakes) {
                    if (blasted)
                        ++s_wokeBlast;
                    else
                        ++s_wokeMover;
                }
                if (blasted)
                    ++s_blastWoke;
                s_pieces.resting[size_t(i)] = 0;
            }
            s_stepList.push_back(i);
        }

        const int stepCount = int(s_stepList.size());
        stepCountForLog = stepCount;
        listMs += std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - listStart).count();

        // Where each moving piece stands before it is stepped, so its particles can be carried
        // by exactly the distance it turns out to travel. Only the moving pieces are recorded:
        // a settled heap does not move, and its particles are already where they belong.
        s_prePos.resize(size_t(stepCount) * 3);
        for (int k = 0; k < stepCount; ++k) {
            const int piece = s_stepList[size_t(k)];
            if (piece < 0 || piece >= s_pieces.count)
                continue;
            for (int a = 0; a < 3; ++a)
                s_prePos[size_t(k) * 3 + a] = s_pieces.translations[size_t(piece) * 3 + a];
        }

        s_stepping.assign(size_t(s_pieces.count), 0);
        for (int k : s_stepList)
            if (k >= 0 && k < int(s_stepping.size()))
                s_stepping[size_t(k)] = 1;

        // One piece's step. Everything it touches is either read-only for the duration of the
        // update (params, tunables, colliders and their meshes, force fields) or indexed by i
        // alone (that piece's position, velocity, spin, orientation and resting flag), so
        // pieces are independent and may run in any order or at once. The caller holds the
        // solver lock across the whole loop, which is what keeps the read-only half read-only.
        auto stepPiece = [&](int slot) {
            const int i = s_stepList[size_t(slot)];
            float* pos = &s_pieces.translations[size_t(i) * 3];
            float* vel = &s_pieces.velocities[size_t(i) * 3];

            float from[3] = {pos[0], pos[1], pos[2]};
            const float pieceDrag =
                dragFor(i < int(s_pieces.mass.size()) ? s_pieces.mass[size_t(i)] : 1.0f);
            for (int a = 0; a < 3; ++a)
                vel[a] = (vel[a] + s->gravity[a] * g_tune.gravityScale * dt) * pieceDrag;

            // Blasts push debris away from their centre, falling off with distance.
            for (const ForceField& f : s_forceFields) {
                float d[3] = {pos[0] - f.pos[0], pos[1] - f.pos[1], pos[2] - f.pos[2]};
                const float dist2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
                if (dist2 > f.radius * f.radius || dist2 < 1e-6f)
                    continue;
                const float dist = std::sqrt(dist2);
                const float falloff = f.linearFalloff ? (1.0f - dist / f.radius)
                                                      : (1.0f - dist2 / (f.radius * f.radius));
                for (int a = 0; a < 3; ++a)
                    vel[a] += (d[a] / dist) * f.strength * falloff * dt;
            }

            // Honour the engine's speed cap where it published one.
            if (s->maxSpeed > 0.0f) {
                const float sp = std::sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]);
                if (sp > s->maxSpeed) {
                    const float k = s->maxSpeed / sp;
                    vel[0] *= k; vel[1] *= k; vel[2] *= k;
                }
            }

            for (int a = 0; a < 3; ++a)
                pos[a] += vel[a] * dt;

            // Tumble while airborne.
            if (i < int(s_pieces.angular.size()) / 3)
                QuatIntegrate(&s_pieces.rotations[size_t(i) * 4],
                              &s_pieces.angular[size_t(i) * 3], dt);

            // Sweep the movement against the world and stop at the first surface crossed.
            float delta[3] = {pos[0] - from[0], pos[1] - from[1], pos[2] - from[2]};
            float dist = std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
            if (dist < 1e-5f)
                return;
            float dir[3] = {delta[0] / dist, delta[1] / dist, delta[2] / dist};

            // Swept as a sphere: the ray runs one clearance beyond where the centre travels,
            // so a surface registers while the piece is still a radius away. Testing the bare
            // centre path would need the piece to fall its whole radius before each contact.
            const float pieceRadius = (i < int(s_pieces.radii.size())) ? s_pieces.radii[size_t(i)]
                                                                      : s->radius;
            const float clearance = pieceRadius + contactSkin;
            const float sweepLen = dist + clearance;

            float bestT = sweepLen;
            float bestN[3] = {0, 0, 1};
            bool hit = false;
            const Collider* hitOn = nullptr;

            // Mesh data is local-space, so the movement is transformed into each collider's
            // frame rather than transforming thousands of vertices into the world each step.
            for (const Collider& sh : s_colliders) {
                if (!sh.mesh)
                    continue;

                float invRot[4];
                QuatConjugate(sh.rot, invRot);

                const float relFrom[3] = {from[0] - sh.pos[0], from[1] - sh.pos[1],
                                          from[2] - sh.pos[2]};
                float localFrom[3], localDir[3];
                QuatRotate(invRot, relFrom, localFrom);
                QuatRotate(invRot, dir, localDir);

                float hitT = 0.0f, localN[3];
                if (SweepMesh(*sh.mesh, localFrom, localDir, bestT, kContactSkin, hitT,
                              localN)) {
                    bestT = hitT;
                    QuatRotate(sh.rot, localN, bestN);   // the normal back into world space
                    hit = true;
                    hitOn = &sh;
                }
            }

            // Land the piece on the nearest surface the sweep crossed.
            //
            // The ray runs a clearance past where the centre travels, so the surface sits at
            // `bestT` and the centre stops a clearance short of it. Everything after that is
            // the same response the hull path gives: the silhouette resolves against the face,
            // and only a surface that could hold the piece up may park it.
            if (hit) {
                const float travel = std::max(0.0f, std::min(dist, bestT - clearance));
                for (int a = 0; a < 3; ++a)
                    pos[a] = from[a] + dir[a] * travel;

                const float surface[3] = {from[0] + dir[0] * bestT, from[1] + dir[1] * bestT,
                                          from[2] + dir[2] * bestT};

                const int shapeIdx = (i < int(s_pieces.hullIndex.size()))
                                         ? s_pieces.hullIndex[size_t(i)] : -1;
                const bool haveShape = shapeIdx >= 0 && shapeIdx < int(s_hulls.size()) &&
                                       s_hulls[size_t(shapeIdx)].numPoints > 0;

                float* q = &s_pieces.rotations[size_t(i) * 4];
                float* w = (i < int(s_pieces.angular.size()) / 3)
                               ? &s_pieces.angular[size_t(i) * 3] : nullptr;
                const float friction = std::max(0.0f, std::min(1.0f,
                    s->dynamicFriction * g_tune.frictionScale));
                const float bounce = s->restitution * g_tune.restitutionScale * heftBounce;

                // A mesh that is being carried about takes the piece with it, resolved in the
                // collider's own frame so the chunk rides the surface instead of being pushed
                // clear of it and dropping straight back.
                float meshVel[3] = {0, 0, 0};
                if (hitOn && hitOn->moved) {
                    const float frameSpan = dt * float(steps);
                    const float share = KickShare(i);
                    for (int a = 0; a < 3; ++a)
                        meshVel[a] = (hitOn->pos[a] - hitOn->prevPos[a]) / frameSpan * share;
                }
                for (int a = 0; a < 3; ++a)
                    vel[a] -= meshVel[a];

                if (haveShape && w) {
                    const fragment::Hull& shp = s_hulls[size_t(shapeIdx)];
                    const HullContact c = FindHullContact(shp, q, pos, surface, bestN);
                    if (c.valid)
                        ResolveHullContact(shp, c, pos, vel, w, bestN, bounce, friction,
                                           contactSkin, contactSpin);
                } else {
                    // No silhouette, so the piece bounces as a sphere about its centre.
                    const float vn = vel[0] * bestN[0] + vel[1] * bestN[1] + vel[2] * bestN[2];
                    if (vn < 0.0f) {
                        const float keep = std::max(0.0f, std::min(1.0f, 1.0f - friction));
                        for (int a = 0; a < 3; ++a) {
                            const float tangent = vel[a] - vn * bestN[a];
                            vel[a] = tangent * keep - vn * bestN[a] * bounce;
                        }
                    }
                }

                for (int a = 0; a < 3; ++a)
                    vel[a] += meshVel[a];

                if (hitOn && hitOn->moved && i < int(s_pieces.resting.size())) {
                    if (s_traceWakes && s_pieces.resting[size_t(i)])
                        s_wokeCollider.fetch_add(1, std::memory_order_relaxed);
                    s_pieces.resting[size_t(i)] = 0;
                }

                contactCount.fetch_add(1, std::memory_order_relaxed);

                // Only a surface facing up against gravity can hold a piece up, so brushing a
                // wall on the way past does not leave a chunk hanging on it.
                const float gLen = std::sqrt(s->gravity[0] * s->gravity[0] +
                                             s->gravity[1] * s->gravity[1] +
                                             s->gravity[2] * s->gravity[2]);
                const bool supported =
                    gLen < 1e-3f ||
                    -(bestN[0] * s->gravity[0] + bestN[1] * s->gravity[1] +
                      bestN[2] * s->gravity[2]) / gLen > 0.5f;

                const float sp = vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2];
                const float spin = w ? (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) : 0.0f;
                if (supported && (!hitOn || !hitOn->moved) && sp < sleepSpeed * sleepSpeed &&
                    spin * pieceRadius * pieceRadius < sleepSpeed * sleepSpeed &&
                    i < int(s_pieces.resting.size())) {
                    vel[0] = vel[1] = vel[2] = 0.0f;
                    if (w)
                        w[0] = w[1] = w[2] = 0.0f;
                    s_pieces.resting[size_t(i)] = 1;
                }
            }

            // Convex hulls, which most of the world's collision geometry is. A hull is a set
            // of planes, so the distance from a point to it is the largest of the per-plane
            // distances and the plane producing it is the nearest face: exact and cheap, with
            // no mesh traversal.
            for (const Collider& sh : s_colliders) {
            if (sh.planeCount <= 0)
                continue;

            // Skip hulls the piece is nowhere near, using the engine's own collider bounds.
            if (sh.haveAabb) {
                // `near` is a macro in windef.h, hence the name.
                const int bIdx = (i < int(s_pieces.hullIndex.size()))
                                     ? s_pieces.hullIndex[size_t(i)] : -1;
                const float bReach = (bIdx >= 0 && bIdx < int(s_hulls.size()) &&
                                      s_hulls[size_t(bIdx)].radius > 0.0f)
                                         ? s_hulls[size_t(bIdx)].radius : pieceRadius;
                if (!SphereTouchesAabb(sh.aabbLo, sh.aabbHi, pos, bReach + contactSkin))
                    continue;
            }

            float invRot[4];
            QuatConjugate(sh.rot, invRot);
            const float rel[3] = {pos[0] - sh.pos[0], pos[1] - sh.pos[1], pos[2] - sh.pos[2]};
            float local[3];
            QuatRotate(invRot, rel, local);

            int bestIdx = -1;
            const float best = ConvexDistance(&s_planes[size_t(sh.planeStart) * 4],
                                              sh.planeCount, local, bestIdx);
            const float* bestPlane =
                (bestIdx >= 0) ? &s_planes[(size_t(sh.planeStart) + size_t(bestIdx)) * 4]
                               : nullptr;
            // The piece touches as soon as any part of its silhouette can reach the face, so
            // this uses the hull's true bounding radius rather than the mean half extent used
            // for broad phase. The smaller figure would miss a corner-first landing until the
            // piece had sunk well in, and the correction would then launch it back out.
            const int probeIdx = (i < int(s_pieces.hullIndex.size()))
                                     ? s_pieces.hullIndex[size_t(i)] : -1;
            const float reach = (probeIdx >= 0 && probeIdx < int(s_hulls.size()) &&
                                 s_hulls[size_t(probeIdx)].radius > 0.0f)
                                    ? s_hulls[size_t(probeIdx)].radius
                                    : pieceRadius;
            if (!bestPlane || best >= reach + contactSkin)
                continue; // clear of this hull

            float normal[3];
            QuatRotate(sh.rot, bestPlane, normal);

            // The point on the hull's surface beneath the piece, which the silhouette then
            // resolves against.
            const float surface[3] = {pos[0] - normal[0] * best, pos[1] - normal[1] * best,
                                      pos[2] - normal[2] * best};

            const int shapeIdx2 = (i < int(s_pieces.hullIndex.size()))
                                      ? s_pieces.hullIndex[size_t(i)] : -1;
            const bool haveShape2 = shapeIdx2 >= 0 && shapeIdx2 < int(s_hulls.size()) &&
                                    s_hulls[size_t(shapeIdx2)].numPoints > 0;

            float* q2 = &s_pieces.rotations[size_t(i) * 4];
            float* w2 = (i < int(s_pieces.angular.size()) / 3)
                            ? &s_pieces.angular[size_t(i) * 3] : nullptr;
            const float friction2 = std::max(0.0f, std::min(1.0f,
                s->dynamicFriction * g_tune.frictionScale));
            const float bounce2 = s->restitution * g_tune.restitutionScale * heftBounce;

            // What the surface itself is doing. Resolving in the collider's frame of reference
            // is what carries the piece along with it; resolving in the world frame would push
            // the chunk clear geometrically and let it drop straight back.
            float hullVel[3] = {0, 0, 0};
            if (sh.moved) {
                const float frameSpan = dt * float(steps);
                const float share = KickShare(i);
                for (int a = 0; a < 3; ++a)
                    hullVel[a] = (sh.pos[a] - sh.prevPos[a]) / frameSpan * share;
            }
            for (int a = 0; a < 3; ++a)
                vel[a] -= hullVel[a];

            if (haveShape2 && w2) {
                const fragment::Hull& shp2 = s_hulls[size_t(shapeIdx2)];
                const HullContact c2 = FindHullContact(shp2, q2, pos, surface, normal);
                if (c2.valid)
                    ResolveHullContact(shp2, c2, pos, vel, w2, normal, bounce2, friction2,
                                        contactSkin, contactSpin);
            } else {
                // No silhouette, so the piece separates and bounces as a sphere.
                const float push = std::min(pieceRadius + contactSkin - best,
                                            std::max(pieceRadius, 1.0f) * kMaxSeparationRadii);
                for (int a = 0; a < 3; ++a)
                    pos[a] += normal[a] * push;
                const float vn2 = vel[0] * normal[0] + vel[1] * normal[1] + vel[2] * normal[2];
                if (vn2 < 0.0f) {
                    const float keep = std::max(0.0f, std::min(1.0f, 1.0f - friction2));
                    for (int a = 0; a < 3; ++a) {
                        const float tangent = vel[a] - vn2 * normal[a];
                        vel[a] = tangent * keep - vn2 * normal[a] * bounce2;
                    }
                }
            }

            for (int a = 0; a < 3; ++a)
                vel[a] += hullVel[a];

            // Being walked into is motion, so a settled chunk wakes.
            if (sh.moved && i < int(s_pieces.resting.size())) {
                if (s_traceWakes && s_pieces.resting[size_t(i)])
                    s_wokeCollider.fetch_add(1, std::memory_order_relaxed);
                s_pieces.resting[size_t(i)] = 0;
            }

            contactCount.fetch_add(1, std::memory_order_relaxed);

            // As on the mesh path, only a surface that can hold the piece up may park it, so
            // brushing a wall does not leave a chunk hanging.
            const float gLen2 = std::sqrt(s->gravity[0] * s->gravity[0] +
                                          s->gravity[1] * s->gravity[1] +
                                          s->gravity[2] * s->gravity[2]);
            const bool supported2 =
                gLen2 < 1e-3f ||
                -(normal[0] * s->gravity[0] + normal[1] * s->gravity[1] +
                  normal[2] * s->gravity[2]) / gLen2 > 0.5f;

            const float sp2 = vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2];
            const float spin2b = w2 ? (w2[0] * w2[0] + w2[1] * w2[1] + w2[2] * w2[2]) : 0.0f;
            if (supported2 && !sh.moved && sp2 < sleepSpeed * sleepSpeed &&
                spin2b * pieceRadius * pieceRadius < sleepSpeed * sleepSpeed &&
                i < int(s_pieces.resting.size())) {
                vel[0] = vel[1] = vel[2] = 0.0f;
                if (w2)
                    w2[0] = w2[1] = w2[2] = 0.0f;
                s_pieces.resting[size_t(i)] = 1;
            }
        }

        // Actors, and anything else modelled as a primitive rather than a mesh. The collider's
        // own velocity is handed to the piece, so being stepped on shoves a chunk rather than
        // merely stopping it. Kept apart from the triangle sweep, since a primitive has a
        // closed form and needs no sweep.
            for (const Collider& sh : s_colliders) {
                if (sh.type != kColliderSphere && sh.type != kColliderCapsule && sh.type != kColliderBox)
                    continue;

                float invRot[4];
                QuatConjugate(sh.rot, invRot);
                const float rel[3] = {pos[0] - sh.pos[0], pos[1] - sh.pos[1], pos[2] - sh.pos[2]};
                float local[3];
                QuatRotate(invRot, rel, local);

                float closest[3];
                bool insideBox = false;
                if (!ClosestOnPrimitive(sh, local, closest, insideBox))
                    continue;

                const float reach = PrimitiveRadius(sh) + pieceRadius + kContactSkin;
                float d[3] = {local[0] - closest[0], local[1] - closest[1], local[2] - closest[2]};
                float gap = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);

                // A piece dead-centre inside a sphere or capsule has no direction to leave by,
                // so it goes straight up rather than following float noise.
                if (gap < 1e-4f) {
                    d[0] = 0.0f; d[1] = 0.0f; d[2] = 1.0f;
                    gap = 0.0f;
                } else if (!insideBox && gap >= reach) {
                    continue;
                } else {
                    for (int a = 0; a < 3; ++a)
                        d[a] /= gap;
                }
                if (insideBox)
                    gap = -gap; // inside, so the surface is the other way

                float normal[3];
                QuatRotate(sh.rot, d, normal);

                // Lift the piece clear of the surface.
                const float push = reach - gap;
                for (int a = 0; a < 3; ++a)
                    pos[a] += normal[a] * push;

                // The collider's own velocity: a stationary one blocks a piece, a moving one
                // carries it.
                float shapeVel[3] = {0, 0, 0};
                if (sh.moved) {
                    for (int a = 0; a < 3; ++a)
                        // The previous transform is a frame old, not a substep old.
                        shapeVel[a] = (sh.pos[a] - sh.prevPos[a]) / (dt * float(steps)) *
                                      KickShare(i);
                }

                // Resolved in the collider's own frame, which is what carries a piece along
                // rather than pushing it clear and letting it drop back. The share is already
                // folded into shapeVel above, so entry and exit use the same velocity.
                {
                    const float bounce = s->restitution * g_tune.restitutionScale * heftBounce;
                    ResolveAgainstMovingSurface(vel, normal, shapeVel, 1.0f, bounce);
                }

                // Being walked into is motion, so the piece wakes rather than sitting wherever
                // the shove left it, mid-air included.
                if (sh.moved && i < int(s_pieces.resting.size())) {
                    if (s_traceWakes && s_pieces.resting[size_t(i)])
                        s_wokeCollider.fetch_add(1, std::memory_order_relaxed);
                    s_pieces.resting[size_t(i)] = 0;

                    // A matching tumble, so a kicked chunk does not slide away perfectly flat.
                    if (i < int(s_pieces.angular.size()) / 3) {
                        float* w = &s_pieces.angular[size_t(i) * 3];
                        const float inv = 1.0f / std::max(pieceRadius, 1e-3f);
                        float t[3];
                        const float sn = shapeVel[0] * normal[0] + shapeVel[1] * normal[1] +
                                         shapeVel[2] * normal[2];
                        for (int a = 0; a < 3; ++a)
                            t[a] = shapeVel[a] - sn * normal[a];
                        w[0] += (normal[1] * t[2] - normal[2] * t[1]) * inv * kRollBlend;
                        w[1] += (normal[2] * t[0] - normal[0] * t[2]) * inv * kRollBlend;
                        w[2] += (normal[0] * t[1] - normal[1] * t[0]) * inv * kRollBlend;
                    }
                }
                contactCount.fetch_add(1, std::memory_order_relaxed);
            }
        };

        // Below this the condition variable round trip costs more than the work it hands out,
        // so a light frame stays on the calling thread.
        constexpr int kMinPiecesForThreads = 48;
        if (stepCount >= kMinPiecesForThreads && threads::Workers() > 0) {
            const auto sweepStart = std::chrono::steady_clock::now();
            threads::ParallelFor(stepCount, 8, stepPiece);
            sweepMs += std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - sweepStart).count();
        } else {
            for (int k = 0; k < stepCount; ++k)
                stepPiece(k);   // timed by the caller below
        }

        rigidsMoved = stepCount;

    // Timestep, gravity and one piece's response, reported for the first few substeps. The
    // three together distinguish a wrong step from wrong gravity from drag eating the fall.
    {
        static int loggedStep = 0;
        if (loggedStep < 40 && stepCount > 0) {
            ++loggedStep;
            const int i0 = s_stepList[0];
            const float* p0 = &s_pieces.translations[size_t(i0) * 3];
            const float* v0 = &s_pieces.velocities[size_t(i0) * 3];
            const float* w0 = &s_pieces.angular[size_t(i0) * 3];
            log::Write("traj %2d: dt=%.4f z=%.2f vz=%+.1f speed=%.1f spin=%.1f rest=%d "
                       "shape=%d r=%.2f", loggedStep, dt, p0[2], v0[2],
                       std::sqrt(v0[0] * v0[0] + v0[1] * v0[1] + v0[2] * v0[2]),
                       std::sqrt(w0[0] * w0[0] + w0[1] * w0[1] + w0[2] * w0[2]),
                       (i0 < int(s_pieces.resting.size())) ? s_pieces.resting[size_t(i0)] : 0,
                       (i0 < int(s_pieces.hullIndex.size())) ? s_pieces.hullIndex[size_t(i0)]
                                                              : -1,
                       (i0 < int(s_pieces.radii.size())) ? s_pieces.radii[size_t(i0)] : 0.0f);
        }
    }

        // Piece against piece is serial: unlike the sweep it writes both sides of every pair,
        // so threads resolving overlapping pairs would fight over the same chunks.
        const auto pairStart = std::chrono::steady_clock::now();
        pairContacts = 0;
        if (g_tune.debrisVsDebris) {
            const int restCount = int(s_pieces.resting.size());

            // Cell size follows the largest piece, so a chunk can only touch something in its
            // own cell or one adjacent to it.
            float widest = s->radius;
            for (int k = 0; k < s_pieces.count && k < int(s_pieces.radii.size()); ++k)
                widest = std::max(widest, s_pieces.radii[size_t(k)]);
            const float cellSize = std::max(2.0f * widest + contactSkin, 8.0f);

            // Which pieces the pile is holding up, rebuilt each substep. Stale support would
            // let a piece sleep somewhere nothing is under it.
            s_supported.assign(size_t(s_pieces.count), 0);

            s_pieceGrid.Build(s_pieces.translations.data(), s_pieces.count, cellSize);

            // Only the pieces that are moving are walked. A settled heap holds its shape
            // without anything being done to it, and iterating the whole pool to discover that
            // costs a grid walk per settled piece per substep, which is what makes this the
            // dominant cost as a pile grows.
            //
            // Each moving piece finds every neighbour, moving or not, so a pair is reached from
            // whichever side is moving. A pair of moving pieces would be reached from both, and
            // the lower index takes it.
            for (int idx = 0; idx < stepCount; ++idx) {
                const int i = s_stepList[size_t(idx)];
                if (i < 0 || i >= s_pieces.count)
                    continue;
                float* pi = &s_pieces.translations[size_t(i) * 3];
                const float ri = (i < int(s_pieces.radii.size())) ? s_pieces.radii[size_t(i)]
                                                                  : s->radius;
                if (!std::isfinite(pi[0]))
                    continue;

                s_pieceGrid.ForEachCandidate(i, pi, [&](int j) {
                    const bool jStepping = j < int(s_stepping.size()) && s_stepping[size_t(j)];
                    if (jStepping && j < i)
                        return;   // the other side of this pair will take it

                    float* pj = &s_pieces.translations[size_t(j) * 3];
                    const float rj = (j < int(s_pieces.radii.size()))
                                         ? s_pieces.radii[size_t(j)] : s->radius;
                    if (!std::isfinite(pj[0]))
                        return;

                    const float d[3] = {pi[0] - pj[0], pi[1] - pj[1], pi[2] - pj[2]};
                    const float dist2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];

                    // Broad phase on the bounding radii, which can only over-estimate.
                    const int si = (i < int(s_pieces.hullIndex.size()))
                                       ? s_pieces.hullIndex[size_t(i)] : -1;
                    const int sj = (j < int(s_pieces.hullIndex.size()))
                                       ? s_pieces.hullIndex[size_t(j)] : -1;
                    const bool hasI = si >= 0 && si < int(s_hulls.size()) &&
                                      s_hulls[size_t(si)].numPoints > 0;
                    const bool hasJ = sj >= 0 && sj < int(s_hulls.size()) &&
                                      s_hulls[size_t(sj)].numPoints > 0;
                    const float bri = hasI ? s_hulls[size_t(si)].radius : ri;
                    const float brj = hasJ ? s_hulls[size_t(sj)].radius : rj;
                    const float bound = bri + brj + contactSkin;
                    if (dist2 >= bound * bound || dist2 < 1e-8f)
                        return;

                    const float dist = std::sqrt(dist2);
                    const float sep[3] = {d[0] / dist, d[1] / dist, d[2] / dist};

                    // Narrow phase along the separation axis, using how far each chunk reaches
                    // that way rather than an averaged radius standing in for both. Collapsing
                    // each to a mean sphere lets chunks sink into one another and flattens the
                    // pile.
                    const float minusSep[3] = {-sep[0], -sep[1], -sep[2]};
                    const float extI = hasI ? SupportExtent(s_hulls[size_t(si)],
                                                            &s_pieces.rotations[size_t(i) * 4],
                                                            minusSep) : ri;
                    const float extJ = hasJ ? SupportExtent(s_hulls[size_t(sj)],
                                                            &s_pieces.rotations[size_t(j) * 4],
                                                            sep) : rj;

                    pairs::Body ba{pi, &s_pieces.velocities[size_t(i) * 3],
                                   (i < int(s_pieces.angular.size()) / 3)
                                       ? &s_pieces.angular[size_t(i) * 3] : nullptr,
                                   (i < int(s_pieces.mass.size())) ? s_pieces.mass[size_t(i)]
                                                                   : 1.0f,
                                   extI,
                                   i < restCount && s_pieces.resting[size_t(i)] != 0};
                    pairs::Body bb{pj, &s_pieces.velocities[size_t(j) * 3],
                                   (j < int(s_pieces.angular.size()) / 3)
                                       ? &s_pieces.angular[size_t(j) * 3] : nullptr,
                                   (j < int(s_pieces.mass.size())) ? s_pieces.mass[size_t(j)]
                                                                   : 1.0f,
                                   extJ,
                                   j < restCount && s_pieces.resting[size_t(j)] != 0};

                    pairs::Settings ps;
                    ps.contactSkin = contactSkin;
                    ps.restitution = s->restitution * heftBounce;
                    ps.mu = std::max(s->particleFriction * g_tune.frictionScale, 0.05f) *
                            g_tune.settleRate;
                    ps.sleepSpeed = sleepSpeed;
                    ps.dt = dt;

                    pairs::Result pr;
                    if (!pairs::Resolve(ba, bb, sep, dist, s->gravity, ps, pr))
                        return;
                    ++pairContacts;

                    if (pr.supported == pairs::kSupportedA && i < int(s_supported.size()))
                        s_supported[size_t(i)] = 1;
                    else if (pr.supported == pairs::kSupportedB && j < int(s_supported.size()))
                        s_supported[size_t(j)] = 1;

                    if (s_traceWakes) {
                        if (pr.wakeA && i < restCount && s_pieces.resting[size_t(i)])
                            ++s_wokePair;
                        if (pr.wakeB && j < restCount && s_pieces.resting[size_t(j)])
                            ++s_wokePair;
                    }
                    if (pr.wakeA && i < restCount)
                        s_pieces.resting[size_t(i)] = 0;
                    if (pr.wakeB && j < restCount)
                        s_pieces.resting[size_t(j)] = 0;
                });      // each neighbour offered by the grid
            }            // each piece

            pairMs += std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - pairStart).count();

            const auto settleStart = std::chrono::steady_clock::now();

            // A chunk held up by the pile can now settle on it. Every other sleep test lives
            // inside a world contact, which a piece buried in a heap never reaches.
            for (int i = 0; i < s_pieces.count && i < int(s_supported.size()); ++i) {
                if (!s_supported[size_t(i)] || s_pieces.resting[size_t(i)])
                    continue;
                float* vel = &s_pieces.velocities[size_t(i) * 3];
                float* w = (i < int(s_pieces.angular.size()) / 3)
                               ? &s_pieces.angular[size_t(i) * 3] : nullptr;
                const float r = (i < int(s_pieces.radii.size())) ? s_pieces.radii[size_t(i)]
                                                                 : s->radius;
                const float sp = vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2];
                const float spin = w ? (w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) : 0.0f;
                if (sp < sleepSpeed * sleepSpeed && spin * r * r < sleepSpeed * sleepSpeed) {
                    vel[0] = vel[1] = vel[2] = 0.0f;
                    if (w)
                        w[0] = w[1] = w[2] = 0.0f;
                    s_pieces.resting[size_t(i)] = 1;
                }
            }
            settleMs += std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - settleStart).count();
        }               // debris against debris

        // Carry each moving piece's particles along with it.
        //
        // These buffers belong to the plugin but the engine reads them, and what it reads has
        // to describe the same debris the transforms do. A particle advanced under its own
        // gravity has nothing to stop it against the world, so it keeps falling after the piece
        // it belongs to has landed, and the two end up thousands of units apart.
        //
        // Last in the substep, so it sees where every piece finished and what it finished
        // doing. Pieces are still being moved by their neighbours after the world sweep, and a
        // chunk held up by the pile is put to sleep later still: carrying the particles any
        // earlier leaves them holding a position and a velocity the piece no longer has.
        {
            const auto pStart = std::chrono::steady_clock::now();
            for (Container* c : s_containers) {
                if (!c || c->maxParticles <= 0)
                    continue;
                for (int k = 0; k < stepCount; ++k) {
                    const int piece = s_stepList[size_t(k)];
                    if (piece < 0 || piece + 1 >= int(s_pieceSlotStart.size()) ||
                        piece >= s_pieces.count)
                        continue;

                    const float* now = &s_pieces.translations[size_t(piece) * 3];
                    const float* was = &s_prePos[size_t(k) * 3];
                    const float delta[3] = {now[0] - was[0], now[1] - was[1], now[2] - was[2]};
                    const float* pieceVel = &s_pieces.velocities[size_t(piece) * 3];

                    for (int e = s_pieceSlotStart[size_t(piece)];
                         e < s_pieceSlotStart[size_t(piece) + 1]; ++e) {
                        const int slot = s_pieceSlots[size_t(e)];
                        if (slot < 0 || slot >= c->maxParticles)
                            continue;
                        float* pp = &c->particles[size_t(slot) * 4];
                        if (!IsLiveParticle(pp))
                            continue;
                        CarryParticle(delta, pieceVel, pp, &c->velocities[size_t(slot) * 3]);
                        ++moved;
                    }
                }
            }
            particleMs += std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - pStart).count();
        }

    } // substep
    loopMs = std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - loopStart).count();

    const int contacts = contactCount.load(std::memory_order_relaxed);

    // Where a burst of waking came from. Six mechanisms can clear a resting flag and they are
    // indistinguishable from outside: the heap stirs. Reported whenever a real share of a
    // settled pile wakes at once.
    // What a blast reached, said once per blast rather than once per update. An explosion's
    // field lives for as long as the explosion does, so the first update that finds settled
    // debris inside it is the interesting one and the rest are the same news repeated.
    if (!s_forceFields.empty()) {
        static int loggedBlast = 0;
        static bool reachedLast = false;
        const bool reached = s_blastWoke > 0;
        if (reached && !reachedLast && loggedBlast < 8) {
            ++loggedBlast;
            log::Write("blast: %zu force field(s), radius %.0f strength %.0f, disturbed %d "
                       "settled piece(s)", s_forceFields.size(), s_forceFields[0].radius,
                       s_forceFields[0].strength, s_blastWoke);
        }
        reachedLast = reached;
    }

    if (s_traceWakes) {
        const int woke = s_wokeShock + s_wokeBlast + s_wokeMover + s_wokePair + s_wokeFresh +
                         s_wokeMigrated + s_wokeCollider.load(std::memory_order_relaxed);
        static int logged = 0;
        if (restingBefore >= 16 && woke * 20 >= restingBefore && logged < 60) {
            ++logged;
            log::Write("WAKES: %d of %d settled woke, by shock=%d blast=%d mover=%d "
                       "collider=%d pair=%d respawn=%d migration=%d",
                       woke, restingBefore, s_wokeShock, s_wokeBlast, s_wokeMover,
                       s_wokeCollider.load(std::memory_order_relaxed), s_wokePair, s_wokeFresh,
                       s_wokeMigrated);
        }
        s_wokeFresh = 0;
        s_wokeMigrated = 0;
    }

    // The cost curve, reported as the pile grows rather than every frame. Each line is the
    // mean over the samples since the last one, so a single unlucky frame does not decide it.
    {
        const double totalMs = std::chrono::duration<double, std::milli>(
                                   std::chrono::steady_clock::now() - stepStart).count();
        s_cost.sweepMs += sweepMs;
        s_cost.pairMs += pairMs;
        s_cost.particleMs += particleMs;
        s_cost.listMs += listMs;
        s_cost.settleMs += settleMs;
        s_cost.loopMs += loopMs;
        s_cost.totalMs += totalMs;
        s_cost.steps += steps;
        ++s_costSamples;

        // A new band of debris, or a long spell at the same one: either is worth a line.
        const int band = (s_pieces.count / 250) * 250;
        if (s_traceWakes && s_costSamples >= 60 &&
            (band > s_costPeak || s_costSamples >= 600)) {
            if (band > s_costPeak)
                s_costPeak = band;
            const double samples = double(s_costSamples);
            const double accounted = (s_cost.sweepMs + s_cost.pairMs + s_cost.particleMs +
                                      s_cost.listMs + s_cost.settleMs) / samples;
            log::Write("cost: %d pieces (%d stepped), %.3f ms/frame = %.3f sweep + %.3f pairs "
                       "+ %.3f particles + %.3f list + %.3f settle, %.3f in-loop gap, "
                       "%.3f outside the loop, %.1f substeps, over %d frames",
                       s_pieces.count, stepCountForLog, s_cost.totalMs / samples,
                       s_cost.sweepMs / samples, s_cost.pairMs / samples,
                       s_cost.particleMs / samples, s_cost.listMs / samples,
                       s_cost.settleMs / samples, s_cost.loopMs / samples - accounted,
                       (s_cost.totalMs - s_cost.loopMs) / samples,
                       double(s_cost.steps) / samples, s_costSamples);
            s_cost = StepCost();
            s_costSamples = 0;
        }
    }

    // Reported on each new high water mark rather than every frame, so the log shows what the
    // budget is actually producing.
    {
        static int peak = 0;
        static int reports = 0;
        if (s_pieces.count > peak + peak / 8 + 8 && reports < 16) {
            peak = s_pieces.count;
            ++reports;
            int resting = 0;
            for (int k = 0; k < s_pieces.count && k < int(s_pieces.resting.size()); ++k)
                if (s_pieces.resting[size_t(k)])
                    ++resting;
            int liveParticles = 0, capacity = 0;
            for (const Container* c : s_containers) {
                if (!c)
                    continue;
                capacity += c->maxParticles;
                for (int k = 0; k < c->liveHigh; ++k)
                    if (IsLiveParticle(&c->particles[size_t(k) * 4]))
                        ++liveParticles;
            }
            log::Write("peak debris: %d pieces (%d settled, %d moving) of a %d cap; engine "
                       "particle container %d live of %d",
                       s_pieces.count, resting, s_pieces.count - resting, g_tune.maxPieces,
                       liveParticles, capacity);
        }
    }

    // How a piece got far above where it appeared.
    //
    // A chunk climbing under its own velocity covers roughly vel*dt in a step; one that is being
    // displaced covers the whole distance in a single step. Reporting the step on which a piece
    // crosses the line, with the distance it moved on that step, separates the two.
    if (s_pieces.count > 0 && log::Verbose()) {
        const size_t tracked = size_t(s_pieces.count);
        if (s_riseFrom.size() < tracked) {
            s_riseFrom.resize(tracked, 1e30f);
            s_risePrev.resize(tracked, 0.0f);
            s_riseLogged.resize(tracked, 0);
        }

        for (size_t i = 0; i < tracked && s_riseReported < kRiseReports; ++i) {
            const float z = s_pieces.translations[i * 3 + 2];
            if (s_riseFrom[i] > 1e29f) {
                s_riseFrom[i] = z;
                s_risePrev[i] = z;
                continue;
            }

            const float rise = z - s_riseFrom[i];
            if (rise > kRiseReport && !s_riseLogged[i]) {
                s_riseLogged[i] = 1;
                ++s_riseReported;
                const float* v = &s_pieces.velocities[i * 3];
                log::Write("RISE slot %zu: %.0f above where it appeared (z %.0f -> %.0f), "
                           "%.0f of it on this step, vel (%.1f, %.1f, %.1f), speed %.0f, "
                           "resting=%d, pieces=%d",
                           i, rise, s_riseFrom[i], z, z - s_risePrev[i], v[0], v[1], v[2],
                           std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]),
                           (i < s_pieces.resting.size()) ? s_pieces.resting[i] : -1,
                           s_pieces.count);
            }
            s_risePrev[i] = z;
        }

        // The mirror case: a piece that has dropped a long way below where it appeared has gone
        // through something it should have landed on. Whether anything is still beneath it says
        // which half is at fault, since the sweep can only stop a piece against geometry the
        // collider set actually holds at the time it falls.
        for (size_t i = 0; i < tracked && s_sankReported < kRiseReports; ++i) {
            if (s_riseFrom[i] > 1e29f || s_riseLogged[i])
                continue;
            const float* p = &s_pieces.translations[i * 3];
            if (s_riseFrom[i] - p[2] < kSankReport)
                continue;

            s_riseLogged[i] = 1;
            ++s_sankReported;

            const float down[3] = {0.0f, 0.0f, -1.0f};
            int meshes = 0, hits = 0;
            float nearest = 1e30f;
            for (const Collider& sh : s_colliders) {
                if (!sh.mesh)
                    continue;
                ++meshes;
                float invRot[4], localFrom[3], localDir[3];
                QuatConjugate(sh.rot, invRot);
                const float rel[3] = {p[0] - sh.pos[0], p[1] - sh.pos[1], p[2] - sh.pos[2]};
                QuatRotate(invRot, rel, localFrom);
                QuatRotate(invRot, down, localDir);
                float hitT = 0.0f;
                if (SweepMesh(*sh.mesh, localFrom, localDir, 8000.0f, 1.0f, hitT, nullptr)) {
                    ++hits;
                    nearest = std::min(nearest, hitT);
                }
            }

            log::Write("SANK slot %zu: %.0f below where it appeared (z %.0f -> %.0f), "
                       "%d of %d meshes beneath it (nearest %.0f), %zu colliders, "
                       "resting=%d, speed %.0f",
                       i, s_riseFrom[i] - p[2], s_riseFrom[i], p[2], hits, meshes,
                       nearest > 1e29f ? -1.0f : nearest, s_colliders.size(),
                       (i < s_pieces.resting.size()) ? s_pieces.resting[i] : -1,
                       std::sqrt(s_pieces.velocities[i * 3 + 0] * s_pieces.velocities[i * 3 + 0] +
                                 s_pieces.velocities[i * 3 + 1] * s_pieces.velocities[i * 3 + 1] +
                                 s_pieces.velocities[i * 3 + 2] * s_pieces.velocities[i * 3 + 2]));
        }
    }

    static int logged = 0;
    if ((moved > 0 || rigidsMoved > 0) && logged < 8) {
        ++logged;
        float rmin = 1e9f, rmax = 0.0f;
        for (int i = 0; i < s_pieces.count && i < int(s_pieces.radii.size()); ++i) {
            rmin = std::min(rmin, s_pieces.radii[size_t(i)]);
            rmax = std::max(rmax, s_pieces.radii[size_t(i)]);
        }
        int shaped = 0;
        for (int i = 0; i < s_pieces.count && i < int(s_pieces.hullIndex.size()); ++i)
            if (s_pieces.hullIndex[size_t(i)] >= 0)

                ++shaped;
        log::Write("advanced %d particles, %d of %d rigid pieces (%d with a real silhouette), "
                   "%d world contacts, %d piece-piece contacts over %d substeps "
                   "(piece radii %.1f..%.1f, %d threads)", moved, rigidsMoved, s_pieces.count,
                   shaped, contacts, pairContacts, steps, rmin, rmax, threads::Workers() + 1);
    }
}
void  WINAPI Shim_flexSetPhases(...)       { Note("flexSetPhases");       }

// flexSetActive(solver, indices, n, memory)
//
// The engine publishes which particle slots are live; the same number goes back out through
// flexGetActiveCount.
void WINAPI Shim_flexSetActive(void* handle, const int* indices, int n, int)
{
    Note("flexSetActive");
    (void)handle;
    (void)indices;
    s_activeCount.store((n > 0 && n < (1 << 22)) ? n : 0, std::memory_order_relaxed);
}

// Reports the count the engine itself last published. Reporting zero would tell the engine its
// debris system is empty and invite it to retire the pieces being simulated.
int WINAPI Shim_flexGetActiveCount(void* handle)
{
    Note("flexGetActiveCount");
    (void)handle;
    const int n = s_activeCount.load(std::memory_order_relaxed);

    static int logged = 0;
    if (n > 0 && logged < 3) {
        ++logged;
        log::Write("GetActiveCount reporting %d active particles", n);
    }
    return n;
}
// flexSetRigids(solver, offsets, indices, restPositions, restNormals, stiffness,
//               rotations, translations, numRigids, memory)
//
// The engine supplies each piece's starting orientation and position, which are taken as the
// initial state and integrated from.
void WINAPI Shim_flexSetRigids(void* handle, const int* offsets, const int* indices,
                               const float* restPositions, const float*, const float*,
                               const float* rotations, const float* translations,
                               int numRigids, int)
{
    Note("flexSetRigids");
    Solver* solver = AsSolver(handle);
    if (numRigids > (1 << 20))
        return;

    if (numRigids <= 0) {
        // The engine has retired every piece, so the pool is cleared rather than left alive to
        // be integrated and reported for the rest of the session.
        std::lock_guard<std::mutex> lock(s_solverMutex);
        s_pieces = Pieces();
        return;
    }

    // Safety valve: simulating only the first N of an unexpectedly large batch degrades
    // gracefully rather than stalling a frame.
    if (numRigids > g_tune.maxPieces) {
        static bool warned = false;
        if (!warned) {
            warned = true;
            log::Write("engine asked for %d pieces, simulating the first %d (MaxPieces)",
                       numRigids, g_tune.maxPieces);
        }
        numRigids = g_tune.maxPieces;
    }

    std::lock_guard<std::mutex> lock(s_solverMutex);

    const float particleRadius = solver ? solver->radius : 3.5f;
    const float pieceRadius = particleRadius * g_tune.pieceRadiusScale;

    const int previous = s_pieces.count;
    s_pieces.count = numRigids;
    s_pieces.rotations.resize(size_t(numRigids) * 4, 0.0f);
    s_pieces.translations.resize(size_t(numRigids) * 3, 0.0f);
    s_pieces.velocities.resize(size_t(numRigids) * 3, 0.0f);
    s_pieces.angular.resize(size_t(numRigids) * 3, 0.0f);
    s_pieces.radii.resize(size_t(numRigids), pieceRadius);
    s_pieces.gyration.resize(size_t(numRigids), pieceRadius);
    s_pieces.hullIndex.resize(size_t(numRigids), -1);
    s_pieces.mass.resize(size_t(numRigids), 1.0f);
    s_pieces.resting.resize(size_t(numRigids), 0);
    s_pieces.lastReported.resize(size_t(numRigids) * 3, 0.0f);
    s_pieces.reported.resize(size_t(numRigids), 0);

    // A newly grown slot has an all-zero quaternion, which is not identity: integrating a spawn
    // spin from it normalises to a half turn about the spin axis and draws the piece flipped.
    for (int i = previous; i < numRigids; ++i) {
        float* q = &s_pieces.rotations[size_t(i) * 4];
        q[0] = q[1] = q[2] = 0.0f;
        q[3] = 1.0f;
    }

    // One-time record of which inputs the engine actually supplies.
    static bool describedInputs = false;
    if (!describedInputs) {
        describedInputs = true;
        log::Write("SetRigids inputs: offsets=%s restPositions=%s rotations=%s "
                   "translations=%s, particle radius %.2f",
                   offsets ? "yes" : "null", restPositions ? "yes" : "null",
                   rotations ? "yes" : "null", translations ? "yes" : "null", particleRadius);
        // An all-zero offsets array with no rest positions means the pieces carry a transform
        // and nothing else, leaving no per-piece geometry to size colliders from.
        if (offsets && offsets[numRigids] == 0)
            log::Write("engine supplies no particles per rigid, pieces collide at the "
                       "shared radius of %.2f units", particleRadius);
    }

    // A piece is seeded from the engine's transform when new and left alone once it is under
    // simulation, since overwriting every frame would snap it back to its spawn point.
    //
    // New is not simply an index past the previous count: the engine recycles a fixed pool, so
    // a fresh burst usually arrives in the same indices as the last. The test is that the
    // engine hands back the positions it was given, so a slot arriving with a position other
    // than the one last reported for it has been re-seeded. Decided once, up front, so position
    // and orientation are adopted for the same set of slots.
    //
    slots::Classification classified;
    slots::Classify(translations, numRigids, s_pieces.lastReported, s_pieces.reported, previous,
                    kRespawnDistance, classified);

    // Why slots are being called new.
    //
    // A steady burst grows the pool by a few dozen pieces a call, so anything much beyond that
    // is the test misreading pieces the engine merely handed back. The three cases are told
    // apart here: a slot past the previous count is new by definition, a slot never reported has
    // nothing to compare against, and a slot that was reported and still differs is the one that
    // matters, so its displacement is what gets measured.
    if (log::Verbose()) {
        static int classifyLogs = 0;
        int fresh = 0, migrated = 0, beyondPrevious = 0, neverReported = 0, differs = 0;
        float worst = 0.0f;
        double totalGap = 0.0;
        for (int i = 0; i < numRigids; ++i) {
            if (classified.change[size_t(i)] == slots::kMigrated)
                ++migrated;
            if (classified.change[size_t(i)] != slots::kFresh)
                continue;
            ++fresh;
            if (i >= previous) {
                ++beyondPrevious;
            } else if (i >= int(s_pieces.reported.size()) || !s_pieces.reported[size_t(i)]) {
                ++neverReported;
            } else {
                ++differs;
                const float* was = &s_pieces.lastReported[size_t(i) * 3];
                const float* now = &translations[size_t(i) * 3];
                const float d[3] = {now[0] - was[0], now[1] - was[1], now[2] - was[2]};
                const float gap = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
                totalGap += gap;
                worst = std::max(worst, gap);
            }
        }

        if (fresh > 0 && classifyLogs < 12) {
            ++classifyLogs;
            log::Write("CLASSIFY: %d of %d slots new (%d past the previous %d, %d never "
                       "reported, %d reported but changed, mean gap %.1f, worst %.1f), "
                       "%d migrated, %d sets since the engine last read back",
                       fresh, numRigids, beyondPrevious, previous, neverReported, differs,
                       differs ? float(totalGap / differs) : 0.0f, worst, migrated,
                       s_setsSinceRead);
        }
    }

    s_fresh.assign(size_t(numRigids), 0);
    s_wasResting.assign(size_t(numRigids), 0);
    s_migrated.clear();
    s_migrateTo.clear();
    int respawned = 0;
    int moved = 0;

    for (int i = 0; i < numRigids; ++i) {
        const uint8_t change = classified.change[size_t(i)];
        const int src = classified.source[size_t(i)];

        if (change == slots::kMigrated && src >= 0 &&
            src < int(s_pieces.velocities.size()) / 3) {
            PieceState st{};
            for (int a = 0; a < 3; ++a) {
                st.vel[a] = s_pieces.velocities[size_t(src) * 3 + size_t(a)];
                st.ang[a] = s_pieces.angular[size_t(src) * 3 + size_t(a)];
            }
            for (int a = 0; a < 4; ++a)
                st.rot[a] = s_pieces.rotations[size_t(src) * 4 + size_t(a)];
            st.radius = s_pieces.radii[size_t(src)];
            st.gyration = s_pieces.gyration[size_t(src)];
            st.shape = s_pieces.hullIndex[size_t(src)];
            st.resting = s_pieces.resting[size_t(src)];
            if (s_traceWakes && !st.resting)
                ++s_wokeMigrated;
            s_migrated.push_back(st);
            s_migrateTo.push_back(i);
            ++moved;
            continue;
        }

        if (change == slots::kFresh) {
            s_fresh[size_t(i)] = 1;
            if (s_traceWakes && i < int(s_pieces.resting.size()) &&
                s_pieces.resting[size_t(i)])
                ++s_wokeFresh;
            if (i < previous)
                ++respawned;
            if (i < int(s_pieces.resting.size()) && s_pieces.resting[size_t(i)])
                s_wasResting[size_t(i)] = 1;
        }
    }

    // Applied after every source has been read, so a pool shuffling several pieces at once
    // cannot have one migration overwrite another's source.
    for (size_t n = 0; n < s_migrateTo.size(); ++n) {
        const int dst = s_migrateTo[n];
        const PieceState& st = s_migrated[n];
        for (int a = 0; a < 3; ++a) {
            s_pieces.translations[size_t(dst) * 3 + size_t(a)] =
                translations[size_t(dst) * 3 + size_t(a)];
            s_pieces.velocities[size_t(dst) * 3 + size_t(a)] = st.vel[a];
            s_pieces.angular[size_t(dst) * 3 + size_t(a)] = st.ang[a];
        }
        for (int a = 0; a < 4; ++a)
            s_pieces.rotations[size_t(dst) * 4 + size_t(a)] = st.rot[a];
        s_pieces.radii[size_t(dst)] = st.radius;
        s_pieces.gyration[size_t(dst)] = st.gyration;
        s_pieces.hullIndex[size_t(dst)] = st.shape;
        s_pieces.resting[size_t(dst)] = st.resting;
    }

    ++s_setsSinceRead;

    // A piece that had already settled being called new is what would relaunch a finished pile,
    // so it is counted separately from ordinary respawns, with the distance it jumped.
    int wokenFromRest = 0;
    float worstJump = 0.0f;
    float sampleOld[3] = {0, 0, 0}, sampleNew[3] = {0, 0, 0};
    if (translations) {
        for (int i = 0; i < numRigids; ++i) {
            if (!s_fresh[size_t(i)] || i >= previous)
                continue;
            if (i >= int(s_pieces.resting.size()) || !s_pieces.resting[size_t(i)])
                continue;
            ++wokenFromRest;
            const float* mine = &s_pieces.lastReported[size_t(i) * 3];
            const float* theirs = &translations[size_t(i) * 3];
            const float d[3] = {theirs[0] - mine[0], theirs[1] - mine[1], theirs[2] - mine[2]};
            const float jump = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            if (jump > worstJump) {
                worstJump = jump;
                for (int a = 0; a < 3; ++a) {
                    sampleOld[a] = mine[a];
                    sampleNew[a] = theirs[a];
                }
            }
        }
    }

    static int loggedMove = 0;
    static uint32_t lastWokenEpoch = 0;
    if (wokenFromRest > 0 && loggedMove < 24 && s_spawnEpoch - lastWokenEpoch > 30) {
        ++loggedMove;
        lastWokenEpoch = s_spawnEpoch;
        log::Write("WAKE: %d settled pieces called new (of %d), worst jump %.1f units, "
                   "reported (%.1f, %.1f, %.1f) -> engine gave (%.1f, %.1f, %.1f); "
                   "%d shuffles, %d respawns, %d sets since the engine last read back",
                   wokenFromRest, numRigids, worstJump, sampleOld[0], sampleOld[1],
                   sampleOld[2], sampleNew[0], sampleNew[1], sampleNew[2], moved, respawned,
                   s_setsSinceRead);
    }

    // Orientation is adopted on the same terms as position. Copying it unconditionally would
    // discard the tumble integrated the frame before: for a piece in flight the incoming
    // rotation is either the value already reported, making the copy pointless, or the spawn
    // pose, which would reset the piece so it never appeared to rotate.
    if (rotations) {
        for (int i = 0; i < numRigids; ++i) {
            if (!s_fresh[size_t(i)])
                continue;
            float* q = &s_pieces.rotations[size_t(i) * 4];
            const float* src = &rotations[size_t(i) * 4];
            const float len2 = src[0] * src[0] + src[1] * src[1] + src[2] * src[2] +
                               src[3] * src[3];
            if (std::isfinite(len2) && len2 > 1e-8f) {
                const float inv = 1.0f / std::sqrt(len2);
                for (int a = 0; a < 4; ++a)
                    q[a] = src[a] * inv;
            } else {
                q[0] = q[1] = q[2] = 0.0f; // degenerate quaternion; spawn upright instead
                q[3] = 1.0f;
            }
        }
    }

    if (translations) {
        // Invert the engine's per-piece runs, so the particle buffers can follow the bodies.
        if (offsets && indices && !s_containers.empty() && s_containers.front()) {
            BuildPieceSlots(offsets, indices, numRigids, s_containers.front()->maxParticles,
                            s_pieceSlotStart, s_pieceSlots);

            // The highest slot the engine has assigned, taken from the assignment itself. It
            // was previously found by scanning down from the container's capacity until a live
            // particle turned up, which walked the whole empty tail every substep whenever
            // nothing was live up there: tens of thousands of reads for a fixed answer.
            int high = 0;
            for (int slot : s_pieceSlots)
                if (slot + 1 > high)
                    high = slot + 1;
            for (Container* c : s_containers)
                if (c && high > c->liveHigh)
                    c->liveHigh = std::min(high, c->maxParticles);
        }

        s_freshList.clear();
        s_seededFromEngine.assign(size_t(numRigids), 0);
        ++s_spawnEpoch;
        int matchedSpawns = 0;
        for (int i = 0; i < numRigids; ++i) {
            if (!s_fresh[size_t(i)])
                continue;

            memcpy(&s_pieces.translations[size_t(i) * 3], &translations[size_t(i) * 3],
                   3 * sizeof(float));
            s_pieces.radii[size_t(i)] = pieceRadius;
            s_pieces.gyration[size_t(i)] = pieceRadius;
            s_pieces.hullIndex[size_t(i)] = -1;
            s_pieces.mass[size_t(i)] = 1.0f;

            // Claim the spawn this slot came from, where one was recorded. Nothing states
            // which slot an instance lands in, so the two are matched by position; spawns in
            // one frame are metres apart, so the match is unambiguous.
            {
                const float* here = &s_pieces.translations[size_t(i) * 3];
                int bestSpawn = -1, bestLayout = s_xformPosLayout;
                float bestDist2 = kSpawnMatchDistance * kSpawnMatchDistance;

                for (size_t sp = 0; sp < s_pendingSpawns.size(); ++sp) {
                    if (s_pendingSpawns[sp].used)
                        continue;
                    // Once the layout is settled only that reading is tried; until then every
                    // candidate is tested and whichever lands is adopted.
                    const int first = (s_xformPosLayout >= 0) ? s_xformPosLayout : 0;
                    const int last = (s_xformPosLayout >= 0) ? s_xformPosLayout
                                                             : kXformLayouts - 1;
                    for (int layout = first; layout <= last; ++layout) {
                        float p[3];
                        CandidatePos(s_pendingSpawns[sp], layout, p);
                        const float d[3] = {p[0] - here[0], p[1] - here[1], p[2] - here[2]};
                        const float dist2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
                        if (dist2 < bestDist2) {
                            bestDist2 = dist2;
                            bestSpawn = int(sp);
                            bestLayout = layout;
                        }
                    }
                }

                if (bestSpawn >= 0) {
                    PendingSpawn& sp = s_pendingSpawns[size_t(bestSpawn)];
                    sp.used = true;
                    ++matchedSpawns;

                    if (s_xformPosLayout < 0) {
                        s_xformPosLayout = bestLayout;
                        log::Write("spawn transforms carry their position at layout %d "
                                   "(0 = 4x4 column-major, 1 = position first, "
                                   "2 = 4x4 row-major), engine spawn velocities are now "
                                   "being used directly", bestLayout);
                    }

                    for (int a = 0; a < 3; ++a)
                        s_pieces.velocities[size_t(i) * 3 + a] = sp.vel[a];
                    if (sp.radius > 0.0f)
                        s_pieces.radii[size_t(i)] = sp.radius * g_tune.pieceRadiusScale;
                    if (sp.gyration > 0.0f)
                        s_pieces.gyration[size_t(i)] = sp.gyration * g_tune.pieceRadiusScale;
                    s_pieces.hullIndex[size_t(i)] = sp.hullIndex;
                    s_seededFromEngine[size_t(i)] = 1;
                }
            }

            // The engine's rest positions for this piece, where supplied: its own particle
            // cloud, so silhouette and inertia are exact and the fallback below is skipped.
            if (offsets && restPositions && s_pieces.hullIndex[size_t(i)] < 0) {
                const int begin = offsets[i];
                const int end = offsets[i + 1];
                const int n = end - begin;
                if (n > 0 && n < 4096 && begin >= 0)
                    s_pieces.mass[size_t(i)] = float(n);
                if (n > 0 && n < 4096 && begin >= 0) {
                    // Settle the packing once, by which reading gives a chunk-sized cloud.
                    if (s_restStride == 0) {
                        for (int stride : {3, 4}) {
                            fragment::Hull probe;
                            if (fragment::BuildFromPoints(
                                    restPositions + size_t(begin) * size_t(stride), n, stride,
                                    particleRadius * 0.5f, probe) &&
                                probe.radius > 0.01f && probe.radius < 500.0f) {
                                s_restStride = stride;
                                break;
                            }
                        }
                        if (s_restStride)
                            log::Write("rest positions are %d floats per particle; pieces now "
                                       "take their silhouette from the engine's own cloud",
                                       s_restStride);
                    }

                    if (s_restStride) {
                        const float* pts = restPositions + size_t(begin) * size_t(s_restStride);
                        uint64_t key = 1469598103934665603ull ^ uint64_t(n);
                        for (int k = 0; k < n * s_restStride; ++k) {
                            key ^= FloatBits(pts[k]);
                            key *= 1099511628211ull;
                        }
                        auto it = s_restHulls.find(key);
                        int idx = -1;
                        if (it != s_restHulls.end()) {
                            idx = it->second;
                        } else {
                            fragment::Hull shp;
                            if (fragment::BuildFromPoints(pts, n, s_restStride,
                                                            particleRadius * 0.5f, shp)) {
                                s_hulls.push_back(shp);
                                idx = int(s_hulls.size()) - 1;
                                s_restHulls[key] = idx;
                            }
                        }
                        if (idx >= 0 && s_hulls[size_t(idx)].collisionRadius > 0.0f) {
                            s_pieces.hullIndex[size_t(i)] = idx;
                            s_pieces.radii[size_t(i)] =
                                s_hulls[size_t(idx)].collisionRadius * g_tune.pieceRadiusScale;
                            s_pieces.gyration[size_t(i)] =
                                s_hulls[size_t(idx)].gyration * g_tune.pieceRadiusScale;
                        }
                    }
                }
            }

            // Fall back to one of the loaded debris models. Which model a fragment uses is not
            // stated anywhere reachable here, so one is chosen from the set and keyed off where
            // the fragment appeared, keeping a piece's shape stable for its whole life.
            //
            // The choice of model is the only approximation: each silhouette itself was measured
            // from geometry the engine supplied. Any real chunk silhouette tumbles and settles
            // like debris, whereas a uniform sphere cannot, since a contact beneath its centre
            // of mass produces no torque.
            if (s_pieces.hullIndex[size_t(i)] < 0 && !s_hulls.empty()) {
                const float* t2 = &s_pieces.translations[size_t(i) * 3];
                const uint32_t pick = Mix(FloatBits(t2[0]) ^ Mix(FloatBits(t2[1]) ^
                                                                 Mix(FloatBits(t2[2]))));
                const int idx = int(pick % uint32_t(s_hulls.size()));
                const fragment::Hull& shp = s_hulls[size_t(idx)];
                if (shp.numPoints > 0 && shp.collisionRadius > 0.0f) {
                    s_pieces.hullIndex[size_t(i)] = idx;
                    s_pieces.radii[size_t(i)] = shp.collisionRadius * g_tune.pieceRadiusScale;
                    s_pieces.gyration[size_t(i)] = shp.gyration * g_tune.pieceRadiusScale;
                }
            }

            // Velocity is settled below, once the whole burst is known.
            s_pieces.velocities[size_t(i) * 3 + 0] = 0.0f;
            s_pieces.velocities[size_t(i) * 3 + 1] = 0.0f;
            s_pieces.velocities[size_t(i) * 3 + 2] = 0.0f;
            s_pieces.resting[size_t(i)] = 0;
            s_freshList.push_back(i);

            // A recycled slot starts its climb from wherever this chunk appeared.
            if (size_t(i) < s_riseFrom.size()) {
                s_riseFrom[size_t(i)] = 1e30f;
                s_riseLogged[size_t(i)] = 0;
            }

            // The engine's spawn velocity for this fragment. It writes one into the particle
            // velocities of each chunk it creates, and those particles live in container
            // buffers owned by this plugin, with `indices` naming the slots this piece holds.
            // So the launch velocity is read directly rather than inferred from the burst.
            if (offsets && indices && !s_containers.empty()) {
                const Container* c = s_containers.front();
                const int begin = offsets[i];
                const int end = offsets[i + 1];
                if (c && end > begin && begin >= 0) {
                    float sum[3] = {0, 0, 0};
                    int n = 0;
                    for (int k = begin; k < end; ++k) {
                        const int slot = indices[k];
                        if (slot < 0 || slot >= c->maxParticles)
                            continue;
                        const float* v = &c->velocities[size_t(slot) * 3];
                        if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2]))
                            continue;
                        for (int a = 0; a < 3; ++a)
                            sum[a] += v[a];
                        ++n;
                    }
                    if (n > 0) {
                        const float mean[3] = {sum[0] / float(n), sum[1] / float(n),
                                               sum[2] / float(n)};
                        const float speed2 =
                            mean[0] * mean[0] + mean[1] * mean[1] + mean[2] * mean[2];
                        // An actual launch: fast enough to be one, and slow enough to be one.
                        if (speed2 > 1.0f &&
                            speed2 < kMaxPlausibleSpawnSpeed * kMaxPlausibleSpawnSpeed) {
                            // How hard the gun throws the chunks it creates, which is separate
                            // from how hard the same shot disturbs rubble already lying there.
                            // ImpactShock governs that half independently.
                            for (int a = 0; a < 3; ++a)
                                s_pieces.velocities[size_t(i) * 3 + a] =
                                    mean[a] * g_tune.spawnVelocityScale;
                            s_seededFromEngine[size_t(i)] = 1;
                            static int loggedVel = 0;
                            if (loggedVel < 4) {
                                ++loggedVel;
                                log::Write("spawn velocity from the engine: (%.1f, %.1f, %.1f) "
                                           "over %d particles", mean[0], mean[1], mean[2], n);
                            }
                        }
                    }
                }
            }

            // Give each new piece a spin, integrated as an angular velocity rather than
            // recovered by matching the particle cloud to its rest pose each step.
            //
            // Seeded from where the fragment appeared and which burst it belongs to, never from
            // the slot it landed in, so a recycled slot tumbles differently each time. The axis
            // is drawn uniformly over the sphere, which keeps a burst from looking
            // choreographed.
            const float* t = &s_pieces.translations[size_t(i) * 3];
            uint32_t seed = Mix(FloatBits(t[0]) ^ Mix(FloatBits(t[1]) ^ Mix(FloatBits(t[2])))) ^
                            Mix(uint32_t(i) * 2654435761u + s_spawnEpoch);

            const float z = NextFloat(seed) * 2.0f - 1.0f;
            const float phi = NextFloat(seed) * 6.28318531f;
            const float rxy = std::sqrt(std::max(0.0f, 1.0f - z * z));
            const float axis[3] = {rxy * std::cos(phi), rxy * std::sin(phi), z};
            const float spin = g_tune.spawnSpin * (0.35f + 1.30f * NextFloat(seed));
            for (int a = 0; a < 3; ++a)
                s_pieces.angular[size_t(i) * 3 + a] = axis[a] * spin;
        }

        // What is under a fresh fragment. Debris resting on some surfaces and sinking through
        // others is a question about the collision set rather than about the solver: either the
        // ground is absent from what the engine hands over, or it is present and the sweep is
        // not finding it. A ray straight down from a new piece, through the same sweep the
        // solver uses, answers which.
        if (log::Verbose() && !s_freshList.empty()) {
            static int probed = 0;
            if (probed < 8) {
                ++probed;
                const int i = s_freshList.front();
                const float* p = &s_pieces.translations[size_t(i) * 3];
                const float down[3] = {0.0f, 0.0f, -1.0f};

                int meshHit = 0, meshNear = 0, hullNear = 0, primNear = 0;
                float nearest = 1e30f;

                for (const Collider& sh : s_colliders) {
                    if (sh.haveAabb &&
                        (p[0] < sh.aabbLo[0] - 256.0f || p[0] > sh.aabbHi[0] + 256.0f ||
                         p[1] < sh.aabbLo[1] - 256.0f || p[1] > sh.aabbHi[1] + 256.0f))
                        continue;

                    if (sh.mesh) {
                        ++meshNear;
                        float invRot[4], localFrom[3], localDir[3];
                        QuatConjugate(sh.rot, invRot);
                        const float rel[3] = {p[0] - sh.pos[0], p[1] - sh.pos[1],
                                              p[2] - sh.pos[2]};
                        QuatRotate(invRot, rel, localFrom);
                        QuatRotate(invRot, down, localDir);
                        float hitT = 0.0f;
                        if (SweepMesh(*sh.mesh, localFrom, localDir, 4000.0f, 1.0f, hitT,
                                      nullptr)) {
                            ++meshHit;
                            if (hitT < nearest)
                                nearest = hitT;
                        }
                    } else if (sh.planeCount > 0) {
                        ++hullNear;
                    } else {
                        ++primNear;
                    }
                }

                log::Write("GROUND: fresh piece at (%.0f, %.0f, %.0f); %d of %d meshes in the "
                           "column are hit going down (nearest %.0f units); also %d hulls, "
                           "%d primitives, of %zu colliders",
                           p[0], p[1], p[2], meshHit, meshNear,
                           nearest > 1e29f ? -1.0f : nearest, hullNear, primNear,
                           s_colliders.size());
            }
        }

        // A piece that was already lying there is not thrown, whatever the slot bookkeeping
        // decided about it.
        //
        // Telling a re-seeded slot from a piece the engine merely moved is unreliable at scale,
        // and getting it wrong in this direction is what is visible: a settled piece handed a
        // spawn spin and a burst velocity leaps out of the pile. So a piece that was at rest
        // adopts the position it was given and nothing else, and simply falls from there.
        // Forcing it back to rest instead would freeze it wherever it happened to be.
        {
            int calmed = 0;
            size_t keep = 0;
            for (size_t n = 0; n < s_freshList.size(); ++n) {
                const int i = s_freshList[n];
                const bool wasResting =
                    i < int(s_wasResting.size()) && s_wasResting[size_t(i)] != 0;

                // How far the engine moved this slot since we last reported it. An unbounded
                // distance stands for a slot with no previous position, which is displaced by
                // definition rather than undisturbed.
                float jump = 1e30f;
                if (i < int(s_pieces.lastReported.size()) / 3) {
                    const float* was = &s_pieces.lastReported[size_t(i) * 3];
                    const float* now = &s_pieces.translations[size_t(i) * 3];
                    const float d[3] = {now[0] - was[0], now[1] - was[1], now[2] - was[2]};
                    jump = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
                }

                const slots::Reseed r = slots::OnReseed(wasResting, jump);
                if (!r.thrown) {
                    if (r.clearMotion) {
                        for (int a = 0; a < 3; ++a) {
                            s_pieces.velocities[size_t(i) * 3 + a] = 0.0f;
                            s_pieces.angular[size_t(i) * 3 + a] = 0.0f;
                        }
                    }
                    if (i < int(s_pieces.resting.size()))
                        s_pieces.resting[size_t(i)] = r.asleep ? 1 : 0;

                    ++calmed;
                    continue;   // and no burst direction or shock from it either
                }
                s_freshList[keep++] = s_freshList[n];
            }
            s_freshList.resize(keep);

            static int loggedCalm = 0;
            if (calmed > 0 && loggedCalm < 8) {
                ++loggedCalm;
                log::Write("%d piece(s) already at rest were re-seeded and left to fall rather "
                           "than thrown (%zu genuine spawns kept)", calmed, s_freshList.size());
            }
        }

        // Fragments from one impact arrive together, spread around the point that was hit, so
        // the outward direction from the centre of their own cluster says which way each piece
        // broke off. Nothing else in the traffic states where a shot came from.
        //
        // The clustering matters: sustained fire puts several separate impacts in one call, and
        // a single centroid across all of them would throw every fragment toward the average of
        // unrelated hits. One shove is placed per cluster, at its middle, with the strength the
        // engine launched that cluster at.
        if (g_tune.impactShock > 0.0f) {
            for (int idx : s_freshList) {
                const float* p = &s_pieces.translations[size_t(idx) * 3];
                const float* v = &s_pieces.velocities[size_t(idx) * 3];
                // Measured back through the launch scale, so scaling the fragments down does
                // not scale the disturbance with them.
                const float scale = (s_seededFromEngine[size_t(idx)] &&
                                     g_tune.spawnVelocityScale > 0.01f)
                                        ? g_tune.spawnVelocityScale : 1.0f;
                const float speed =
                    std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) / scale;
                if (!(speed > 1.0f))
                    continue;

                Record(s_impacts, p, speed * kBurstStrength * g_tune.impactShock);
            }
        }

        // Spawns no slot claimed are aged out, so the list cannot grow without bound.
        {
            size_t keep = 0;
            for (size_t sp = 0; sp < s_pendingSpawns.size(); ++sp) {
                if (s_pendingSpawns[sp].used || ++s_pendingSpawns[sp].age > kSpawnMaxAge)
                    continue;
                s_pendingSpawns[keep++] = s_pendingSpawns[sp];
            }
            s_pendingSpawns.resize(keep);
        }

        static int loggedMatch = 0;
        if (loggedMatch < 4 && !s_freshList.empty()) {
            ++loggedMatch;
            log::Write("%d of %d newly seeded pieces matched an engine spawn (velocity and "
                       "size taken from the engine); %zu spawns still unclaimed",
                       matchedSpawns, int(s_freshList.size()), s_pendingSpawns.size());
        }

        // Only pieces the engine did not describe infer a direction from the burst's
        // arrangement; the rest leave at the velocity it stated.
        if (g_tune.spawnBurst > 0.0f && s_freshList.size() > 1) {
            const bool cluster = s_freshList.size() <= 512; // else O(n^2) stops being free
            float all[3] = {0, 0, 0};
            if (!cluster) {
                for (int idx : s_freshList)
                    for (int a = 0; a < 3; ++a)
                        all[a] += s_pieces.translations[size_t(idx) * 3 + a];
                for (int a = 0; a < 3; ++a)
                    all[a] /= float(s_freshList.size());
            }

            for (int idx : s_freshList) {
                if (s_seededFromEngine[size_t(idx)])
                    continue; // the engine already said how fast this one is going
                const float* p = &s_pieces.translations[size_t(idx) * 3];
                float centre[3] = {all[0], all[1], all[2]};

                if (cluster) {
                    float sum[3] = {0, 0, 0};
                    int n = 0;
                    for (int other : s_freshList) {
                        const float* q = &s_pieces.translations[size_t(other) * 3];
                        const float d[3] = {q[0] - p[0], q[1] - p[1], q[2] - p[2]};
                        if (d[0] * d[0] + d[1] * d[1] + d[2] * d[2] >
                            kBurstRadius * kBurstRadius)
                            continue;
                        for (int a = 0; a < 3; ++a)
                            sum[a] += q[a];
                        ++n;
                    }
                    if (n < 2)
                        continue; // a lone fragment has no cluster to point away from
                    for (int a = 0; a < 3; ++a)
                        centre[a] = sum[a] / float(n);
                }

                float dir[3] = {p[0] - centre[0], p[1] - centre[1], p[2] - centre[2]};
                const float len = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
                if (len < 1e-3f)
                    continue; // sitting on the centre, so no direction to leave by

                uint32_t seed = Mix(FloatBits(p[0]) ^ Mix(FloatBits(p[2]))) ^ s_spawnEpoch;
                const float speed = g_tune.spawnBurst * (0.5f + NextFloat(seed));
                for (int a = 0; a < 3; ++a)
                    s_pieces.velocities[size_t(idx) * 3 + a] = dir[a] / len * speed;
            }
        }

        static int loggedRespawn = 0;
        if (respawned > 0 && loggedRespawn < 4) {
            ++loggedRespawn;
            log::Write("%d of %d slots re-seeded by the engine, new debris adopted into "
                       "recycled pool slots", respawned, numRigids);
        }
    }

    static int logged = 0;
    if (logged < 4 && numRigids > 0) {
        ++logged;
        log::Write("SetRigids numRigids=%d first translation=(%.1f, %.1f, %.1f)",
                   numRigids, s_pieces.translations[0], s_pieces.translations[1],
                   s_pieces.translations[2]);
    }
}

// flexGetRigidTransforms(solver, rotations, translations, memory)
void WINAPI Shim_flexGetRigidTransforms(void* handle, float* rotations, float* translations,
                                        int)
{
    Note("flexGetRigidTransforms");
    (void)handle;
    std::lock_guard<std::mutex> lock(s_solverMutex);
    if (s_pieces.count <= 0)
        return;

    if (rotations)
        memcpy(rotations, s_pieces.rotations.data(),
               size_t(s_pieces.count) * 4 * sizeof(float));
    if (translations) {
        const size_t n = size_t(s_pieces.count) * 3;
        memcpy(translations, s_pieces.translations.data(), n * sizeof(float));

        // Record exactly what was reported, so the next flexSetRigids can tell a round trip of
        // these values from a genuinely new fragment.
        s_pieces.lastReported.resize(n);
        memcpy(s_pieces.lastReported.data(), s_pieces.translations.data(),
               n * sizeof(float));
        s_pieces.reported.assign(size_t(s_pieces.count), 1);
        s_setsSinceRead = 0;
    }

    static int logged = 0;
    if (logged < 4) {
        ++logged;
        log::Write("GetRigidTransforms n=%d returning first=(%.1f, %.1f, %.1f)",
                   s_pieces.count, s_pieces.translations[0], s_pieces.translations[1],
                   s_pieces.translations[2]);
    }
}
// flexGetBounds(solver, lower, upper)
//
// The solver's world-space AABB, which the engine uses to decide whether the debris draw call
// is on screen. Leaving the buffer untouched would hand back whatever it already held and cull
// the debris everywhere the box did not happen to cover.
void WINAPI Shim_flexGetBounds(void* handle, float* lower, float* upper)
{
    Note("flexGetBounds");
    (void)handle;
    if (!lower || !upper)
        return;

    std::lock_guard<std::mutex> lock(s_solverMutex);

    float lo[3] = {1e30f, 1e30f, 1e30f};
    float hi[3] = {-1e30f, -1e30f, -1e30f};
    bool any = false;

    // Pieces are what the engine draws, and each extends its own radius beyond its centre, so
    // the box includes that or debris at the edge of the batch clips early.
    for (int i = 0; i < s_pieces.count; ++i) {
        const float* p = &s_pieces.translations[size_t(i) * 3];
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]))
            continue;
        const float r = (i < int(s_pieces.radii.size())) ? s_pieces.radii[size_t(i)] : 0.0f;
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], p[a] - r);
            hi[a] = std::max(hi[a], p[a] + r);
        }
        any = true;
    }

    for (const Container* c : s_containers) {
        if (!c)
            continue;
        for (int i = 0; i < c->maxParticles; ++i) {
            const float* p = &c->particles[size_t(i) * 4];
            if (!IsLiveParticle(p))
                continue;
            for (int a = 0; a < 3; ++a) {
                lo[a] = std::min(lo[a], p[a]);
                hi[a] = std::max(hi[a], p[a]);
            }
            any = true;
        }
    }

    // An empty solver has nothing to bound, so a degenerate box is the right answer.
    for (int a = 0; a < 3; ++a) {
        lower[a] = any ? lo[a] : 0.0f;
        upper[a] = any ? hi[a] : 0.0f;
    }

    static int logged = 0;
    if (any && logged < 3) {
        ++logged;
        log::Write("GetBounds (%.0f %.0f %.0f)-(%.0f %.0f %.0f) over %d pieces",
                   lower[0], lower[1], lower[2], upper[0], upper[1], upper[2],
                   s_pieces.count);
    }
}
// flexSetShapes(solver, geometry, numGeometryEntries, shapeAabbMin, shapeAabbMax,
//               shapeOffsets, shapePositions, shapeRotations, shapePrevPositions,
//               shapePrevRotations, shapeFlags, numShapes, memory)
//
// A geometry entry for a triangle mesh begins with the mesh handle. Those handles were issued
// by this module, so the entry stride is established by finding the spacing at which the
// read-back pointers match known handles.
void WINAPI Shim_flexSetShapes(void* solver, const void* geometry, int numGeometryEntries,
                               const float* aabbMin, const float* aabbMax,
                               const int* shapeOffsets,
                               const float* positions, const float* rotations,
                               const float* prevPositions, const float*,
                               const int* shapeFlags, int numShapes, int)
{
    Note("flexSetShapes");
    (void)solver;
    if (!positions || numShapes <= 0 || numShapes > 65536)
        return;

    std::lock_guard<std::mutex> lock(s_solverMutex);

    // Locate the issued mesh handles inside the geometry block and derive the stride from
    // where they land, rather than assuming one.
    if (geometry && numGeometryEntries > 0 && s_geometryStride == 0 && !s_meshes.empty() &&
        !s_geometryScanned) {
        s_geometryScanned = true;

        const size_t bytes = std::min<size_t>(size_t(numGeometryEntries) * 64, 256 * 1024);
        size_t firstOffset = 0, secondOffset = 0;
        const int found = ScanForMeshHandles(geometry, bytes, firstOffset, secondOffset);

        if (found < 0) {
            log::Write("geometry scan faulted, block shorter than assumed");
        } else if (found >= 2) {
            s_geometryStride = int(secondOffset - firstOffset);
            log::Write("found %d mesh handles in the geometry block; first at "
                       "+%zu, spacing %d bytes", found, firstOffset, s_geometryStride);
        } else if (found == 1) {
            log::Write("only one mesh handle found at +%zu, cannot derive spacing",
                       firstOffset);
        } else {
            log::Write("no mesh handles present in the geometry block "
                       "(entries=%d, known meshes=%zu), meshes are referenced some other "
                       "way, e.g. by index", numGeometryEntries, s_meshes.size());
        }
    }

    s_colliders.clear();
    s_colliders.reserve(size_t(numShapes));

    // The flags array is trusted only once it has proved itself: if the reading is right,
    // every collider it calls a triangle mesh resolves to an issued handle. Failing that,
    // colliders are identified by handle alone and the primitives are lost.
    static int flagsUsable = -1; // -1 unknown, 0 rejected, 1 accepted
    if (flagsUsable < 0 && shapeFlags && geometry && shapeOffsets && !s_meshes.empty()) {
        int claimedMeshes = 0, agreed = 0, badType = 0;
        auto base = static_cast<const uint8_t*>(geometry);
        for (int i = 0; i < numShapes; ++i) {
            const int type = shapeFlags[i] & kColliderKindMask;
            if (type < kColliderSphere || type > kColliderSDF) {
                ++badType;
                continue;
            }
            if (type != kColliderTriangleMesh)
                continue;
            ++claimedMeshes;
            const int g = shapeOffsets[i];
            if (g < 0 || g >= numGeometryEntries)
                continue;
            auto p = *reinterpret_cast<void* const*>(base + size_t(g) * kGeometryEntrySize);
            if (s_meshes.find(p) != s_meshes.end())
                ++agreed;
        }
        // With nothing claiming to be a mesh there is nothing to check against, so the question
        // stays open for a later call rather than being decided by a cell holding no meshes.
        if (claimedMeshes > 0 || badType > 0) {
            flagsUsable = (badType == 0 && claimedMeshes > 0 && agreed * 4 >= claimedMeshes * 3);
            log::Write("shape flags %s, %d shapes, %d called triangle meshes, %d of those "
                       "resolved to handles we issued, %d had an unrecognised type",
                       flagsUsable ? "accepted" : "rejected, falling back to meshes only",
                       numShapes, claimedMeshes, agreed, badType);
        }
    }

    // Keep the previous frame's colliders, so one that has moved can be told from one that has
    // always been where it is.
    s_prevColliders.swap(s_colliders);
    s_colliders.clear();
    s_colliders.reserve(size_t(numShapes));
    s_planes.clear();

    for (int i = 0; i < numShapes; ++i) {
        Collider sh;
        // Positions and rotations are float4 per collider.
        sh.pos[0] = positions[size_t(i) * 4 + 0];
        sh.pos[1] = positions[size_t(i) * 4 + 1];
        sh.pos[2] = positions[size_t(i) * 4 + 2];
        if (rotations) {
            for (int a = 0; a < 4; ++a)
                sh.rot[a] = rotations[size_t(i) * 4 + a];
        }

        // Prefer the engine's own previous transform; where it is absent, last frame's serves
        // the same purpose.
        if (prevPositions) {
            for (int a = 0; a < 3; ++a)
                sh.prevPos[a] = prevPositions[size_t(i) * 4 + a];
        } else if (i < int(s_prevColliders.size())) {
            for (int a = 0; a < 3; ++a)
                sh.prevPos[a] = s_prevColliders[size_t(i)].pos[a];
        } else {
            for (int a = 0; a < 3; ++a)
                sh.prevPos[a] = sh.pos[a];
        }

        const float step[3] = {sh.pos[0] - sh.prevPos[0], sh.pos[1] - sh.prevPos[1],
                               sh.pos[2] - sh.prevPos[2]};
        sh.moved = (step[0] * step[0] + step[1] * step[1] + step[2] * step[2]) >
                   kColliderMotionEpsilon * kColliderMotionEpsilon;

        if (shapeFlags) {
            // The dynamic bit stands on its own: flags are `kind | (dynamic ? 8 : 0)`, so bit 3
            // is readable without trusting the kind encoding.
            sh.dynamic = (shapeFlags[i] & kColliderDynamicBit) != 0;
            if (flagsUsable == 1)
                sh.type = shapeFlags[i] & kColliderKindMask;
        }

        // Each collider owns a run of geometry entries given by shapeOffsets. This walks the
        // run and takes the first entry it can read: an issued handle gives a triangle mesh,
        // and a sphere, capsule or box carries its dimensions at the start of the entry.
        if (geometry && shapeOffsets) {
            const int begin = shapeOffsets[i];
            const int end = (i + 1 < numShapes) ? shapeOffsets[i + 1] : numGeometryEntries;
            auto base = static_cast<const uint8_t*>(geometry);
            for (int g = begin; g < end && g < numGeometryEntries; ++g) {
                if (g < 0)
                    break;
                auto entry = base + size_t(g) * kGeometryEntrySize;
                auto p = *reinterpret_cast<void* const*>(entry);
                auto it = s_meshes.find(p);
                if (it != s_meshes.end()) {
                    sh.mesh = &it->second;
                    sh.type = kColliderTriangleMesh;
                    break;
                }
                if (sh.type == kColliderSphere || sh.type == kColliderCapsule ||
                    sh.type == kColliderBox) {
                    auto f = reinterpret_cast<const float*>(entry);
                    // Refuse implausible dimensions, which would otherwise plant an invisible
                    // collider in the world.
                    bool sane = true;
                    const int used = (sh.type == kColliderBox) ? 3
                                   : (sh.type == kColliderCapsule) ? 2 : 1;
                    for (int a = 0; a < used; ++a) {
                        if (!std::isfinite(f[a]) || f[a] <= 0.0f || f[a] > 100000.0f)
                            sane = false;
                        sh.dims[a] = f[a];
                    }
                    if (!sane)
                        sh.type = kColliderUnknown;
                    break;
                }
            }
        }
        // Actors. A single geometry entry holding two positive lengths is a capsule: radius
        // then half height. Recognised by that signature rather than by the kind flag, since
        // this build's kind encoding does not match the published one and the data is
        // unambiguous on its own. These are the colliders that move, and they set neither the
        // dynamic bit nor a decodable kind.
        if (geometry && shapeOffsets && !sh.mesh && sh.planeCount == 0) {
            const int begin = shapeOffsets[i];
            const int end = (i + 1 < numShapes) ? shapeOffsets[i + 1] : numGeometryEntries;
            if (end - begin == 1 && begin >= 0 && begin < numGeometryEntries) {
                auto f = reinterpret_cast<const float*>(
                    static_cast<const uint8_t*>(geometry) + size_t(begin) * kGeometryEntrySize);
                // Two plausible positive lengths are the whole signature. The remaining two
                // components are the unused tail of the entry and hold whatever occupied it
                // before, in practice denormals around 1e-39 that print as 0.0000, so nothing
                // may be required of them.
                const bool looksCapsule =
                    std::isfinite(f[0]) && std::isfinite(f[1]) &&
                    f[0] > 0.01f && f[0] < 1000.0f &&
                    f[1] > 0.0f && f[1] < 1000.0f;
                if (looksCapsule) {
                    sh.type = kColliderCapsule;
                    sh.dims[0] = f[0];   // radius
                    sh.dims[1] = f[1];   // half height
                }
            }
        }

        // Most of the world is a convex hull rather than a mesh or a primitive. Planes are
        // recognised by their content: a float4 whose first three components form a unit vector
        // is a plane, and nothing else in the block resembles one, so this holds regardless of
        // how the flags are encoded.
        if (geometry && shapeOffsets && sh.type != kColliderTriangleMesh && !sh.mesh) {
            const int begin = shapeOffsets[i];
            const int end = (i + 1 < numShapes) ? shapeOffsets[i + 1] : numGeometryEntries;
            auto base = static_cast<const uint8_t*>(geometry);
            const int firstPlane = int(s_planes.size() / 4);
            int planes = 0;

            for (int g = begin; g >= 0 && g < end && g < numGeometryEntries; ++g) {
                auto f = reinterpret_cast<const float*>(base + size_t(g) * kGeometryEntrySize);
                const float len2 = f[0] * f[0] + f[1] * f[1] + f[2] * f[2];
                if (!std::isfinite(len2) || std::fabs(len2 - 1.0f) > 0.02f ||
                    !std::isfinite(f[3]))
                    break; // not a plane, so the run has ended
                for (int a = 0; a < 4; ++a)
                    s_planes.push_back(f[a]);
                ++planes;
            }

            // Fewer than four planes cannot bound a volume, so anything shorter is noise.
            if (planes >= 4) {
                sh.planeStart = firstPlane;
                sh.planeCount = planes;
            } else {
                s_planes.resize(size_t(firstPlane) * 4);
            }
        }

        // The engine's per-collider bounds, used only to skip hulls a piece is nowhere near.
        if (aabbMin && aabbMax) {
            bool sane = true;
            for (int a = 0; a < 3 && sane; ++a) {
                const float lo = aabbMin[size_t(i) * 4 + size_t(a)];
                const float hi = aabbMax[size_t(i) * 4 + size_t(a)];
                sane = std::isfinite(lo) && std::isfinite(hi) && hi >= lo &&
                       std::fabs(lo) < 1e9f && std::fabs(hi) < 1e9f;
                sh.aabbLo[a] = lo;
                sh.aabbHi[a] = hi;
            }
            sh.haveAabb = sane;
        }

        s_colliders.push_back(sh);
    }

    // Whether the collider list holds an actor, and whether anything moves. Both must be true
    // for walking through rubble to disturb it.
    {
        static int loggedMotion = 0;
        static bool sawMotion = false;
        int dyn = 0, movedByEngine = 0, movedByUs = 0;
        for (int i = 0; i < numShapes; ++i) {
            if (shapeFlags && (shapeFlags[i] & kColliderDynamicBit))
                ++dyn;
            if (prevPositions) {
                float d2 = 0.0f;
                for (int a = 0; a < 3; ++a) {
                    const float e = positions[size_t(i) * 4 + size_t(a)] -
                                    prevPositions[size_t(i) * 4 + size_t(a)];
                    d2 += e * e;
                }
                if (d2 > kColliderMotionEpsilon * kColliderMotionEpsilon)
                    ++movedByEngine;
            }
            if (i < int(s_prevColliders.size())) {
                float d2 = 0.0f;
                for (int a = 0; a < 3; ++a) {
                    const float e = positions[size_t(i) * 4 + size_t(a)] -
                                    s_prevColliders[size_t(i)].pos[a];
                    d2 += e * e;
                }
                if (d2 > kColliderMotionEpsilon * kColliderMotionEpsilon)
                    ++movedByUs;
            }
        }
        const bool interesting = (movedByEngine > 0 || movedByUs > 0) && !sawMotion;
        if (interesting)
            sawMotion = true;
        if (loggedMotion < 6 && (interesting || loggedMotion < 2)) {
            ++loggedMotion;
            log::Write("shape motion: %d of %d flagged dynamic, %d moved per the engine's own "
                       "previous transforms, %d moved compared with our last frame "
                       "(prevPositions=%s)", dyn, numShapes, movedByEngine, movedByUs,
                       prevPositions ? "supplied" : "null");
        }
    }

    // Colliders that move but could not be decoded, dumped raw so the encoding can be
    // identified.
    {
        static int loggedUnknown = 0;
        if (loggedUnknown < 3 && prevPositions && geometry && shapeOffsets) {
            for (int i = 0; i < numShapes && loggedUnknown < 3; ++i) {
                const Collider& sh = s_colliders[size_t(i)];
                if (sh.mesh || sh.planeCount > 0)
                    continue;
                (void)0;
                float d2 = 0.0f;
                for (int a = 0; a < 3; ++a) {
                    const float e = positions[size_t(i) * 4 + size_t(a)] -
                                    prevPositions[size_t(i) * 4 + size_t(a)];
                    d2 += e * e;
                }
                if (d2 <= kColliderMotionEpsilon * kColliderMotionEpsilon)
                    continue;

                ++loggedUnknown;
                const int begin = shapeOffsets[i];
                const int end = (i + 1 < numShapes) ? shapeOffsets[i + 1] : numGeometryEntries;
                char geo[420] = {};
                int n = 0;
                auto base = static_cast<const uint8_t*>(geometry);
                for (int g = begin; g < end && g < begin + 5 && g < numGeometryEntries; ++g) {
                    auto f = reinterpret_cast<const float*>(base + size_t(g) *
                                                            kGeometryEntrySize);
                    n += _snprintf_s(geo + n, sizeof(geo) - size_t(n), _TRUNCATE,
                                     "[%d]%.4f,%.4f,%.4f,%.4f ", g, f[0], f[1], f[2], f[3]);
                }
                log::Write("MOVING shape %d: decodedType=%d dims=(%.2f, %.2f, %.2f) "
                           "flags=%08X entries %d..%d (%d) moved %.2f units raw %s",
                           i, sh.type, sh.dims[0], sh.dims[1], sh.dims[2],
                           shapeFlags ? unsigned(shapeFlags[i]) : 0u, begin, end, end - begin,
                           std::sqrt(d2), geo);
            }
        }
    }

    static bool describedFlags = false;
    if (!describedFlags) {
        describedFlags = true;
        if (!shapeFlags) {
            log::Write("SetShapes: the engine passes no shape flags, so only shapes resolving "
                       "to a mesh handle can be identified at all");
        } else {
            char raw[256] = {};
            int n = 0;
            for (int i = 0; i < numShapes && i < 12; ++i)
                n += _snprintf_s(raw + n, sizeof(raw) - size_t(n), _TRUNCATE, "%08X ",
                                 unsigned(shapeFlags[i]));
            log::Write("SetShapes: first shape flags = %s", raw);
        }
        if (geometry && shapeOffsets && numShapes > 0) {
            char off[256] = {};
            int n = 0;
            for (int i = 0; i < numShapes && i < 14; ++i)
                n += _snprintf_s(off + n, sizeof(off) - size_t(n), _TRUNCATE, "%d ",
                                 shapeOffsets[i]);
            log::Write("SetShapes: first shape offsets = %s", off);
            auto base = static_cast<const uint8_t*>(geometry);
            char geo[400] = {};
            n = 0;
            for (int g = 0; g < 10; ++g) {
                auto e = base + size_t(g) * kGeometryEntrySize;
                auto f = reinterpret_cast<const float*>(e);
                n += _snprintf_s(geo + n, sizeof(geo) - size_t(n), _TRUNCATE,
                                 "[%d]%p|%.2f,%.2f ", g,
                                 *reinterpret_cast<void* const*>(e), f[2], f[3]);
            }
            log::Write("SetShapes: first geometry entries = %s", geo);
        }
    }

    static int logged = 0;
    static uint32_t lastShapeLog = 0;
    if (logged < 10 && (logged < 2 || s_spawnEpoch - lastShapeLog > 200)) {
        ++logged;
        lastShapeLog = s_spawnEpoch;
        int counts[6] = {};
        int withMesh = 0, movers = 0, dynamics = 0, hulls = 0, hullPlanes = 0, withAabb = 0;
        for (const Collider& sh : s_colliders) {
            if (sh.mesh)
                ++withMesh;
            if (sh.planeCount > 0) { ++hulls; hullPlanes += sh.planeCount; }
            if (sh.haveAabb)
                ++withAabb;
            if (sh.moved)
                ++movers;
            if (sh.dynamic)
                ++dynamics;
            if (sh.type >= 0 && sh.type <= kColliderSDF)
                ++counts[sh.type];
        }
        log::Write("SetShapes numShapes=%d geomEntries=%d meshes=%d hulls=%d (%d planes) "
                   "capsules=%d aabbs=%d dynamic=%d moving=%d first pos=(%.1f, %.1f, %.1f)",
                   numShapes, numGeometryEntries, withMesh, hulls, hullPlanes,
                   counts[kColliderCapsule], withAabb, dynamics, movers, s_colliders[0].pos[0],
                   s_colliders[0].pos[1], s_colliders[0].pos[2]);
    }
}
void* WINAPI Shim_flexCreateTriangleMesh(...)  { Note("flexCreateTriangleMesh");  return NewHandle(); }
// flexUpdateTriangleMesh(mesh, vertices, indices, numVertices, numTriangles, lower, upper,
//                        memory)
//
// Vertices may be packed float3 or padded to float4. The stride is decided once, by which
// reading puts every vertex inside the bounds the engine supplied alongside them.
void WINAPI Shim_flexUpdateTriangleMesh(void* mesh, const float* vertices, const int* indices,
                                        int numVertices, int numTriangles, const float* lower,
                                        const float* upper, int)
{
    Note("flexUpdateTriangleMesh");
    if (!mesh || !vertices || !indices || numVertices <= 0 || numTriangles <= 0)
        return;
    if (numVertices > 1'000'000 || numTriangles > 1'000'000)
        return; // implausible; refuse rather than read wild memory

    auto fitsBounds = [&](int stride) {
        if (!lower || !upper)
            return false;
        const int probes = std::min(numVertices, 64);
        for (int i = 0; i < probes; ++i) {
            const float* v = vertices + size_t(i) * stride;
            for (int a = 0; a < 3; ++a) {
                if (!std::isfinite(v[a]))
                    return false;
                // A little slack, for bounds that were rounded.
                const float pad = 1.0f + 0.001f * std::fabs(upper[a] - lower[a]);
                if (v[a] < lower[a] - pad || v[a] > upper[a] + pad)
                    return false;
            }
        }
        return true;
    };

    std::lock_guard<std::mutex> lock(s_solverMutex);

    // Three is tried first, since collision vertices are packed float3 here; four remains as a
    // fallback for a build that pads them.
    if (s_vertexStride == 0) {
        if (fitsBounds(3))
            s_vertexStride = 3;
        else if (fitsBounds(4))
            s_vertexStride = 4;
        if (s_vertexStride)
            log::Write("collision mesh vertex stride = %d floats (verified against "
                       "the engine's own bounds)", s_vertexStride);
        else
            log::Write("could not determine vertex stride, collision disabled");
    }
    if (s_vertexStride == 0)
        return;

    TriMesh& m = s_meshes[mesh];
    if (!IsKnownMeshHandle(mesh) && s_meshHandleCount < 64)
        s_meshHandleList[s_meshHandleCount++] = mesh;

    // The engine re-uploads the same static mesh whenever a cell is touched, so an unchanged
    // upload skips the copy and the index rebuild. Matching counts, bounds and end vertices
    // identify it: a handle reused for different geometry differs in at least one.
    if (m.grid.valid && m.verts.size() == size_t(numVertices) * 3 &&
        m.indices.size() == size_t(numTriangles) * 3 && lower && upper) {
        bool same = true;
        for (int a = 0; a < 3 && same; ++a)
            same = (m.lower[a] == lower[a] && m.upper[a] == upper[a]);
        const float* last = vertices + size_t(numVertices - 1) * s_vertexStride;
        for (int a = 0; a < 3 && same; ++a)
            same = (m.verts[size_t(a)] == vertices[a] &&
                    m.verts[m.verts.size() - 3 + size_t(a)] == last[a]);
        if (same)
            return;
    }

    m.verts.resize(size_t(numVertices) * 3);
    for (int i = 0; i < numVertices; ++i) {
        const float* v = vertices + size_t(i) * s_vertexStride;
        m.verts[size_t(i) * 3 + 0] = v[0];
        m.verts[size_t(i) * 3 + 1] = v[1];
        m.verts[size_t(i) * 3 + 2] = v[2];
    }
    m.indices.assign(indices, indices + size_t(numTriangles) * 3);
    for (int a = 0; a < 3; ++a) {
        m.lower[a] = lower ? lower[a] : -1e30f;
        m.upper[a] = upper ? upper[a] : 1e30f;
    }

    // Indexed once here rather than scanned in full on every sweep of every piece for as long
    // as the cell is loaded.
    BuildTriGrid(m);

    static int logged = 0;
    if (logged < 3) {
        ++logged;
        log::Write("collision mesh %d verts / %d tris, bounds (%.0f %.0f %.0f)-"
                   "(%.0f %.0f %.0f), grid %dx%dx%d %s", numVertices, numTriangles,
                   m.lower[0], m.lower[1], m.lower[2], m.upper[0], m.upper[1], m.upper[2],
                   m.grid.dim[0], m.grid.dim[1], m.grid.dim[2],
                   m.grid.valid ? "indexed" : "(linear scan)");
    }
}
void  WINAPI Shim_flexDestroyTriangleMesh(void* m) { Note("flexDestroyTriangleMesh"); free(m); }

// A real allocator: the engine writes particle data straight into what this returns.
void* WINAPI Shim_flexAlloc(int size)
{
    Note("flexAlloc");
    return size > 0 ? calloc(1, size_t(size)) : NewHandle();
}
void  WINAPI Shim_flexFree(void* p)        { Note("flexFree");            free(p); }
void  WINAPI Shim_flexSetFence(...)        { Note("flexSetFence");        }
void  WINAPI Shim_flexWaitFence(...)       { Note("flexWaitFence");       }
void  WINAPI Shim_flexStartRecord(...)     { Note("flexStartRecord");     }
void  WINAPI Shim_flexStopRecord(...)      { Note("flexStopRecord");      }
void  WINAPI Shim_flexSnapshot(...)        { Note("flexSnapshot");        }

// The container owns the particle storage the engine writes spawn data into and reads results
// back from, which flexExtGetParticleData hands out pointers to.
void* WINAPI Shim_flexExtCreateContainer(void* solver, int maxParticles)
{
    Note("flexExtCreateContainer");
    auto* c = new (std::nothrow) Container();
    if (!c)
        return nullptr;

    c->maxParticles = (maxParticles > 0 && maxParticles < 1 << 20) ? maxParticles : 6000;
    c->particles.assign(size_t(c->maxParticles) * 4, 0.0f);
    c->velocities.assign(size_t(c->maxParticles) * 3, 0.0f);
    c->phases.assign(size_t(c->maxParticles), 0);
    c->normals.assign(size_t(c->maxParticles) * 4, 0.0f);
    c->active.assign(size_t(c->maxParticles), 0);
    c->solver = AsSolver(solver);

    {
        std::lock_guard<std::mutex> lock(s_solverMutex);
        s_containers.push_back(c);
    }
    // Whether the requested budget took. The engine latches it from Fallout4Prefs.ini long
    // before any plugin loads, so a short allocation can only be corrected there.
    if (s_expectedBudget > 0 && c->maxParticles < s_expectedBudget) {
        const int tier = (c->maxParticles >= 32768) ? 2 : (c->maxParticles >= 16000 ? 1 : 0);
        log::Write("ext container created (maxParticles=%d), that is debris quality tier %d, "
                   "below the %d particles you asked for. The game reads its tier from "
                   "iQuality:NVFlex in Fallout4Prefs.ini during startup, before any plugin "
                   "loads, so this cannot be changed from here. Set iQuality=%d under [NVFlex] "
                   "and restart. Debris volume is bounded by this budget: the engine refuses a "
                   "chunk once the particles in flight plus the chunk's own would exceed it.",
                   c->maxParticles, tier, s_expectedBudget, config::Get().debrisQuality);
    } else {
        log::Write("ext container created (maxParticles=%d)", c->maxParticles);
    }
    return c;
}

void WINAPI Shim_flexExtDestroyContainer(void* handle)
{
    Note("flexExtDestroyContainer");
    auto* c = static_cast<Container*>(handle);
    std::lock_guard<std::mutex> lock(s_solverMutex);
    s_containers.erase(std::remove(s_containers.begin(), s_containers.end(), c),
                       s_containers.end());
    delete c;
}
// flexExtCreateRigidFromMesh(vertices, numVertices, indices, numTriangleIndices, radius,
//                            expand)
//
// The engine describes the chunk it is about to spawn. That description is what gives each
// piece its own size rather than one shared radius. The model is built here rather than
// forwarded, so a single allocator owns it through create and destroy alike.
//
// The engine gates spawning on `activeParticles + model->numParticles < iMaxParticles`, so the
// particle count the voxeliser produces sets how much debris fits on screen: too few and the
// budget never fills, too many and it throttles early. The count is therefore compared against
// what the shipped library produces for the same mesh.
using PFN_ExtCreateRigidFromMesh = void*(WINAPI*)(const float*, int, const int*, int, float,
                                                  float);
using PFN_ExtDestroyAsset = void(WINAPI*)(void*);
PFN_ExtCreateRigidFromMesh s_realExtCreateRigidFromMesh = nullptr;
PFN_ExtDestroyAsset s_realExtDestroyAsset = nullptr;

// Reads the particle count out of a model the original library built. Guarded, and read-only:
// the allocation and the layout are theirs.
int OriginalParticleCount(void* asset)
{
    __try {
        auto base = static_cast<const uint8_t*>(asset);
        const int n = *reinterpret_cast<const int*>(base + 8);
        return (n >= 0 && n < 1000000) ? n : -1;
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        return -1;
    }
}

void* WINAPI Shim_flexExtCreateRigidFromMesh(const float* vertices, int numVertices,
                                             const int* indices, int numTriangleIndices,
                                             float radius, float expand)
{
    Note("flexExtCreateRigidFromMesh");

    void* handle = nullptr;
    if (g_tune.useEngineSpawnData)
        handle = fragment::BuildFromMesh(vertices, numVertices, indices, numTriangleIndices,
                                            radius, expand, g_tune.engineParticles);

    // An opaque block leaves the engine with a chunk carrying no particles: it costs the
    // per-piece geometry but is never unsafe.
    if (!handle)
        handle = NewHandle();
    if (!handle)
        return nullptr;

    ModelInfo a;
    a.verts = numVertices;
    a.tris = (numTriangleIndices > 0) ? numTriangleIndices / 3 : 0;

    float centre[3] = {0, 0, 0};
    float measuredRadius = 0.0f, measuredGyration = 0.0f;
    int particles = 0;
    if (fragment::Describe(handle, centre, measuredRadius, measuredGyration, particles)) {
        // The particle cloud describes the chunk better than its bounding box: a splinter and
        // a cube of the same extent share a box and behave nothing alike.
        a.particles = particles;
        a.radius = measuredRadius + expand;
        a.gyration = measuredGyration;
        for (int i = 0; i < 3; ++i)
            a.centre[i] = centre[i];

        fragment::Hull shp;
        if (fragment::GetHull(handle, shp)) {
            std::lock_guard<std::mutex> lock(s_solverMutex);
            s_hulls.push_back(shp);
            a.hullIndex = int(s_hulls.size()) - 1;
        }
    } else if (vertices && numVertices > 0 && numVertices <= 1'000'000) {
        // No model, so the mesh is measured directly. Vertices are tightly packed float3, and
        // the read is probed rather than trusted.
        float lo[3] = {1e30f, 1e30f, 1e30f};
        float hi[3] = {-1e30f, -1e30f, -1e30f};
        if (ProbeVertexBounds(vertices, std::min(numVertices, 256), 3, lo, hi)) {
            float extent = 0.0f;
            for (int i = 0; i < 3; ++i) {
                a.centre[i] = (lo[i] + hi[i]) * 0.5f;
                extent = std::max(extent, (hi[i] - lo[i]) * 0.5f);
            }
            if (extent > 0.05f && extent < 5000.0f) {
                a.radius = extent + expand;
                a.gyration = extent;
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(s_solverMutex);
        s_models[handle] = a;
    }

    static int logged = 0;
    if (logged < 5) {
        ++logged;
        // What the shipped library makes of the same mesh, built and released through its own
        // matched pair purely to compare particle counts. The engine is never given this one.
        int theirs = -1;
        if (s_realExtCreateRigidFromMesh && s_realExtDestroyAsset) {
            void* ref = s_realExtCreateRigidFromMesh(vertices, numVertices, indices,
                                                     numTriangleIndices, radius, expand);
            if (ref) {
                theirs = OriginalParticleCount(ref);
                s_realExtDestroyAsset(ref);
            }
        }
        log::Write("chunk asset: %d verts / %d tris, %d particles (original library makes %d), "
                   "radius %.2f, gyration %.2f (solver particle radius %.2f, expand %.2f)",
                   numVertices, a.tris, a.particles, theirs, a.radius, a.gyration, radius,
                   expand);
    }
    return handle;
}

void WINAPI Shim_flexExtDestroyAsset(void* a)
{
    Note("flexExtDestroyAsset");
    {
        std::lock_guard<std::mutex> lock(s_solverMutex);
        s_models.erase(a);
    }

    // Whatever allocated it releases it: voxelised models go back to that module, and the
    // opaque fallback blocks are plain allocations.
    if (fragment::IsOurs(a))
        fragment::Destroy(a);
    else
        free(a);
}

// flexExtCreateInstance(container, model, transform, vx, vy, vz, phase, invMassScale)
//
// Where the engine states how fast a fragment is leaving and in which direction.
//
// This shim only watches: it records the arguments, calls through to the original and returns
// what that returns, so the engine gets exactly the instance it would have got.
using PFN_ExtCreateInstance = void*(WINAPI*)(void*, const void*, const float*, float, float,
                                             float, int, float);
PFN_ExtCreateInstance s_realExtCreateInstance = nullptr;

void* WINAPI Shim_flexExtCreateInstance(void* container, const void* asset,
                                        const float* transform, float vx, float vy, float vz,
                                        int phase, float invMassScale)
{
    Note("flexExtCreateInstance");

    if (transform) {
        PendingSpawn sp;
        sp.xformFloats = ReadTransform(transform, sp.xform);
        sp.vel[0] = vx;
        sp.vel[1] = vy;
        sp.vel[2] = vz;

        // If the signature is wrong the arguments land in the wrong registers, so the values
        // are checked and any mismatch is logged.
        const bool sane = std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vz) &&
                          std::fabs(vx) < 1e6f && std::fabs(vy) < 1e6f && std::fabs(vz) < 1e6f &&
                          phase >= 0 && std::isfinite(invMassScale) && invMassScale >= 0.0f &&
                          invMassScale < 1e4f;
        if (!sane) {
            sp.vel[0] = sp.vel[1] = sp.vel[2] = 0.0f;
            static bool warned = false;
            if (!warned) {
                warned = true;
                log::Write("ExtCreateInstance arguments are implausible (vel %.3g %.3g %.3g, "
                           "phase %d, invMassScale %.3g), the assumed signature is probably "
                           "wrong for this build. Set UseEngineSpawnData=0 in FlexRevive.ini "
                           "if the game is unstable.", vx, vy, vz, phase, invMassScale);
            }
        }

        std::lock_guard<std::mutex> lock(s_solverMutex);
        auto it = s_models.find(const_cast<void*>(asset));
        if (it != s_models.end()) {
            sp.radius = it->second.radius;
            sp.gyration = it->second.gyration;
            sp.hullIndex = it->second.hullIndex;
        }

        if (sp.xformFloats > 0 && s_pendingSpawns.size() < 8192)
            s_pendingSpawns.push_back(sp);

        static int logged = 0;
        if (logged < 6) {
            ++logged;
            log::Write("ExtCreateInstance vel=(%.1f, %.1f, %.1f) phase=%d invMassScale=%.3f "
                       "assetRadius=%.2f xform[0..3]=%.1f %.1f %.1f %.1f "
                       "xform[12..14]=%.1f %.1f %.1f", vx, vy, vz, phase, invMassScale,
                       sp.radius, sp.xform[0], sp.xform[1], sp.xform[2], sp.xform[3],
                       sp.xform[12], sp.xform[13], sp.xform[14]);
        }
    }

    if (s_realExtCreateInstance)
        return s_realExtCreateInstance(container, asset, transform, vx, vy, vz, phase,
                                       invMassScale);
    return NewHandle();
}
// flexExtGetParticleData(container, particles, velocities, phases, normals, activeIndices)
//
// Hands the engine pointers into this module's own storage, so what is integrated is what it
// sees.
//
// Six outputs, not five: the sixth is the engine's active particle list, which it fills with
// 0..n-1 as soon as a chunk reports a non-zero particle count. Leaving it unwritten leaves the
// engine writing through whatever the caller had there.
void WINAPI Shim_flexExtGetParticleData(void* handle, float** particles, float** velocities,
                                        int** phases, float** normals, int** activeIndices)
{
    Note("flexExtGetParticleData");
    auto* c = static_cast<Container*>(handle);
    if (!c)
        return;

    std::lock_guard<std::mutex> lock(s_solverMutex);
    if (particles)     *particles     = c->particles.data();
    if (velocities)    *velocities    = c->velocities.data();
    if (phases)        *phases        = c->phases.data();
    if (normals)       *normals       = c->normals.data();
    if (activeIndices) *activeIndices = c->active.data();

    static int logged = 0;
    if (logged < 3) {
        ++logged;
        log::Write("ExtGetParticleData -> our buffers (p=%p v=%p ph=%p n=%p active=%p, "
                   "capacity %d)",
                   (void*)(particles ? *particles : nullptr),
                   (void*)(velocities ? *velocities : nullptr),
                   (void*)(phases ? *phases : nullptr),
                   (void*)(normals ? *normals : nullptr),
                   (void*)(activeIndices ? *activeIndices : nullptr), c->maxParticles);
    }
}
// flexExtSetForceFields(container, forceFields, numForceFields, memory)
//
// Explosions publish these, and they are what throws debris outward. The entry this build
// writes is wider than the published Flex struct and carries fields Flex does not document,
// so it is read by offset rather than by declaring it:
//
//   0, 4, 8  position          16  strength          32  a further scaled value
//        12  radius            20  a second strength 36  an int, always 1
//                              24  mode, always 0    40  linear falloff, a byte, always 1
//                              28  padding
//
// Radius comes from the explosion's own radius and strength from its Force, each scaled by
// the settings entry the engine looked up for that explosion.
constexpr size_t kFieldRadius = 12;
constexpr size_t kFieldStrength = 16;
constexpr size_t kFieldLinearFalloff = 40;
void WINAPI Shim_flexExtSetForceFields(void* container, const void* fields, int numFields,
                                       int)
{
    Note("flexExtSetForceFields");
    (void)container;

    std::lock_guard<std::mutex> lock(s_solverMutex);
    s_forceFields.clear();
    if (!fields || numFields <= 0 || numFields > 256)
        return;

    auto base = static_cast<const uint8_t*>(fields);
    for (int i = 0; i < numFields; ++i) {
        const uint8_t* entry = base + size_t(i) * kForceFieldSize;
        auto f = reinterpret_cast<const float*>(entry);
        ForceField ff;
        ff.pos[0] = f[0];
        ff.pos[1] = f[1];
        ff.pos[2] = f[2];
        memcpy(&ff.radius, entry + kFieldRadius, sizeof(ff.radius));
        memcpy(&ff.strength, entry + kFieldStrength, sizeof(ff.strength));
        // A byte, not a word: the engine stores the flag with a byte write and the three bytes
        // above it are never initialised, so reading wider picks up whatever was on its stack.
        ff.linearFalloff = entry[kFieldLinearFalloff] != 0;

        // Reject anything that does not look like a blast, rather than trusting the layout.
        if (!std::isfinite(ff.pos[0]) || !std::isfinite(ff.radius) || ff.radius <= 0.0f ||
            ff.radius > 100000.0f || !std::isfinite(ff.strength))
            continue;
        s_forceFields.push_back(ff);
    }

    // Reported when a blast begins rather than every frame it lasts. The engine republishes a
    // live explosion's field on every update, so logging each call describes the first
    // explosion of the session several times over and every later one not at all.
    static int logged = 0;
    static bool hadFields = false;
    if (!s_forceFields.empty() && !hadFields && logged < 8) {
        ++logged;
        const ForceField& f = s_forceFields[0];
        log::Write("force field at (%.0f, %.0f, %.0f) radius=%.0f strength=%.1f falloff=%s "
                   "(%d of %d accepted)", f.pos[0], f.pos[1], f.pos[2], f.radius, f.strength,
                   f.linearFalloff ? "linear" : "quadratic", int(s_forceFields.size()),
                   numFields);
    }
    hadFields = !s_forceFields.empty();
}

// Every entry point this plugin takes over, and what it points at instead.
const f4kit::imports::Redirect kReplacements[] = {
    {"flexInit", reinterpret_cast<void*>(&Shim_flexInit)},
    {"flexShutdown", reinterpret_cast<void*>(&Shim_flexShutdown)},
    {"flexAcquireContext", reinterpret_cast<void*>(&Shim_flexAcquireContext)},
    {"flexCreateSolver", reinterpret_cast<void*>(&Shim_flexCreateSolver)},
    {"flexDestroySolver", reinterpret_cast<void*>(&Shim_flexDestroySolver)},
    {"flexUpdateSolver", reinterpret_cast<void*>(&Shim_flexUpdateSolver)},
    {"flexSetParams", reinterpret_cast<void*>(&Shim_flexSetParams)},
    {"flexSetParticles", reinterpret_cast<void*>(&Shim_flexSetParticles)},
    {"flexGetParticles", reinterpret_cast<void*>(&Shim_flexGetParticles)},
    {"flexSetVelocities", reinterpret_cast<void*>(&Shim_flexSetVelocities)},
    {"flexGetVelocities", reinterpret_cast<void*>(&Shim_flexGetVelocities)},
    {"flexSetPhases", reinterpret_cast<void*>(&Shim_flexSetPhases)},
    {"flexSetActive", reinterpret_cast<void*>(&Shim_flexSetActive)},
    {"flexGetActiveCount", reinterpret_cast<void*>(&Shim_flexGetActiveCount)},
    {"flexSetRigids", reinterpret_cast<void*>(&Shim_flexSetRigids)},
    {"flexGetRigidTransforms", reinterpret_cast<void*>(&Shim_flexGetRigidTransforms)},
    {"flexGetBounds", reinterpret_cast<void*>(&Shim_flexGetBounds)},
    {"flexSetShapes", reinterpret_cast<void*>(&Shim_flexSetShapes)},
    {"flexCreateTriangleMesh", reinterpret_cast<void*>(&Shim_flexCreateTriangleMesh)},
    {"flexUpdateTriangleMesh", reinterpret_cast<void*>(&Shim_flexUpdateTriangleMesh)},
    {"flexDestroyTriangleMesh", reinterpret_cast<void*>(&Shim_flexDestroyTriangleMesh)},
    {"flexAlloc", reinterpret_cast<void*>(&Shim_flexAlloc)},
    {"flexFree", reinterpret_cast<void*>(&Shim_flexFree)},
    {"flexSetFence", reinterpret_cast<void*>(&Shim_flexSetFence)},
    {"flexWaitFence", reinterpret_cast<void*>(&Shim_flexWaitFence)},
    {"flexStartRecord", reinterpret_cast<void*>(&Shim_flexStartRecord)},
    {"flexStopRecord", reinterpret_cast<void*>(&Shim_flexStopRecord)},
    {"flexSnapshot", reinterpret_cast<void*>(&Shim_flexSnapshot)},
    {"flexExtCreateContainer", reinterpret_cast<void*>(&Shim_flexExtCreateContainer)},
    {"flexExtDestroyContainer", reinterpret_cast<void*>(&Shim_flexExtDestroyContainer)},
    {"flexExtCreateRigidFromMesh", reinterpret_cast<void*>(&Shim_flexExtCreateRigidFromMesh)},
    {"flexExtCreateInstance", reinterpret_cast<void*>(&Shim_flexExtCreateInstance)},
    {"flexExtDestroyAsset", reinterpret_cast<void*>(&Shim_flexExtDestroyAsset)},
    {"flexExtGetParticleData", reinterpret_cast<void*>(&Shim_flexExtGetParticleData)},
    {"flexExtSetForceFields", reinterpret_cast<void*>(&Shim_flexExtSetForceFields)},
};

// Named so a bug report can show the game reached one, rather than leaving it to be inferred
// from a crash address. Imports without a name arrive by ordinal, with nothing to match on.
void NoteLeftAlone(const char* moduleName, const char* importName, void*)
{
    if (importName)
        log::Write("  UNREPLACED %s!%s, still routed to the original library", moduleName,
                   importName);
    else
        log::Write("  UNREPLACED %s ordinal import, still routed to the original library",
                   moduleName);
}

// What each thunk pointed at before it was rewritten, so a shim that only watches a call can
// hand it straight on.
f4kit::imports::Originals s_originals;

void* OriginalFor(const char* name)
{
    return s_originals.For(name);
}

} // namespace

bool Install()
{
    if (s_installed)
        return true;

    // The spawn hook is the one interception that depends on a function signature rather than
    // a struct layout, so it can be opted out of. Left out here, its thunk keeps pointing at
    // the original and that call is untouched.
    const char* skip[] = {"flexExtCreateInstance"};
    const int skipCount = g_tune.useEngineSpawnData ? 0 : 1;

    const f4kit::imports::Report report =
        f4kit::imports::Patch("flex", kReplacements,
                              int(sizeof(kReplacements) / sizeof(kReplacements[0])), skip,
                              skipCount, s_originals, &NoteLeftAlone, nullptr);
    const int patched = report.patched;
    log::Write("patched %d imports across %d module(s); %d left by name, %d by ordinal",
               report.patched, report.modules, report.unmatched, report.byOrdinal);
    if (patched == 0) {
        log::Write("no Flex imports found to patch");
        return false;
    }

    s_installed = true;
    log::Write("%d Flex entry points redirected, the game cannot enter the "
               "unusable CUDA 7.5 solver", patched);

    // flexExtCreateInstance is intercepted only to read the spawn velocity, so it needs the
    // original to hand the call on to. Without one the engine gets an opaque instance instead.
    s_realExtCreateInstance =
        reinterpret_cast<PFN_ExtCreateInstance>(OriginalFor("flexExtCreateInstance"));
    log::Write("flexExtCreateInstance %s", s_realExtCreateInstance
                   ? "intercepted and forwarded to the original (spawn velocity is readable)"
                   : "not imported by this build, spawn velocity will be inferred");

    log::Write("debris chunks are described natively, the game "
               "never enters the original library for them");

    // Used only to compare particle counts against, never to build what the engine gets.
    s_realExtCreateRigidFromMesh = reinterpret_cast<PFN_ExtCreateRigidFromMesh>(
        OriginalFor("flexExtCreateRigidFromMesh"));
    s_realExtDestroyAsset =
        reinterpret_cast<PFN_ExtDestroyAsset>(OriginalFor("flexExtDestroyAsset"));
    if (!s_realExtCreateRigidFromMesh || !s_realExtDestroyAsset) {
        s_realExtCreateRigidFromMesh = nullptr;
        s_realExtDestroyAsset = nullptr;
    }

    threads::Start(g_tune.solverThreads);

    return true;
}

void ExpectParticleBudget(int tier)
{
    static const int kTierParticles[3] = {6000, 16000, 32768};
    s_expectedBudget = (tier >= 0 && tier <= 2) ? kTierParticles[tier] : 0;
}

bool Installed()
{
    return s_installed;
}

}
