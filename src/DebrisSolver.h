#pragma once

// The debris solver: a CPU rigid-body simulation standing in for the GPU particle library
// the engine expects.
//
// Fallout 4 drives weapon debris through a C API imported from flexRelease_x64.dll and
// flexExtRelease_x64.dll. This redirects the game's import table for both into the
// implementations here and simulates the debris itself. The engine keeps spawning, mesh
// rasterisation and rendering; what it needs back is integrated positions and orientations.
//
// The import table is patched by name, so nothing here is tied to a game build.
namespace flexrevive::solver {

// Redirects the debris imports. Patches the import table rather than the exports, so it may
// be called before or after those DLLs load.
bool Install();

bool Installed();

// Solver tuning, driven by FlexRevive.ini and read afresh on each step.
struct Tunables {
    float gravityScale = 1.0f;       // multiplies the engine's own gravity
    float dragScale = 1.0f;          // multiplies the engine's damping
    float restitutionScale = 1.0f;   // bounciness
    float frictionScale = 1.0f;      // grip
    float spawnSpin = 12.0f;         // radians/sec given to a new chunk
    float spawnBurst = 150.0f;       // fallback launch speed, units/sec
    float spawnVelocityScale = 0.3f; // scales the launch speed the engine asks for
    float impactTorque = 1.0f;       // how much off-centre hits set pieces spinning
    bool rolling = true;             // round pieces roll instead of only sliding
    bool debrisVsDebris = true;      // chunks collide with each other
    float pieceRadiusScale = 1.0f;   // scales the shared collision radius every piece uses
    float settleRate = 1.5f;        // how fast a heap of rubble stops jostling itself
    float impactShock = 1.0f;        // how hard a fresh burst shoves the rubble around it
    float heft = 1.6f;               // how heavy the debris feels: less drag, less bounce
    int maxPieces = 32768;           // ceiling on simultaneously simulated pieces
    // Threads the per-piece sweep runs across, counting the game thread. 0 sizes it from the
    // machine. Read once when the pool starts, so a change needs a restart.
    int solverThreads = 0;
    // GPU backend: 0 off, 1 automatic, 2 forced.
    int gpuSolver = 0;
    // Intercept flexExtCreateInstance for the engine's spawn velocity and chunk size. Off
    // infers both and leaves that import untouched.
    bool useEngineSpawnData = true;
    // Step from the clock rather than the fixed value the engine passes.
    bool realTimestep = true;
    // Hand the engine chunks carrying real particles, so its budget bounds the debris.
    bool engineParticles = true;
};
extern Tunables g_tune;

// Records the configured quality tier, so the size the engine allocates can be checked
// against it when the particle container appears.
void ExpectParticleBudget(int tier);

}
