#pragma once

namespace flexrevive::config {

// FlexRevive.ini, read from the folder containing the plugin DLL (Data\F4SE\Plugins).
// Written back with full commentary when absent, so the file documents itself.
struct Values {
    // ---- general ----------------------------------------------------------------------
    bool enabled = true;
    // Turn weapon debris on at load even when the user's prefs disable it.
    bool forceEnableWeaponDebris = true;
    bool verboseLog = false;
    // Read spawn velocity and per-chunk size from the engine by intercepting
    // flexExtCreateInstance and forwarding it unchanged. Off infers both and leaves that
    // import alone.
    bool useEngineSpawnData = true;
    // Step from measured elapsed time rather than the fixed 10 ms the engine passes. The
    // engine steps once per rendered frame, so below 100 fps its value advances less physics
    // than a second of real time contains.
    bool realTimestep = true;
    // The engine's debris quality tier, selecting its particle budget, cull distance and
    // neighbour limit. -1 leaves whatever the prefs hold.
    int debrisQuality = 2;
    // Hand the engine chunks that carry real particles, so its iMaxParticles budget bounds
    // how much debris reaches the screen.
    bool engineParticles = true;

    // ---- physics ----------------------------------------------------------------------
    // Multiplies the engine's gravity. At 1.0 the engine's 686.6 units/s^2, over this game's
    // 69.99 units per metre, is 9.81 m/s^2.
    float gravityScale = 1.0f;
    // Air resistance. Above 1 settles debris sooner, below 1 lets it sail further.
    float dragScale = 1.0f;
    // Bounciness on impact. 0 lands debris dead, higher values make it skip.
    float restitutionScale = 1.0f;
    // Grip. Higher shortens the slide on landing.
    float frictionScale = 1.0f;

    // ---- rotation ---------------------------------------------------------------------
    // Radians/sec of spin given to a freshly spawned chunk.
    float spawnSpin = 12.0f;
    // Fallback launch speed in units/sec, used when the engine supplies no velocity.
    // Direction is taken from where the fragment sits within its own burst. 0 drops from rest.
    float spawnBurst = 150.0f;
    // Scales the launch speed the engine gives a freshly created chunk. Independent of
    // ImpactShock, which governs the same shot's effect on rubble already present.
    float spawnVelocityScale = 0.3f;
    // How much off-centre impacts set pieces tumbling. Scales the torque from the friction
    // impulse, which is the only part of a contact that can create spin.
    float impactTorque = 1.0f;
    // Round pieces roll down slopes instead of only sliding.
    bool rolling = true;

    // ---- collision --------------------------------------------------------------------
    // Debris collides with other debris, so chunks pile rather than interpenetrate.
    bool debrisVsDebris = true;
    // Scales every piece's collision radius together. Raise it if debris sinks into surfaces,
    // lower it if debris rests visibly above them.
    float pieceRadiusScale = 1.0f;
    // How quickly chunks resting against each other bleed off relative motion. Higher settles
    // a heap sooner; 0 leaves them grinding indefinitely.
    float settleRate = 1.5f;
    // How hard a burst of fresh debris shoves settled rubble nearby.
    float impactShock = 1.0f;
    // How heavy the debris feels. Divides air resistance, bounce, and how far a chunk is
    // carried when something walks into it, on top of the per-piece mass from particle counts.
    float heft = 1.6f;

    // ---- lifetime ---------------------------------------------------------------------
    // Upper bound on simultaneously simulated pieces. A safety valve, not a quality setting:
    // how much debris spawns is the engine's decision.
    int maxPieces = 32768;

    // Use a GPU compute backend for the solver where the hardware suits it.
    // 0 off, 1 automatic, 2 forced.
    int gpuSolver = 0;

    // ---- performance ------------------------------------------------------------------
    // Threads the per-piece sweep spreads across, counting the game thread. 0 sizes it from
    // the machine, 1 keeps everything on the calling thread. Read once when the pool starts,
    // so a change needs a restart.
    int solverThreads = 0;
};

Values& Get();

// Loads FlexRevive.ini, writing a fully commented default if absent, then applies it.
void Load();

// Pushes the current values into the live solver, taking effect on the next step.
void Apply();

// Directory containing the plugin DLL, with trailing backslash. Stable after first call.
const wchar_t* PluginDir();

}
