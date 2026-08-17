// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

#include "gpu/Scene.h"

#include <cstdint>

// The GPU compute backend for the debris step.
//
// Why the whole loop, or none of it
// ---------------------------------
// The obvious design, moving one pass to the card and leaving the rest, does not work here. A
// dispatch costs about a microsecond to submit; reading the results back costs about 0.6 ms on
// a fast card, measured, because it is a pipeline stall rather than a transfer. So any split
// that has to hand positions back to the CPU mid-substep pays that stall several times a frame
// to save arithmetic measured in microseconds, and loses outright.
//
// What works is keeping the state resident: upload once, run every substep as a chain of
// dispatches with nothing read back between them, and read the transforms once at the end of
// the frame. That is what this is arranged to do, and it is why the passes had to be ported
// together rather than one at a time.
//
// World geometry is uploaded separately from the frame, because it changes rarely and is by
// far the largest thing here: a cell's collision meshes run to tens of thousands of triangles,
// against a few hundred pieces.
namespace flexrevive::gpu {

// One collision mesh, in its own local space, as the solver already holds it.
struct MeshUpload {
    const float* verts = nullptr;      // xyz triples
    int vertCount = 0;
    const int* indices = nullptr;      // 3 per triangle
    int triCount = 0;
    float lower[3] = {0, 0, 0};
    float upper[3] = {0, 0, 0};

    // The mesh's triangle index, when it has one. With gridStart null the shader falls back to
    // testing every triangle, which is slower and equally correct.
    const int* gridStart = nullptr;    // one entry per cell plus a terminator
    int gridCellCount = 0;
    const int* gridTris = nullptr;
    int gridTriCount = 0;
    int gridDim[3] = {0, 0, 0};
    float gridOrigin[3] = {0, 0, 0};
    float gridInvCell[3] = {1, 1, 1};
};

// One blast, as the integration reads it.
struct Blast {
    float pos[3] = {0, 0, 0};
    float radius = 0.0f;
    float strength = 0.0f;
    uint32_t linearFalloff = 0;
};

// Everything one frame needs. The piece arrays are the solver's own and are written back in
// place; the rest is read only.
struct Frame {
    float* positions = nullptr;    // xyz per piece, updated in place
    float* velocities = nullptr;   // xyz per piece, updated in place
    float* rotations = nullptr;    // xyzw per piece, updated in place
    float* angular = nullptr;      // xyz per piece, updated in place
    const float* mass = nullptr;
    const float* radii = nullptr;
    uint8_t* resting = nullptr;    // updated in place
    int count = 0;

    const int* stepList = nullptr; // the moving subset
    int stepCount = 0;

    const Blast* blasts = nullptr;
    int blastCount = 0;

    // Tunables and per-step constants, filled by the caller so this stays free of the solver's
    // configuration.
    float gravity[3] = {0, 0, 0};  // already scaled
    float dt = 0.0f;               // one substep
    int substeps = 1;
    float dragBase = 0.0f;
    float maxSpeed = 0.0f;
    float contactSkin = 0.0f;
    float restitution = 0.0f;
    float dynamicFriction = 0.0f;
    float sleepSpeed = 0.0f;
    float noBounceSpeed = 0.0f;
    float rollBlend = 0.0f;
    bool rolling = true;
    // Piece against piece, and the largest collision radius in the pool. The cell size follows
    // the widest piece, so a chunk can only reach something in its own cell or one beside it,
    // which is the property the colouring rests on.
    bool debrisVsDebris = true;
    float widestPiece = 0.0f;
    float pieceFriction = 0.0f;
    float settleRate = 1.0f;
    float heftBounce = 1.0f;
    float spinDamp = 36.0f;
};

bool Start();

// Whether the backend will actually step debris, as opposed to merely having a device.
bool Ready();

// Why Ready is false, for the log. Null when it is true.
const char* NotReadyReason();

// Uploads world geometry, replacing whatever was there. Call when the collision set changes,
// not every frame: this is the expensive one.
//
// `colliders` is already in the shader's layout, since only the caller knows how to read the
// engine's shape block. A collider referring to mesh k must set refs[0] to k.
bool SetWorld(const scene::Collider* colliders, int colliderCount, const MeshUpload* meshes,
              int meshCount, const float* planes, int planeCount);

// Runs `substeps` substeps and writes the results back into the frame's arrays. Returns false,
// having changed nothing, when the backend is unusable or an upload fails, so a caller can fall
// through to the CPU path without checking anything first.
bool StepFrame(const Frame& f);

// How long the last StepFrame spent, in milliseconds, split into the work and the readback.
// The readback is the interesting half: it is the fixed cost the whole design is arranged
// around, and the only way to know it on a given machine is to have measured it there.
void LastTiming(double& outDispatchMs, double& outReadbackMs);

void StopForTesting();

}
