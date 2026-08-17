// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>

// The GPU compute backend for the debris step.
//
// What is here and what is not
// ----------------------------
// This is the first half of the backend. It owns the device, the buffers and the dispatch, and
// it runs the integration pass: gravity, air resistance, force fields and the speed cap. The
// collision passes, which is where the time actually goes, are still the CPU's.
//
// That split is not yet worth enabling, and the backend says so rather than pretending
// otherwise. Every substep would have to end with a readback to hand the positions to the CPU
// collision passes, and a readback costs about 0.6 ms on a fast card against the 0.001 ms the
// dispatch itself takes: the transfer is the whole cost, and paying it several times a frame
// to save a few microseconds of arithmetic is a straight loss. Ready() therefore reports false
// until the collision passes land and the loop can stay resident on the card, reading back
// once a frame.
//
// So ComputeBackend=gpu currently initialises the device, reports what it found, and steps on
// the CPU. It does not silently claim otherwise: that mistake has been made in this plugin
// before, in a GpuSolver setting that chose a backend, logged it, and ran the same CPU code
// either way.
namespace flexrevive::gpu {

// One blast, as the integration pass reads it. Mirrors the ForceField the solver keeps.
struct Blast {
    float pos[3] = {0, 0, 0};
    float radius = 0.0f;
    float strength = 0.0f;
    uint32_t linearFalloff = 0;
};

// Everything one integration substep needs that is not per-piece.
struct StepParams {
    float gravity[3] = {0, 0, 0};   // already scaled by GravityScale
    float dt = 0.0f;
    float dragBase = 0.0f;          // damping * DragScale / max(Heft, 0.05)
    float maxSpeed = 0.0f;          // the engine's cap; 0 means none
};

// Brings up the device and the shaders. Returns whether the backend is usable at all, which is
// weaker than Ready: a device may exist while the backend still declines to run the step.
//
// Safe to call more than once; only the first does the work.
bool Start();

// Whether the backend will actually step debris, as opposed to merely having a device. False
// while the collision passes are still the CPU's, so the solver keeps its own path.
bool Ready();

// Why Ready is false, for the log. Null when it is true.
const char* NotReadyReason();

// Runs one integration substep over `stepList` and writes the results back into `positions`
// and `velocities`, which are the solver's own arrays, indexed by piece.
//
// Returns false, having changed nothing, if the backend is unusable or the upload fails, so a
// caller can fall through to the CPU path without checking anything first.
//
// `count` is the number of pieces, `stepList`/`stepCount` the moving subset, and `mass` one
// entry per piece.
bool IntegrateStep(float* positions, float* velocities, const float* mass, int count,
                   const int* stepList, int stepCount, const Blast* blasts, int blastCount,
                   const StepParams& params);

// Releases everything. For the tests; the plugin holds its device for the life of the process.
void StopForTesting();

}
