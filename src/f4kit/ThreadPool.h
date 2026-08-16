// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

#include <functional>

// A persistent worker pool for the solver's per-piece work.
//
// Workers sleep on a condition variable between frames rather than spinning, and the pool is
// sized below the core count so the game keeps its own headroom.
//
// The pool is created once and never destroyed. Joining a thread from a static destructor
// would run during DLL_PROCESS_DETACH under the loader lock and deadlock against that
// thread's exit; the process exiting reclaims it instead. Start() is called from
// F4SEPlugin_Load, after LoadLibrary has returned, so thread creation is not itself under
// the loader lock.
namespace f4kit::threads {

// Starts the pool if it is not already running; later calls do nothing. `totalThreads` counts
// the calling thread, so 1 means no workers and everything runs inline, and 0 means size it
// from the machine.
void Start(int totalThreads);

// Extra worker threads, not counting the caller. 0 when the pool is inert.
int Workers();

// Runs body(k) for every k in [0, count) across the workers and the calling thread, returning
// once all have finished. Work is claimed in `grain`-sized chunks from one shared cursor
// rather than split into fixed ranges, since per-piece cost varies by an order of magnitude
// between a chunk sweeping a detailed mesh and one falling through open air.
//
// body must be safe to run concurrently for different k. Nothing here takes the solver lock;
// the caller holds it across the whole call, which is what keeps the state body reads
// immutable for the duration.
void ParallelFor(int count, int grain, const std::function<void(int)>& body);

}
