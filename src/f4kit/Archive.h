// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

// Mounts one of the game's own BA2 archives after startup has passed.
//
// The engine mounts most archives from the lists in Fallout4.ini, but a few are mounted from
// code, behind a condition tested once during startup. An archive whose condition was false at
// that moment is skipped for the rest of the session, and every file in it is simply absent.
//
// Nothing here is hard-coded to a build. The archive's name is a string literal in the image,
// referenced by the call site that mounts it, so the mount routine is reached from the name
// rather than from an address.
namespace f4kit::archive {

// Mounts `archiveName` the way the engine would have. `archiveName` must be a name the engine
// itself mounts by literal, since that call site is what locates the routine.
//
// Returns false, having done nothing, when the call site cannot be found or does not look like
// one. Safe to call when the archive is already mounted, though there is no reason to.
bool Mount(const char* archiveName);

}
