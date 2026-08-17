// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

// Parks a piece the pile is holding up.
//
// Every other sleep test in this solver lives inside a world contact, which a piece buried in a
// heap never reaches: it is held off the ground by its neighbours and never touches geometry
// again. Without this pass a heap grinds against itself indefinitely.
//
// Runs after the pair pass, which is what fills gSupported, and reads it rather than deciding
// support for itself. Pieces are independent here, so it is one thread each.

#include "common.hlsli"

StructuredBuffer<uint> gSupported : register(t0);

RWStructuredBuffer<Piece> gPieces : register(u0);

[numthreads(64, 1, 1)]
void main(uint3 tid : SV_DispatchThreadID)
{
    if (tid.x >= gCounts.x)
        return;

    const uint i = tid.x;
    Piece p = gPieces[i];

    if (gSupported[i] == 0 || p.state.x != 0)
        return;

    const float sp = dot(p.velMass.xyz, p.velMass.xyz);
    const float spin = dot(p.angGyr.xyz, p.angGyr.xyz);
    const float r = p.posRadius.w;

    // Spin is compared against the speed its rim would carry, so a large piece has to be
    // turning more slowly than a small one to count as still.
    if (sp < gSleepSpeed * gSleepSpeed && spin * r * r < gSleepSpeed * gSleepSpeed) {
        p.velMass.xyz = float3(0, 0, 0);
        p.angGyr.xyz = float3(0, 0, 0);
        p.state.x = 1;
        gPieces[i] = p;
    }
}
