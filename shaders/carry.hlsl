// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

// Carries each moving piece's particles along with it.
//
// These buffers belong to the plugin but the engine reads them, and what it reads has to
// describe the same debris the transforms do. A particle advanced under its own gravity has
// nothing to stop it against the world, so it keeps falling after the piece it belongs to has
// landed and the two end up thousands of units apart.
//
// Last in the substep, so it sees where every piece finished and what it finished doing:
// pieces are still being moved by their neighbours after the world sweep, and one held up by
// the pile is put to sleep later still.
//
// Safe to run per piece because no two pieces own the same container slot, which the host
// checks when it builds the runs and refuses to dispatch this if it ever fails.

#include "common.hlsli"

struct SlotRun {
    uint4 range;   // x first entry in gPieceSlots, y how many, zw spare
};

StructuredBuffer<uint> gStepList : register(t0);
StructuredBuffer<SlotRun> gSlotRuns : register(t1);   // per piece
StructuredBuffer<uint> gPieceSlots : register(t2);    // container slots, grouped by piece
StructuredBuffer<float4> gPrePos : register(t3);      // where each stepped piece began, xyz
StructuredBuffer<Piece> gPieces : register(t4);

RWStructuredBuffer<float4> gParticles : register(u0);   // xyz position, w inverse mass
RWStructuredBuffer<float4> gVelocities : register(u1);  // xyz velocity, w unused

[numthreads(64, 1, 1)]
void main(uint3 tid : SV_DispatchThreadID)
{
    if (tid.x >= gStepCount)
        return;

    const uint piece = gStepList[tid.x];
    const Piece p = gPieces[piece];
    if (!isfinite(p.posRadius.x))
        return;

    // Exactly the distance the piece turned out to travel, rather than an integration of its
    // own: the piece is the thing that was stopped by the world, and the particles are attached
    // to it.
    const float3 delta = p.posRadius.xyz - gPrePos[tid.x].xyz;

    const SlotRun run = gSlotRuns[piece];
    for (uint e = 0; e < run.range.y; ++e) {
        const uint slot = gPieceSlots[run.range.x + e];

        // A slot the engine has retired holds an inverse mass of zero and a position that means
        // nothing; the CPU path skips those and so does this.
        float4 pos = gParticles[slot];
        if (pos.w == 0.0f)
            continue;

        pos.xyz += delta;                       // the fourth component is inverse mass
        gParticles[slot] = pos;
        gVelocities[slot] = float4(p.velMass.xyz, gVelocities[slot].w);
    }
}
