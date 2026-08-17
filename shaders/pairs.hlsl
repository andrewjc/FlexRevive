// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

// Piece against piece, one colour of the cell partition per dispatch.
//
// Unlike the sweep, this writes both sides of every pair, so it cannot simply be handed out per
// piece: two threads resolving overlapping pairs would fight over the same chunks. The CPU path
// solves that by colouring cells on their coordinates modulo three, and this reuses the same
// partition. Two cells of one colour differ by three on some axis, so the blocks of twenty-seven
// they reach into do not overlap and their pieces cannot meet; the host dispatches the colours
// one after another, which is what keeps each pair seeing what the pairs before it left behind.
//
// The unit of work is a cell, not a piece. Colouring separates cells; two pieces sitting in one
// cell share their whole neighbourhood, so a cell's pieces are resolved by a single thread in a
// fixed order. Getting that wrong is what the CPU tests caught, and it would be a data race
// here rather than merely a different answer.
//
// A transcription of pairs::Resolve and the loop around it in DebrisSolver.cpp.

#include "common.hlsli"

// One cell's worth of work: where its pieces start in gCellPieces, and how many there are.
struct CellRun {
    uint4 range;   // x start, y count, zw spare
};

StructuredBuffer<CellRun> gRuns : register(t0);      // the runs of the colour being dispatched
StructuredBuffer<uint> gCellPieces : register(t1);   // piece indices, grouped by cell
StructuredBuffer<uint> gGridHead : register(t2);     // spatial hash: first piece in a bucket
StructuredBuffer<uint> gGridNext : register(t3);     // and the next in the same bucket
StructuredBuffer<int4> gPieceCell : register(t4);    // each piece's cell, w non-zero if indexed

RWStructuredBuffer<Piece> gPieces : register(u0);
RWStructuredBuffer<uint> gSupported : register(u1);  // set where the pile holds a piece up

// Matches grid::kBuckets.
static const uint kBuckets = 8192;

uint BucketOf(int3 c)
{
    const uint h = uint(c.x) * 73856093u ^ uint(c.y) * 19349663u ^ uint(c.z) * 83492791u;
    return h & (kBuckets - 1);
}

// response::SeparationSplit: the lighter piece yields more, and the two shares sum to one.
void SeparationSplit(float massI, float massJ, out float shareI, out float shareJ)
{
    const float mi = max(massI, 1.0f);
    const float mj = max(massJ, 1.0f);
    const float total = mi + mj;
    shareI = mj / total;
    shareJ = mi / total;
}

// pairs::Resolve. Both bodies are read and written, which is why only one thread may hold a
// cell. `sep` runs from j toward i and `dist` is the distance between their centres.
void ResolvePair(uint i, uint j, float3 sep, float dist, float extentI, float extentJ)
{
    Piece a = gPieces[i];
    Piece b = gPieces[j];

    const float minDist = extentI + extentJ + gContactSkin;
    if (!(dist < minDist))
        return;

    const float overlap = minDist - dist;

    // How much overlap counts as settled rather than as a collision. A piece parks as soon as
    // its correction drops below this rather than at exactly zero separation.
    const float settleGap = max(gContactSkin, 0.25f * min(extentI, extentJ));

    const float wake2 = gSleepSpeed * gSleepSpeed;
    const bool inMotion = dot(a.velMass.xyz, a.velMass.xyz) > wake2 ||
                          dot(b.velMass.xyz, b.velMass.xyz) > wake2;

    float shareA, shareB;
    SeparationSplit(a.velMass.w, b.velMass.w, shareA, shareB);
    // pairs::kDefaultRelaxation. A piece in a heap is touched once per overlapping neighbour
    // and every correction lands in the same pass, so taking a share converges over the
    // substeps instead of packing the cluster with stored displacement.
    const float relax = 0.5f;
    a.posRadius.xyz += sep * overlap * shareA * relax;
    b.posRadius.xyz -= sep * overlap * shareB * relax;

    // Chunk on chunk loses spin and relative motion just as it would to the ground. A settled
    // stack exchanges almost no impulse, so these are rates rather than proportional to one.
    const float mu = max(gFriction.y * gFriction.z, 0.0f);
    const float keep = max(0.0f, 1.0f - mu * gSleep.w * gDt);
    a.angGyr.xyz *= keep;
    b.angGyr.xyz *= keep;

    // Pulled toward the velocity of their shared centre of mass, so their common motion is
    // untouched and only the grinding between them is removed. Weighting by mass is what makes
    // this conserve momentum: blending toward the plain average would let a light chunk drag a
    // heavy one along with it.
    const float ma = max(a.velMass.w, 1.0f);
    const float mb = max(b.velMass.w, 1.0f);
    const float total = ma + mb;
    const float blend = min(0.5f, mu * 14.0f * gDt);   // pairs::kDefaultLinearDamp
    const float3 vcm = (ma * a.velMass.xyz + mb * b.velMass.xyz) / total;
    a.velMass.xyz += (vcm - a.velMass.xyz) * blend;
    b.velMass.xyz += (vcm - b.velMass.xyz) * blend;

    // The separation runs from b toward a, so pointing against gravity puts a on top.
    //
    // Only a piece at rest holds another one up. A chunk lying on one that is itself falling is
    // not standing on anything, and treating it as though it were lets a cluster of falling
    // debris support itself in mid-air.
    const float gl = length(gGravity);
    if (gl > 1e-3f) {
        const float up = -dot(sep, gGravity) / gl;
        if (up > 0.5f && b.state.x != 0)
            gSupported[i] = 1;
        else if (up < -0.5f && a.state.x != 0)
            gSupported[j] = 1;
    }

    bool wake = overlap > settleGap && (inMotion || overlap > 2.0f * settleGap);

    const float approach = dot(a.velMass.xyz - b.velMass.xyz, sep);
    if (approach < 0.0f) {
        // Exchange the closing velocity, damped by restitution and shared by mass, so the
        // heavier piece barely notices.
        const float impulse = -approach * (1.0f + max(gRestitution * gFriction.w, 0.0f)) / total;
        a.velMass.xyz += sep * impulse * mb;
        b.velMass.xyz -= sep * impulse * ma;
        if (abs(approach) > gSleepSpeed)
            wake = true;
    }

    if (wake) {
        a.state.x = 0;
        b.state.x = 0;
    }

    gPieces[i] = a;
    gPieces[j] = b;
}

[numthreads(64, 1, 1)]
void main(uint3 tid : SV_DispatchThreadID)
{
    if (tid.x >= gCounts.x)   // run count for this colour, passed in gCounts.x
        return;

    const CellRun run = gRuns[tid.x];

    // Every piece of this cell, in the order the host laid them out, so the result does not
    // depend on how the work happened to be shared out.
    for (uint k = 0; k < run.range.y; ++k) {
        const uint i = gCellPieces[run.range.x + k];
        const Piece pi = gPieces[i];
        if (!isfinite(pi.posRadius.x))
            continue;

        const int4 cellI = gPieceCell[i];
        if (cellI.w == 0)
            continue;

        // The piece's own cell and the twenty-six around it. Buckets are shared between distant
        // cells, so a candidate may be nowhere near; dropping those on the cell rather than on
        // the distance is what keeps a piece from reading a neighbour another colour owns.
        for (int ox = -1; ox <= 1; ++ox)
        for (int oy = -1; oy <= 1; ++oy)
        for (int oz = -1; oz <= 1; ++oz) {
            const uint bucket = BucketOf(cellI.xyz + int3(ox, oy, oz));
            for (uint j = gGridHead[bucket]; j != 0xFFFFFFFFu; j = gGridNext[j]) {
                if (j == i)
                    continue;

                // A pair of moving pieces is reached from both sides; the lower index takes it.
                const Piece pj = gPieces[j];
                if (pj.state.z != 0 && j < i)
                    continue;
                if (!isfinite(pj.posRadius.x))
                    continue;

                const int4 cellJ = gPieceCell[j];
                if (cellJ.w != 0 && any(abs(cellI.xyz - cellJ.xyz) > 1))
                    continue;

                const float3 d = pi.posRadius.xyz - pj.posRadius.xyz;
                const float dist2 = dot(d, d);
                const float bound = pi.posRadius.w + pj.posRadius.w + gContactSkin;
                if (dist2 >= bound * bound || dist2 < 1e-8f)
                    continue;

                const float dist = sqrt(dist2);
                ResolvePair(i, j, d / dist, dist, pi.posRadius.w, pj.posRadius.w);
            }
        }
    }
}
