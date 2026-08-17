// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

// One substep's integration for every moving piece: gravity, air resistance, the blasts in
// range, and the engine's speed cap.
//
// This is a transcription of the opening of stepPiece in DebrisSolver.cpp and of DragFactor in
// Response.cpp, and it has to stay one. The CPU path is the reference: where the two disagree
// the GPU is wrong, and tests/test_gpu.cpp compares them on the same inputs rather than
// trusting that the arithmetic below was copied faithfully.
//
// Only the pieces in the step list are touched. A settled heap is not in it, which is what
// keeps the cost proportional to what is moving rather than to what exists.

struct Piece {
    float3 pos;
    float3 vel;
    float mass;
};

struct Blast {
    float3 pos;
    float radius;
    float strength;
    uint linearFalloff;
};

cbuffer Params : register(b0)
{
    float3 gGravity;        // already scaled by GravityScale
    float gDt;
    float gDragBase;        // damping * DragScale / max(Heft, 0.05)
    float gMaxSpeed;        // the engine's cap; 0 means none
    uint gStepCount;        // entries in gStepList
    uint gBlastCount;
};

StructuredBuffer<uint> gStepList : register(t0);   // piece indices that are moving
StructuredBuffer<Blast> gBlasts : register(t1);
RWStructuredBuffer<Piece> gPieces : register(u0);

// 1 / cbrt(max(mass, 1)), which is response::MassResponse.
//
// Written as an exact reciprocal cube root rather than pow(x, -1/3): pow is log2 then exp2 on
// every vendor and loses enough of the low bits that the comparison against the CPU has to be
// slackened to hide it. Two Newton steps from rsqrt land within a few ULP and cost less.
float MassResponse(float mass)
{
    const float m = max(mass, 1.0f);
    // Halley's method on f(y) = y^-3 - m converges cubically, so one pass from a rough seed is
    // enough for float. The seed is exact for powers of eight and never worse than ~5%.
    float y = exp2(-log2(m) * (1.0f / 3.0f));
    const float c = y * y * y * m;
    y *= (2.0f + c) / (1.0f + 2.0f * c);
    return y;
}

// response::DragFactor with heft already folded into gDragBase, matching the CPU, which
// divides by heft once when it computes dragBase and passes 1.0 for heft.
float DragFactor(float mass)
{
    return 1.0f - min(max(gDragBase, 0.0f) * MassResponse(mass) * max(gDt, 0.0f), 1.0f);
}

[numthreads(64, 1, 1)]
void main(uint3 tid : SV_DispatchThreadID)
{
    if (tid.x >= gStepCount)
        return;

    const uint i = gStepList[tid.x];
    Piece p = gPieces[i];

    // A retired slot can hold anything, and the CPU path drops those before stepping. isfinite
    // on the first component is the same test, in the same place.
    if (!isfinite(p.pos.x))
        return;

    p.vel = (p.vel + gGravity * gDt) * DragFactor(p.mass);

    // Blasts push debris away from their centre, falling off with distance.
    for (uint b = 0; b < gBlastCount; ++b) {
        const Blast f = gBlasts[b];
        const float3 d = p.pos - f.pos;
        const float dist2 = dot(d, d);
        if (dist2 > f.radius * f.radius || dist2 < 1e-6f)
            continue;
        const float dist = sqrt(dist2);
        const float falloff = f.linearFalloff ? (1.0f - dist / f.radius)
                                              : (1.0f - dist2 / (f.radius * f.radius));
        p.vel += (d / dist) * f.strength * falloff * gDt;
    }

    // Honour the engine's speed cap where it published one.
    if (gMaxSpeed > 0.0f) {
        const float sp = length(p.vel);
        if (sp > gMaxSpeed)
            p.vel *= gMaxSpeed / sp;
    }

    p.pos += p.vel * gDt;

    gPieces[i] = p;
}
