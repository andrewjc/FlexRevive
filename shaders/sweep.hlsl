// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

// One substep of a moving piece against the world: integrate it, sweep where it travelled, and
// land it on the first surface it crossed.
//
// This is the pass the frame time actually goes into. Measured in a real cell: of 0.565 ms per
// frame across the whole solver, 0.322 ms was here, against 0.131 for piece-versus-piece and
// hundredths for everything else. It is also the pass that suits a card best, because pieces
// are independent of one another for its whole duration.
//
// It is a transcription of stepPiece in DebrisSolver.cpp and has to stay one. Where the two
// disagree the GPU is wrong, and tests/test_gpu.cpp runs both over the same scene rather than
// trusting that the arithmetic below was copied faithfully.

#include "common.hlsli"

StructuredBuffer<uint> gStepList : register(t0);
StructuredBuffer<Blast> gBlasts : register(t1);
StructuredBuffer<Collider> gColliders : register(t2);
StructuredBuffer<MeshDesc> gMeshes : register(t3);
StructuredBuffer<float4> gMeshVerts : register(t4);   // xyz used, w padding
StructuredBuffer<uint> gMeshIndices : register(t5);
StructuredBuffer<uint> gGridStart : register(t6);
StructuredBuffer<uint> gGridTris : register(t7);
StructuredBuffer<float4> gPlanes : register(t8);

RWStructuredBuffer<Piece> gPieces : register(u0);

// A triangle's vertices, from the shared arrays.
void TriVerts(MeshDesc m, uint tri, out float3 v0, out float3 v1, out float3 v2)
{
    const uint base = m.offsets.y + tri * 3;
    const uint vbase = m.offsets.x;
    v0 = gMeshVerts[vbase + gMeshIndices[base + 0]].xyz;
    v1 = gMeshVerts[vbase + gMeshIndices[base + 1]].xyz;
    v2 = gMeshVerts[vbase + gMeshIndices[base + 2]].xyz;
}

// collision::SweepMesh, in the mesh's own space. Where the mesh carries a grid only the cells
// the segment passes through are visited; a triangle listed in several cells may be tested
// more than once, which is harmless because the test is pure and the best distance only
// shrinks.
bool SweepMesh(uint meshIndex, float3 from, float3 dir, float maxDist, float skin,
               out float outDist, out float3 outNormal)
{
    outDist = maxDist;
    outNormal = float3(0, 0, 1);

    const MeshDesc m = gMeshes[meshIndex];
    if (m.counts.y == 0)
        return false;

    // Cheap reject against the mesh's own bounds, before a triangle is touched.
    const float3 to = from + dir * maxDist;
    const float3 lo = min(from, to) - skin;
    const float3 hi = max(from, to) + skin;
    if (any(hi < m.lower.xyz) || any(lo > m.upper.xyz))
        return false;

    float best = maxDist;
    bool hit = false;
    float3 bestN = float3(0, 0, 1);

    if (m.gridOrigin.w == 0.0f) {
        for (uint t = 0; t < m.counts.y; ++t) {
            float3 v0, v1, v2;
            TriVerts(m, t, v0, v1, v2);
            float hitT;
            float3 n;
            if (SegmentHitsTriangle(from, dir, best, v0, v1, v2, hitT, n)) {
                best = hitT;
                bestN = n;
                hit = true;
            }
        }
    } else {
        const int3 dim = int3(m.gridDim.xyz);
        const int3 cLo = clamp(int3((lo - m.gridOrigin.xyz) * m.gridInvCell.xyz), int3(0, 0, 0),
                               dim - 1);
        const int3 cHi = clamp(int3((hi - m.gridOrigin.xyz) * m.gridInvCell.xyz), int3(0, 0, 0),
                               dim - 1);
        for (int z = cLo.z; z <= cHi.z; ++z)
            for (int y = cLo.y; y <= cHi.y; ++y)
                for (int x = cLo.x; x <= cHi.x; ++x) {
                    const uint cell = uint((z * dim.y + y) * dim.x + x);
                    const uint e0 = gGridStart[m.offsets.z + cell];
                    const uint e1 = gGridStart[m.offsets.z + cell + 1];
                    for (uint e = e0; e < e1; ++e) {
                        float3 v0, v1, v2;
                        TriVerts(m, gGridTris[m.offsets.w + e], v0, v1, v2);
                        float hitT;
                        float3 n;
                        if (SegmentHitsTriangle(from, dir, best, v0, v1, v2, hitT, n)) {
                            best = hitT;
                            bestN = n;
                            hit = true;
                        }
                    }
                }
    }

    if (hit) {
        outDist = best;
        outNormal = bestN;
    }
    return hit;
}

[numthreads(64, 1, 1)]
void main(uint3 tid : SV_DispatchThreadID)
{
    if (tid.x >= gStepCount)
        return;

    const uint i = gStepList[tid.x];
    Piece p = gPieces[i];

    // A retired slot can hold anything, and the CPU path drops those before stepping.
    if (!isfinite(p.posRadius.x))
        return;

    const float3 from = p.posRadius.xyz;
    const float mass = p.velMass.w;

    // ---- integrate ------------------------------------------------------------------------
    p.velMass.xyz = (p.velMass.xyz + gGravity * gDt) * DragFactor(mass);

    for (uint b = 0; b < gBlastCount; ++b) {
        const Blast f = gBlasts[b];
        const float3 d = p.posRadius.xyz - f.posRadius.xyz;
        const float dist2 = dot(d, d);
        const float radius = f.posRadius.w;
        if (dist2 > radius * radius || dist2 < 1e-6f)
            continue;
        const float dist = sqrt(dist2);
        const float falloff = f.strength.y != 0.0f ? (1.0f - dist / radius)
                                                   : (1.0f - dist2 / (radius * radius));
        p.velMass.xyz += (d / dist) * f.strength.x * falloff * gDt;
    }

    if (gMaxSpeed > 0.0f) {
        const float sp = length(p.velMass.xyz);
        if (sp > gMaxSpeed)
            p.velMass.xyz *= gMaxSpeed / sp;
    }

    p.posRadius.xyz += p.velMass.xyz * gDt;

    // Tumble while airborne.
    p.rot = QuatIntegrate(p.rot, p.angGyr.xyz, gDt);

    // ---- sweep ----------------------------------------------------------------------------
    const float3 delta = p.posRadius.xyz - from;
    const float dist = length(delta);
    if (dist < 1e-5f) {
        gPieces[i] = p;
        return;
    }
    const float3 dir = delta / dist;

    // Swept as a sphere: the ray runs one clearance past where the centre travels, so a
    // surface registers while the piece is still a radius away. Testing the bare centre path
    // would need the piece to fall its whole radius before each contact.
    const float pieceRadius = p.posRadius.w;
    const float clearance = pieceRadius + gContactSkin;
    const float sweepLen = dist + clearance;

    float bestT = sweepLen;
    float3 bestN = float3(0, 0, 1);
    bool hit = false;

    for (uint c = 0; c < gColliderCount; ++c) {
        const Collider sh = gColliders[c];
        if (sh.refs.x == 0xFFFFFFFFu)
            continue;   // not a triangle mesh; handled by the primitive pass

        // Mesh data is local space, so the movement is transformed into the collider's frame
        // rather than transforming thousands of vertices into the world every step.
        const float3 localFrom = QuatRotateInv(sh.rot, from - sh.posType.xyz);
        const float3 localDir = QuatRotateInv(sh.rot, dir);

        float hitT;
        float3 localN;
        if (SweepMesh(sh.refs.x, localFrom, localDir, bestT, gContactSkin, hitT, localN)) {
            bestT = hitT;
            bestN = QuatRotate(sh.rot, localN);   // the normal back into world space
            hit = true;
        }
    }

    if (hit) {
        // Land the piece on the nearest surface the sweep crossed. The ray runs a clearance
        // past where the centre travels, so the surface sits at bestT and the centre stops a
        // clearance short of it.
        const float travel = max(0.0f, min(dist, bestT - clearance));
        p.posRadius.xyz = from + dir * travel;

        // Resolve the normal component: a little bounce, the rest of the approach removed.
        const float vn = dot(p.velMass.xyz, bestN);
        if (vn < 0.0f) {
            const float bounce = abs(vn) < gSleep.y ? 0.0f : gRestitution;
            p.velMass.xyz -= (1.0f + bounce) * vn * bestN;

            // Coulomb friction on what is left tangentially, capped against the normal
            // impulse so a light touch scrubs lightly.
            const float3 tangent = p.velMass.xyz - dot(p.velMass.xyz, bestN) * bestN;
            const float tlen = length(tangent);
            if (tlen > 1e-5f) {
                const float scrub = min(gFriction.x * abs(vn) * (1.0f + bounce), tlen);
                p.velMass.xyz -= (tangent / tlen) * scrub;

                // The sideways part of a contact is the only part that can spin a piece, since
                // an impulse through the centre of mass produces no torque.
                if (gMore.z != 0) {
                    const float inv = 1.0f / max(pieceRadius, 1e-3f);
                    p.angGyr.xyz += cross(bestN, tangent) * inv * gSleep.z;
                }
            }
        }
    }

    gPieces[i] = p;
}
