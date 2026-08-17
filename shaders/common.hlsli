// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

// The state the debris solver keeps on the card, and the small routines every kernel shares.
//
// Layout discipline
// -----------------
// Every struct here is built from float4 and uint4 and nothing else. That is not a style
// preference, it is the fix for a whole class of bug: a structured buffer packs its members
// tightly while a constant buffer pushes any vector that would straddle a sixteen-byte
// register into the next one, so the same declaration means two different things depending on
// where it is bound. The first version of this backend assumed the constant-buffer rule for a
// structured buffer and read every field of every piece four bytes late, which the CPU
// comparison caught only because it existed. A struct of float4s is aligned identically under
// both rules and matches an array of float[4] in C++ with nothing left to reason about.
//
// The cost is naming: values are packed into the spare lanes of a vector they have nothing to
// do with. Each one says where it lives.

#ifndef FLEXREVIVE_COMMON_HLSLI
#define FLEXREVIVE_COMMON_HLSLI

// ---- state ---------------------------------------------------------------------------------

struct Piece {
    float4 posRadius;   // xyz world position, w collision radius
    float4 velMass;     // xyz velocity, w mass
    float4 rot;         // orientation, xyzw quaternion
    float4 angGyr;      // xyz angular velocity, w radius of gyration
    uint4 state;        // x resting, y hull index or ~0, z stepping, w spare
};

// A placed piece of world geometry. Meshes carry an index into the mesh table; hulls carry a
// run of planes; primitives carry their dimensions.
struct Collider {
    float4 posType;     // xyz position, w the kind, as a float so the vector stays uniform
    float4 rot;         // orientation
    float4 dimsRadius;  // xyz half extents or radius/half height, w primitive radius
    float4 prevMoved;   // xyz previous position, w non-zero when it travelled this frame
    float4 aabbLo;      // xyz lower bound, w non-zero when the bounds are meaningful
    float4 aabbHi;      // xyz upper bound, w unused
    uint4 refs;         // x mesh index or ~0, y plane start, z plane count, w spare
};

// One uploaded collision mesh: where its vertices, indices and grid live in the shared arrays.
struct MeshDesc {
    float4 lower;       // xyz lower bound, w unused
    float4 upper;       // xyz upper bound, w unused
    float4 gridOrigin;  // xyz grid origin, w non-zero when the grid is usable
    float4 gridInvCell; // xyz reciprocal cell size, w unused
    uint4 gridDim;      // xyz cells per axis, w unused
    uint4 offsets;      // x vertex base, y index base, z grid start base, w grid triangle base
    uint4 counts;       // x vertex count, y triangle count, z spare, w spare
};

struct Blast {
    float4 posRadius;   // xyz centre, w radius
    float4 strength;    // x strength, y non-zero for linear falloff, zw spare
};

// ---- parameters ----------------------------------------------------------------------------
//
// A constant buffer, so this one really is packed into sixteen-byte registers. Kept to whole
// float4s for the same reason as everything else.
cbuffer Params : register(b0)
{
    float4 gGravityDt;      // xyz gravity, already scaled, w the substep
    float4 gDragMaxSpeed;   // x drag base, y engine speed cap, z contact skin, w restitution
    float4 gFriction;       // x dynamic friction, y piece friction, z settle rate, w heft bounce
    float4 gSleep;          // x sleep speed, y static friction speed, z roll blend, w spin damp
    uint4 gCounts;          // x pieces, y step list length, z colliders, w blasts
    uint4 gMore;            // x mesh count, y substeps, z rolling, w spare
    // Set per dispatch rather than per frame: the pair pass runs once for each
    // colour, over a different span of the same run array each time.
    uint4 gPass;            // x runs in this dispatch, y where they start, zw spare
};

#define gGravity      gGravityDt.xyz
#define gDt           gGravityDt.w
#define gDragBase     gDragMaxSpeed.x
#define gMaxSpeed     gDragMaxSpeed.y
#define gContactSkin  gDragMaxSpeed.z
#define gRestitution  gDragMaxSpeed.w
#define gSleepSpeed   gSleep.x
#define gStepCount    gCounts.y
#define gColliderCount gCounts.z
#define gBlastCount   gCounts.w

// Collider kinds, matching flexrevive::collision::ColliderKind.
static const uint kSphere = 0;
static const uint kCapsule = 1;
static const uint kBox = 2;
static const uint kConvexMesh = 3;
static const uint kTriangleMesh = 4;

// ---- maths ---------------------------------------------------------------------------------

// 1 / cbrt(max(m, 1)), which is response::MassResponse.
//
// A refined reciprocal cube root rather than pow(m, -1/3): pow is log2 then exp2 on every
// vendor and loses enough low bits that the comparison against the CPU has to be slackened to
// hide it. One Halley step from the same seed converges cubically and lands within a few ULP.
float MassResponse(float mass)
{
    const float m = max(mass, 1.0f);
    float y = exp2(-log2(m) * (1.0f / 3.0f));
    const float c = y * y * y * m;
    return y * (2.0f + c) / (1.0f + 2.0f * c);
}

float DragFactor(float mass)
{
    return 1.0f - min(max(gDragBase, 0.0f) * MassResponse(mass) * max(gDt, 0.0f), 1.0f);
}

// Rotates v by the quaternion q, and by its conjugate. Same identity the CPU uses in Math3D.
float3 QuatRotate(float4 q, float3 v)
{
    const float3 u = q.xyz;
    const float3 t = 2.0f * cross(u, v);
    return v + q.w * t + cross(u, t);
}

float3 QuatRotateInv(float4 q, float3 v)
{
    return QuatRotate(float4(-q.xyz, q.w), v);
}

// Advances an orientation by an angular velocity, then renormalises. Matches QuatIntegrate.
float4 QuatIntegrate(float4 q, float3 w, float dt)
{
    const float4 dq = 0.5f * dt * float4(
        w.x * q.w + w.y * q.z - w.z * q.y,
        w.y * q.w + w.z * q.x - w.x * q.z,
        w.z * q.w + w.x * q.y - w.y * q.x,
       -w.x * q.x - w.y * q.y - w.z * q.z);
    const float4 r = q + dq;
    const float len = length(r);
    return len > 1e-8f ? r / len : float4(0, 0, 0, 1);
}

// Möller-Trumbore, transcribed from collision::SegmentHitsTriangle including its unscaled
// barycentric tests: the divide is paid only by a triangle actually hit, and nearly every
// triangle a sweep visits is rejected.
bool SegmentHitsTriangle(float3 from, float3 dir, float maxDist, float3 v0, float3 v1,
                         float3 v2, out float outDist, out float3 outNormal)
{
    outDist = 0.0f;
    outNormal = float3(0, 0, 1);

    const float3 e1 = v1 - v0;
    const float3 e2 = v2 - v0;
    const float3 pv = cross(dir, e2);
    const float det = dot(e1, pv);
    if (abs(det) < 1e-8f)
        return false;

    const float3 tv = from - v0;
    const float u = dot(tv, pv);
    if (det > 0.0f) { if (u < 0.0f || u > det) return false; }
    else            { if (u > 0.0f || u < det) return false; }

    const float3 qv = cross(tv, e1);
    const float v = dot(dir, qv);
    if (det > 0.0f) { if (v < 0.0f || u + v > det) return false; }
    else            { if (v > 0.0f || u + v < det) return false; }

    const float t = dot(e2, qv) / det;
    if (t < 0.0f || t > maxDist)
        return false;

    float3 n = normalize(cross(e1, e2));
    if (dot(n, dir) > 0.0f)
        n = -n;   // oriented against travel, so it always points back at the piece

    outDist = t;
    outNormal = n;
    return true;
}

// The signed distance from a point to a convex hull, the point already in the hull's own
// space. A convex set is the intersection of its half spaces, so the largest per-plane
// distance is the distance to the hull. Positive is outside.
float ConvexDistance(StructuredBuffer<float4> planes, uint start, uint count, float3 local,
                     out uint outPlane)
{
    outPlane = 0;
    if (count == 0)
        return 1e30f;

    float best = -1e30f;
    for (uint k = 0; k < count; ++k) {
        const float4 pl = planes[start + k];
        const float d = dot(pl.xyz, local) + pl.w;
        if (d > best) {
            best = d;
            outPlane = k;
        }
    }
    return best;
}

// `at` rather than `point`, which HLSL reserves for a geometry shader primitive type.
bool SphereTouchesAabb(float3 lo, float3 hi, float3 at, float reach)
{
    return all(at >= lo - reach) && all(at <= hi + reach);
}

#endif
