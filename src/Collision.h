// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

#include <vector>

// World collision geometry and the queries run against it.
//
// Everything here is pure geometry: no engine state, no Windows, no globals. A collider is a
// placed piece of geometry, and the queries answer where a piece crosses it or how far a point
// sits from its surface.
namespace flexrevive::collision {

// A uniform grid over a mesh's triangles, so a sweep tests only the geometry it passes near.
// A piece moves a few units per step against meshes spanning hundreds, so the cells its sweep
// touches hold a small fraction of the triangles.
struct TriGrid {
    int dim[3] = {0, 0, 0};
    float origin[3] = {0, 0, 0};
    float invCell[3] = {1, 1, 1};
    std::vector<int> start; // one entry per cell plus a terminator; offsets into tris
    std::vector<int> tris;  // triangle indices, grouped by cell
    bool valid = false;
};

// World collision geometry, as uploaded through flexUpdateTriangleMesh.
struct TriMesh {
    std::vector<float> verts;   // xyz triples
    std::vector<int> indices;   // 3 per triangle
    float lower[3] = {0, 0, 0};
    float upper[3] = {0, 0, 0};
    TriGrid grid;
};

// Below this a linear scan beats the indirection; above the cell cap the index costs more
// memory than it saves time.
constexpr int kGridMinTriangles = 64;
constexpr int kGridMaxCells = 65536;

int GridCell(const TriGrid& g, int x, int y, int z);

// Builds the grid for a freshly uploaded mesh. On failure grid.valid stays false and the sweep
// falls back to testing every triangle, which is slower but equally correct.
void BuildTriGrid(TriMesh& m);

// Collider flags pack the primitive kind into the low three bits with a dynamic bit directly
// above them, so the kind is `flags & kColliderKindMask` and the dynamic bit is
// `flags & kColliderDynamicBit`. The world is mostly triangle meshes and convex hulls; actors
// are capsules.
enum ColliderKind {
    kColliderSphere = 0,
    kColliderCapsule = 1,
    kColliderBox = 2,
    kColliderConvexMesh = 3,
    kColliderTriangleMesh = 4,
    kColliderSDF = 5,
    kColliderUnknown = -1,
};
constexpr int kColliderKindMask = 0x7;
constexpr int kColliderDynamicBit = 0x8;

// A placed piece of collision geometry. Mesh data is local-space, so each collider carries the
// transform that puts it in the world.
struct Collider {
    const TriMesh* mesh = nullptr;
    int type = kColliderUnknown;
    float pos[3] = {0, 0, 0};
    float rot[4] = {0, 0, 0, 1};     // quaternion x, y, z, w
    float prevPos[3] = {0, 0, 0};    // last frame, so a moving collider has a velocity
    // Sphere: radius. Capsule: radius, half height. Box: half extents.
    float dims[3] = {0, 0, 0};
    bool moved = false;              // travelled far enough this frame to shove debris
    bool dynamic = false;            // the engine's own dynamic bit
    // A convex hull, as a run of planes held by the caller, each a float4 of unit normal plus
    // plane distance.
    int planeStart = 0;
    int planeCount = 0;
    float aabbLo[3] = {0, 0, 0};
    float aabbHi[3] = {0, 0, 0};
    bool haveAabb = false;
};

// A collider must move by more than float noise to count as moving, so static geometry is not
// treated as a pusher.
constexpr float kColliderMotionEpsilon = 0.25f;

// Möller-Trumbore. Returns true and the hit distance along the ray if the segment crosses the
// triangle, from either face. `outNormal`, when given, receives the unit face normal oriented
// against the direction of travel.
bool SegmentHitsTriangle(const float* from, const float* dir, float maxDist, const float* v0,
                         const float* v1, const float* v2, float& outDist,
                         float* outNormal = nullptr);

// Closest point on a collision primitive to a point already in the collider's local space, plus
// whether that point sits inside it. Returns false for kinds with no analytic form here:
// triangle meshes go through the sweep, and convex meshes and SDFs are not reconstructible from
// what the engine passes.
//
// This is a discrete overlap test rather than a swept one. An actor is slow relative to its own
// size, so there is nothing to tunnel through, and an overlap test also catches pieces already
// resting against the collider, which is what lets one shove a settled pile aside.
bool ClosestOnPrimitive(const Collider& sh, const float* local, float* closest, bool& inside);

// Sweeps a segment through a mesh, in the mesh's own space, and reports the nearest triangle
// it crosses within `maxDist`. `outNormal` receives that face's unit normal oriented against
// the direction of travel.
//
// Where the mesh carries an index only the cells the segment passes through are visited. A
// triangle listed in several cells may be tested more than once, which is harmless: the test
// is a pure function and the best distance only shrinks. `skin` widens the cell range and the
// bounds check so a surface just outside the segment is still found.
bool SweepMesh(const TriMesh& m, const float* from, const float* dir, float maxDist, float skin,
               float& outDist, float* outNormal);

// The signed distance from a point to a convex hull, with the point already in the hull's own
// space. `planes` holds float4 each: a unit outward normal and the plane's offset.
//
// A convex set is the intersection of its half spaces, so the largest of the per-plane
// distances is the distance to the hull itself, and the plane that produced it is the nearest
// face. Positive is outside, zero is on the surface, negative is within. `outPlane` receives
// the index of that face, or -1 when there are no planes to test.
float ConvexDistance(const float* planes, int planeCount, const float* local, int& outPlane);

// Whether a sphere of `reach` about `point` can touch an axis-aligned box. Used to drop hulls
// a piece is nowhere near before doing any real work.
bool SphereTouchesAabb(const float* lo, const float* hi, const float* point, float reach);

// The surface radius a primitive adds to its closest-point form. A sphere and a capsule are
// both everything within R of a core; a box is the core itself.
float PrimitiveRadius(const Collider& sh);

}
