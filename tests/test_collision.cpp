// Collision primitives: ray/triangle intersection, the triangle index, and closest-point
// queries against the analytic shapes.

#include "Collision.h"
#include "TestHarness.h"

#include <algorithm>
#include <vector>

using namespace flexrevive;
using namespace f4kit;
using namespace flexrevive::collision;

// A single triangle in the z = 0 plane, spanning (0,0) to (10,10).
static void FlatTriangle(float* v0, float* v1, float* v2)
{
    v0[0] = 0;  v0[1] = 0;  v0[2] = 0;
    v1[0] = 10; v1[1] = 0;  v1[2] = 0;
    v2[0] = 0;  v2[1] = 10; v2[2] = 0;
}

static void TestSegmentHitsTriangle()
{
    test::Suite("segment against triangle");

    float v0[3], v1[3], v2[3];
    FlatTriangle(v0, v1, v2);
    float t = 0.0f;

    // Straight down through the middle, from 5 units up.
    const float from[3] = {2, 2, 5};
    const float down[3] = {0, 0, -1};
    CHECK(SegmentHitsTriangle(from, down, 10.0f, v0, v1, v2, t));
    CHECK_NEAR(t, 5.0, 1e-4);

    // Too short to reach.
    CHECK(!SegmentHitsTriangle(from, down, 4.0f, v0, v1, v2, t));

    // Outside the triangle in the plane's own coordinates.
    const float wide[3] = {9, 9, 5};
    CHECK(!SegmentHitsTriangle(wide, down, 10.0f, v0, v1, v2, t));

    // Travelling away from it.
    const float up[3] = {0, 0, 1};
    CHECK(!SegmentHitsTriangle(from, up, 10.0f, v0, v1, v2, t));

    // Parallel to the plane never registers, however long the ray.
    const float across[3] = {1, 0, 0};
    const float inPlane[3] = {-5, 2, 0};
    CHECK(!SegmentHitsTriangle(inPlane, across, 1000.0f, v0, v1, v2, t));

    // A hit is found from either face, since debris can arrive from below.
    const float below[3] = {2, 2, -5};
    CHECK(SegmentHitsTriangle(below, up, 10.0f, v0, v1, v2, t));
    CHECK_NEAR(t, 5.0, 1e-4);

    // A degenerate triangle bounds nothing.
    float d0[3] = {0, 0, 0}, d1[3] = {0, 0, 0}, d2[3] = {0, 0, 0};
    CHECK(!SegmentHitsTriangle(from, down, 10.0f, d0, d1, d2, t));
}

// A grid of `n` by `n` quads in the z = 0 plane, each 10 units across: enough triangles to
// cross the indexing threshold.
static void MakeFloor(int n, TriMesh& m)
{
    m = TriMesh();
    for (int y = 0; y <= n; ++y)
        for (int x = 0; x <= n; ++x) {
            m.verts.push_back(float(x) * 10.0f);
            m.verts.push_back(float(y) * 10.0f);
            m.verts.push_back(0.0f);
        }
    auto at = [&](int x, int y) { return y * (n + 1) + x; };
    for (int y = 0; y < n; ++y)
        for (int x = 0; x < n; ++x) {
            m.indices.push_back(at(x, y));     m.indices.push_back(at(x + 1, y));
            m.indices.push_back(at(x + 1, y + 1));
            m.indices.push_back(at(x, y));     m.indices.push_back(at(x + 1, y + 1));
            m.indices.push_back(at(x, y + 1));
        }
    m.lower[0] = m.lower[1] = m.lower[2] = 0.0f;
    m.upper[0] = m.upper[1] = float(n) * 10.0f;
    m.upper[2] = 0.0f;
}

static void TestTriGrid()
{
    test::Suite("triangle index");

    // Below the threshold the index is deliberately not built; the linear path is correct.
    TriMesh small;
    MakeFloor(2, small);            // 8 triangles
    BuildTriGrid(small);
    CHECK(!small.grid.valid);

    TriMesh floor;
    MakeFloor(16, floor);           // 512 triangles
    BuildTriGrid(floor);
    CHECK(floor.grid.valid);
    CHECK(floor.grid.dim[0] > 0 && floor.grid.dim[1] > 0 && floor.grid.dim[2] > 0);

    // Flat geometry must collapse its empty axis rather than build a needlessly deep grid.
    CHECK_EQ(floor.grid.dim[2], 1);

    // Every triangle is reachable through the index: the union of all cells covers them all.
    std::vector<int> seen(floor.indices.size() / 3, 0);
    for (int c = 0; c + 1 < int(floor.grid.start.size()); ++c)
        for (int k = floor.grid.start[c]; k < floor.grid.start[c + 1]; ++k)
            if (floor.grid.tris[k] >= 0 && floor.grid.tris[k] < int(seen.size()))
                seen[floor.grid.tris[k]] = 1;
    CHECK_EQ(std::count(seen.begin(), seen.end(), 0), 0);

    // The index is a superset of what a linear scan would find, which is what makes it safe:
    // a sweep may test a triangle twice but must never miss one.
    const float from[3] = {55.0f, 55.0f, 20.0f};
    const float dir[3] = {0, 0, -1};
    float linearT = 1e30f;
    bool linearHit = false;
    for (size_t t = 0; t < floor.indices.size() / 3; ++t) {
        const float* a = &floor.verts[size_t(floor.indices[t * 3 + 0]) * 3];
        const float* b = &floor.verts[size_t(floor.indices[t * 3 + 1]) * 3];
        const float* c = &floor.verts[size_t(floor.indices[t * 3 + 2]) * 3];
        float hit = 0.0f;
        if (SegmentHitsTriangle(from, dir, 50.0f, a, b, c, hit) && hit < linearT) {
            linearT = hit; linearHit = true;
        }
    }
    CHECK(linearHit);
    CHECK_NEAR(linearT, 20.0, 1e-3);

    // A mesh with no triangles is handled rather than indexed.
    TriMesh empty;
    BuildTriGrid(empty);
    CHECK(!empty.grid.valid);
}

static void TestClosestOnSphere()
{
    test::Suite("closest point on a sphere");

    Collider s;
    s.type = kColliderSphere;
    s.dims[0] = 10.0f;

    float closest[3];
    bool inside = false;

    // A point outside projects onto the centre, with the radius added by PrimitiveRadius.
    const float outside[3] = {20, 0, 0};
    CHECK(ClosestOnPrimitive(s, outside, closest, inside));
    CHECK(!inside);
    CHECK_NEAR(PrimitiveRadius(s), 10.0, 1e-6);

    // A point at the centre still yields a usable answer rather than a division by zero.
    const float centre[3] = {0, 0, 0};
    CHECK(ClosestOnPrimitive(s, centre, closest, inside));
    CHECK(std::isfinite(closest[0]) && std::isfinite(closest[1]) && std::isfinite(closest[2]));
}

static void TestClosestOnCapsule()
{
    test::Suite("closest point on a capsule");

    Collider c;
    c.type = kColliderCapsule;
    c.dims[0] = 5.0f;    // radius
    c.dims[1] = 20.0f;   // half height, running along local x

    float closest[3];
    bool inside = false;

    // Beside the shaft: the nearest core point is directly alongside, at the same x.
    const float beside[3] = {0, 30, 0};
    CHECK(ClosestOnPrimitive(c, beside, closest, inside));
    CHECK_NEAR(closest[0], 0.0, 1e-4);
    CHECK_NEAR(closest[1], 0.0, 1e-4);

    // Past the end cap: the core point clamps to the end of the shaft, not beyond it.
    const float pastEnd[3] = {100, 0, 0};
    CHECK(ClosestOnPrimitive(c, pastEnd, closest, inside));
    CHECK_NEAR(closest[0], 20.0, 1e-4);

    // And past the other end.
    const float pastOther[3] = {-100, 0, 0};
    CHECK(ClosestOnPrimitive(c, pastOther, closest, inside));
    CHECK_NEAR(closest[0], -20.0, 1e-4);

    CHECK_NEAR(PrimitiveRadius(c), 5.0, 1e-6);
}

static void TestClosestOnBox()
{
    test::Suite("closest point on a box");

    Collider b;
    b.type = kColliderBox;
    b.dims[0] = 10.0f; b.dims[1] = 20.0f; b.dims[2] = 5.0f;

    float closest[3];
    bool inside = false;

    // Outside on one axis clamps to the face.
    const float out[3] = {50, 0, 0};
    CHECK(ClosestOnPrimitive(b, out, closest, inside));
    CHECK(!inside);
    CHECK_NEAR(closest[0], 10.0, 1e-4);

    // A box is its own surface, so it adds no radius on top.
    CHECK_NEAR(PrimitiveRadius(b), 0.0, 1e-6);

    // A point strictly inside is reported as such, and leaves by its nearest face: at
    // (0, 0, 4) the z face at 5 is closest.
    const float in[3] = {0, 0, 4};
    CHECK(ClosestOnPrimitive(b, in, closest, inside));
    CHECK(inside);
    CHECK_NEAR(closest[2], 5.0, 1e-4);
}

static void TestUnsupportedKinds()
{
    test::Suite("kinds with no analytic form");

    // Triangle meshes go through the sweep, and these two cannot be reconstructed from what
    // the engine passes, so all three must decline rather than invent a surface.
    for (int kind : {kColliderTriangleMesh, kColliderConvexMesh, kColliderSDF,
                     kColliderUnknown}) {
        Collider c;
        c.type = kind;
        c.dims[0] = c.dims[1] = c.dims[2] = 5.0f;
        float closest[3];
        bool inside = false;
        const float p[3] = {1, 2, 3};
        CHECK(!ClosestOnPrimitive(c, p, closest, inside));
    }
}

static void TestFlagDecoding()
{
    test::Suite("collider flag decoding");

    // Flags are `kind | (dynamic ? 8 : 0)`. A dynamic capsule is 1 | 8 = 9, and must decode as
    // a capsule that is dynamic, not as an out-of-range kind.
    CHECK_EQ(9 & kColliderKindMask, kColliderCapsule);
    CHECK((9 & kColliderDynamicBit) != 0);

    CHECK_EQ(kColliderTriangleMesh & kColliderKindMask, kColliderTriangleMesh);
    CHECK((kColliderTriangleMesh & kColliderDynamicBit) == 0);

    // A dynamic box is 2 | 8 = 10, and the dynamic bit is readable without trusting the kind.
    CHECK((10 & kColliderDynamicBit) != 0);
    CHECK_EQ(10 & kColliderKindMask, kColliderBox);

    // Every kind survives the round trip with and without the dynamic bit set.
    for (int kind : {kColliderSphere, kColliderCapsule, kColliderBox, kColliderConvexMesh,
                     kColliderTriangleMesh, kColliderSDF}) {
        CHECK_EQ(kind & kColliderKindMask, kind);
        CHECK_EQ((kind | kColliderDynamicBit) & kColliderKindMask, kind);
        CHECK(((kind | kColliderDynamicBit) & kColliderDynamicBit) != 0);
    }
}


// A cube of half extent `h`, as six outward planes: n dot p + w, with w = -h.
static std::vector<float> CubePlanes(float h)
{
    std::vector<float> planes;
    const float n[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    for (auto& v : n) {
        planes.push_back(v[0]); planes.push_back(v[1]); planes.push_back(v[2]);
        planes.push_back(-h);
    }
    return planes;
}

static void TestConvexDistance()
{
    test::Suite("distance to a convex hull");

    const std::vector<float> cube = CubePlanes(10.0f);
    int plane = -1;

    // Outside along one axis: the distance is how far past that face the point sits, and the
    // face that produced it is the one it is outside of.
    {
        const float p[3] = {25.0f, 0, 0};
        CHECK_NEAR(ConvexDistance(cube.data(), 6, p, plane), 15.0, 1e-4);
        CHECK_EQ(plane, 0);
    }
    {
        const float p[3] = {0, 0, -30.0f};
        CHECK_NEAR(ConvexDistance(cube.data(), 6, p, plane), 20.0, 1e-4);
        CHECK_EQ(plane, 5);
    }

    // On a face: exactly zero, which is what makes the contact test a plain comparison.
    {
        const float p[3] = {10.0f, 3.0f, -2.0f};
        CHECK_NEAR(ConvexDistance(cube.data(), 6, p, plane), 0.0, 1e-4);
    }

    // Inside: negative, and the magnitude is the distance to the nearest face rather than to
    // the furthest, since the largest per-plane value is the least negative one.
    {
        const float p[3] = {8.0f, 0, 0};
        CHECK_NEAR(ConvexDistance(cube.data(), 6, p, plane), -2.0, 1e-4);
        CHECK_EQ(plane, 0);
    }
    {
        const float centre[3] = {0, 0, 0};
        CHECK_NEAR(ConvexDistance(cube.data(), 6, centre, plane), -10.0, 1e-4);
    }

    // Past a corner, the nearest face is whichever it is furthest beyond.
    {
        const float p[3] = {40.0f, 12.0f, 0};
        CHECK_NEAR(ConvexDistance(cube.data(), 6, p, plane), 30.0, 1e-4);
        CHECK_EQ(plane, 0);
    }

    // A point inside is inside every plane; a point outside is outside at least one. That is
    // what makes the maximum the right answer for a convex set.
    {
        const float inside[3] = {2.0f, -3.0f, 4.0f};
        CHECK(ConvexDistance(cube.data(), 6, inside, plane) < 0.0f);
        const float outside[3] = {2.0f, -30.0f, 4.0f};
        CHECK(ConvexDistance(cube.data(), 6, outside, plane) > 0.0f);
    }

    // No planes bound nothing, and the caller is told there was no face.
    {
        const float p[3] = {1, 2, 3};
        plane = 99;
        const float d = ConvexDistance(nullptr, 0, p, plane);
        CHECK_EQ(plane, -1);
        CHECK(d > 0.0f);
        CHECK(std::isfinite(d));
        CHECK_EQ(ConvexDistance(cube.data(), 0, p, plane) > 0.0f, 1);
    }
}

static void TestSphereTouchesAabb()
{
    test::Suite("sphere against a box")

    ;
    const float lo[3] = {0, 0, 0};
    const float hi[3] = {100.0f, 100.0f, 100.0f};

    const float inside[3] = {50.0f, 50.0f, 50.0f};
    CHECK(SphereTouchesAabb(lo, hi, inside, 0.0f));

    // Just outside on one axis, reached only once the radius covers the gap.
    const float near[3] = {110.0f, 50.0f, 50.0f};
    CHECK(!SphereTouchesAabb(lo, hi, near, 5.0f));
    CHECK(SphereTouchesAabb(lo, hi, near, 15.0f));

    // Far away on any single axis is enough to reject.
    const float far[3] = {50.0f, 50.0f, -900.0f};
    CHECK(!SphereTouchesAabb(lo, hi, far, 50.0f));

    // A degenerate box still answers rather than faulting.
    const float same[3] = {5.0f, 5.0f, 5.0f};
    CHECK(SphereTouchesAabb(same, same, same, 0.0f));
    CHECK(!SphereTouchesAabb(same, same, far, 1.0f));
}


// The nearest hit a full scan of every triangle would find, as a reference for the indexed one.
static bool BruteForceSweep(const TriMesh& m, const float* from, const float* dir, float maxDist,
                            float& outDist, float* outNormal)
{
    bool hit = false;
    float best = maxDist;
    const int tris = int(m.indices.size() / 3);
    for (int t = 0; t < tris; ++t) {
        const float* a = &m.verts[size_t(m.indices[size_t(t) * 3 + 0]) * 3];
        const float* b = &m.verts[size_t(m.indices[size_t(t) * 3 + 1]) * 3];
        const float* c = &m.verts[size_t(m.indices[size_t(t) * 3 + 2]) * 3];
        float d = 0.0f, n[3];
        if (SegmentHitsTriangle(from, dir, best, a, b, c, d, n) && d <= best) {
            best = d;
            hit = true;
            for (int i = 0; i < 3; ++i) outNormal[i] = n[i];
        }
    }
    outDist = best;
    return hit;
}

static void TestSweepMesh()
{
    test::Suite("sweeping a mesh");

    TriMesh floor;
    MakeFloor(16, floor);
    BuildTriGrid(floor);
    CHECK(floor.grid.valid);

    const float down[3] = {0, 0, -1};
    float dist = 0.0f, normal[3] = {0, 0, 0};

    // Straight down onto the floor from above.
    {
        const float from[3] = {55.0f, 55.0f, 30.0f};
        CHECK(SweepMesh(floor, from, down, 100.0f, 0.5f, dist, normal));
        CHECK_NEAR(dist, 30.0, 1e-3);
        CHECK_NEAR(normal[2], 1.0, 1e-3);   // oriented against travel
    }

    // Short of the surface.
    {
        const float from[3] = {55.0f, 55.0f, 30.0f};
        CHECK(!SweepMesh(floor, from, down, 10.0f, 0.5f, dist, normal));
    }

    // Beyond the mesh entirely, rejected by its bounds.
    {
        const float from[3] = {5000.0f, 5000.0f, 30.0f};
        CHECK(!SweepMesh(floor, from, down, 100.0f, 0.5f, dist, normal));
    }

    // Travelling away from it.
    {
        const float from[3] = {55.0f, 55.0f, 30.0f};
        const float up[3] = {0, 0, 1};
        CHECK(!SweepMesh(floor, from, up, 100.0f, 0.5f, dist, normal));
    }

    // A normal is always oriented against travel, so a piece arriving from below is pushed
    // back the way it came rather than through the surface.
    {
        const float from[3] = {55.0f, 55.0f, -30.0f};
        const float up[3] = {0, 0, 1};
        CHECK(SweepMesh(floor, from, up, 100.0f, 0.5f, dist, normal));
        CHECK_NEAR(normal[2], -1.0, 1e-3);
    }
}

// The index exists to avoid testing every triangle. It is only safe if it finds exactly what
// testing every triangle would, so the two are compared across the whole surface.
static void TestSweepMatchesFullScan()
{
    test::Suite("the index finds what a full scan finds");

    TriMesh floor;
    MakeFloor(16, floor);
    BuildTriGrid(floor);

    const float down[3] = {0, 0, -1};
    int compared = 0, disagreed = 0, hits = 0;

    for (int gx = 0; gx < 40; ++gx)
        for (int gy = 0; gy < 40; ++gy) {
            const float from[3] = {float(gx) * 4.3f, float(gy) * 4.1f, 25.0f};
            float indexedDist = 0.0f, indexedN[3] = {0, 0, 0};
            float scanDist = 0.0f, scanN[3] = {0, 0, 0};

            const bool a = SweepMesh(floor, from, down, 60.0f, 0.5f, indexedDist, indexedN);
            const bool b = BruteForceSweep(floor, from, down, 60.0f, scanDist, scanN);
            ++compared;
            if (a != b || (a && std::fabs(indexedDist - scanDist) > 1e-3f))
                ++disagreed;
            if (a) ++hits;
        }

    CHECK(compared > 1000);
    CHECK(hits > 500);        // the rays are actually landing on the surface
    CHECK_EQ(disagreed, 0);

    // Angled sweeps cross cell boundaries, which is where a traversal is most likely to miss.
    disagreed = 0;
    for (int k = 0; k < 200; ++k) {
        const float from[3] = {float(k) * 0.8f, 5.0f + float(k) * 0.3f, 40.0f};
        const float len = std::sqrt(0.4f * 0.4f + 0.2f * 0.2f + 1.0f);
        const float dir[3] = {0.4f / len, 0.2f / len, -1.0f / len};
        float ad = 0.0f, an[3], bd = 0.0f, bn[3];
        const bool a = SweepMesh(floor, from, dir, 120.0f, 0.5f, ad, an);
        const bool b = BruteForceSweep(floor, from, dir, 120.0f, bd, bn);
        if (a != b || (a && std::fabs(ad - bd) > 1e-3f))
            ++disagreed;
    }
    CHECK_EQ(disagreed, 0);
}

static void TestSweepWithoutIndex()
{
    test::Suite("a mesh too small to index");

    // Below the threshold no grid is built, and the sweep must still work.
    TriMesh small;
    MakeFloor(2, small);
    BuildTriGrid(small);
    CHECK(!small.grid.valid);

    const float from[3] = {10.0f, 10.0f, 20.0f};
    const float down[3] = {0, 0, -1};
    float dist = 0.0f, normal[3];
    CHECK(SweepMesh(small, from, down, 100.0f, 0.5f, dist, normal));
    CHECK_NEAR(dist, 20.0, 1e-3);

    // An empty mesh is swept without incident.
    TriMesh empty;
    CHECK(!SweepMesh(empty, from, down, 100.0f, 0.5f, dist, normal));
}

int main()
{
    printf("Collision\n");
    TestSegmentHitsTriangle();
    TestTriGrid();
    TestClosestOnSphere();
    TestClosestOnCapsule();
    TestClosestOnBox();
    TestUnsupportedKinds();
    TestFlagDecoding();
    TestConvexDistance();
    TestSphereTouchesAabb();
    TestSweepMesh();
    TestSweepMatchesFullScan();
    TestSweepWithoutIndex();
    return test::Report("Collision");
}
