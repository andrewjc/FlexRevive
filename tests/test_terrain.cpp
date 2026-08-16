// A piece falling onto ground, against the sweep the solver actually runs.
//
// Debris settles on concrete but sinks through dirt. The two differ in what the geometry is:
// concrete arrives as convex hulls, landscape as a triangle mesh. So this drops a chunk onto a
// mesh the way the solver does it, a short sweep per substep from where the piece was to where
// gravity is about to put it, and reports whether the surface stopped it.
//
// The mesh is built at the scale and position landscape actually occupies. Terrain quads span
// hundreds of units and sit tens of thousands from the world origin, where a float carries far
// fewer digits after the point than it does near it, and both of those are things a test built
// around a small mesh at the origin would never exercise.

#include "Collision.h"
#include "TestHarness.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace flexrevive;
using namespace f4kit;
using namespace flexrevive::collision;

namespace {

constexpr float kGravityZ = -686.6f;
constexpr float kContactSkin = 0.5f;
constexpr float kPieceRadius = 5.0f;

// A sheet of quads, each split into two triangles, centred on `origin`. `relief` is how far the
// surface rises and falls: real landscape is never flat, and a flat sheet has no thickness at
// all, which leaves the index unbuildable and quietly sends every sweep down the brute-force
// path instead of the one the game uses.
TriMesh Ground(const float* origin, float quad, int quads, float relief)
{
    TriMesh m;
    const float half = quad * float(quads) * 0.5f;
    const int side = quads + 1;

    for (int y = 0; y < side; ++y) {
        for (int x = 0; x < side; ++x) {
            m.verts.push_back(origin[0] - half + float(x) * quad);
            m.verts.push_back(origin[1] - half + float(y) * quad);
            m.verts.push_back(origin[2] +
                              relief * std::sin(float(x) * 0.7f) * std::cos(float(y) * 0.5f));
        }
    }
    for (int y = 0; y < quads; ++y) {
        for (int x = 0; x < quads; ++x) {
            const int a = y * side + x, b = a + 1, c = a + side, d = c + 1;
            m.indices.insert(m.indices.end(), {a, b, c});
            m.indices.insert(m.indices.end(), {b, d, c});
        }
    }

    for (int i = 0; i < 3; ++i) {
        m.lower[i] = 1e30f;
        m.upper[i] = -1e30f;
    }
    for (size_t v = 0; v < m.verts.size(); v += 3)
        for (int i = 0; i < 3; ++i) {
            m.lower[i] = std::min(m.lower[i], m.verts[v + size_t(i)]);
            m.upper[i] = std::max(m.upper[i], m.verts[v + size_t(i)]);
        }

    BuildTriGrid(m);
    return m;
}

struct Drop {
    bool stopped = false;
    float restZ = 0.0f;
    int steps = 0;
};

// Falls a piece from `startZ` onto the mesh, sweeping each substep exactly as the solver does.
Drop FallAt(const TriMesh& m, const float* at, float startZ, float startVz, float dt,
            int maxSteps)
{
    Drop out;
    float pos[3] = {at[0], at[1], startZ};
    float vel[3] = {0.0f, 0.0f, startVz};

    for (int step = 0; step < maxSteps; ++step) {
        out.steps = step;

        float from[3] = {pos[0], pos[1], pos[2]};
        vel[2] += kGravityZ * dt;
        for (int a = 0; a < 3; ++a)
            pos[a] += vel[a] * dt;

        float delta[3] = {pos[0] - from[0], pos[1] - from[1], pos[2] - from[2]};
        const float dist =
            std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
        if (dist < 1e-5f)
            continue;

        const float dir[3] = {delta[0] / dist, delta[1] / dist, delta[2] / dist};
        const float clearance = kPieceRadius + kContactSkin;
        const float sweepLen = dist + clearance;

        float hitT = 0.0f, n[3];
        if (SweepMesh(m, from, dir, sweepLen, kContactSkin, hitT, n)) {
            // Placed at the surface and stopped, which is what the contact does.
            for (int a = 0; a < 3; ++a)
                pos[a] = from[a] + dir[a] * std::max(0.0f, hitT - clearance);
            vel[0] = vel[1] = vel[2] = 0.0f;
            out.stopped = true;
            out.restZ = pos[2];
            return out;
        }

        // Gone below the surface without ever registering it.
        if (pos[2] < m.lower[2] - 200.0f) {
            out.restZ = pos[2];
            return out;
        }
    }
    out.restZ = pos[2];
    return out;
}

Drop Fall(const TriMesh& m, const float* at, float startZ, float dt, int maxSteps)
{
    return FallAt(m, at, startZ, 0.0f, dt, maxSteps);
}

// A piece thrown across the ground rather than dropped onto it, so the sweep runs at an angle
// and crosses the index sideways as well as downward.
Drop Launch(const TriMesh& m, const float* start, const float* velocity, float dt, int maxSteps)
{
    Drop out;
    float pos[3] = {start[0], start[1], start[2]};
    float vel[3] = {velocity[0], velocity[1], velocity[2]};

    for (int step = 0; step < maxSteps; ++step) {
        out.steps = step;

        const float from[3] = {pos[0], pos[1], pos[2]};
        vel[2] += kGravityZ * dt;
        for (int a = 0; a < 3; ++a)
            pos[a] += vel[a] * dt;

        float delta[3] = {pos[0] - from[0], pos[1] - from[1], pos[2] - from[2]};
        const float dist =
            std::sqrt(delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
        if (dist < 1e-5f)
            continue;

        const float dir[3] = {delta[0] / dist, delta[1] / dist, delta[2] / dist};
        const float clearance = kPieceRadius + kContactSkin;

        float hitT = 0.0f, n[3];
        if (SweepMesh(m, from, dir, dist + clearance, kContactSkin, hitT, n)) {
            out.stopped = true;
            out.restZ = from[2] + dir[2] * std::max(0.0f, hitT - clearance);
            return out;
        }

        if (pos[2] < m.lower[2] - 200.0f) {
            out.restZ = pos[2];
            return out;
        }
    }
    out.restZ = pos[2];
    return out;
}

} // namespace

static void TestGridBuilds()
{
    test::Suite("the index over a sheet of ground");

    const float here[3] = {0.0f, 0.0f, 0.0f};
    TriMesh m = Ground(here, 128.0f, 32, 240.0f);
    CHECK_EQ(int(m.indices.size() / 3), 2048);
    CHECK(m.grid.valid);
}

// Sweeps the two things landscape does differently from a wall: how far it is from the world
// origin, and how large its triangles are.
static void TestGroundStopsAPiece()
{
    test::Suite("ground stops what lands on it");

    struct Case {
        const char* what;
        float origin[3];
        float quad;
        int quads;
        float relief;
    };

    const Case cases[] = {
        {"flat, at the origin",               {0.0f, 0.0f, 0.0f},            16.0f,  8,   0.0f},
        {"rolling, at the origin",            {0.0f, 0.0f, 0.0f},            64.0f, 16,  40.0f},
        {"terrain quads, at the origin",      {0.0f, 0.0f, 0.0f},           128.0f, 32, 240.0f},
        {"flat, far out",                     {-78433.0f, 84415.0f, 7530.0f}, 16.0f,  8,   0.0f},
        {"rolling, far out",                  {-78433.0f, 84415.0f, 7530.0f}, 64.0f, 16,  40.0f},
        {"terrain quads, far out",            {-78433.0f, 84415.0f, 7530.0f},128.0f, 32, 240.0f},
        {"a whole terrain cell, far out",     {-78433.0f, 84415.0f, 7530.0f},512.0f, 32, 900.0f},
    };

    for (const Case& c : cases) {
        TriMesh m = Ground(c.origin, c.quad, c.quads, c.relief);

        // Dropped just off a vertex, so the fall lands inside a triangle rather than along a
        // shared edge where either of two triangles could claim it.
        const float at[3] = {c.origin[0] + c.quad * 0.37f, c.origin[1] + c.quad * 0.21f, 0.0f};
        const Drop d = Fall(m, at, c.origin[2] + 600.0f, 1.0f / 240.0f, 4000);

        printf("      %-32s grid=%d  stopped=%d  rest z %.1f  (surface spans %.1f..%.1f)\n",
               c.what, int(m.grid.valid), int(d.stopped), d.restZ, m.lower[2], m.upper[2]);

        CHECK(d.stopped);
        CHECK(d.restZ >= m.lower[2]);
    }
}

// A piece arriving quickly, which is what debris does after any real fall.
//
// The sweep covers the whole of a step, so speed alone should not matter. What changes with
// speed is how much geometry the segment crosses: a slow piece stays inside one cell of the
// index, a fast one runs through many, and only the fast case exercises walking the index.
static void TestFastPieceIsStopped()
{
    test::Suite("ground stops a piece arriving fast");

    const float origin[3] = {-78433.0f, 84415.0f, 7530.0f};
    TriMesh m = Ground(origin, 128.0f, 32, 240.0f);
    const float at[3] = {origin[0] + 47.0f, origin[1] + 27.0f, 0.0f};

    // Terminal-ish speeds, and the coarse substep a busy frame produces.
    const float speeds[] = {0.0f, 200.0f, 600.0f, 1500.0f, 3000.0f, 6000.0f};
    const float steps[] = {1.0f / 240.0f, 1.0f / 60.0f, 1.0f / 30.0f};

    for (float dt : steps) {
        for (float speed : speeds) {
            const Drop d = FallAt(m, at, origin[2] + 900.0f, -speed, dt, 8000);
            printf("      dt 1/%-3.0f  entry speed %-6.0f  moves %6.1f/step  stopped=%d  "
                   "rest z %.1f\n",
                   1.0f / dt, speed, speed * dt, int(d.stopped), d.restZ);
            CHECK(d.stopped);
            CHECK(d.restZ >= m.lower[2]);
        }
    }
}

// Debris thrown across the ground, which is how it actually leaves a wall.
static void TestSkimmingPieceIsStopped()
{
    test::Suite("ground stops a piece travelling across it");

    const float origin[3] = {-78433.0f, 84415.0f, 7530.0f};
    TriMesh m = Ground(origin, 128.0f, 32, 240.0f);

    // Starting above the middle, thrown outward at a range of angles. A shallow throw covers a
    // lot of ground per step and crosses many cells of the index; a steep one barely leaves the
    // column it started in.
    const float speeds[] = {300.0f, 900.0f, 2400.0f};
    const float angles[] = {0.0f, 5.0f, 15.0f, 45.0f, 80.0f};

    int missed = 0;
    for (float dt : {1.0f / 240.0f, 1.0f / 60.0f}) {
        for (float speed : speeds) {
            for (float deg : angles) {
                const float rad = deg * 3.14159265f / 180.0f;
                const float vel[3] = {std::cos(rad) * speed, 0.0f, -std::sin(rad) * speed};
                const float start[3] = {origin[0] - 1400.0f, origin[1] + 33.0f,
                                        m.upper[2] + 120.0f};
                const Drop d = Launch(m, start, vel, dt, 12000);
                if (!d.stopped) {
                    ++missed;
                    printf("      MISSED dt 1/%-3.0f speed %-5.0f angle %2.0f deg -> fell to "
                           "%.1f (surface bottoms at %.1f)\n",
                           1.0f / dt, speed, deg, d.restZ, m.lower[2]);
                }
                CHECK(d.stopped);
            }
        }
    }
    printf("      %d of 30 trajectories passed through the ground\n", missed);
}

int main()
{
    printf("Terrain\n");
    TestGridBuilds();
    TestGroundStopsAPiece();
    TestFastPieceIsStopped();
    TestSkimmingPieceIsStopped();
    return test::Report("Terrain");
}
