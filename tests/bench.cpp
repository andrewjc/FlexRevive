// Throughput benchmarks for the CPU-bound inner loops.
//
// Not a test: it asserts nothing and never fails a build. It measures the paths that dominate
// a frame under sustained fire, so an optimisation can be shown to have worked rather than
// assumed to have. Run it before and after a change; the suites next door decide correctness.

#include "Collision.h"
#include "Contact.h"
#include "Math3D.h"
#include "Shock.h"

#include <chrono>
#include <cstdio>
#include <vector>

using namespace flexrevive;

namespace {

// Kept from being optimised away without forcing a store: the compiler must assume the value
// escapes, which is what stops a benchmark measuring an empty loop.
volatile float g_sink = 0.0f;

template <typename F>
double TimeMs(int reps, F&& body)
{
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < reps; ++i)
        body(i);
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

void Report(const char* name, double ms, long long ops)
{
    printf("  %-34s %8.2f ms   %10.1f M ops/s\n", name, ms,
           ms > 0.0 ? double(ops) / ms / 1000.0 : 0.0);
}

// A floor of `n` by `n` quads, the shape most world collision actually takes.
collision::TriMesh MakeFloor(int n)
{
    collision::TriMesh m;
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
    m.upper[0] = m.upper[1] = float(n) * 10.0f;
    return m;
}

fragment::Hull MakeChunk()
{
    std::vector<float> pts;
    for (float x = -4; x <= 4.01f; x += 1.0f)
        for (float y = -4; y <= 4.01f; y += 1.0f)
            for (float z = -2; z <= 2.01f; z += 1.0f) {
                pts.push_back(x); pts.push_back(y); pts.push_back(z);
            }
    fragment::Hull h;
    fragment::BuildFromPoints(pts.data(), int(pts.size() / 3), 3, 0.5f, h);
    return h;
}

} // namespace

int main()
{
    printf("FlexRevive benchmarks\n");

    // ---- the triangle sweep: the dominant cost of a piece in flight ----------------------
    {
        const collision::TriMesh floor = MakeFloor(40);   // 3200 triangles
        const int tris = int(floor.indices.size() / 3);
        const float dir[3] = {0, 0, -1};
        const int reps = 300;

        const double ms = TimeMs(reps, [&](int r) {
            const float from[3] = {float(r % 400), float((r * 7) % 400), 50.0f};
            float best = 1e30f, t = 0.0f;
            for (int k = 0; k < tris; ++k) {
                const float* a = &floor.verts[size_t(floor.indices[k * 3 + 0]) * 3];
                const float* b = &floor.verts[size_t(floor.indices[k * 3 + 1]) * 3];
                const float* c = &floor.verts[size_t(floor.indices[k * 3 + 2]) * 3];
                if (collision::SegmentHitsTriangle(from, dir, 100.0f, a, b, c, t) && t < best)
                    best = t;
            }
            g_sink = best;
        });
        Report("segment vs triangle", ms, 1LL * reps * tris);
    }

    // ---- quaternion work: every piece, every substep -------------------------------------
    {
        const int reps = 2000000;
        float q[4] = {0, 0, 0, 1};
        const float omega[3] = {1.5f, -2.0f, 0.5f};
        const double ms = TimeMs(reps, [&](int) {
            math::QuatIntegrate(q, omega, 0.001f);
            float out[3];
            const float v[3] = {1, 0, 0};
            math::QuatRotate(q, v, out);
            g_sink = out[0];
        });
        Report("quaternion integrate + rotate", ms, 1LL * reps);
    }

    // ---- contact resolution: every piece that touches anything ---------------------------
    {
        const fragment::Hull chunk = MakeChunk();
        const float identity[4] = {0, 0, 0, 1};
        const float up[3] = {0, 0, 1};
        const float surface[3] = {0, 0, 0};
        const int reps = 500000;

        const double ms = TimeMs(reps, [&](int) {
            float pos[3] = {0, 0, 1.5f};
            float vel[3] = {10.0f, 0, -80.0f};
            float w[3] = {0.5f, 0, 0};
            const contact::HullContact c =
                contact::FindHullContact(chunk, identity, pos, surface, up);
            if (c.valid)
                contact::ResolveHullContact(chunk, c, pos, vel, w, up, 0.25f, 0.6f, 0.5f);
            g_sink = pos[2];
        });
        Report("find + resolve hull contact", ms, 1LL * reps);
    }

    // ---- the contact search alone, separated from resolving it ---------------------------
    {
        const fragment::Hull chunk = MakeChunk();
        const float identity[4] = {0, 0, 0, 1};
        const float up[3] = {0, 0, 1};
        const float surface[3] = {0, 0, 0};
        const float pos[3] = {0, 0, 1.5f};
        const int reps = 2000000;
        const double ms = TimeMs(reps, [&](int) {
            const contact::HullContact c =
                contact::FindHullContact(chunk, identity, pos, surface, up);
            g_sink = c.depth;
        });
        Report("  of which: find contact", ms, 1LL * reps);
    }

    // ---- support extents: piece against piece, the narrow phase ---------------------------
    {
        const fragment::Hull chunk = MakeChunk();
        const float identity[4] = {0, 0, 0, 1};
        const float dir[3] = {0.577f, 0.577f, 0.577f};
        const int reps = 2000000;
        const double ms = TimeMs(reps, [&](int) {
            g_sink = contact::SupportExtent(chunk, identity, dir);
        });
        Report("support extent", ms, 1LL * reps);
    }

    // ---- impact shock: once per shocked piece per burst -----------------------------------
    {
        std::vector<shock::Impact> impacts;
        for (int i = 0; i < 8; ++i) {
            const float p[3] = {float(i) * 200.0f, 0, 0};
            shock::Record(impacts, p, 40.0f);
        }
        const int reps = 2000000;
        const double ms = TimeMs(reps, [&](int r) {
            const float pos[3] = {float(r % 1600), 10.0f, 5.0f};
            float push[3] = {0, 0, 0};
            shock::PushFor(impacts, pos, 3.0f, push);
            g_sink = push[0];
        });
        Report("impact shock push", ms, 1LL * reps);
    }

    return 0;
}
