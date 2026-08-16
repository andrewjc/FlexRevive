// Vector, quaternion, hashing and sampling primitives.

#include "Math3D.h"
#include "TestHarness.h"

#include <algorithm>
#include <vector>

using namespace flexrevive;
using namespace f4kit;
using namespace flexrevive::math;

static void TestDotCrossMat3()
{
    test::Suite("dot, cross and 3x3 multiply");

    const float x[3] = {1, 0, 0}, y[3] = {0, 1, 0}, z[3] = {0, 0, 1};

    CHECK_NEAR(Dot(x, x), 1.0, 1e-6);
    CHECK_NEAR(Dot(x, y), 0.0, 1e-6);

    float c[3];
    Cross(x, y, c);
    CHECK_VEC3(c, 0, 0, 1, 1e-6);   // right handed
    Cross(y, x, c);
    CHECK_VEC3(c, 0, 0, -1, 1e-6);  // anticommutative
    Cross(x, x, c);
    CHECK_VEC3(c, 0, 0, 0, 1e-6);   // parallel vectors have no cross product

    // A cross product is perpendicular to both of its inputs.
    const float a[3] = {1.5f, -2.0f, 0.75f}, b[3] = {-0.25f, 3.0f, 2.0f};
    Cross(a, b, c);
    CHECK_NEAR(Dot(c, a), 0.0, 1e-4);
    CHECK_NEAR(Dot(c, b), 0.0, 1e-4);

    const float identity[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float out[3];
    Mat3Mul(identity, a, out);
    CHECK_VEC3(out, a[0], a[1], a[2], 1e-6);

    // Row major: row 0 scales x, row 1 scales y, row 2 scales z.
    const float scale[9] = {2, 0, 0, 0, 3, 0, 0, 0, 4};
    Mat3Mul(scale, z, out);
    CHECK_VEC3(out, 0, 0, 4, 1e-6);
}

static void TestQuatRotate()
{
    test::Suite("quaternion rotation");

    const float identity[4] = {0, 0, 0, 1};
    const float v[3] = {1, 2, 3};
    float out[3];

    QuatRotate(identity, v, out);
    CHECK_VEC3(out, 1, 2, 3, 1e-6);

    // A quarter turn about z carries x onto y.
    const float s = std::sqrt(0.5f);
    const float qz90[4] = {0, 0, s, s};
    const float x[3] = {1, 0, 0};
    QuatRotate(qz90, x, out);
    CHECK_VEC3(out, 0, 1, 0, 1e-5);

    // Rotation preserves length whatever the axis.
    const float axis = std::sqrt(1.0f / 3.0f);
    const float qd[4] = {axis * 0.5f, axis * 0.5f, axis * 0.5f, std::sqrt(0.75f)};
    const float lenBefore = std::sqrt(Dot(v, v));
    QuatRotate(qd, v, out);
    CHECK_NEAR(std::sqrt(Dot(out, out)), lenBefore, 1e-4);
}

static void TestQuatConjugate()
{
    test::Suite("quaternion conjugate");

    const float q[4] = {0.2f, -0.4f, 0.5f, 0.74162f};
    float inv[4];
    QuatConjugate(q, inv);
    CHECK_NEAR(inv[0], -0.2, 1e-6);
    CHECK_NEAR(inv[3], q[3], 1e-6);

    // Rotating by q then by its conjugate is the identity, which is what the collision code
    // relies on to move between world space and a collider's local space.
    const float v[3] = {3.0f, -1.5f, 2.25f};
    float rotated[3], back[3];
    QuatRotate(q, v, rotated);
    QuatRotate(inv, rotated, back);
    CHECK_VEC3(back, v[0], v[1], v[2], 1e-4);
}

static void TestQuatIntegrate()
{
    test::Suite("quaternion integration");

    // No spin leaves the orientation alone.
    float q[4] = {0, 0, 0, 1};
    const float still[3] = {0, 0, 0};
    QuatIntegrate(q, still, 0.016f);
    CHECK_VEC3(q, 0, 0, 0, 1e-6);
    CHECK_NEAR(q[3], 1.0, 1e-6);

    // A known spin for a known time lands on the expected angle, given small enough steps.
    // The integrator is first order, so a quarter turn taken in one step is around 14 degrees
    // short; the contract is that the error shrinks with the step, not that any step is exact.
    const float spin[3] = {0, 0, 3.14159265f};   // pi rad/s, so 0.5 s is a quarter turn
    const float x[3] = {1, 0, 0};
    float out[3];

    auto quarterTurnError = [&](int steps) {
        float q[4] = {0, 0, 0, 1};
        const float dt = 0.5f / float(steps);
        for (int i = 0; i < steps; ++i)
            QuatIntegrate(q, spin, dt);
        float r[3];
        QuatRotate(q, x, r);
        return std::sqrt((r[0] - 0.0f) * (r[0] - 0.0f) + (r[1] - 1.0f) * (r[1] - 1.0f) +
                         r[2] * r[2]);
    };

    const double coarse = quarterTurnError(1);
    const double fine = quarterTurnError(500);
    CHECK(fine < coarse);          // more steps, less error
    CHECK_NEAR(fine, 0.0, 5e-3);   // and at a realistic substep it is accurate

    // Direction is right even when the step is coarse: a quarter turn about z carries x toward
    // +y, never toward -y.
    float qz[4] = {0, 0, 0, 1};
    QuatIntegrate(qz, spin, 0.5f);
    QuatRotate(qz, x, out);
    CHECK(out[1] > 0.9f);

    // The result stays normalised however long it is integrated, which is what stops a piece's
    // orientation drifting into a scale over a long tumble.
    float qd[4] = {0, 0, 0, 1};
    const float fast[3] = {5.0f, -3.0f, 2.0f};
    for (int i = 0; i < 2000; ++i)
        QuatIntegrate(qd, fast, 0.01f);
    const float len = std::sqrt(qd[0]*qd[0] + qd[1]*qd[1] + qd[2]*qd[2] + qd[3]*qd[3]);
    CHECK_NEAR(len, 1.0, 1e-4);

    // An all-zero quaternion is not a rotation. It normalises to identity rather than to a
    // half turn, which is what a freshly grown piece slot arrives holding.
    float zero[4] = {0, 0, 0, 0};
    QuatIntegrate(zero, still, 0.016f);
    CHECK_VEC3(zero, 0, 0, 0, 1e-6);
    CHECK_NEAR(zero[3], 1.0, 1e-6);
}

static void TestMix()
{
    test::Suite("integer hash");

    CHECK_EQ(Mix(12345u), Mix(12345u));          // deterministic
    CHECK(Mix(0u) != Mix(1u));
    CHECK(Mix(7u) != 7u);

    // Consecutive inputs must not produce consecutive or clustered outputs: adjacent piece
    // slots would otherwise tumble along a visibly regular sweep.
    int distinctHighBits = 0;
    unsigned seen = 0;
    for (unsigned i = 0; i < 32; ++i) {
        const unsigned bucket = Mix(1000u + i) >> 27;   // top 5 bits
        if (!(seen & (1u << bucket))) { seen |= (1u << bucket); ++distinctHighBits; }
    }
    CHECK(distinctHighBits >= 16);
}

static void TestNextFloat()
{
    test::Suite("uniform sampling");

    uint32_t state = 12345u;
    float lo = 1.0f, hi = 0.0f;
    double sum = 0.0;
    const int n = 20000;
    for (int i = 0; i < n; ++i) {
        const float f = NextFloat(state);
        CHECK(f >= 0.0f && f < 1.0f) || printf("        sample %d was %.9g\n", i, f);
        if (f < lo) lo = f;
        if (f > hi) hi = f;
        sum += f;
    }
    CHECK_NEAR(sum / n, 0.5, 0.02);   // roughly uniform
    CHECK(lo < 0.02f);                // covers the bottom of the range
    CHECK(hi > 0.98f);                // and the top

    // Same seed, same sequence: a piece's tumble is reproducible from where it spawned.
    uint32_t a = 999u, b = 999u;
    for (int i = 0; i < 8; ++i)
        CHECK_NEAR(NextFloat(a), NextFloat(b), 0.0);
}

static void TestPositionKey()
{
    test::Suite("position fingerprint");

    const float p[3] = {123.5f, -64.25f, 900.125f};
    const float same[3] = {123.5f, -64.25f, 900.125f};
    CHECK(PositionKey(p) == PositionKey(same));

    // The engine hands transforms back bit for bit, so the key must separate positions that
    // differ by a single float step. A piece found at its old key is the same piece.
    float nudged[3] = {123.5f, -64.25f, 900.125f};
    nudged[2] = std::nextafter(nudged[2], 1e30f);
    CHECK(PositionKey(p) != PositionKey(nudged));

    // Distinct positions across a realistic spread must not collide.
    std::vector<uint64_t> keys;
    for (int i = 0; i < 4096; ++i) {
        const float q[3] = {float(i) * 0.5f, float(i % 37) * 3.25f, float(i % 71) * -1.75f};
        keys.push_back(PositionKey(q));
    }
    std::sort(keys.begin(), keys.end());
    CHECK_EQ(std::adjacent_find(keys.begin(), keys.end()) == keys.end() ? 0 : 1, 0);
}

int main()
{
    printf("Math3D\n");
    TestDotCrossMat3();
    TestQuatRotate();
    TestQuatConjugate();
    TestQuatIntegrate();
    TestMix();
    TestNextFloat();
    TestPositionKey();
    return test::Report("Math3D");
}
