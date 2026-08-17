// Many pieces stepped over time, rather than one interaction in isolation.
//
// The unit tests cover each rule on its own, and each of them can be right while the system
// built from them is wrong. Sleeping is the clearest case: a contact correctly reports which
// piece is on top, a settle pass correctly parks a piece that is held up and barely moving, and
// together they will park a cluster of falling debris in the sky, because every piece in it is
// held up by another piece that is also falling.
//
// So these run a small world forward: gravity, a ground plane, pieces against pieces, and the
// same settle and wake rules the solver applies, then assert on where everything ended up. The
// invariant that matters is that nothing sleeps in mid-air and nothing sinks through the floor.

#include "PairContact.h"
#include "TestHarness.h"

#include <cmath>
#include <cstdint>
#include <vector>

using namespace flexrevive;
using namespace f4kit;
using namespace flexrevive::pairs;

namespace {

// The solver substeps; a chunk resting on another gains only a few units/s of closing speed per
// step. Stepping at frame rate instead adds 11 units/s each time, which is enough on its own to
// keep a settling piece above the sleep threshold for ever.
constexpr float kDt = 1.0f / 240.0f;
constexpr float kGround = 0.0f;
constexpr float kRadius = 5.0f;
constexpr float kSleepSpeed = 25.0f;
const float kGravity[3] = {0.0f, 0.0f, -686.6f};

// A world of equal chunks over a flat floor.
struct Scene {
    std::vector<float> pos, vel, ang;
    std::vector<uint8_t> resting;
    std::vector<uint8_t> supported;

    int Count() const { return int(resting.size()); }

    void Add(float x, float y, float z)
    {
        pos.insert(pos.end(), {x, y, z});
        vel.insert(vel.end(), {0.0f, 0.0f, 0.0f});
        ang.insert(ang.end(), {0.0f, 0.0f, 0.0f});
        resting.push_back(0);
        supported.push_back(0);
    }

    float Z(int i) const { return pos[size_t(i) * 3 + 2]; }
    float Speed(int i) const
    {
        const float* v = &vel[size_t(i) * 3];
        return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    Body At(int i)
    {
        return Body{&pos[size_t(i) * 3], &vel[size_t(i) * 3], &ang[size_t(i) * 3], 1.0f, kRadius,
                    resting[size_t(i)] != 0};
    }

    Settings Rules() const
    {
        Settings s;
        s.contactSkin = 0.5f;
        s.restitution = 0.25f;
        s.mu = 0.6f;
        s.spinDamp = 36.0f;
        s.linearDamp = 14.0f;
        s.sleepSpeed = kSleepSpeed;
        s.dt = kDt;
        s.relaxation = 0.5f;
        return s;
    }

    // One step, in the order the solver runs it: pieces against pieces, then the pieces the pile
    // is holding up settle, then whatever is still awake moves and meets the floor.
    void Step()
    {
        const int n = Count();
        supported.assign(size_t(n), 0);

        for (int i = 0; i < n; ++i) {
            for (int j = i + 1; j < n; ++j) {
                float sep[3] = {pos[size_t(i) * 3 + 0] - pos[size_t(j) * 3 + 0],
                                pos[size_t(i) * 3 + 1] - pos[size_t(j) * 3 + 1],
                                pos[size_t(i) * 3 + 2] - pos[size_t(j) * 3 + 2]};
                const float d = std::sqrt(sep[0] * sep[0] + sep[1] * sep[1] + sep[2] * sep[2]);
                if (d < 1e-4f || d > 2.0f * kRadius + 1.0f)
                    continue;
                for (int a = 0; a < 3; ++a)
                    sep[a] /= d;

                Body bi = At(i), bj = At(j);
                Result r;
                if (!Resolve(bi, bj, sep, d, kGravity, Rules(), r))
                    continue;

                if (r.supported == kSupportedA)
                    supported[size_t(i)] = 1;
                else if (r.supported == kSupportedB)
                    supported[size_t(j)] = 1;

                if (r.wakeA)
                    resting[size_t(i)] = 0;
                if (r.wakeB)
                    resting[size_t(j)] = 0;
            }
        }

        for (int i = 0; i < n; ++i) {
            if (!supported[size_t(i)] || resting[size_t(i)])
                continue;
            if (Speed(i) < kSleepSpeed) {
                for (int a = 0; a < 3; ++a)
                    vel[size_t(i) * 3 + a] = 0.0f;
                resting[size_t(i)] = 1;
            }
        }

        for (int i = 0; i < n; ++i) {
            if (!resting[size_t(i)]) {
                for (int a = 0; a < 3; ++a) {
                    vel[size_t(i) * 3 + a] += kGravity[a] * kDt;
                    pos[size_t(i) * 3 + a] += vel[size_t(i) * 3 + a] * kDt;
                }
            }

            // The floor. This applies to every piece, awake or not: a sleeping chunk at the
            // bottom of a heap is still pushed about by the pieces above it, and only the floor
            // stops that from walking it downward out of the world.
            float& z = pos[size_t(i) * 3 + 2];
            if (z < kGround + kRadius) {
                z = kGround + kRadius;
                float* v = &vel[size_t(i) * 3];
                if (v[2] < 0.0f)
                    v[2] = 0.0f;
                if (Speed(i) < kSleepSpeed) {
                    v[0] = v[1] = v[2] = 0.0f;
                    resting[size_t(i)] = 1;
                }
            }
        }
    }

    void Run(int steps)
    {
        for (int s = 0; s < steps; ++s)
            Step();
    }

    // The highest piece that has gone to sleep without reaching the floor.
    float HighestSleepingInAir() const
    {
        float worst = kGround;
        for (int i = 0; i < Count(); ++i)
            if (resting[size_t(i)] && Z(i) > kGround + kRadius * 4.0f)
                worst = std::max(worst, Z(i));
        return worst;
    }

    float Lowest() const
    {
        float lo = 1e30f;
        for (int i = 0; i < Count(); ++i)
            lo = std::min(lo, Z(i));
        return lo;
    }
};

} // namespace

static void TestSingleChunkFalls()
{
    test::Suite("one chunk over a floor");

    Scene s;
    s.Add(0.0f, 0.0f, 1000.0f);
    s.Run(1200);

    CHECK_NEAR(s.Z(0), kGround + kRadius, 0.5);
    CHECK(s.resting[0]);
}

static void TestFallingClusterDoesNotHoldItselfUp()
{
    test::Suite("a cluster of falling debris");

    // Pieces packed close enough to touch, dropped together from a height. Every one of them is
    // in contact with several others the whole way down. None of them may take that contact for
    // support: this is the mid-air debris, and the reason it was visible as a flat sheet is that
    // the whole cluster slept at whatever height it happened to be at.
    Scene s;
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 5; ++y)
            for (int z = 0; z < 3; ++z)
                s.Add(float(x) * 9.0f, float(y) * 9.0f, 4000.0f + float(z) * 9.0f);

    const int n = s.Count();
    CHECK_EQ(n, 75);

    // Part way down, nothing has parked in the air.
    s.Run(120);
    for (int i = 0; i < n; ++i)
        CHECK(!s.resting[size_t(i)]);
    CHECK(s.Lowest() < 4000.0f);

    s.Run(2000);

    // Everything is on the floor, and nothing is asleep above it.
    CHECK_NEAR(s.HighestSleepingInAir(), kGround, 1e-6);
    for (int i = 0; i < n; ++i)
        CHECK(s.Z(i) < kGround + kRadius * 8.0f);
}

static void TestPileOnTheFloorStillSettles()
{
    test::Suite("a heap standing on the floor");

    // The fix must not stop a real pile settling. Support has to climb from the floor upward: a
    // piece resting on the ground holds up the one above it, which holds up the next. Stacked
    // just inside the touching distance, so every neighbour is in contact.
    Scene s;
    for (int z = 0; z < 6; ++z)
        s.Add(0.0f, 0.0f, kGround + kRadius + float(z) * 10.4f);

    s.Run(2400);

    for (int i = 0; i < s.Count(); ++i) {
        CHECK(s.resting[size_t(i)]);
        CHECK_NEAR(s.Speed(i), 0.0, 1e-6);
    }

    // And it is still a stack, not a pile that has sunk into the floor.
    CHECK(s.Lowest() >= kGround + kRadius - 0.5f);
}

static void TestNothingSinksThroughTheFloor()
{
    test::Suite("the floor holds");

    Scene s;
    for (int i = 0; i < 40; ++i)
        s.Add(float(i % 8) * 9.0f, float(i / 8) * 9.0f, 2000.0f + float(i) * 3.0f);

    s.Run(2000);

    CHECK(s.Lowest() >= kGround + kRadius - 0.5f);
}

static void TestDisplacedSleeperFalls()
{
    test::Suite("a settled piece moved somewhere else");

    // What the engine does when it recycles a slot: a piece that was asleep on the floor is
    // handed back high in the air. It has to wake and fall, or it hangs where it was put.
    Scene s;
    s.Add(0.0f, 0.0f, kGround + kRadius);
    s.Run(40);
    CHECK(s.resting[0]);

    s.pos[2] = 5000.0f;
    s.resting[0] = 0;      // what OnReseed decides for a displaced piece
    s.vel[0] = s.vel[1] = s.vel[2] = 0.0f;

    s.Run(1600);
    CHECK_NEAR(s.Z(0), kGround + kRadius, 0.5);
}

int main()
{
    printf("Scene\n");
    TestSingleChunkFalls();
    TestFallingClusterDoesNotHoldItselfUp();
    TestPileOnTheFloorStillSettles();
    TestNothingSinksThroughTheFloor();
    TestDisplacedSleeperFalls();
    return test::Report("Scene");
}
