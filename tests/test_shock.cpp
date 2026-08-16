// Impact shock: how a burst of fresh debris disturbs the rubble already lying around it.

#include "Shock.h"
#include "TestHarness.h"

#include <vector>

using namespace flexrevive;
using namespace f4kit;
using namespace flexrevive::shock;

static std::vector<Impact> OneImpact(float x, float y, float z, float deltaV, float radius)
{
    std::vector<Impact> v;
    Impact im;
    im.pos[0] = x; im.pos[1] = y; im.pos[2] = z;
    im.deltaV = deltaV;
    im.radius = radius;
    v.push_back(im);
    return v;
}

static float Length(const float* v)
{
    return std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
}

static void TestReach()
{
    test::Suite("reach and falloff");

    const auto impacts = OneImpact(0, 0, 0, 50.0f, 100.0f);
    float push[3];

    // Outside the radius nothing is felt.
    const float far[3] = {200, 0, 0};
    CHECK(!PushFor(impacts, far, 1.0f, push));

    // Just inside, the push is small; close in, it is large. The falloff is linear in
    // distance, so at half the radius it is half strength.
    const float near[3] = {10, 0, 0};
    const float mid[3] = {50, 0, 0};
    const float edge[3] = {99, 0, 0};
    CHECK(PushFor(impacts, near, 1.0f, push));
    const float nearMag = Length(push);
    CHECK(PushFor(impacts, mid, 1.0f, push));
    const float midMag = Length(push);
    CHECK(PushFor(impacts, edge, 1.0f, push));
    const float edgeMag = Length(push);

    CHECK(nearMag > midMag);
    CHECK(midMag > edgeMag);
    CHECK_NEAR(edgeMag, 0.0, 2.0);   // vanishes at the boundary rather than cutting off

    // A piece exactly at the centre has no direction to leave by, and must not produce a NaN.
    const float centre[3] = {0, 0, 0};
    PushFor(impacts, centre, 1.0f, push);
    for (int a = 0; a < 3; ++a)
        CHECK(std::isfinite(push[a]));
}

static void TestDirectionAndLift()
{
    test::Suite("direction and lift");

    const auto impacts = OneImpact(0, 0, 0, 60.0f, 100.0f);
    float push[3];

    // The shove runs outward from the impact, not inward.
    const float east[3] = {40, 0, 0};
    CHECK(PushFor(impacts, east, 1.0f, push));
    CHECK(push[0] > 0.0f);

    const float west[3] = {-40, 0, 0};
    CHECK(PushFor(impacts, west, 1.0f, push));
    CHECK(push[0] < 0.0f);

    // A little lift is added, so a heap scatters rather than sliding flat along the floor.
    // A piece level with the impact still gets upward motion.
    const float level[3] = {40, 0, 0};
    CHECK(PushFor(impacts, level, 1.0f, push));
    CHECK(push[2] > 0.0f);
}

static void TestMassScaling()
{
    test::Suite("mass scaling");

    const auto impacts = OneImpact(0, 0, 0, 40.0f, 200.0f);
    float light[3], heavy[3];

    const float at[3] = {50, 0, 0};
    CHECK(PushFor(impacts, at, 1.0f, light));
    CHECK(PushFor(impacts, at, 8.0f, heavy));

    // A shock carries momentum: pressure follows cross section while inertia follows volume,
    // so the speed picked up falls off as the cube root of mass. Eight times the mass moves at
    // half the speed, not an eighth of it.
    CHECK_NEAR(Length(heavy), Length(light) / 2.0, 0.5);

    // A heavier chunk always moves less, never more.
    float previous = 1e30f;
    for (float m : {1.0f, 2.0f, 10.0f, 100.0f, 1000.0f}) {
        float p[3];
        CHECK(PushFor(impacts, at, m, p));
        CHECK(Length(p) < previous);
        previous = Length(p);
    }

    // Mass below one unit is clamped, so a degenerate chunk cannot be launched absurdly.
    float tiny[3], unit[3];
    CHECK(PushFor(impacts, at, 0.0001f, tiny));
    CHECK(PushFor(impacts, at, 1.0f, unit));
    CHECK_NEAR(Length(tiny), Length(unit), 1e-3);
}

static void TestCap()
{
    test::Suite("speed cap");

    float push[3];

    // A single enormous burst is still capped.
    const auto huge = OneImpact(0, 0, 0, 10000.0f, 500.0f);
    const float at[3] = {10, 0, 0};
    CHECK(PushFor(huge, at, 1.0f, push));
    CHECK(Length(push) <= kMaxShockSpeed + 1e-3f);

    // And so is a pile-up of many. Sustained fire into one spot lands impacts inside the heap
    // many times a second; applied one at a time they would compound until the rubble was
    // thrown clear, so everything reaching one chunk is summed and capped together.
    std::vector<Impact> many;
    for (int i = 0; i < 40; ++i) {
        Impact im;
        im.pos[0] = 0; im.pos[1] = 0; im.pos[2] = 0;
        im.deltaV = 80.0f;
        im.radius = 300.0f;
        many.push_back(im);
    }
    CHECK(PushFor(many, at, 1.0f, push));
    CHECK(Length(push) <= kMaxShockSpeed + 1e-3f);

    // The cap limits magnitude without changing which way the piece goes.
    CHECK(push[0] > 0.0f);
}

static void TestAccumulation()
{
    test::Suite("several impacts at once");

    // Two impacts on opposite sides of a piece largely cancel, which is what summing before
    // applying gives and what applying one at a time would not.
    std::vector<Impact> pair;
    for (float x : {-50.0f, 50.0f}) {
        Impact im;
        im.pos[0] = x; im.pos[1] = 0; im.pos[2] = 0;
        im.deltaV = 30.0f;
        im.radius = 100.0f;
        pair.push_back(im);
    }
    float push[3];
    const float between[3] = {0, 0, 0};
    CHECK(PushFor(pair, between, 1.0f, push));
    CHECK_NEAR(push[0], 0.0, 1.0);   // horizontal components oppose
    CHECK(push[2] > 0.0f);           // the lift from both still adds

    // An empty list moves nothing.
    std::vector<Impact> none;
    const float anywhere[3] = {5, 5, 5};
    CHECK(!PushFor(none, anywhere, 1.0f, push));
}

static void TestRecordMergesBursts()
{
    test::Suite("recording bursts");

    std::vector<Impact> impacts;

    const float origin[3] = {0, 0, 0};
    const float close[3] = {10, 0, 0};
    const float distant[3] = {1000, 0, 0};

    // Fragments from one impact arrive as a cluster, so shoves close together are one event.
    Record(impacts, origin, 40.0f);
    Record(impacts, close, 40.0f);
    CHECK_EQ(int(impacts.size()), 1);

    // Two hits a weapon's spread apart are separate events, not an average of the pair.
    Record(impacts, distant, 40.0f);
    CHECK_EQ(int(impacts.size()), 2);

    // Merging keeps the stronger of the two rather than the later one.
    std::vector<Impact> strength;
    const float near2[3] = {5, 0, 0};
    Record(strength, origin, 90.0f);
    Record(strength, near2, 10.0f);
    CHECK_EQ(int(strength.size()), 1);
    CHECK_NEAR(strength[0].deltaV, 90.0, 1e-3);

    // The list is bounded, so a pathological frame cannot grow it without limit.
    std::vector<Impact> flood;
    for (int i = 0; i < 500; ++i) {
        const float p[3] = {float(i) * 1000.0f, 0, 0};
        Record(flood, p, 20.0f);
    }
    CHECK(int(flood.size()) <= kMaxImpacts);
}

int main()
{
    printf("Shock\n");
    TestReach();
    TestDirectionAndLift();
    TestMassScaling();
    TestCap();
    TestAccumulation();
    TestRecordMergesBursts();
    return test::Report("Shock");
}
