// Two chunks resolving against each other: separating, shedding their relative motion,
// exchanging a closing impulse, and deciding what should wake.
//
// The checks lean on conservation rather than on particular numbers. Separation must leave the
// pair's centre of mass where it was, the impulse must leave their momentum unchanged, and
// damping must remove only the motion between them. A sign or share error breaks one of those
// even when the visible result looks plausible.

#include <cmath>
#include "PairContact.h"
#include "TestHarness.h"

#include <vector>

using namespace flexrevive;
using namespace f4kit;
using namespace flexrevive::pairs;

namespace {

struct Chunk {
    float pos[3] = {0, 0, 0};
    float vel[3] = {0, 0, 0};
    float ang[3] = {0, 0, 0};
    float mass = 1.0f;
    float extent = 5.0f;
    bool resting = false;

    Body ref() { return Body{pos, vel, ang, mass, extent, resting}; }
};

Settings Defaults()
{
    Settings s;
    s.contactSkin = 0.5f;
    s.restitution = 0.25f;
    s.mu = 0.6f;
    s.spinDamp = 36.0f;
    s.linearDamp = 14.0f;
    s.sleepSpeed = 25.0f;
    s.dt = 0.01f;
    s.relaxation = 0.5f;
    return s;
}

const float kDown[3] = {0, 0, -686.6f};
const float kAlongX[3] = {1, 0, 0};   // separation runs from B toward A

float CentreOfMass(const Chunk& a, const Chunk& b, int axis)
{
    return (a.mass * a.pos[axis] + b.mass * b.pos[axis]) / (a.mass + b.mass);
}

float Momentum(const Chunk& a, const Chunk& b, int axis)
{
    return a.mass * a.vel[axis] + b.mass * b.vel[axis];
}

} // namespace

static void TestNoContact()
{
    test::Suite("pieces that are not touching");

    Chunk a, b;
    a.pos[0] = 100.0f;
    b.pos[0] = 0.0f;
    Body ba = a.ref(), bb = b.ref();
    Result r;

    // Well apart: nothing happens and nothing is written.
    CHECK(!Resolve(ba, bb, kAlongX, 100.0f, kDown, Defaults(), r));
    CHECK_NEAR(a.pos[0], 100.0, 1e-6);
    CHECK_NEAR(b.pos[0], 0.0, 1e-6);
    CHECK(!r.wakeA && !r.wakeB);

    // Exactly at the touching distance is still not an overlap.
    CHECK(!Resolve(ba, bb, kAlongX, 5.0f + 5.0f + 0.5f, kDown, Defaults(), r));
}

static void TestSeparationPreservesCentreOfMass()
{
    test::Suite("separation");

    // Equal masses yield equally.
    {
        Chunk a, b;
        a.pos[0] = 6.0f;
        Body ba = a.ref(), bb = b.ref();
        Result r;
        const float before = CentreOfMass(a, b, 0);
        CHECK(Resolve(ba, bb, kAlongX, 6.0f, kDown, Defaults(), r));
        CHECK(r.overlap > 0.0f);
        CHECK(a.pos[0] > 6.0f);   // pushed apart, along the separation
        CHECK(b.pos[0] < 0.0f);
        // Only a share of the overlap is taken out in one pass. A piece in a heap is touched
        // once per overlapping neighbour and every correction lands in the same pass, so
        // resolving each contact in full would move it as many times further as it has
        // neighbours and fire it out of the pile.
        CHECK_NEAR(a.pos[0] - b.pos[0] - 6.0f, r.overlap * Defaults().relaxation, 1e-3);
        CHECK_NEAR(CentreOfMass(a, b, 0), before, 1e-3);
    }

    // A heavy chunk barely yields; the light one does the moving. The pair's centre of mass
    // still does not shift, which is what makes the split a redistribution rather than a shove.
    {
        Chunk a, b;
        a.pos[0] = 6.0f;
        a.mass = 100.0f;
        b.mass = 1.0f;
        Body ba = a.ref(), bb = b.ref();
        Result r;
        const float before = CentreOfMass(a, b, 0);
        CHECK(Resolve(ba, bb, kAlongX, 6.0f, kDown, Defaults(), r));
        CHECK(std::fabs(a.pos[0] - 6.0f) < std::fabs(b.pos[0]));
        CHECK_NEAR(CentreOfMass(a, b, 0), before, 1e-3);
    }
}

static void TestDampingRemovesOnlyRelativeMotion()
{
    test::Suite("damping");

    // Two chunks travelling together keep travelling together: a pile sliding down a slope
    // still slides, and only the grinding between neighbours is taken out.
    {
        Chunk a, b;
        a.pos[0] = 6.0f;
        for (int i = 0; i < 3; ++i) a.vel[i] = b.vel[i] = 30.0f;
        Body ba = a.ref(), bb = b.ref();
        Result r;
        Resolve(ba, bb, kAlongX, 6.0f, kDown, Defaults(), r);
        for (int i = 0; i < 3; ++i)
            CHECK_NEAR(a.vel[i], b.vel[i], 1e-3);   // still moving as one
    }

    // Grinding against each other is reduced, and their mean motion is untouched by it.
    {
        Chunk a, b;
        a.pos[0] = 6.0f;
        a.vel[1] = 40.0f;
        b.vel[1] = -40.0f;
        Body ba = a.ref(), bb = b.ref();
        Result r;
        const float meanBefore = (a.vel[1] + b.vel[1]) * 0.5f;
        const float gapBefore = std::fabs(a.vel[1] - b.vel[1]);
        Resolve(ba, bb, kAlongX, 6.0f, kDown, Defaults(), r);
        CHECK(std::fabs(a.vel[1] - b.vel[1]) < gapBefore);
        CHECK_NEAR((a.vel[1] + b.vel[1]) * 0.5f, meanBefore, 1e-3);
    }

    // Spin is shed too, and never reversed.
    {
        Chunk a, b;
        a.pos[0] = 6.0f;
        a.ang[0] = 10.0f;
        b.ang[0] = -4.0f;
        Body ba = a.ref(), bb = b.ref();
        Result r;
        Resolve(ba, bb, kAlongX, 6.0f, kDown, Defaults(), r);
        CHECK(a.ang[0] < 10.0f && a.ang[0] >= 0.0f);
        CHECK(b.ang[0] > -4.0f && b.ang[0] <= 0.0f);
    }
}

static void TestImpulseConservesMomentum()
{
    test::Suite("closing impulse");

    // A light chunk driven into a heavy one bounces off it; the heavy one barely notices, and
    // between them momentum is unchanged.
    {
        Chunk a, b;
        a.pos[0] = 6.0f;
        a.mass = 1.0f;
        b.mass = 50.0f;
        a.vel[0] = -200.0f;   // closing on b
        Body ba = a.ref(), bb = b.ref();
        Result r;
        const float before = Momentum(a, b, 0);
        CHECK(Resolve(ba, bb, kAlongX, 6.0f, kDown, Defaults(), r));
        CHECK(a.vel[0] > -200.0f);                  // slowed or reversed
        CHECK(std::fabs(b.vel[0]) < 20.0f);         // hardly moved
        CHECK_NEAR(Momentum(a, b, 0), before, 1.0);
    }

    // Pieces already separating are not pulled back together.
    {
        Chunk a, b;
        a.pos[0] = 6.0f;
        a.vel[0] = 100.0f;    // moving away
        Body ba = a.ref(), bb = b.ref();
        Result r;
        Resolve(ba, bb, kAlongX, 6.0f, kDown, Defaults(), r);
        CHECK(a.vel[0] > 0.0f);
    }
}

static void TestWakeDecision()
{
    test::Suite("what wakes");

    Settings s = Defaults();

    // A settled pair sharing a little residual overlap wakes nothing. A heap holds this state
    // everywhere, so waking on it alone would spread from neighbour to neighbour until the
    // whole pile was awake with nothing having pushed it.
    {
        Chunk a, b;
        a.pos[0] = 10.2f;     // overlap just under the settle gap
        a.resting = b.resting = true;
        Body ba = a.ref(), bb = b.ref();
        Result r;
        Resolve(ba, bb, kAlongX, 10.2f, kDown, s, r);
        CHECK(!r.wakeA && !r.wakeB);
    }

    // The same overlap with one of them genuinely moving does wake both.
    {
        Chunk a, b;
        a.pos[0] = 8.0f;
        a.vel[1] = 200.0f;
        a.resting = b.resting = true;
        Body ba = a.ref(), bb = b.ref();
        Result r;
        Resolve(ba, bb, kAlongX, 8.0f, kDown, s, r);
        CHECK(r.wakeA && r.wakeB);
    }

    // A piece driven deep into another wakes regardless of speed, so one that is genuinely
    // displaced is never left hanging where it was pushed.
    {
        Chunk a, b;
        a.pos[0] = 1.0f;      // buried
        a.resting = b.resting = true;
        Body ba = a.ref(), bb = b.ref();
        Result r;
        Resolve(ba, bb, kAlongX, 1.0f, kDown, s, r);
        CHECK(r.wakeA && r.wakeB);
    }
}

static void TestSupport()
{
    test::Suite("which piece is held up");

    const float up[3] = {0, 0, 1};
    const float down[3] = {0, 0, -1};

    // The separation runs from B toward A, so pointing against gravity puts A on top. A is held
    // up only because B is at rest: support is something a stationary piece provides.
    {
        Chunk a, b;
        b.resting = true;
        Body ba = a.ref(), bb = b.ref();
        Result r;
        Resolve(ba, bb, up, 6.0f, kDown, Defaults(), r);
        CHECK_EQ(r.supported, kSupportedA);
    }
    {
        Chunk a, b;
        a.resting = true;
        Body ba = a.ref(), bb = b.ref();
        Result r;
        Resolve(ba, bb, down, 6.0f, kDown, Defaults(), r);
        CHECK_EQ(r.supported, kSupportedB);
    }

    // A piece resting on one that is itself falling is not held up by it. Both are on their way
    // down; nothing here is standing on anything.
    {
        Chunk a, b;
        Body ba = a.ref(), bb = b.ref();
        Result r;
        Resolve(ba, bb, up, 6.0f, kDown, Defaults(), r);
        CHECK_EQ(r.supported, kSupportedNeither);
    }

    // Side by side, neither holds the other up, whatever their state.
    {
        Chunk a, b;
        a.resting = b.resting = true;
        Body ba = a.ref(), bb = b.ref();
        Result r;
        Resolve(ba, bb, kAlongX, 6.0f, kDown, Defaults(), r);
        CHECK_EQ(r.supported, kSupportedNeither);
    }

    // Support never comes from a piece in motion, at any contact angle. Without this a cluster
    // of falling debris holds itself up: each piece is "supported" by a neighbour that is also
    // falling, the whole group is put to sleep, and it hangs in mid-air.
    for (int step = 0; step <= 32; ++step) {
        const float t = float(step) / 32.0f * 6.2831853f;
        const float dir[3] = {std::cos(t), 0.0f, std::sin(t)};
        Chunk a, b;
        Body ba = a.ref(), bb = b.ref();
        Result r;
        Resolve(ba, bb, dir, 6.0f, kDown, Defaults(), r);
        CHECK_EQ(r.supported, kSupportedNeither);
    }

    // With gravity switched off nothing is holding anything up either way.
    {
        Chunk a, b;
        b.resting = true;
        const float none[3] = {0, 0, 0};
        Body ba = a.ref(), bb = b.ref();
        Result r;
        Resolve(ba, bb, up, 6.0f, none, Defaults(), r);
        CHECK_EQ(r.supported, kSupportedNeither);
    }
}

static void TestDegenerate()
{
    test::Suite("degenerate inputs");

    Chunk a, b;
    a.pos[0] = 1.0f;
    a.mass = 0.0f;
    b.mass = 0.0f;
    a.extent = 0.0f;
    b.extent = 0.0f;
    Body ba = a.ref(), bb = b.ref();
    Result r;
    Settings s = Defaults();
    s.dt = 0.0f;
    Resolve(ba, bb, kAlongX, 0.0f, kDown, s, r);
    for (int i = 0; i < 3; ++i) {
        CHECK(std::isfinite(a.pos[i]) && std::isfinite(a.vel[i]) && std::isfinite(a.ang[i]));
        CHECK(std::isfinite(b.pos[i]) && std::isfinite(b.vel[i]) && std::isfinite(b.ang[i]));
    }

    // A body with no angular storage is handled rather than dereferenced.
    Chunk c, d;
    c.pos[0] = 6.0f;
    Body bc = c.ref(), bd = d.ref();
    bc.ang = nullptr;
    bd.ang = nullptr;
    Resolve(bc, bd, kAlongX, 6.0f, kDown, Defaults(), r);
    CHECK(std::isfinite(c.pos[0]));
}


static void TestRelaxationConverges()
{
    test::Suite("relaxation");

    // Taking a share per pass still resolves the overlap, because the pass repeats: each
    // substep and each frame applies it again against whatever is left.
    Chunk a, b;
    a.pos[0] = 6.0f;
    Body ba = a.ref(), bb = b.ref();
    Result r;

    float gap = a.pos[0] - b.pos[0];
    for (int i = 0; i < 40; ++i) {
        ba = a.ref();
        bb = b.ref();
        if (!Resolve(ba, bb, kAlongX, a.pos[0] - b.pos[0], kDown, Defaults(), r))
            break;
        const float next = a.pos[0] - b.pos[0];
        CHECK(next >= gap);        // never pushed back together
        gap = next;
    }

    // Settled apart, at the touching distance rather than short of it.
    CHECK_NEAR(gap, 5.0 + 5.0 + Defaults().contactSkin, 0.2);

    // A relaxation of one takes the whole overlap in a single pass.
    Settings full = Defaults();
    full.relaxation = 1.0f;
    Chunk c, d;
    c.pos[0] = 6.0f;
    Body bc = c.ref(), bd = d.ref();
    Result r2;
    Resolve(bc, bd, kAlongX, 6.0f, kDown, full, r2);
    CHECK_NEAR(c.pos[0] - d.pos[0] - 6.0f, r2.overlap, 1e-3);

    // A nonsensical value cannot invert the correction or overshoot wildly.
    Settings odd = Defaults();
    odd.relaxation = -3.0f;
    Chunk e, f;
    e.pos[0] = 6.0f;
    Body be = e.ref(), bf = f.ref();
    Result r3;
    Resolve(be, bf, kAlongX, 6.0f, kDown, odd, r3);
    CHECK(e.pos[0] >= 6.0f);
    CHECK(std::isfinite(e.pos[0]) && std::isfinite(f.pos[0]));
}

int main()
{
    printf("PairContact\n");
    TestNoContact();
    TestSeparationPreservesCentreOfMass();
    TestRelaxationConverges();
    TestDampingRemovesOnlyRelativeMotion();
    TestImpulseConservesMomentum();
    TestWakeDecision();
    TestSupport();
    TestDegenerate();
    return test::Report("PairContact");
}
