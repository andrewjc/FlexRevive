// Rigid body contact against a chunk's own silhouette: support points, penetration depth, and
// the impulses that separate a piece and set it tumbling.

#include "Contact.h"
#include "TestHarness.h"

#include <algorithm>
#include <vector>

using namespace flexrevive;
using namespace f4kit;
using namespace flexrevive::contact;

// A hull built the way the plugin builds them: from a particle cloud, through the real
// builder. A hand-made box of eight corners would have no face-centre points, so every contact
// would land on a corner and nothing could ever rest flat.
static fragment::Hull MakeSlab(float hx, float hy, float hz)
{
    const float spacing = 1.0f;
    std::vector<float> pts;
    for (float x = -hx; x <= hx + 1e-3f; x += spacing)
        for (float y = -hy; y <= hy + 1e-3f; y += spacing)
            for (float z = -hz; z <= hz + 1e-3f; z += spacing) {
                pts.push_back(x); pts.push_back(y); pts.push_back(z);
            }
    fragment::Hull h;
    const bool ok = fragment::BuildFromPoints(pts.data(), int(pts.size() / 3), 3,
                                              spacing * 0.5f, h);
    if (!ok)
        h = fragment::Hull();
    return h;
}

static void TestSupportExtent()
{
    test::Suite("support extent");

    const fragment::Hull slab = MakeSlab(10.0f, 10.0f, 2.0f);
    const float identity[4] = {0, 0, 0, 1};

    const float down[3] = {0, 0, -1};
    const float alongX[3] = {1, 0, 0};

    // Unrotated, the slab reaches its half extent along each axis, plus the particle radius:
    // every hull point is a particle, so the chunk's surface is the hull swept by one.
    const double inflate = slab.inflate;
    CHECK_NEAR(SupportExtent(slab, identity, down), 2.0 + inflate, 0.05);
    CHECK_NEAR(SupportExtent(slab, identity, alongX), 10.0 + inflate, 0.05);

    // Stood on its edge, a quarter turn about y, the thin axis now points along x and the wide
    // one downward. This is the whole point of carrying a silhouette: reach depends on pose.
    const float s = std::sqrt(0.5f);
    const float qy90[4] = {0, s, 0, s};
    CHECK_NEAR(SupportExtent(slab, qy90, down), 10.0 + inflate, 0.05);
    CHECK_NEAR(SupportExtent(slab, qy90, alongX), 2.0 + inflate, 0.05);

    // A hull with no points has no reach rather than an undefined one.
    fragment::Hull empty;
    CHECK_NEAR(SupportExtent(empty, identity, down), 0.0, 1e-6);
}

static void TestFindHullContact()
{
    test::Suite("finding the contact");

    const fragment::Hull slab = MakeSlab(10.0f, 10.0f, 2.0f);
    const float identity[4] = {0, 0, 0, 1};
    const float up[3] = {0, 0, 1};

    // Sitting with its hull points level with the surface: the swept surface still reaches a
    // particle radius below them, so that is the depth reported.
    {
        const float pos[3] = {0, 0, 2.0f};
        const float surface[3] = {0, 0, 0};
        const HullContact c = FindHullContact(slab, identity, pos, surface, up);
        CHECK(c.valid);
        CHECK_NEAR(c.depth, slab.inflate, 0.05);
    }

    // Sunk one unit in: depth reports how far past the surface the support point sits.
    {
        const float pos[3] = {0, 0, 1.0f};
        const float surface[3] = {0, 0, 0};
        const HullContact c = FindHullContact(slab, identity, pos, surface, up);
        CHECK(c.valid);
        CHECK_NEAR(c.depth, 1.0 + slab.inflate, 0.05);
        // The lever arm points from the centre of mass down to the contact.
        CHECK(c.r[2] < 0.0f);
    }

    // Well clear of the surface: no contact.
    {
        const float pos[3] = {0, 0, 50.0f};
        const float surface[3] = {0, 0, 0};
        const HullContact c = FindHullContact(slab, identity, pos, surface, up);
        CHECK(!c.valid || c.depth < 0.0f);
    }
}

static void TestResolveSeparates()
{
    test::Suite("contact separates the piece");

    const fragment::Hull slab = MakeSlab(10.0f, 10.0f, 2.0f);
    const float identity[4] = {0, 0, 0, 1};
    const float up[3] = {0, 0, 1};
    const float surface[3] = {0, 0, 0};

    float pos[3] = {0, 0, 1.0f};              // one unit into the ground
    float vel[3] = {0, 0, -100.0f};           // falling
    float w[3] = {0, 0, 0};

    const HullContact c = FindHullContact(slab, identity, pos, surface, up);
    CHECK(c.valid);
    ResolveHullContact(slab, c, pos, vel, w, up, 0.25f, 0.6f, 0.5f);

    CHECK(pos[2] > 1.0f);        // lifted out
    CHECK(vel[2] > -100.0f);     // and no longer driving downward
}

static void TestResolveProducesTorque()
{
    test::Suite("off-centre contact produces spin");

    const fragment::Hull slab = MakeSlab(10.0f, 10.0f, 2.0f);
    const float up[3] = {0, 0, 1};
    const float surface[3] = {0, 0, 0};

    // Tilted, so the support point is off to one side of the centre of mass. A normal impulse
    // through an off-centre contact must generate a torque, which is what tips a landing chunk
    // down onto a flat face instead of leaving it at the angle it arrived at.
    const float a = 0.3f;
    const float qTilt[4] = {0, std::sin(a * 0.5f), 0, std::cos(a * 0.5f)};

    float pos[3] = {0, 0, 3.0f};
    float vel[3] = {0, 0, -80.0f};
    float w[3] = {0, 0, 0};

    const HullContact c = FindHullContact(slab, qTilt, pos, surface, up);
    CHECK(c.valid);
    const float spinBefore = std::fabs(w[0]) + std::fabs(w[1]) + std::fabs(w[2]);
    ResolveHullContact(slab, c, pos, vel, w, up, 0.25f, 0.6f, 0.5f);
    const float spinAfter = std::fabs(w[0]) + std::fabs(w[1]) + std::fabs(w[2]);
    CHECK(spinAfter > spinBefore);

    // A flat landing produces less torque than a tilted one, but not none. The hull records
    // one extreme point per direction, and for a flat face every point on it is equally
    // extreme, so the tie is broken by build order and lands on a corner. A single-point
    // contact solver therefore gives a flat-bottomed chunk a small tipping torque it would not
    // have against a distributed contact. Rolling resistance and pile damping absorb it.
    const float identity[4] = {0, 0, 0, 1};
    float pos2[3] = {0, 0, 1.0f};
    float vel2[3] = {0, 0, -80.0f};
    float w2[3] = {0, 0, 0};
    const HullContact flat = FindHullContact(slab, identity, pos2, surface, up);
    CHECK(flat.valid);
    ResolveHullContact(slab, flat, pos2, vel2, w2, up, 0.25f, 0.6f, 0.5f);
    const float flatSpin = std::fabs(w2[0]) + std::fabs(w2[1]) + std::fabs(w2[2]);
    CHECK(flatSpin < spinAfter);
    CHECK(std::isfinite(flatSpin));
}

static void TestRestitution()
{
    test::Suite("restitution");

    const fragment::Hull slab = MakeSlab(5.0f, 5.0f, 5.0f);
    const float identity[4] = {0, 0, 0, 1};
    const float up[3] = {0, 0, 1};
    const float surface[3] = {0, 0, 0};

    // The impulse is solved at the contact point, not at the centre of mass, so what
    // restitution governs is the normal speed of the material in contact. An off-centre contact
    // converts part of the impulse into spin, which is why the body's own velocity is not the
    // quantity to measure.
    //
    // Friction is zeroed here because rolling resistance runs after the normal impulse and
    // scales with it, so it would perturb exactly the quantity under test.
    auto contactNormalSpeed = [&](float restitution) {
        float pos[3] = {0, 0, 4.0f};
        float vel[3] = {0, 0, -200.0f};
        float w[3] = {0, 0, 0};
        const HullContact c = FindHullContact(slab, identity, pos, surface, up);
        ResolveHullContact(slab, c, pos, vel, w, up, restitution, 0.0f, 0.5f);
        const float wxr[3] = {w[1]*c.r[2] - w[2]*c.r[1],
                              w[2]*c.r[0] - w[0]*c.r[2],
                              w[0]*c.r[1] - w[1]*c.r[0]};
        return (vel[0]+wxr[0])*up[0] + (vel[1]+wxr[1])*up[1] + (vel[2]+wxr[2])*up[2];
    };

    // Restitution is the fraction of approach speed returned: e = 0 arrests the contact, and
    // e = 0.9 sends it back at nine tenths of the 200 units/s it arrived with.
    CHECK_NEAR(contactNormalSpeed(0.0f), 0.0, 1.0);
    CHECK_NEAR(contactNormalSpeed(0.9f), 180.0, 2.0);
    CHECK_NEAR(contactNormalSpeed(0.5f), 100.0, 2.0);

    // Monotonic in between, and never more than it arrived with.
    CHECK(contactNormalSpeed(0.9f) > contactNormalSpeed(0.5f));
    CHECK(contactNormalSpeed(1.0f) <= 200.0f + 1.0f);
}

static void TestFrictionScrubsTangentialSpeed()
{
    test::Suite("friction");

    const fragment::Hull slab = MakeSlab(5.0f, 5.0f, 5.0f);
    const float identity[4] = {0, 0, 0, 1};
    const float up[3] = {0, 0, 1};
    const float surface[3] = {0, 0, 0};

    auto slideAfter = [&](float friction) {
        float pos[3] = {0, 0, 4.0f};
        float vel[3] = {150.0f, 0, -50.0f};
        float w[3] = {0, 0, 0};
        const HullContact c = FindHullContact(slab, identity, pos, surface, up);
        ResolveHullContact(slab, c, pos, vel, w, up, 0.25f, friction, 0.5f);
        return vel[0];
    };

    const float slippy = slideAfter(0.0f);
    const float grippy = slideAfter(1.2f);
    CHECK(grippy < slippy);        // more grip, less slide
    CHECK(grippy >= -1.0f);        // friction never reverses the motion it opposes
}

static void TestDegenerateInputs()
{
    test::Suite("degenerate inputs");

    // A hull with no points must not produce a contact, and resolving one must leave the piece
    // untouched rather than writing a NaN into its position.
    fragment::Hull empty;
    const float identity[4] = {0, 0, 0, 1};
    const float up[3] = {0, 0, 1};
    const float surface[3] = {0, 0, 0};

    float pos[3] = {0, 0, 1.0f};
    float vel[3] = {0, 0, -50.0f};
    float w[3] = {0, 0, 0};
    const HullContact c = FindHullContact(empty, identity, pos, surface, up);
    if (c.valid)
        ResolveHullContact(empty, c, pos, vel, w, up, 0.25f, 0.6f, 0.5f);
    for (int a = 0; a < 3; ++a) {
        CHECK(std::isfinite(pos[a]));
        CHECK(std::isfinite(vel[a]));
        CHECK(std::isfinite(w[a]));
    }
}


static void TestSeparationIsBounded()
{
    test::Suite("separation is bounded");

    const fragment::Hull slab = MakeSlab(5.0f, 5.0f, 5.0f);
    const float identity[4] = {0, 0, 0, 1};
    const float up[3] = {0, 0, 1};

    // A piece that has ended up far inside geometry reports an enormous penetration. Lifting it
    // clear in one step would fling it the whole distance, which puts debris in the sky rather
    // than on the ground. The correction is capped and the piece climbs out over several steps.
    HullContact deep;
    deep.valid = true;
    deep.depth = 5000.0f;
    deep.r[0] = 0.0f; deep.r[1] = 0.0f; deep.r[2] = -5.0f;

    float pos[3] = {0, 0, 0};
    float vel[3] = {0, 0, 0};
    float w[3] = {0, 0, 0};
    ResolveHullContact(slab, deep, pos, vel, w, up, 0.25f, 0.6f, 0.5f);

    CHECK(pos[2] > 0.0f);                       // it does move outward
    CHECK(pos[2] < 100.0f);                     // but nothing like five thousand
    CHECK(pos[2] <= slab.radius * 8.0f + 1.0f); // bounded by the chunk's own size

    // An ordinary shallow contact is unaffected by the cap: it still separates fully.
    HullContact shallow;
    shallow.valid = true;
    shallow.depth = 1.0f;
    shallow.r[0] = 0.0f; shallow.r[1] = 0.0f; shallow.r[2] = -5.0f;

    float pos2[3] = {0, 0, 0};
    float vel2[3] = {0, 0, 0};
    float w2[3] = {0, 0, 0};
    ResolveHullContact(slab, shallow, pos2, vel2, w2, up, 0.25f, 0.6f, 0.5f);
    CHECK_NEAR(pos2[2], 1.0 + 0.5, 1e-3);

    // Repeated steps get a deeply buried piece out eventually rather than never.
    float climb[3] = {0, 0, 0};
    float cv[3] = {0, 0, 0}, cw[3] = {0, 0, 0};
    for (int i = 0; i < 20; ++i)
        ResolveHullContact(slab, deep, climb, cv, cw, up, 0.25f, 0.6f, 0.5f);
    CHECK(climb[2] > 100.0f);
}

static void TestSlowContactStopsBouncing()
{
    test::Suite("a piece settling stops rebounding");

    const fragment::Hull slab = MakeSlab(10.0f, 10.0f, 2.0f);
    const float up[3] = {0, 0, 1};
    const float surface[3] = {0, 0, 0};
    const float identity[4] = {0, 0, 0, 1};

    auto after = [&](float approach, float restitution) {
        float pos[3] = {0, 0, 1.9f};
        float vel[3] = {0, 0, -approach};
        float w[3] = {0, 0, 0};
        const HullContact c = FindHullContact(slab, identity, pos, surface, up);
        ResolveHullContact(slab, c, pos, vel, w, up, restitution, 0.6f, 0.5f);
        return vel[2];
    };

    // Gravity adds a few units/s of closing speed every substep. Returning a share of it on
    // each contact is what keeps a heap that has visibly stopped still twitching, so below the
    // threshold the surface must answer as though it had no bounce at all. Comparing a springy
    // surface against a dead one says that without depending on the geometry.
    for (float approach = 1.0f; approach < kNoBounceSpeed; approach += 2.0f)
        CHECK_NEAR(after(approach, 0.9f), after(approach, 0.0f), 1e-4);

    // Above it a springy surface must still behave differently from a dead one, or debris
    // would land dead on everything.
    CHECK(after(200.0f, 0.9f) > after(200.0f, 0.0f) + 1.0f);
    CHECK(after(400.0f, 0.9f) > after(400.0f, 0.0f) + 1.0f);
}

static void TestSlowPieceGripsTheSurface()
{
    test::Suite("a resting piece grips rather than creeping");

    const fragment::Hull slab = MakeSlab(10.0f, 10.0f, 2.0f);
    const float up[3] = {0, 0, 1};
    const float surface[3] = {0, 0, 0};
    const float identity[4] = {0, 0, 0, 1};

    // What friction acts on is the sliding at the contact itself, body motion plus the spin
    // carried round by the lever arm, so that is what has to come out. The body's own velocity
    // barely moves either way, because the impulse is mostly absorbed as rotation.
    auto sliding = [&](float tangential, float& before, float& after) {
        float pos[3] = {0, 0, 1.9f};
        float vel[3] = {tangential, 0.0f, -4.0f};
        float w[3] = {0, 0, 0};
        const HullContact c = FindHullContact(slab, identity, pos, surface, up);

        auto slide = [&](const float* v, const float* ww) {
            const float wxr[3] = {ww[1] * c.r[2] - ww[2] * c.r[1],
                                  ww[2] * c.r[0] - ww[0] * c.r[2],
                                  ww[0] * c.r[1] - ww[1] * c.r[0]};
            const float vc[3] = {v[0] + wxr[0], v[1] + wxr[1], v[2] + wxr[2]};
            const float vn = vc[0] * up[0] + vc[1] * up[1] + vc[2] * up[2];
            const float t[3] = {vc[0] - up[0] * vn, vc[1] - up[1] * vn, vc[2] - up[2] * vn};
            return std::sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
        };

        before = slide(vel, w);
        ResolveHullContact(slab, c, pos, vel, w, up, 0.25f, 0.6f, 0.5f);
        after = slide(vel, w);
    };

    // Coulomb friction is capped against the normal impulse, so a piece pressing on a surface
    // under nothing but its own weight is barely held and creeps along indefinitely. Below the
    // static threshold the surface holds it and a real share of the drift is taken out each
    // contact, which compounds over the substeps into rubble that stays where it landed.
    for (float t = 1.0f; t < kStaticFrictionSpeed; t += 4.0f) {
        float before = 0.0f, after = 0.0f;
        sliding(t, before, after);
        CHECK(after < before * 0.75f);
    }

    // Something genuinely travelling keeps almost all of it, so the grip cannot seize debris
    // that is still moving across a surface.
    {
        float before = 0.0f, after = 0.0f;
        sliding(kStaticFrictionSpeed + 1.0f, before, after);
        CHECK(after > before * 0.9f);

        sliding(300.0f, before, after);
        CHECK(after > before * 0.9f);
    }
}

static void TestSpinFromFrictionIsScaled()
{
    test::Suite("how strongly a contact sets a piece tumbling");

    const fragment::Hull slab = MakeSlab(10.0f, 10.0f, 2.0f);
    const float up[3] = {0, 0, 1};
    const float surface[3] = {0, 0, 0};
    const float identity[4] = {0, 0, 0, 1};

    auto run = [&](float scale, float* outW, float* outVel) {
        float pos[3] = {0, 0, 1.9f};
        float vel[3] = {150.0f, 0.0f, -120.0f};
        float w[3] = {0, 0, 0};
        const HullContact c = FindHullContact(slab, identity, pos, surface, up);
        ResolveHullContact(slab, c, pos, vel, w, up, 0.25f, 0.6f, 0.5f, scale);
        for (int a = 0; a < 3; ++a) { outW[a] = w[a]; outVel[a] = vel[a]; }
    };

    float wOff[3], wOne[3], wTwice[3], vOff[3], vOne[3], vTwice[3];
    run(0.0f, wOff, vOff);
    run(1.0f, wOne, vOne);
    run(2.0f, wTwice, vTwice);

    // The normal impulse produces spin of its own, and that part is physics rather than taste,
    // so it is the tangential contribution alone that the setting scales. That contribution is
    // linear in the scale, which pins it exactly: the step from off to normal must equal the
    // step from normal to double.
    for (int a = 0; a < 3; ++a) {
        CHECK_NEAR(wOne[a] - wOff[a], wTwice[a] - wOne[a], 1e-4);
        CHECK_NEAR(wTwice[a] - wOff[a], 2.0f * (wOne[a] - wOff[a]), 1e-4);
    }

    // And it is a real contribution, not a rounding difference, so turning it up genuinely
    // makes a chunk cartwheel harder off the same contact.
    float delta = 0.0f;
    for (int a = 0; a < 3; ++a)
        delta += (wOne[a] - wOff[a]) * (wOne[a] - wOff[a]);
    CHECK(std::sqrt(delta) > 0.05f);

    // The scale governs spin only. How fast the piece leaves the surface must not change with
    // it, or the setting would quietly become a speed control as well.
    for (int a = 0; a < 3; ++a) {
        CHECK_NEAR(vOne[a], vOff[a], 1e-4);
        CHECK_NEAR(vTwice[a], vOff[a], 1e-4);
    }

}

int main()
{
    printf("Contact\n");
    TestSupportExtent();
    TestFindHullContact();
    TestResolveSeparates();
    TestResolveProducesTorque();
    TestRestitution();
    TestFrictionScrubsTangentialSpeed();
    TestSeparationIsBounded();
    TestDegenerateInputs();
    TestSlowContactStopsBouncing();
    TestSlowPieceGripsTheSurface();
    TestSpinFromFrictionIsScaled();
    return test::Report("Contact");
}
