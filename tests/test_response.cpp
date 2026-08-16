// How a piece's mass modulates what a contact does to it, and how a contact against a surface
// that is itself moving is resolved.
//
// Each law is checked for its shape, not only its value at one point: a response that is
// monotonic in mass, a split that accounts for the whole overlap, a frame transform that
// cannot put a piece outside the interval its inputs bound.

#include "Response.h"
#include "TestHarness.h"

#include <vector>

using namespace flexrevive;
using namespace f4kit;
using namespace flexrevive::response;

static void TestCarryShare()
{
    test::Suite("carry share");

    // A share is always a usable fraction: never negative, never more than all of the motion.
    for (float m : {0.0f, 0.5f, 1.0f, 5.0f, 50.0f, 5000.0f})
        for (float h : {0.25f, 1.0f, 1.6f, 4.0f}) {
            const float s = CarryShare(m, h);
            CHECK(s > 0.0f && s <= 1.0f);
            CHECK(std::isfinite(s));
        }

    // Heavier chunks respond less, strictly.
    float previous = 2.0f;
    for (float m : {1.0f, 2.0f, 8.0f, 27.0f, 64.0f, 500.0f}) {
        const float s = CarryShare(m, 1.0f);
        CHECK(s < previous);
        previous = s;
    }

    // The law is the cube root of mass: eight times the mass responds half as much, because
    // contact force follows cross section while inertia follows volume.
    CHECK_NEAR(CarryShare(8.0f, 1.0f), CarryShare(1.0f, 1.0f) / 2.0, 1e-4);
    CHECK_NEAR(CarryShare(27.0f, 1.0f), CarryShare(1.0f, 1.0f) / 3.0, 1e-4);

    // Mass below one unit is clamped, so a degenerate chunk responds as a whole one.
    CHECK_NEAR(CarryShare(0.001f, 1.0f), CarryShare(1.0f, 1.0f), 1e-6);

    // Heft scales the whole effect, and a nonsense Heft cannot divide by zero.
    CHECK(CarryShare(4.0f, 3.0f) < CarryShare(4.0f, 1.0f));
    CHECK(std::isfinite(CarryShare(4.0f, 0.0f)));
}

static void TestDragFactor()
{
    test::Suite("drag factor");

    const float dt = 0.01f;

    // A retention factor: never negative, never gains energy.
    for (float m : {1.0f, 10.0f, 100.0f})
        for (float base : {0.0f, 0.5f, 5.0f, 500.0f}) {
            const float f = DragFactor(m, base, dt, 1.0f);
            CHECK(f >= 0.0f && f <= 1.0f);
        }

    // No drag leaves the velocity alone.
    CHECK_NEAR(DragFactor(1.0f, 0.0f, dt, 1.0f), 1.0, 1e-6);

    // A bigger chunk keeps more of its speed: drag per unit mass falls with size.
    CHECK(DragFactor(64.0f, 5.0f, dt, 1.0f) > DragFactor(1.0f, 5.0f, dt, 1.0f));

    // Heft makes everything keep more of its speed, cutting through the air better.
    CHECK(DragFactor(1.0f, 5.0f, dt, 2.0f) > DragFactor(1.0f, 5.0f, dt, 1.0f));

    // An absurd drag stops the piece rather than reversing it.
    CHECK(DragFactor(1.0f, 1e6f, dt, 1.0f) >= 0.0f);
}

static void TestSeparationSplit()
{
    test::Suite("separation split");

    float a = 0.0f, b = 0.0f;

    // The two shares always account for exactly the whole overlap.
    for (float mi : {1.0f, 3.0f, 40.0f})
        for (float mj : {1.0f, 7.0f, 900.0f}) {
            SeparationSplit(mi, mj, a, b);
            CHECK_NEAR(a + b, 1.0, 1e-5);
            CHECK(a >= 0.0f && b >= 0.0f);
        }

    // Equal masses yield equally.
    SeparationSplit(5.0f, 5.0f, a, b);
    CHECK_NEAR(a, 0.5, 1e-5);
    CHECK_NEAR(b, 0.5, 1e-5);

    // The lighter piece yields more: a slab shoulders a splinter aside.
    SeparationSplit(1.0f, 99.0f, a, b);
    CHECK(a > b);
    CHECK_NEAR(a, 0.99, 1e-4);

    // Degenerate masses still produce a usable split rather than a NaN.
    SeparationSplit(0.0f, 0.0f, a, b);
    CHECK(std::isfinite(a) && std::isfinite(b));
    CHECK_NEAR(a + b, 1.0, 1e-5);
}

// A contact against a moving surface leaves the piece with a velocity between what it had and
// what the surface is doing. Anything outside that interval is energy the contact invented.
static bool WithinInterval(float result, float pieceWas, float surface)
{
    const float lo = std::min(pieceWas, surface) - 1e-3f;
    const float hi = std::max(pieceWas, surface) + 1e-3f;
    return result >= lo && result <= hi;
}

static void TestMovingSurface()
{
    test::Suite("contact against a moving surface");

    const float up[3] = {0, 0, 1};

    // A stationary surface: the piece bounces and nothing carries it sideways.
    {
        float vel[3] = {0, 0, -100.0f};
        const float still[3] = {0, 0, 0};
        ResolveAgainstMovingSurface(vel, up, still, 1.0f, 0.0f);
        CHECK_NEAR(vel[0], 0.0, 1e-4);
        CHECK_NEAR(vel[1], 0.0, 1e-4);
        CHECK(vel[2] >= -1e-3f);   // no longer driving into the surface
    }

    // Walking into a heap pushes along the contact normal, which is the direction that
    // actually carries a piece. `into` points from the collider out toward the piece.
    const float into[3] = {1, 0, 0};

    {
        float vel[3] = {0, 0, -50.0f};
        const float advancing[3] = {200.0f, 0, 0};
        ResolveAgainstMovingSurface(vel, into, advancing, 1.0f, 0.0f);
        CHECK(vel[0] > 0.0f);
        CHECK(WithinInterval(vel[0], 0.0f, 200.0f));
        CHECK_NEAR(vel[2], -50.0, 1e-4);   // motion across the normal is untouched
    }

    // Tangential motion is not carried: this resolves the normal contact only, and a surface
    // dragging a piece sideways is friction's job, handled where the contact is solved.
    {
        float vel[3] = {0, 0, -50.0f};
        const float sliding[3] = {200.0f, 0, 0};
        ResolveAgainstMovingSurface(vel, up, sliding, 1.0f, 0.0f);
        CHECK_NEAR(vel[0], 0.0, 1e-4);
    }

    // The share scales the carry and never inverts it. A piece must not leave the contact
    // moving against a surface that is advancing on it, at any share.
    for (float share : {0.0f, 0.15f, 0.4f, 0.75f, 1.0f}) {
        float vel[3] = {0, 0, -50.0f};
        const float advancing[3] = {200.0f, 0, 0};
        ResolveAgainstMovingSurface(vel, into, advancing, share, 0.0f);
        CHECK(vel[0] >= -1e-3f);                              // never driven backwards
        CHECK(WithinInterval(vel[0], 0.0f, 200.0f * share));  // never past the surface
    }

    // More share means more carry, strictly, at every step.
    float previous = -1.0f;
    for (float share : {0.0f, 0.2f, 0.5f, 0.8f, 1.0f}) {
        float vel[3] = {0, 0, -50.0f};
        const float advancing[3] = {200.0f, 0, 0};
        ResolveAgainstMovingSurface(vel, into, advancing, share, 0.0f);
        CHECK(vel[0] > previous);
        previous = vel[0];
    }

    // A share of zero is exactly the stationary-surface answer: a heavy chunk brushed by
    // something is left alone, not shoved in some other direction.
    {
        float moving[3] = {10.0f, 0, -50.0f};
        float still[3] = {10.0f, 0, -50.0f};
        const float advancing[3] = {200.0f, 0, 0};
        const float none[3] = {0, 0, 0};
        ResolveAgainstMovingSurface(moving, into, advancing, 0.0f, 0.25f);
        ResolveAgainstMovingSurface(still, into, none, 1.0f, 0.25f);
        for (int a = 0; a < 3; ++a)
            CHECK_NEAR(moving[a], still[a], 1e-4);
    }

    // A piece already leaving the surface is not grabbed back by it.
    {
        float vel[3] = {0, 0, 120.0f};
        const float advancing[3] = {200.0f, 0, 0};
        float before[3] = {vel[0], vel[1], vel[2]};
        ResolveAgainstMovingSurface(vel, up, advancing, 1.0f, 0.0f);
        CHECK_NEAR(vel[2], before[2], 1e-4);
    }

    // Restitution rebounds the piece off a moving surface without inventing speed along it.
    {
        float vel[3] = {0, 0, -100.0f};
        const float rising[3] = {0, 0, 40.0f};
        ResolveAgainstMovingSurface(vel, up, rising, 1.0f, 0.5f);
        CHECK(vel[2] > 0.0f);
        CHECK(std::isfinite(vel[0]) && std::isfinite(vel[1]));
    }

    // Nothing produces a NaN, whatever it is handed.
    {
        float vel[3] = {0, 0, -10.0f};
        const float odd[3] = {0, 0, 0};
        ResolveAgainstMovingSurface(vel, up, odd, -1.0f, -5.0f);
        for (int a = 0; a < 3; ++a)
            CHECK(std::isfinite(vel[a]));
    }
}

// The two laws must agree about what "heavier" means, or a chunk that shrugs off a footfall
// would still be flung by a shot landing beside it.
static void TestLawsAgree()
{
    test::Suite("the mass laws agree");

    for (float m : {2.0f, 20.0f, 200.0f}) {
        CHECK(CarryShare(m, 1.0f) < CarryShare(1.0f, 1.0f));
        float a = 0.0f, b = 0.0f;
        SeparationSplit(m, 1.0f, a, b);
        CHECK(a < b);   // the heavy piece yields less of the separation
    }
}

int main()
{
    printf("Response\n");
    TestCarryShare();
    TestDragFactor();
    TestSeparationSplit();
    TestMovingSurface();
    TestLawsAgree();
    return test::Report("Response");
}
