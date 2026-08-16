// Working out what the engine did to each debris slot between one call and the next.
//
// The engine hands back the transforms it was given, bit for bit, and recycles a fixed pool of
// slots. So a slot arriving with a position other than the one last reported for it has either
// been re-seeded with a new fragment or been handed a piece that lived in a different slot. The
// two are told apart by looking for the incoming position among everything previously reported:
// a piece that moved seats carries its old position exactly, a new fragment does not.

#include "SlotTracker.h"
#include "TestHarness.h"

#include <cmath>
#include <vector>

using namespace flexrevive;
using namespace f4kit;
using namespace flexrevive::slots;

namespace {

// A pool of `n` slots at spaced-out positions, all of them already reported once.
struct Pool {
    std::vector<float> reportedAt;
    std::vector<uint8_t> reported;
    int count = 0;

    explicit Pool(int n) : reportedAt(size_t(n) * 3, 0.0f), reported(size_t(n), 1), count(n)
    {
        for (int i = 0; i < n; ++i) {
            reportedAt[size_t(i) * 3 + 0] = 1000.0f + float(i) * 100.0f;
            reportedAt[size_t(i) * 3 + 1] = 2000.0f - float(i) * 37.0f;
            reportedAt[size_t(i) * 3 + 2] = 500.0f + float(i) * 11.0f;
        }
    }

    // What the engine would hand back if it changed nothing.
    std::vector<float> RoundTrip() const { return reportedAt; }

    void Put(std::vector<float>& t, int slot, const float* p) const
    {
        for (int a = 0; a < 3; ++a) t[size_t(slot) * 3 + size_t(a)] = p[a];
    }

    void Copy(std::vector<float>& t, int dstSlot, int srcSlot) const
    {
        for (int a = 0; a < 3; ++a)
            t[size_t(dstSlot) * 3 + size_t(a)] = reportedAt[size_t(srcSlot) * 3 + size_t(a)];
    }
};

constexpr float kThreshold = 16.0f;

Classification Run(const Pool& pool, const std::vector<float>& incoming, int count)
{
    Classification c;
    Classify(incoming.data(), count, pool.reportedAt, pool.reported, pool.count, kThreshold, c);
    return c;
}

} // namespace

static void TestRoundTripIsUnchanged()
{
    test::Suite("values handed straight back");

    Pool pool(8);
    const Classification c = Run(pool, pool.RoundTrip(), 8);

    for (int i = 0; i < 8; ++i) {
        CHECK_EQ(c.change[size_t(i)], kUnchanged);
        CHECK_EQ(c.source[size_t(i)], -1);
    }
}

static void TestSmallMovementIsStillUnchanged()
{
    test::Suite("a slot nudged less than the threshold");

    Pool pool(4);
    std::vector<float> t = pool.RoundTrip();

    // The engine shifts a piece by a few units. Below the threshold that is a round trip with
    // noise on it, not a new fragment.
    t[3] += 4.0f;
    t[4] -= 3.0f;

    const Classification c = Run(pool, t, 4);
    CHECK_EQ(c.change[1], kUnchanged);
}

static void TestGrownSlotsAreFresh()
{
    test::Suite("slots beyond the previous count");

    Pool pool(3);
    std::vector<float> t(size_t(6) * 3, 0.0f);
    for (int i = 0; i < 3; ++i)
        pool.Copy(t, i, i);
    for (int i = 3; i < 6; ++i) {
        const float p[3] = {-9000.0f, float(i), 42.0f};
        pool.Put(t, i, p);
    }

    const Classification c = Run(pool, t, 6);
    for (int i = 0; i < 3; ++i)
        CHECK_EQ(c.change[size_t(i)], kUnchanged);
    for (int i = 3; i < 6; ++i)
        CHECK_EQ(c.change[size_t(i)], kFresh);
}

static void TestNewFragmentIsFresh()
{
    test::Suite("a slot re-seeded with new debris");

    Pool pool(5);
    std::vector<float> t = pool.RoundTrip();

    // Somewhere else entirely, and matching nothing previously reported.
    const float elsewhere[3] = {-4000.0f, 8000.0f, 120.0f};
    pool.Put(t, 2, elsewhere);

    const Classification c = Run(pool, t, 5);
    CHECK_EQ(c.change[2], kFresh);
    CHECK_EQ(c.source[2], -1);
    for (int i : {0, 1, 3, 4})
        CHECK_EQ(c.change[size_t(i)], kUnchanged);
}

static void TestPieceChangingSeats()
{
    test::Suite("a piece moved to a different slot");

    Pool pool(6);
    std::vector<float> t = pool.RoundTrip();

    // The engine compacts its pool: whatever was in slot 4 now sits in slot 1. Slot 1 must be
    // recognised as that piece rather than treated as a brand new fragment and relaunched.
    pool.Copy(t, 1, 4);
    const float elsewhere[3] = {-7000.0f, 0.0f, 0.0f};
    pool.Put(t, 4, elsewhere);

    const Classification c = Run(pool, t, 6);
    CHECK_EQ(c.change[1], kMigrated);
    CHECK_EQ(c.source[1], 4);
    CHECK_EQ(c.change[4], kFresh);
}

static void TestTwoPiecesSwapping()
{
    test::Suite("two pieces exchanging slots");

    Pool pool(4);
    std::vector<float> t = pool.RoundTrip();
    pool.Copy(t, 0, 3);
    pool.Copy(t, 3, 0);

    const Classification c = Run(pool, t, 4);
    CHECK_EQ(c.change[0], kMigrated);
    CHECK_EQ(c.source[0], 3);
    CHECK_EQ(c.change[3], kMigrated);
    CHECK_EQ(c.source[3], 0);
}

static void TestExactMatchRequired()
{
    test::Suite("matching is exact");

    Pool pool(4);
    std::vector<float> t = pool.RoundTrip();

    // Slot 1 receives slot 3's position with a single float step added. The engine returns its
    // values bit for bit, so this is not slot 3's piece and must not inherit its state.
    pool.Copy(t, 1, 3);
    t[3] = std::nextafter(t[3], 1e30f);

    const Classification c = Run(pool, t, 4);
    CHECK(c.change[1] != kMigrated);
    CHECK_EQ(c.source[1], -1);
}

static void TestNeverMigratesFromItself()
{
    test::Suite("a slot is never its own source");

    Pool pool(3);
    std::vector<float> t = pool.RoundTrip();

    // A slot far from where it was, but whose new position happens to be its own old one is
    // impossible; this pins that a match against itself is not treated as a migration.
    for (int i = 0; i < 3; ++i) {
        const Classification c = Run(pool, t, 3);
        CHECK(c.source[size_t(i)] != i);
    }
}

static void TestNeverReportedIsFresh()
{
    test::Suite("slots never reported");

    Pool pool(4);
    pool.reported[2] = 0;           // this slot has never been handed back
    std::vector<float> t = pool.RoundTrip();
    const float elsewhere[3] = {-1000.0f, -1000.0f, -1000.0f};
    pool.Put(t, 2, elsewhere);

    const Classification c = Run(pool, t, 4);
    CHECK_EQ(c.change[2], kFresh);
}

static void TestDegenerate()
{
    test::Suite("degenerate inputs");

    Pool pool(3);
    Classification c;

    // No incoming transforms at all.
    Classify(nullptr, 3, pool.reportedAt, pool.reported, 3, kThreshold, c);
    CHECK_EQ(int(c.change.size()), 3);

    // A count of zero yields empty results rather than reading anything.
    std::vector<float> t = pool.RoundTrip();
    Classify(t.data(), 0, pool.reportedAt, pool.reported, 3, kThreshold, c);
    CHECK_EQ(int(c.change.size()), 0);

    // A previous count larger than the arrays actually hold is clamped.
    Classify(t.data(), 3, pool.reportedAt, pool.reported, 999, kThreshold, c);
    CHECK_EQ(int(c.change.size()), 3);
    for (int i = 0; i < 3; ++i)
        CHECK(c.source[size_t(i)] >= -1 && c.source[size_t(i)] < 3);
}

static void TestReseed()
{
    test::Suite("a slot reported as new");

    // A piece that was moving is a genuine spawn: thrown, with its own motion.
    {
        const Reseed r = OnReseed(false, 0.0f);
        CHECK(r.thrown);
        CHECK(!r.clearMotion);
        CHECK(!r.asleep);
    }

    // Distance is irrelevant for a moving piece; it is a spawn either way.
    {
        const Reseed r = OnReseed(false, 9000.0f);
        CHECK(r.thrown);
    }

    // A settled piece handed back where it lay has not been touched. Throwing it would fling a
    // chunk out of a heap that nothing hit, so it keeps still and stays asleep.
    {
        const Reseed r = OnReseed(true, 1.0f);
        CHECK(!r.thrown);
        CHECK(r.clearMotion);
        CHECK(r.asleep);
    }

    // A settled piece handed back somewhere else is a different fragment in a recycled slot. It
    // must be awake: left asleep with its motion cleared it hangs at the position it was handed
    // and never falls, which is debris stuck in mid-air.
    {
        const Reseed r = OnReseed(true, 5000.0f);
        CHECK(!r.thrown);
        CHECK(r.clearMotion);
        CHECK(!r.asleep);
    }

    // Nothing may end up both asleep and displaced, at any distance. This is the invariant the
    // mid-air debris violated.
    for (float jump = 0.0f; jump < 8000.0f; jump += 7.0f) {
        const Reseed r = OnReseed(true, jump);
        if (jump >= kSettledJump)
            CHECK(!r.asleep);
    }

    // The threshold itself: below it the piece is undisturbed, at it and above it is displaced.
    CHECK(OnReseed(true, kSettledJump - 0.1f).asleep);
    CHECK(!OnReseed(true, kSettledJump).asleep);

    // A slot with no previous position reports an unbounded jump rather than zero, so it must
    // not be mistaken for a piece that has not moved.
    CHECK(!OnReseed(true, 1e30f).asleep);
}

int main()
{
    printf("SlotTracker\n");
    TestRoundTripIsUnchanged();
    TestSmallMovementIsStillUnchanged();
    TestGrownSlotsAreFresh();
    TestNewFragmentIsFresh();
    TestPieceChangingSeats();
    TestTwoPiecesSwapping();
    TestExactMatchRequired();
    TestNeverMigratesFromItself();
    TestNeverReportedIsFresh();
    TestDegenerate();
    TestReseed();
    return test::Report("SlotTracker");
}
