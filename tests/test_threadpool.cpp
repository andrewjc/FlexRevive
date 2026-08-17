// How SolverThreads=0 sizes the solver's thread pool.
//
// The arithmetic is three lines and it was wrong for two years' worth of budget machines: the
// old rule held two threads back unconditionally, which on a quad core left the solver one
// worker while a six core got three. That is the shape of mistake nothing catches at runtime,
// because a pool that is half the size it should be still works perfectly, just slower.

#include "f4kit/ThreadPool.h"
#include "TestHarness.h"

using namespace f4kit;
using namespace f4kit::threads;

namespace {

// Workers, not counting the calling thread, which is what the solver actually gains.
int Workers(unsigned hw)
{
    return AutoThreadCount(hw) - 1;
}

void TestSmallMachines()
{
    test::Suite("small machines");

    // Nothing to hand work to, so everything runs on the caller.
    CHECK_EQ(AutoThreadCount(0), 1);
    CHECK_EQ(AutoThreadCount(1), 1);
    CHECK_EQ(Workers(1), 0);

    // Two threads is one worker: the caller is blocked inside the solver either way, so the
    // second is not being taken from anything that could use it.
    CHECK_EQ(AutoThreadCount(2), 2);
    CHECK_EQ(AutoThreadCount(3), 2);
}

void TestQuadCore()
{
    test::Suite("the case this exists for");

    // The regression: hardware_concurrency() - 2 gave 2 here, so one worker.
    CHECK_EQ(AutoThreadCount(4), 3);
    CHECK_EQ(Workers(4), 2);
}

void TestLargerMachinesAreUnchanged()
{
    test::Suite("larger machines are left alone");

    CHECK_EQ(AutoThreadCount(6), 4);
    CHECK_EQ(AutoThreadCount(8), 6);
    CHECK_EQ(AutoThreadCount(12), 10);
    CHECK_EQ(AutoThreadCount(16), 14);
    CHECK_EQ(AutoThreadCount(32), 30);
}

void TestNeverShrinksAsTheMachineGrows()
{
    test::Suite("monotonic");

    // A machine with more threads must never be given fewer, which is exactly what a step in
    // the rule invites. The step at six is where the two-thread reserve resumes.
    int previous = 0;
    for (unsigned hw = 0; hw <= 64; ++hw) {
        const int n = AutoThreadCount(hw);
        CHECK(n >= previous);
        CHECK(n >= 1);
        previous = n;
    }
}

void TestAlwaysLeavesTheMachineSomething()
{
    test::Suite("headroom");

    // At least one thread's worth of the machine stays out of the pool from four up, so the
    // game's background threads are never entirely displaced.
    for (unsigned hw = 4; hw <= 64; ++hw)
        CHECK(AutoThreadCount(hw) <= int(hw) - 1);
}

} // namespace

int main()
{
    printf("ThreadPool\n");
    TestSmallMachines();
    TestQuadCore();
    TestLargerMachinesAreUnchanged();
    TestNeverShrinksAsTheMachineGrows();
    TestAlwaysLeavesTheMachineSomething();
    return test::Report("ThreadPool");
}
