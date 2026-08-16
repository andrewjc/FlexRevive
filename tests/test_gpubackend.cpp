// Choosing between the CPU solver and a GPU one.
//
// A dispatch and its readback cost the same whether they carry ten pieces or ten thousand, so
// the choice is not simply "use the GPU if there is one". The policy has to account for what
// the hardware can do, what the user asked for, and whether there is enough work to be worth
// the round trip. Every refusal carries a reason, because a backend silently not engaging is
// the hardest kind of problem to report.

#include "GpuBackend.h"
#include "TestHarness.h"

using namespace flexrevive;
using namespace f4kit;
using namespace flexrevive::gpu;

namespace {

// A device that would be chosen if nothing else ruled it out.
Capability GoodDevice()
{
    Capability c;
    c.present = true;
    c.computeCapable = true;
    c.vendorId = kVendorNvidia;
    c.dedicatedMemory = 4ull * 1024 * 1024 * 1024;
    return c;
}

constexpr int kPlenty = 8000;

} // namespace

static void TestDisabled()
{
    test::Suite("switched off");

    // Off means off, whatever the hardware is.
    const Choice c = Select(GoodDevice(), kModeOff, kPlenty, kMinPiecesForGpu);
    CHECK_EQ(c.backend, kBackendCpu);
    CHECK_EQ(c.reason, kReasonDisabled);
}

static void TestNoUsableDevice()
{
    test::Suite("hardware that cannot run it");

    // Nothing there at all.
    {
        Capability c;
        const Choice choice = Select(c, kModeAuto, kPlenty, kMinPiecesForGpu);
        CHECK_EQ(choice.backend, kBackendCpu);
        CHECK_EQ(choice.reason, kReasonNoDevice);
    }

    // A device that cannot run compute at the level required.
    {
        Capability c = GoodDevice();
        c.computeCapable = false;
        const Choice choice = Select(c, kModeAuto, kPlenty, kMinPiecesForGpu);
        CHECK_EQ(choice.backend, kBackendCpu);
        CHECK_EQ(choice.reason, kReasonNoCompute);
    }

    // Not enough memory to hold the working set.
    {
        Capability c = GoodDevice();
        c.dedicatedMemory = 16ull * 1024 * 1024;
        const Choice choice = Select(c, kModeAuto, kPlenty, kMinPiecesForGpu);
        CHECK_EQ(choice.backend, kBackendCpu);
        CHECK_EQ(choice.reason, kReasonNotEnoughMemory);
    }

    // Forcing it on cannot conjure hardware that is not there: a forced choice still falls
    // back rather than dispatching into nothing.
    {
        Capability c;
        const Choice choice = Select(c, kModeForce, kPlenty, kMinPiecesForGpu);
        CHECK_EQ(choice.backend, kBackendCpu);
        CHECK_EQ(choice.reason, kReasonNoDevice);
    }
}

static void TestVendor()
{
    test::Suite("vendor");

    // Automatic selection only picks a device this has been built and tested against.
    {
        Capability c = GoodDevice();
        c.vendorId = kVendorAmd;
        const Choice choice = Select(c, kModeAuto, kPlenty, kMinPiecesForGpu);
        CHECK_EQ(choice.backend, kBackendCpu);
        CHECK_EQ(choice.reason, kReasonUntestedVendor);
    }

    // Forcing it accepts any compute-capable device, since the user has asked for it
    // deliberately and the reason says so.
    {
        Capability c = GoodDevice();
        c.vendorId = kVendorAmd;
        const Choice choice = Select(c, kModeForce, kPlenty, kMinPiecesForGpu);
        CHECK_EQ(choice.backend, kBackendGpu);
        CHECK_EQ(choice.reason, kReasonForced);
    }

    CHECK(IsNvidia(GoodDevice()));
    Capability amd = GoodDevice();
    amd.vendorId = kVendorAmd;
    CHECK(!IsNvidia(amd));
}

static void TestWorkloadThreshold()
{
    test::Suite("whether there is enough work");

    // A dispatch and its readback cost the same at any size, so a handful of pieces is not
    // worth the round trip and stays on the CPU.
    {
        const Choice c = Select(GoodDevice(), kModeAuto, 12, kMinPiecesForGpu);
        CHECK_EQ(c.backend, kBackendCpu);
        CHECK_EQ(c.reason, kReasonTooFewPieces);
    }

    // Exactly at the threshold is enough.
    {
        const Choice c = Select(GoodDevice(), kModeAuto, kMinPiecesForGpu, kMinPiecesForGpu);
        CHECK_EQ(c.backend, kBackendGpu);
        CHECK_EQ(c.reason, kReasonSelected);
    }

    // One short of it is not.
    {
        const Choice c = Select(GoodDevice(), kModeAuto, kMinPiecesForGpu - 1, kMinPiecesForGpu);
        CHECK_EQ(c.backend, kBackendCpu);
    }

    // Forcing it overrides the workload test: the user gets what they asked for.
    {
        const Choice c = Select(GoodDevice(), kModeForce, 1, kMinPiecesForGpu);
        CHECK_EQ(c.backend, kBackendGpu);
        CHECK_EQ(c.reason, kReasonForced);
    }

    // The threshold decision must not flip about frame to frame as a pile drains, or the
    // backend would swap constantly. Once above, it stays chosen for that piece count.
    for (int n = kMinPiecesForGpu; n < kMinPiecesForGpu + 200; ++n)
        CHECK_EQ(Select(GoodDevice(), kModeAuto, n, kMinPiecesForGpu).backend, kBackendGpu);
}

static void TestSelected()
{
    test::Suite("a device that is chosen");

    const Choice c = Select(GoodDevice(), kModeAuto, kPlenty, kMinPiecesForGpu);
    CHECK_EQ(c.backend, kBackendGpu);
    CHECK_EQ(c.reason, kReasonSelected);
}

static void TestReasonsAreReportable()
{
    test::Suite("every outcome can be explained");

    // A backend that quietly does not engage is the hardest thing to diagnose from a bug
    // report, so every reason has to render as something a user can read back.
    for (int r = 0; r <= kReasonForced; ++r) {
        const char* text = ReasonText(Reason(r));
        CHECK(text != nullptr);
        CHECK(text[0] != '\0');
    }

    // An unknown value is still answered rather than running off the end of a table.
    CHECK(ReasonText(Reason(999)) != nullptr);
}

static void TestModeParsing()
{
    test::Suite("what the setting means");

    CHECK_EQ(ModeFromSetting(0), kModeOff);
    CHECK_EQ(ModeFromSetting(1), kModeAuto);
    CHECK_EQ(ModeFromSetting(2), kModeForce);

    // Anything else is the safe default rather than an error, since this comes from a file a
    // user edits by hand.
    CHECK_EQ(ModeFromSetting(-5), kModeOff);
    CHECK_EQ(ModeFromSetting(99), kModeOff);
}

int main()
{
    printf("GpuBackend\n");
    TestDisabled();
    TestNoUsableDevice();
    TestVendor();
    TestWorkloadThreshold();
    TestSelected();
    TestReasonsAreReportable();
    TestModeParsing();
    return test::Report("GpuBackend");
}
