// The GPU backend, against the CPU it has to agree with.
//
// A compute backend is only worth having if it computes the same answer, and "the same" is not
// a thing to assume across a transcription from C++ into HLSL: the two have different
// intrinsics, different precision rules, and a compiler that is free to reassociate. So every
// case here runs the same inputs through both paths and compares.
//
// What the cases assert on is the worst relative difference across the whole scene, and they
// print it. Two independent implementations of the same formula in two languages do not agree
// bit for bit, so the useful question is not whether they differ but by how much, and whether
// that figure moves.
//
// A machine with no capable adapter skips rather than fails, since that is a fact about the
// machine and not about the code.

#include "gpu/Device.h"
#include "gpu/GpuSolver.h"
#include "Response.h"
#include "TestHarness.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

using namespace flexrevive;
using namespace f4kit;

namespace {

// Positions and velocities near the magnitudes Fallout 4 actually uses: world coordinates in
// the tens of thousands, launch speeds around a couple of hundred units a second.
struct Scene {
    std::vector<float> pos, vel, mass;
    std::vector<int> stepList;
    int count = 0;
};

Scene MakeScene(int n, uint32_t seed = 12345u)
{
    Scene s;
    s.count = n;
    s.pos.resize(size_t(n) * 3);
    s.vel.resize(size_t(n) * 3);
    s.mass.resize(size_t(n));
    uint32_t h = seed;
    auto next = [&] {
        h ^= h << 13; h ^= h >> 17; h ^= h << 5;
        return float(h % 100000u) / 100000.0f;
    };
    for (int i = 0; i < n; ++i) {
        s.pos[size_t(i) * 3 + 0] = (next() - 0.5f) * 160000.0f;
        s.pos[size_t(i) * 3 + 1] = (next() - 0.5f) * 160000.0f;
        s.pos[size_t(i) * 3 + 2] = next() * 8000.0f;
        for (int a = 0; a < 3; ++a)
            s.vel[size_t(i) * 3 + size_t(a)] = (next() - 0.5f) * 400.0f;
        // Masses run from a splinter to a slab; the drag term is where mass matters.
        s.mass[size_t(i)] = 1.0f + next() * 600.0f;
        s.stepList.push_back(i);
    }
    return s;
}

// The CPU integration, lifted from stepPiece so the two are compared rather than one being
// compared against a paraphrase of itself.
void CpuIntegrate(Scene& s, const std::vector<gpu::Blast>& blasts, const gpu::StepParams& p)
{
    for (int k = 0; k < int(s.stepList.size()); ++k) {
        const int i = s.stepList[size_t(k)];
        float* pos = &s.pos[size_t(i) * 3];
        float* vel = &s.vel[size_t(i) * 3];
        if (!std::isfinite(pos[0]))
            continue;

        const float drag = response::DragFactor(s.mass[size_t(i)], p.dragBase, p.dt, 1.0f);
        for (int a = 0; a < 3; ++a)
            vel[a] = (vel[a] + p.gravity[a] * p.dt) * drag;

        for (const gpu::Blast& f : blasts) {
            const float d[3] = {pos[0] - f.pos[0], pos[1] - f.pos[1], pos[2] - f.pos[2]};
            const float dist2 = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
            if (dist2 > f.radius * f.radius || dist2 < 1e-6f)
                continue;
            const float dist = std::sqrt(dist2);
            const float falloff = f.linearFalloff ? (1.0f - dist / f.radius)
                                                  : (1.0f - dist2 / (f.radius * f.radius));
            for (int a = 0; a < 3; ++a)
                vel[a] += (d[a] / dist) * f.strength * falloff * p.dt;
        }

        if (p.maxSpeed > 0.0f) {
            const float sp = std::sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]);
            if (sp > p.maxSpeed) {
                const float k2 = p.maxSpeed / sp;
                vel[0] *= k2; vel[1] *= k2; vel[2] *= k2;
            }
        }

        for (int a = 0; a < 3; ++a)
            pos[a] += vel[a] * p.dt;
    }
}

gpu::StepParams DefaultParams()
{
    gpu::StepParams p;
    p.gravity[2] = -686.6f;   // the engine's gravity, 9.81 m/s^2 at this game's scale
    p.dt = 1.0f / 240.0f;     // 60 fps, four substeps
    p.dragBase = 0.35f;
    p.maxSpeed = 0.0f;
    return p;
}

// The largest relative difference anywhere in the scene, rather than a count of how many
// exceeded some threshold.
//
// A count only says whether the tolerance was met, which makes the tolerance a thing to tune
// until the test passes. The worst error says how far apart the two implementations actually
// are, so the tolerance can be set from what float arithmetic costs and a regression shows up
// as the number moving rather than as a pass becoming a fail.
//
// Relative, since world coordinates run to 1e5 and an absolute epsilon there is meaningless.
struct Diff {
    float worst = 0.0f;
    bool ranAtAll = true;
    int notFinite = 0;   // one side finite and the other not, which is never a rounding matter
};

void Accumulate(const std::vector<float>& a, const std::vector<float>& b, Diff& d)
{
    for (size_t i = 0; i < a.size(); ++i) {
        if (!std::isfinite(a[i]) || !std::isfinite(b[i])) {
            if (std::isfinite(a[i]) != std::isfinite(b[i]))
                ++d.notFinite;
            continue;
        }
        const float scale = std::max(1.0f, std::max(std::fabs(a[i]), std::fabs(b[i])));
        d.worst = std::max(d.worst, std::fabs(a[i] - b[i]) / scale);
    }
}

// Both paths over the same scene.
Diff RunBoth(Scene scene, const std::vector<gpu::Blast>& blasts, const gpu::StepParams& p,
             int substeps)
{
    Scene cpu = scene;
    for (int s = 0; s < substeps; ++s)
        CpuIntegrate(cpu, blasts, p);

    Diff d;
    for (int s = 0; s < substeps; ++s)
        if (!gpu::IntegrateStep(scene.pos.data(), scene.vel.data(), scene.mass.data(),
                                scene.count, scene.stepList.data(), int(scene.stepList.size()),
                                blasts.empty() ? nullptr : blasts.data(), int(blasts.size()),
                                p)) {
            d.ranAtAll = false;
            return d;
        }

    Accumulate(cpu.pos, scene.pos, d);
    Accumulate(cpu.vel, scene.vel, d);
    return d;
}

// What two independent implementations of the same formula cost, per case.
//
// The two do not agree bit for bit and cannot be made to. The shader computes the cube root in
// the drag term by refinement where the CPU calls std::cbrt, and the HLSL compiler contracts a
// multiply and an add into one rounding where MSVC does not. Both are correct; they round
// differently in the last place or two, and the substeps compound it.
//
// The bounds are per case rather than one global figure, because the cases are not equally
// well conditioned and a single loose bound would stop the well-behaved ones from catching
// anything. Each is roughly three times the error measured on the reference machine, which is
// slack enough for another vendor's rounding and tight enough that a wrong formula blows
// through it by orders of magnitude. Every case prints its real figure, so drift shows up as
// the number moving rather than as a sudden failure.

// Gravity and drag: a multiply and an add per component per substep, over sixteen of them.
// The only interesting term is the cube root. Measured 7.4e-07.
constexpr float kRelGravity = 2.5e-6f;

// The speed cap adds a length and a divide, and its branch is a discontinuity: a piece within
// a rounding of the cap can clamp on one side and not the other. The scale factor there is
// within a rounding of 1, so the divergence stays tiny. Measured 1.9e-06.
constexpr float kRelSpeedCap = 6e-6f;

// Force fields are the worst conditioned expression here by some way, and legitimately so.
// The offset from a blast is a difference of world coordinates in the tens of thousands, which
// loses low bits before anything else happens; it is then squared, rooted and divided. Worse,
// the falloff is 1 - dist/radius, which is a subtraction of nearly equal numbers for any piece
// near the edge of the blast, so its own relative error is unbounded even where its absolute
// error is negligible. Measured 6.4e-06, which on a velocity of a few hundred units a second
// is a difference of some thousandths of a unit per second: below anything the eye or the
// engine could distinguish.
constexpr float kRelBlast = 2e-5f;

bool g_haveDevice = false;

void TestDeviceComesUp()
{
    test::Suite("the device comes up, or says why not");

    g_haveDevice = gpu::Start();
    if (!g_haveDevice) {
        printf("        no compute device on this machine: %s\n", gpu::NotReadyReason());
        printf("        (the remaining GPU cases are skipped, which is not a failure)\n");
        return;
    }

    const gpu::device::DeviceInfo& info = gpu::device::Info();
    printf("        %s (%s), %llu MB, feature level 0x%X\n", info.description,
           gpu::device::VendorName(info.vendorId), (unsigned long long)info.dedicatedMB,
           unsigned(info.featureLevel));

    // cs_5_0 needs 11_0, and nothing below that may be reported as usable.
    CHECK(info.featureLevel >= 0xB000);
    CHECK(!info.software);
    CHECK(info.description[0] != 0);
}

void TestMatchesTheCpuUnderGravity()
{
    test::Suite("gravity and drag agree with the CPU");
    if (!g_haveDevice)
        return;

    // Many substeps, so any difference in the drag term compounds instead of hiding in the
    // noise of a single step.
    const Diff d = RunBoth(MakeScene(2000), {}, DefaultParams(), 16);
    printf("        worst relative difference over 16 substeps: %.3g (bound %.3g)\n",
           double(d.worst), double(kRelGravity));
    CHECK(d.ranAtAll);
    CHECK_EQ(d.notFinite, 0);
    CHECK(d.worst < kRelGravity);
}

void TestMatchesTheCpuUnderBlasts()
{
    test::Suite("force fields agree with the CPU");
    if (!g_haveDevice)
        return;

    // Both falloff modes, overlapping, and one field centred on the debris so the near-zero
    // distance guard is exercised.
    std::vector<gpu::Blast> blasts(3);
    blasts[0] = {{0.0f, 0.0f, 2000.0f}, 40000.0f, 2000.0f, 1};
    blasts[1] = {{10000.0f, -5000.0f, 0.0f}, 60000.0f, 550.0f, 0};
    blasts[2] = {{0.0f, 0.0f, 0.0f}, 90000.0f, 1200.0f, 1};

    const Diff d = RunBoth(MakeScene(2000, 999u), blasts, DefaultParams(), 8);
    printf("        worst relative difference: %.3g (bound %.3g)\n",
           double(d.worst), double(kRelBlast));
    CHECK(d.ranAtAll);
    CHECK_EQ(d.notFinite, 0);
    CHECK(d.worst < kRelBlast);
}

void TestMatchesTheCpuUnderTheSpeedCap()
{
    test::Suite("the speed cap agrees with the CPU");
    if (!g_haveDevice)
        return;

    gpu::StepParams p = DefaultParams();
    p.maxSpeed = 120.0f;   // low enough that most pieces are clamped every step

    const Diff d = RunBoth(MakeScene(2000, 4242u), {}, p, 8);
    printf("        worst relative difference: %.3g (bound %.3g)\n",
           double(d.worst), double(kRelSpeedCap));
    CHECK(d.ranAtAll);
    CHECK_EQ(d.notFinite, 0);
    CHECK(d.worst < kRelSpeedCap);
}

void TestRetiredSlotsAreLeftAlone()
{
    test::Suite("slots that are not numbers are left alone");
    if (!g_haveDevice)
        return;

    // A recycled slot can hold anything at all, and both paths must skip it rather than
    // propagate it into its neighbours.
    Scene scene = MakeScene(512, 77u);
    scene.pos[size_t(10) * 3 + 0] = std::nanf("");
    scene.pos[size_t(20) * 3 + 1] = std::numeric_limits<float>::infinity();
    scene.vel[size_t(30) * 3 + 2] = std::nanf("");

    const Diff d = RunBoth(scene, {}, DefaultParams(), 4);
    CHECK(d.ranAtAll);
    // A slot that is not a number must stay not a number on both sides, rather than one path
    // quietly turning it into a coordinate.
    CHECK_EQ(d.notFinite, 0);
    CHECK(d.worst < kRelGravity);
}

void TestOnlyTheStepListMoves()
{
    test::Suite("a settled piece is not touched");
    if (!g_haveDevice)
        return;

    // The step list is the moving subset; everything else must come back byte for byte, since
    // a settled heap that drifts is the bug this whole design is arranged to avoid.
    Scene scene = MakeScene(1000, 31337u);
    scene.stepList.clear();
    for (int i = 0; i < scene.count; i += 3)
        scene.stepList.push_back(i);

    const std::vector<float> beforePos = scene.pos;
    const std::vector<float> beforeVel = scene.vel;

    const gpu::StepParams p = DefaultParams();
    const bool ran = gpu::IntegrateStep(scene.pos.data(), scene.vel.data(), scene.mass.data(),
                                        scene.count, scene.stepList.data(),
                                        int(scene.stepList.size()), nullptr, 0, p);
    CHECK(ran);
    if (!ran)
        return;

    int movedWhenItShouldNot = 0, unmovedWhenItShould = 0;
    for (int i = 0; i < scene.count; ++i) {
        const bool stepping = (i % 3) == 0;
        bool changed = false;
        for (int a = 0; a < 3; ++a)
            changed = changed || scene.pos[size_t(i) * 3 + size_t(a)] !=
                                     beforePos[size_t(i) * 3 + size_t(a)] ||
                      scene.vel[size_t(i) * 3 + size_t(a)] !=
                          beforeVel[size_t(i) * 3 + size_t(a)];
        if (!stepping && changed)
            ++movedWhenItShouldNot;
        if (stepping && !changed)
            ++unmovedWhenItShould;
    }
    CHECK_EQ(movedWhenItShouldNot, 0);
    CHECK_EQ(unmovedWhenItShould, 0);
}

void TestBackendDeclinesUntilItIsWorthIt()
{
    test::Suite("the backend does not claim more than it does");

    // The setting must never report a GPU step while the CPU does the work. That exact mistake
    // is in this plugin's history: a GpuSolver option that chose a backend, logged it, and ran
    // identical CPU code either way. Ready() stays false until the collision passes are on the
    // card and the loop can stop coming back mid-step.
    CHECK(!gpu::Ready());
    CHECK(gpu::NotReadyReason() != nullptr);
    CHECK(gpu::NotReadyReason()[0] != 0);
}

} // namespace

int main()
{
    printf("Gpu\n");
    TestDeviceComesUp();
    TestMatchesTheCpuUnderGravity();
    TestMatchesTheCpuUnderBlasts();
    TestMatchesTheCpuUnderTheSpeedCap();
    TestRetiredSlotsAreLeftAlone();
    TestOnlyTheStepListMoves();
    TestBackendDeclinesUntilItIsWorthIt();
    gpu::StopForTesting();
    gpu::StopForTesting();
    return test::Report("Gpu");
}
