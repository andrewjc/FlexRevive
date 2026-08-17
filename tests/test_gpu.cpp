// The GPU backend, against the CPU it has to agree with.
//
// A compute backend is only worth having if it computes the same answer, and "the same" is not
// a thing to assume across a transcription from C++ into HLSL: the two have different
// intrinsics, different precision rules, and a compiler free to contract a multiply and an add
// into one rounding where MSVC is not. So every case runs the same inputs through both paths.
//
// What the cases assert on is the worst relative difference across the whole scene, and they
// print it. Two independent implementations of one formula do not agree bit for bit, so the
// useful question is not whether they differ but by how much, and whether that figure moves.
//
// A machine with no capable adapter skips rather than fails, since that is a fact about the
// machine and not about the code.

#include "gpu/Device.h"
#include "gpu/GpuSolver.h"
#include "Collision.h"
#include "Math3D.h"
#include "Response.h"
#include "TestHarness.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

using namespace flexrevive;
using namespace f4kit;

namespace {

struct Scene {
    std::vector<float> pos, vel, rot, ang, mass, radii;
    std::vector<uint8_t> resting;
    std::vector<int> stepList;
    int count = 0;
};

uint32_t g_hash = 12345u;
float Next()
{
    g_hash ^= g_hash << 13; g_hash ^= g_hash >> 17; g_hash ^= g_hash << 5;
    return float(g_hash % 100000u) / 100000.0f;
}

// Positions and speeds near the magnitudes the game actually uses, above a ground plane the
// sweep will find rather than anywhere at all.
Scene MakeScene(int n, uint32_t seed = 12345u)
{
    g_hash = seed;
    Scene s;
    s.count = n;
    s.pos.resize(size_t(n) * 3);
    s.vel.resize(size_t(n) * 3);
    s.rot.resize(size_t(n) * 4);
    s.ang.resize(size_t(n) * 3);
    s.mass.resize(size_t(n));
    s.radii.resize(size_t(n));
    s.resting.assign(size_t(n), 0);
    for (int i = 0; i < n; ++i) {
        s.pos[size_t(i) * 3 + 0] = (Next() - 0.5f) * 2000.0f;
        s.pos[size_t(i) * 3 + 1] = (Next() - 0.5f) * 2000.0f;
        s.pos[size_t(i) * 3 + 2] = 20.0f + Next() * 600.0f;
        for (int a = 0; a < 3; ++a)
            s.vel[size_t(i) * 3 + size_t(a)] = (Next() - 0.5f) * 400.0f;
        for (int a = 0; a < 3; ++a)
            s.ang[size_t(i) * 3 + size_t(a)] = (Next() - 0.5f) * 8.0f;

        // A unit quaternion: an unnormalised one would drift differently on each side and the
        // comparison would be measuring that rather than the code.
        float q[4] = {Next() - 0.5f, Next() - 0.5f, Next() - 0.5f, Next() - 0.5f};
        float len = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        if (len < 1e-4f) { q[0] = q[1] = q[2] = 0.0f; q[3] = 1.0f; len = 1.0f; }
        for (int a = 0; a < 4; ++a)
            s.rot[size_t(i) * 4 + size_t(a)] = q[a] / len;

        s.mass[size_t(i)] = 1.0f + Next() * 600.0f;
        s.radii[size_t(i)] = 1.5f + Next() * 3.0f;
        s.stepList.push_back(i);
    }
    return s;
}

// A ground plane as a triangle mesh, which is what the sweep exists to hit.
collision::TriMesh MakeGround(float span, float z, int divisions)
{
    collision::TriMesh m;
    const float step = 2.0f * span / float(divisions);
    for (int y = 0; y <= divisions; ++y)
        for (int x = 0; x <= divisions; ++x) {
            m.verts.push_back(-span + float(x) * step);
            m.verts.push_back(-span + float(y) * step);
            m.verts.push_back(z);
        }
    const int stride = divisions + 1;
    for (int y = 0; y < divisions; ++y)
        for (int x = 0; x < divisions; ++x) {
            const int a = y * stride + x, b = a + 1, c = a + stride, d = c + 1;
            m.indices.insert(m.indices.end(), {a, b, c});
            m.indices.insert(m.indices.end(), {b, d, c});
        }
    m.lower[0] = m.lower[1] = -span; m.lower[2] = z - 1.0f;
    m.upper[0] = m.upper[1] = span;  m.upper[2] = z + 1.0f;
    collision::BuildTriGrid(m);
    return m;
}

gpu::Frame MakeFrame(Scene& s, const std::vector<gpu::Blast>& blasts, int substeps)
{
    gpu::Frame f;
    f.positions = s.pos.data();
    f.velocities = s.vel.data();
    f.rotations = s.rot.data();
    f.angular = s.ang.data();
    f.mass = s.mass.data();
    f.radii = s.radii.data();
    f.resting = s.resting.data();
    f.count = s.count;
    f.stepList = s.stepList.data();
    f.stepCount = int(s.stepList.size());
    f.blasts = blasts.empty() ? nullptr : blasts.data();
    f.blastCount = int(blasts.size());
    f.gravity[2] = -686.6f;   // the engine's gravity, 9.81 m/s^2 at this game's scale
    f.dt = 1.0f / 240.0f;     // 60 fps, four substeps
    f.substeps = substeps;
    f.dragBase = 0.35f;
    f.maxSpeed = 0.0f;
    f.contactSkin = 0.5f;
    f.restitution = 0.25f;
    f.dynamicFriction = 0.6f;
    f.sleepSpeed = 25.0f;
    f.noBounceSpeed = 45.0f;
    f.rollBlend = 0.35f;
    f.rolling = true;
    return f;
}

// One CPU substep, transcribed from stepPiece so the two are compared rather than one being
// compared against a paraphrase of itself.
void CpuStep(Scene& s, const collision::TriMesh& ground, const std::vector<gpu::Blast>& blasts,
             const gpu::Frame& f)
{
    for (int k = 0; k < int(s.stepList.size()); ++k) {
        const int i = s.stepList[size_t(k)];
        float* pos = &s.pos[size_t(i) * 3];
        float* vel = &s.vel[size_t(i) * 3];
        if (!std::isfinite(pos[0]))
            continue;

        const float from[3] = {pos[0], pos[1], pos[2]};
        const float drag = response::DragFactor(s.mass[size_t(i)], f.dragBase, f.dt, 1.0f);
        for (int a = 0; a < 3; ++a)
            vel[a] = (vel[a] + f.gravity[a] * f.dt) * drag;

        for (const gpu::Blast& b : blasts) {
            const float d[3] = {pos[0] - b.pos[0], pos[1] - b.pos[1], pos[2] - b.pos[2]};
            const float dist2 = d[0]*d[0] + d[1]*d[1] + d[2]*d[2];
            if (dist2 > b.radius * b.radius || dist2 < 1e-6f)
                continue;
            const float dist = std::sqrt(dist2);
            const float falloff = b.linearFalloff ? (1.0f - dist / b.radius)
                                                  : (1.0f - dist2 / (b.radius * b.radius));
            for (int a = 0; a < 3; ++a)
                vel[a] += (d[a] / dist) * b.strength * falloff * f.dt;
        }

        if (f.maxSpeed > 0.0f) {
            const float sp = std::sqrt(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]);
            if (sp > f.maxSpeed) {
                const float c = f.maxSpeed / sp;
                vel[0] *= c; vel[1] *= c; vel[2] *= c;
            }
        }
        for (int a = 0; a < 3; ++a)
            pos[a] += vel[a] * f.dt;

        math::QuatIntegrate(&s.rot[size_t(i) * 4], &s.ang[size_t(i) * 3], f.dt);

        const float delta[3] = {pos[0] - from[0], pos[1] - from[1], pos[2] - from[2]};
        const float dist = std::sqrt(delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2]);
        if (dist < 1e-5f)
            continue;
        const float dir[3] = {delta[0] / dist, delta[1] / dist, delta[2] / dist};

        const float clearance = s.radii[size_t(i)] + f.contactSkin;
        float bestT = dist + clearance;
        float n[3] = {0, 0, 1};
        if (!collision::SweepMesh(ground, from, dir, bestT, f.contactSkin, bestT, n))
            continue;

        const float travel = std::max(0.0f, std::min(dist, bestT - clearance));
        for (int a = 0; a < 3; ++a)
            pos[a] = from[a] + dir[a] * travel;

        const float vn = vel[0]*n[0] + vel[1]*n[1] + vel[2]*n[2];
        if (vn < 0.0f) {
            const float bounce = std::fabs(vn) < f.noBounceSpeed ? 0.0f : f.restitution;
            for (int a = 0; a < 3; ++a)
                vel[a] -= (1.0f + bounce) * vn * n[a];

            const float vn2 = vel[0]*n[0] + vel[1]*n[1] + vel[2]*n[2];
            float t[3] = {vel[0] - vn2*n[0], vel[1] - vn2*n[1], vel[2] - vn2*n[2]};
            const float tlen = std::sqrt(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]);
            if (tlen > 1e-5f) {
                const float scrub =
                    std::min(f.dynamicFriction * std::fabs(vn) * (1.0f + bounce), tlen);
                for (int a = 0; a < 3; ++a)
                    vel[a] -= (t[a] / tlen) * scrub;
                if (f.rolling) {
                    float* w = &s.ang[size_t(i) * 3];
                    const float inv = 1.0f / std::max(s.radii[size_t(i)], 1e-3f);
                    w[0] += (n[1]*t[2] - n[2]*t[1]) * inv * f.rollBlend;
                    w[1] += (n[2]*t[0] - n[0]*t[2]) * inv * f.rollBlend;
                    w[2] += (n[0]*t[1] - n[1]*t[0]) * inv * f.rollBlend;
                }
            }
        }
    }
}

struct Diff {
    float worst = 0.0f;
    int notFinite = 0;
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

// The world the card is given: one ground mesh and one collider referring to it.
bool UploadGround(const collision::TriMesh& m)
{
    gpu::MeshUpload up;
    up.verts = m.verts.data();
    up.vertCount = int(m.verts.size() / 3);
    up.indices = m.indices.data();
    up.triCount = int(m.indices.size() / 3);
    for (int a = 0; a < 3; ++a) {
        up.lower[a] = m.lower[a];
        up.upper[a] = m.upper[a];
    }
    if (m.grid.valid) {
        up.gridStart = m.grid.start.data();
        up.gridCellCount = int(m.grid.start.size());
        up.gridTris = m.grid.tris.data();
        up.gridTriCount = int(m.grid.tris.size());
        for (int a = 0; a < 3; ++a) {
            up.gridDim[a] = m.grid.dim[a];
            up.gridOrigin[a] = m.grid.origin[a];
            up.gridInvCell[a] = m.grid.invCell[a];
        }
    }

    gpu::scene::Collider c{};
    c.rot[3] = 1.0f;   // identity: the mesh is already in world space
    c.refs[0] = 0;     // mesh 0
    return gpu::SetWorld(&c, 1, &up, 1, nullptr, 0);
}

// Free flight is a comparison of arithmetic alone, and holds to a few parts in ten million.
constexpr float kRelIntegrate = 2.5e-6f;

// A sweep is far less forgiving, and legitimately so: a contact is a branch. A piece within a
// rounding of a surface lands on one side on one implementation and the other on the other,
// and from that substep on the two answers are different rather than close. The bound is on
// the worst single component over the whole scene, set from what this machine reports with
// room for another vendor's rounding, and the case that really matters is the one beside it:
// nothing may fall through the floor, whatever the last bits do.
constexpr float kRelSweep = 3e-4f;

bool g_haveDevice = false;
collision::TriMesh g_ground;

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
    CHECK(info.featureLevel >= 0xB000);   // cs_5_0 needs 11_0
    CHECK(!info.software);

    g_ground = MakeGround(4000.0f, 0.0f, 40);
    printf("        ground: %d triangles, grid %s\n", int(g_ground.indices.size() / 3),
           g_ground.grid.valid ? "built" : "absent");
    CHECK(UploadGround(g_ground));
}

void TestFreeFallMatchesTheCpu()
{
    test::Suite("gravity and drag agree with the CPU");
    if (!g_haveDevice)
        return;

    // Well above anything, so no contact fires and the difference is arithmetic, not a branch.
    Scene onGpu = MakeScene(2000);
    for (int i = 0; i < onGpu.count; ++i)
        onGpu.pos[size_t(i) * 3 + 2] += 5000.0f;
    Scene cpu = onGpu;

    const std::vector<gpu::Blast> none;
    const gpu::Frame f = MakeFrame(onGpu, none, 8);
    const bool ran = gpu::StepFrame(f);
    CHECK(ran);
    if (!ran)
        return;

    gpu::Frame cf = MakeFrame(cpu, none, 8);
    for (int s = 0; s < 8; ++s)
        CpuStep(cpu, g_ground, none, cf);

    Diff d;
    Accumulate(cpu.pos, onGpu.pos, d);
    Accumulate(cpu.vel, onGpu.vel, d);
    Accumulate(cpu.rot, onGpu.rot, d);
    printf("        worst relative difference over 8 substeps: %.3g (bound %.3g)\n",
           double(d.worst), double(kRelIntegrate));
    CHECK_EQ(d.notFinite, 0);
    CHECK(d.worst < kRelIntegrate);
}

void TestSweepMatchesTheCpu()
{
    test::Suite("landing on a mesh agrees with the CPU");
    if (!g_haveDevice)
        return;

    Scene onGpu = MakeScene(2000, 777u);
    Scene cpu = onGpu;

    const std::vector<gpu::Blast> none;
    const gpu::Frame f = MakeFrame(onGpu, none, 24);   // long enough for everything to land
    const bool ran = gpu::StepFrame(f);
    CHECK(ran);
    if (!ran)
        return;

    gpu::Frame cf = MakeFrame(cpu, none, 24);
    for (int s = 0; s < 24; ++s)
        CpuStep(cpu, g_ground, none, cf);

    Diff d;
    Accumulate(cpu.pos, onGpu.pos, d);
    Accumulate(cpu.vel, onGpu.vel, d);
    printf("        worst relative difference over 24 substeps: %.3g (bound %.3g)\n",
           double(d.worst), double(kRelSweep));
    CHECK_EQ(d.notFinite, 0);
    CHECK(d.worst < kRelSweep);

    // What matters more than the last bits: nothing went through the floor. A sweep that
    // silently misses is the failure this pass exists to prevent, and it would not show up as
    // a large difference, only as debris underground.
    int below = 0, cpuBelow = 0;
    for (int i = 0; i < onGpu.count; ++i) {
        if (onGpu.pos[size_t(i) * 3 + 2] < -50.0f)
            ++below;
        if (cpu.pos[size_t(i) * 3 + 2] < -50.0f)
            ++cpuBelow;
    }
    printf("        pieces below the ground: %d on the gpu, %d on the cpu\n", below, cpuBelow);
    CHECK_EQ(below, 0);
    CHECK_EQ(cpuBelow, 0);
}

void TestBlastsMatchTheCpu()
{
    test::Suite("force fields agree with the CPU");
    if (!g_haveDevice)
        return;

    std::vector<gpu::Blast> blasts(3);
    blasts[0] = {{0.0f, 0.0f, 200.0f}, 4000.0f, 2000.0f, 1};
    blasts[1] = {{1000.0f, -500.0f, 0.0f}, 6000.0f, 550.0f, 0};
    blasts[2] = {{0.0f, 0.0f, 0.0f}, 9000.0f, 1200.0f, 1};

    Scene onGpu = MakeScene(2000, 999u);
    for (int i = 0; i < onGpu.count; ++i)
        onGpu.pos[size_t(i) * 3 + 2] += 5000.0f;
    Scene cpu = onGpu;

    const gpu::Frame f = MakeFrame(onGpu, blasts, 8);
    const bool ran = gpu::StepFrame(f);
    CHECK(ran);
    if (!ran)
        return;

    gpu::Frame cf = MakeFrame(cpu, blasts, 8);
    for (int s = 0; s < 8; ++s)
        CpuStep(cpu, g_ground, blasts, cf);

    Diff d;
    Accumulate(cpu.pos, onGpu.pos, d);
    Accumulate(cpu.vel, onGpu.vel, d);
    printf("        worst relative difference: %.3g (bound %.3g)\n",
           double(d.worst), double(kRelIntegrate));
    CHECK_EQ(d.notFinite, 0);
    CHECK(d.worst < kRelIntegrate);
}

void TestRetiredSlotsAreLeftAlone()
{
    test::Suite("slots that are not numbers are left alone");
    if (!g_haveDevice)
        return;

    Scene s = MakeScene(512, 77u);
    s.pos[size_t(10) * 3 + 0] = std::nanf("");
    s.pos[size_t(20) * 3 + 1] = std::numeric_limits<float>::infinity();

    const std::vector<gpu::Blast> none;
    const gpu::Frame f = MakeFrame(s, none, 4);
    CHECK(gpu::StepFrame(f));

    // A broken slot stays broken rather than becoming a coordinate, and must not have spread.
    CHECK(!std::isfinite(s.pos[size_t(10) * 3 + 0]));
    int broken = 0;
    for (int i = 0; i < s.count; ++i)
        if (i != 10 && i != 20 && !std::isfinite(s.pos[size_t(i) * 3 + 0]))
            ++broken;
    CHECK_EQ(broken, 0);
}

void TestTheReadbackIsMeasured()
{
    test::Suite("the readback is measured, not assumed");
    if (!g_haveDevice)
        return;

    double dispatchMs = 0.0, readbackMs = 0.0;
    gpu::LastTiming(dispatchMs, readbackMs);
    printf("        last frame: %.3f ms dispatched, %.3f ms read back\n", dispatchMs, readbackMs);

    // The whole design rests on the readback being the expensive half, so the figure has to be
    // available rather than assumed. Whether it is worth paying is a question about a given
    // machine, which is why it is printed rather than checked against a threshold.
    CHECK(readbackMs >= 0.0);
    CHECK(dispatchMs >= 0.0);
}

void TestBackendDeclinesUntilItIsWorthIt()
{
    test::Suite("the backend does not claim more than it does");

    // The setting must never report a GPU step while the CPU does the work. That exact mistake
    // is in this plugin's history: a GpuSolver option that chose a backend, logged it, and ran
    // identical CPU code either way.
    CHECK(!gpu::Ready());
    CHECK(gpu::NotReadyReason() != nullptr);
    CHECK(gpu::NotReadyReason()[0] != 0);
}

} // namespace

int main()
{
    // Unbuffered, so a case that faults still shows which one it was. A driver fault takes
    // the process with it and anything sitting in stdout's buffer goes with it.
    setvbuf(stdout, nullptr, _IONBF, 0);
    printf("Gpu\n");
    TestDeviceComesUp();
    TestFreeFallMatchesTheCpu();
    TestSweepMatchesTheCpu();
    TestBlastsMatchTheCpu();
    TestRetiredSlotsAreLeftAlone();
    TestTheReadbackIsMeasured();
    TestBackendDeclinesUntilItIsWorthIt();
    gpu::StopForTesting();
    return test::Report("Gpu");
}
