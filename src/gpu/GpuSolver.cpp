// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "gpu/GpuSolver.h"

#include "gpu/Device.h"
#include "f4kit/Log.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <d3d11.h>

#include <algorithm>
#include <chrono>
#include <mutex>
#include <vector>

#if FLEXREVIVE_HAVE_GPU
#include "sweep_cs.h"
#include "pairs_cs.h"
#include "settle_cs.h"
#endif

using namespace f4kit;

namespace flexrevive::gpu {

namespace {

std::mutex s_mutex;
bool s_tried = false;
bool s_usable = false;
const char* s_notReady = "the backend has not been started";

ID3D11ComputeShader* s_sweep = nullptr;
ID3D11ComputeShader* s_pairs = nullptr;
ID3D11ComputeShader* s_settle = nullptr;
ID3D11Buffer* s_params = nullptr;

double s_dispatchMs = 0.0;
double s_readbackMs = 0.0;

ID3D11Device* Dev() { return static_cast<ID3D11Device*>(device::Device()); }
ID3D11DeviceContext* Ctx() { return static_cast<ID3D11DeviceContext*>(device::Context()); }

// A structured buffer with its views, grown to fit and never shrunk, so a steady scene stops
// allocating after its first few frames.
struct Buffer {
    ID3D11Buffer* buffer = nullptr;
    ID3D11ShaderResourceView* srv = nullptr;
    ID3D11UnorderedAccessView* uav = nullptr;
    UINT stride = 0;
    UINT capacity = 0;
    bool writable = false;

    void Release()
    {
        if (uav) { uav->Release(); uav = nullptr; }
        if (srv) { srv->Release(); srv = nullptr; }
        if (buffer) { buffer->Release(); buffer = nullptr; }
        capacity = 0;
    }
};

// Piece state and the moving subset, replaced every frame.
Buffer s_pieces, s_stepList, s_blasts;
// World geometry, replaced only when it changes.
Buffer s_colliders, s_meshes, s_verts, s_indices, s_gridStart, s_gridTris, s_planes;
// The pair pass's neighbourhood, rebuilt each frame from the piece positions.
Buffer s_runs, s_cellPieces, s_gridHead, s_gridNext, s_pieceCell, s_supported;
ID3D11Buffer* s_readback = nullptr;
UINT s_readbackBytes = 0;

std::vector<scene::Piece> s_pieceStage;
std::vector<uint32_t> s_stepStage;
std::vector<scene::Blast> s_blastStage;
std::vector<uint32_t> s_headStage, s_nextStage, s_cellPieceStage, s_looseStage;
std::vector<int32_t> s_cellStage;
std::vector<scene::CellRun> s_runStage;
std::vector<uint32_t> s_colourFirst, s_colourCount;
std::vector<uint32_t> s_supportedStage;

// Flattened world geometry, kept so an upload does not have to be assembled twice.
std::vector<scene::MeshDesc> s_meshStage;
std::vector<float> s_vertStage;      // float4 per vertex, w unused
std::vector<uint32_t> s_indexStage;
std::vector<uint32_t> s_gridStartStage;
std::vector<uint32_t> s_gridTriStage;
// How many colliders were actually uploaded. A buffer is grown in powers of two
// and its tail holds whatever was there before, so the shader must be told the
// real count rather than the capacity.
int s_colliderCountUploaded = 0;
bool s_haveWorld = false;

bool EnsureBuffer(Buffer& b, UINT stride, UINT count, bool writable)
{
    if (b.buffer && b.capacity >= count && b.stride == stride && b.writable == writable)
        return true;
    b.Release();

    UINT capacity = 256;
    while (capacity < count)
        capacity *= 2;

    D3D11_BUFFER_DESC bd{};
    bd.ByteWidth = stride * capacity;
    bd.Usage = writable ? D3D11_USAGE_DEFAULT : D3D11_USAGE_DYNAMIC;
    bd.BindFlags = writable ? (D3D11_BIND_UNORDERED_ACCESS | D3D11_BIND_SHADER_RESOURCE)
                            : D3D11_BIND_SHADER_RESOURCE;
    bd.CPUAccessFlags = writable ? 0 : D3D11_CPU_ACCESS_WRITE;
    bd.MiscFlags = D3D11_RESOURCE_MISC_BUFFER_STRUCTURED;
    bd.StructureByteStride = stride;
    if (FAILED(Dev()->CreateBuffer(&bd, nullptr, &b.buffer)))
        return false;

    D3D11_SHADER_RESOURCE_VIEW_DESC sv{};
    sv.Format = DXGI_FORMAT_UNKNOWN;
    sv.ViewDimension = D3D11_SRV_DIMENSION_BUFFER;
    sv.Buffer.NumElements = capacity;
    if (FAILED(Dev()->CreateShaderResourceView(b.buffer, &sv, &b.srv))) {
        b.Release();
        return false;
    }

    if (writable) {
        D3D11_UNORDERED_ACCESS_VIEW_DESC uv{};
        uv.Format = DXGI_FORMAT_UNKNOWN;
        uv.ViewDimension = D3D11_UAV_DIMENSION_BUFFER;
        uv.Buffer.NumElements = capacity;
        if (FAILED(Dev()->CreateUnorderedAccessView(b.buffer, &uv, &b.uav))) {
            b.Release();
            return false;
        }
    }
    b.stride = stride;
    b.capacity = capacity;
    b.writable = writable;
    return true;
}

// A dynamic buffer is written whole each time, which is what MAP_WRITE_DISCARD is for. A
// default buffer is the shader's own output and is seeded through UpdateSubresource instead.
bool Fill(Buffer& b, const void* data, size_t bytes)
{
    if (!b.buffer)
        return false;
    if (!data || bytes == 0)
        return true;   // nothing to copy is not a failure, and memcpy from null is a fault
    if (b.writable) {
        D3D11_BOX box{};
        box.right = UINT(bytes);
        box.bottom = 1;
        box.back = 1;
        Ctx()->UpdateSubresource(b.buffer, 0, &box, data, 0, 0);
        return true;
    }
    D3D11_MAPPED_SUBRESOURCE m{};
    if (FAILED(Ctx()->Map(b.buffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &m)))
        return false;
    memcpy(m.pData, data, bytes);
    Ctx()->Unmap(b.buffer, 0);
    return true;
}

// Upload `data` into a freshly sized buffer, for the world geometry.
//
// An empty array still gets a buffer: a structured buffer cannot have zero elements, and the
// shader stays bound to all of them whether or not it reads any. What it does not get is a
// copy, because the source pointer for an empty vector is null and memcpy from null is a fault
// rather than a no-op. A mesh with no triangle index and a scene with no hulls both take that
// path, so it is the ordinary case rather than an edge one.
bool Upload(Buffer& b, UINT stride, const void* data, size_t elements)
{
    if (!EnsureBuffer(b, stride, UINT(std::max<size_t>(elements, 1)), false))
        return false;
    if (!data || elements == 0)
        return true;
    return Fill(b, data, size_t(stride) * elements);
}

void ReleaseAll()
{
    for (Buffer* b : {&s_pieces, &s_stepList, &s_blasts, &s_colliders, &s_meshes, &s_verts,
                      &s_indices, &s_gridStart, &s_gridTris, &s_planes, &s_runs, &s_cellPieces,
                      &s_gridHead, &s_gridNext, &s_pieceCell, &s_supported})
        b->Release();
    if (s_params) { s_params->Release(); s_params = nullptr; }
    if (s_readback) { s_readback->Release(); s_readback = nullptr; }
    s_readbackBytes = 0;
    for (ID3D11ComputeShader** cs : {&s_sweep, &s_pairs, &s_settle})
        if (*cs) { (*cs)->Release(); *cs = nullptr; }
    s_haveWorld = false;
    s_colliderCountUploaded = 0;
    s_usable = false;
}

} // namespace

bool Start()
{
    std::lock_guard<std::mutex> lock(s_mutex);
    if (s_tried)
        return s_usable;
    s_tried = true;

#if !FLEXREVIVE_HAVE_GPU
    s_notReady = "this build has no compute shaders; fxc was not found when it was built";
    log::Write("gpu: %s", s_notReady);
    return false;
#else
    if (!device::Start()) {
        s_notReady = "no compute device is available";
        return false;
    }

    struct { const void* blob; size_t size; ID3D11ComputeShader** out; const char* name; }
    kernels[] = {
        {g_sweepCS, sizeof(g_sweepCS), &s_sweep, "world collision"},
        {g_pairsCS, sizeof(g_pairsCS), &s_pairs, "piece against piece"},
        {g_settleCS, sizeof(g_settleCS), &s_settle, "settle"},
    };
    for (const auto& k : kernels) {
        if (FAILED(Dev()->CreateComputeShader(k.blob, k.size, nullptr, k.out))) {
            s_notReady = "a compute shader was rejected by the driver";
            log::Write("gpu: the %s shader was rejected by the driver, staying on the CPU",
                       k.name);
            ReleaseAll();
            return false;
        }
    }

    D3D11_BUFFER_DESC cb{};
    cb.ByteWidth = sizeof(scene::Params);
    cb.Usage = D3D11_USAGE_DYNAMIC;
    cb.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
    cb.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
    if (FAILED(Dev()->CreateBuffer(&cb, nullptr, &s_params))) {
        s_notReady = "the parameter buffer could not be created";
        log::Write("gpu: %s, staying on the CPU", s_notReady);
        ReleaseAll();
        return false;
    }

    s_usable = true;
    s_notReady = "the piece-versus-piece, settle and particle passes are not yet dispatched, "
                 "so a frame would still have to come back to the CPU part-way through";
    log::Write("gpu: compute backend initialised on %s", device::Info().description);
    return true;
#endif
}

bool Ready()
{
    // Deliberately not s_usable. The device works and the world pass runs, but the rest of the
    // substep is still the CPU's, and a frame that has to return mid-way pays the readback it
    // was arranged to avoid. See the note at the top of the header.
    return false;
}

const char* NotReadyReason()
{
    return s_notReady;
}

bool SetWorld(const scene::Collider* colliders, int colliderCount, const MeshUpload* meshes,
              int meshCount, const float* planes, int planeCount)
{
#if !FLEXREVIVE_HAVE_GPU
    (void)colliders; (void)colliderCount; (void)meshes; (void)meshCount;
    (void)planes; (void)planeCount;
    return false;
#else
    std::lock_guard<std::mutex> lock(s_mutex);
    if (!s_usable)
        return false;

    s_meshStage.clear();
    s_vertStage.clear();
    s_indexStage.clear();
    s_gridStartStage.clear();
    s_gridTriStage.clear();

    // Every mesh is concatenated into one set of arrays, each descriptor recording where its
    // own run begins. A shader cannot hold an array of buffers, and binding one per mesh would
    // mean a dispatch per mesh.
    for (int i = 0; i < meshCount; ++i) {
        const MeshUpload& m = meshes[i];
        scene::MeshDesc d{};
        for (int a = 0; a < 3; ++a) {
            d.lower[a] = m.lower[a];
            d.upper[a] = m.upper[a];
            d.gridOrigin[a] = m.gridOrigin[a];
            d.gridInvCell[a] = m.gridInvCell[a];
            d.gridDim[a] = uint32_t(std::max(m.gridDim[a], 0));
        }
        d.offsets[0] = uint32_t(s_vertStage.size() / 4);
        d.offsets[1] = uint32_t(s_indexStage.size());
        d.offsets[2] = uint32_t(s_gridStartStage.size());
        d.offsets[3] = uint32_t(s_gridTriStage.size());
        d.counts[0] = uint32_t(std::max(m.vertCount, 0));
        d.counts[1] = uint32_t(std::max(m.triCount, 0));

        // The grid is usable only if it is entirely present. A partly uploaded index would send
        // the sweep to cells that hold nothing and quietly stop finding surfaces.
        const bool haveGrid = m.gridStart && m.gridTris && m.gridCellCount > 0 &&
                              m.gridDim[0] > 0 && m.gridDim[1] > 0 && m.gridDim[2] > 0;
        d.gridOrigin[3] = haveGrid ? 1.0f : 0.0f;
        s_meshStage.push_back(d);

        // Vertices widen to float4 so the buffer is indexed in whole 16-byte elements, which is
        // the same discipline the structs follow and for the same reason.
        for (int v = 0; v < m.vertCount; ++v) {
            for (int a = 0; a < 3; ++a)
                s_vertStage.push_back(m.verts ? m.verts[size_t(v) * 3 + size_t(a)] : 0.0f);
            s_vertStage.push_back(0.0f);
        }
        for (int k = 0; k < m.triCount * 3; ++k)
            s_indexStage.push_back(uint32_t(m.indices ? std::max(m.indices[k], 0) : 0));
        if (haveGrid) {
            for (int k = 0; k < m.gridCellCount; ++k)
                s_gridStartStage.push_back(uint32_t(std::max(m.gridStart[k], 0)));
            for (int k = 0; k < m.gridTriCount; ++k)
                s_gridTriStage.push_back(uint32_t(std::max(m.gridTris[k], 0)));
        }
    }

    const bool ok =
        Upload(s_colliders, sizeof(scene::Collider), colliders,
               size_t(std::max(colliderCount, 0))) &&
        Upload(s_meshes, sizeof(scene::MeshDesc), s_meshStage.data(), s_meshStage.size()) &&
        Upload(s_verts, 16, s_vertStage.data(), s_vertStage.size() / 4) &&
        Upload(s_indices, 4, s_indexStage.data(), s_indexStage.size()) &&
        Upload(s_gridStart, 4, s_gridStartStage.data(), s_gridStartStage.size()) &&
        Upload(s_gridTris, 4, s_gridTriStage.data(), s_gridTriStage.size()) &&
        Upload(s_planes, 16, planes, size_t(std::max(planeCount, 0)));

    s_colliderCountUploaded = ok ? std::max(colliderCount, 0) : 0;
    s_haveWorld = ok;
    if (!ok)
        log::Write("gpu: world geometry could not be uploaded, staying on the CPU");
    return ok;
#endif
}

bool StepFrame(const Frame& f)
{
#if !FLEXREVIVE_HAVE_GPU
    (void)f;
    return false;
#else
    std::lock_guard<std::mutex> lock(s_mutex);
    s_dispatchMs = s_readbackMs = 0.0;
    if (!s_usable || !s_haveWorld || !f.positions || !f.velocities || !f.rotations ||
        !f.angular || !f.mass || !f.radii || !f.resting || f.count <= 0 || !f.stepList ||
        f.stepCount <= 0)
        return false;

    const auto start = std::chrono::steady_clock::now();

    s_pieceStage.resize(size_t(f.count));
    for (int i = 0; i < f.count; ++i) {
        scene::Piece& g = s_pieceStage[size_t(i)];
        for (int a = 0; a < 3; ++a) {
            g.posRadius[a] = f.positions[size_t(i) * 3 + size_t(a)];
            g.velMass[a] = f.velocities[size_t(i) * 3 + size_t(a)];
            g.angGyr[a] = f.angular[size_t(i) * 3 + size_t(a)];
        }
        for (int a = 0; a < 4; ++a)
            g.rot[a] = f.rotations[size_t(i) * 4 + size_t(a)];
        g.posRadius[3] = f.radii[size_t(i)];
        g.velMass[3] = f.mass[size_t(i)];
        g.angGyr[3] = 0.0f;
        g.state[0] = f.resting[size_t(i)] ? 1u : 0u;
        g.state[1] = scene::kNoIndex;
        g.state[2] = 0u;
        g.state[3] = 0u;
    }

    s_stepStage.resize(size_t(f.stepCount));
    for (int k = 0; k < f.stepCount; ++k) {
        const int piece = f.stepList[k];
        s_stepStage[size_t(k)] = uint32_t(piece >= 0 && piece < f.count ? piece : 0);
        if (piece >= 0 && piece < f.count)
            s_pieceStage[size_t(piece)].state[2] = 1u;   // stepping, for the pair pass
    }

    const int blastN = std::max(f.blastCount, 0);
    s_blastStage.resize(size_t(std::max(blastN, 1)));
    for (int b = 0; b < blastN; ++b) {
        scene::Blast& g = s_blastStage[size_t(b)];
        for (int a = 0; a < 3; ++a)
            g.posRadius[a] = f.blasts[b].pos[a];
        g.posRadius[3] = f.blasts[b].radius;
        g.strength[0] = f.blasts[b].strength;
        g.strength[1] = f.blasts[b].linearFalloff ? 1.0f : 0.0f;
        g.strength[2] = g.strength[3] = 0.0f;
    }

    if (!EnsureBuffer(s_pieces, sizeof(scene::Piece), UINT(f.count), true) ||
        !EnsureBuffer(s_stepList, 4, UINT(f.stepCount), false) ||
        !EnsureBuffer(s_blasts, sizeof(scene::Blast), UINT(s_blastStage.size()), false))
        return false;
    if (!Fill(s_pieces, s_pieceStage.data(), s_pieceStage.size() * sizeof(scene::Piece)) ||
        !Fill(s_stepList, s_stepStage.data(), s_stepStage.size() * 4) ||
        !Fill(s_blasts, s_blastStage.data(), s_blastStage.size() * sizeof(scene::Blast)))
        return false;

    scene::Params p{};
    for (int a = 0; a < 3; ++a)
        p.gravityDt[a] = f.gravity[a];
    p.gravityDt[3] = f.dt;
    p.dragMaxSpeed[0] = f.dragBase;
    p.dragMaxSpeed[1] = f.maxSpeed;
    p.dragMaxSpeed[2] = f.contactSkin;
    p.dragMaxSpeed[3] = f.restitution;
    p.friction[0] = f.dynamicFriction;
    p.friction[1] = f.pieceFriction;
    p.friction[2] = f.settleRate;
    p.friction[3] = f.heftBounce;
    p.sleep[0] = f.sleepSpeed;
    p.sleep[1] = f.noBounceSpeed;
    p.sleep[2] = f.rollBlend;
    p.sleep[3] = f.spinDamp;
    p.counts[0] = uint32_t(f.count);
    p.counts[1] = uint32_t(f.stepCount);
    p.counts[3] = uint32_t(blastN);
    p.more[0] = uint32_t(s_meshStage.size());
    p.more[1] = uint32_t(std::max(f.substeps, 1));
    p.more[2] = f.rolling ? 1u : 0u;
    p.counts[2] = uint32_t(std::max(0, s_colliderCountUploaded));

    // The pair pass's neighbourhood. Built from the positions at the start of the frame and
    // held for all its substeps, where the CPU rebuilds it each one. Pieces move a fraction of
    // a cell in a substep, so the same neighbours are offered either way; rebuilding it would
    // mean a readback per substep, which is the cost this design exists to avoid.
    const float cellSize = std::max(2.0f * f.widestPiece + f.contactSkin, 8.0f);
    s_headStage.assign(size_t(scene::kBuckets), scene::kNoIndex);
    s_nextStage.assign(size_t(f.count), scene::kNoIndex);
    s_cellStage.assign(size_t(f.count) * 4, 0);
    scene::BuildHash(f.positions, f.count, cellSize, s_headStage.data(), s_nextStage.data(),
                     s_cellStage.data());
    scene::BuildCellRuns(s_cellStage.data(), f.stepList, f.stepCount, s_cellPieceStage,
                         s_runStage, s_colourFirst, s_colourCount, s_looseStage);

    s_supportedStage.assign(size_t(f.count), 0u);

    const bool pairs = f.debrisVsDebris && !s_runStage.empty();
    if (pairs) {
        if (!EnsureBuffer(s_runs, sizeof(scene::CellRun), UINT(s_runStage.size()), false) ||
            !EnsureBuffer(s_cellPieces, 4, UINT(s_cellPieceStage.size()), false) ||
            !EnsureBuffer(s_gridHead, 4, UINT(s_headStage.size()), false) ||
            !EnsureBuffer(s_gridNext, 4, UINT(s_nextStage.size()), false) ||
            !EnsureBuffer(s_pieceCell, 16, UINT(f.count), false) ||
            !EnsureBuffer(s_supported, 4, UINT(f.count), true))
            return false;
        if (!Fill(s_runs, s_runStage.data(), s_runStage.size() * sizeof(scene::CellRun)) ||
            !Fill(s_cellPieces, s_cellPieceStage.data(), s_cellPieceStage.size() * 4) ||
            !Fill(s_gridHead, s_headStage.data(), s_headStage.size() * 4) ||
            !Fill(s_gridNext, s_nextStage.data(), s_nextStage.size() * 4) ||
            !Fill(s_pieceCell, s_cellStage.data(), s_cellStage.size() * 4))
            return false;
    }

    ID3D11ShaderResourceView* sweepSrvs[] = {
        s_stepList.srv, s_blasts.srv, s_colliders.srv, s_meshes.srv,
        s_verts.srv, s_indices.srv, s_gridStart.srv, s_gridTris.srv, s_planes.srv,
    };
    ID3D11ShaderResourceView* pairSrvs[] = {
        s_runs.srv, s_cellPieces.srv, s_gridHead.srv, s_gridNext.srv, s_pieceCell.srv,
    };
    ID3D11ShaderResourceView* settleSrvs[] = {s_supported.srv};
    ID3D11ShaderResourceView* noSrvs[9] = {};
    ID3D11UnorderedAccessView* noUavs[2] = {};

    auto setParams = [&]() {
        D3D11_MAPPED_SUBRESOURCE m{};
        if (FAILED(Ctx()->Map(s_params, 0, D3D11_MAP_WRITE_DISCARD, 0, &m)))
            return false;
        memcpy(m.pData, &p, sizeof(p));
        Ctx()->Unmap(s_params, 0);
        Ctx()->CSSetConstantBuffers(0, 1, &s_params);
        return true;
    };

    // Every substep is a chain of dispatches with nothing read between them. That is the whole
    // point: the state stays on the card for the frame and only the result comes back.
    for (int s = 0; s < std::max(f.substeps, 1); ++s) {
        // World collision, one thread per moving piece.
        p.pass[0] = p.pass[1] = 0;
        if (!setParams())
            return false;
        Ctx()->CSSetShader(s_sweep, nullptr, 0);
        Ctx()->CSSetShaderResources(0, UINT(sizeof(sweepSrvs) / sizeof(sweepSrvs[0])),
                                    sweepSrvs);
        Ctx()->CSSetUnorderedAccessViews(0, 1, &s_pieces.uav, nullptr);
        Ctx()->Dispatch(UINT((f.stepCount + 63) / 64), 1, 1);
        Ctx()->CSSetUnorderedAccessViews(0, 2, noUavs, nullptr);
        Ctx()->CSSetShaderResources(0, 9, noSrvs);

        if (pairs) {
            // Which pieces the pile holds up, rebuilt each substep: stale support would let a
            // piece sleep with nothing under it.
            const UINT zero[4] = {0, 0, 0, 0};
            Ctx()->ClearUnorderedAccessViewUint(s_supported.uav, zero);

            // One dispatch per colour, in turn. Two cells of a colour cannot reach the same
            // piece, so a colour is safe to run at once; the colours are sequenced because each
            // has to see what the one before it left behind.
            ID3D11UnorderedAccessView* pairUavs[] = {s_pieces.uav, s_supported.uav};
            Ctx()->CSSetShader(s_pairs, nullptr, 0);
            Ctx()->CSSetShaderResources(0, UINT(sizeof(pairSrvs) / sizeof(pairSrvs[0])),
                                        pairSrvs);
            Ctx()->CSSetUnorderedAccessViews(0, 2, pairUavs, nullptr);
            for (int c = 0; c < scene::kColours; ++c) {
                const uint32_t runs = s_colourCount[size_t(c)];
                if (runs == 0)
                    continue;
                p.pass[0] = runs;
                p.pass[1] = s_colourFirst[size_t(c)];
                if (!setParams())
                    return false;
                Ctx()->Dispatch(UINT((runs + 63) / 64), 1, 1);
            }
            Ctx()->CSSetUnorderedAccessViews(0, 2, noUavs, nullptr);
            Ctx()->CSSetShaderResources(0, 9, noSrvs);

            // A chunk the pile is holding up can now be parked. Every other sleep test lives
            // inside a world contact, which a piece buried in a heap never reaches.
            p.pass[0] = p.pass[1] = 0;
            if (!setParams())
                return false;
            Ctx()->CSSetShader(s_settle, nullptr, 0);
            Ctx()->CSSetShaderResources(0, 1, settleSrvs);
            Ctx()->CSSetUnorderedAccessViews(0, 1, &s_pieces.uav, nullptr);
            Ctx()->Dispatch(UINT((f.count + 63) / 64), 1, 1);
            Ctx()->CSSetUnorderedAccessViews(0, 2, noUavs, nullptr);
            Ctx()->CSSetShaderResources(0, 9, noSrvs);
        }
    }

    s_dispatchMs = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - start).count();

    // The one readback, at the end of the frame. Timed separately because it is the fixed cost
    // the whole design is arranged around.
    const auto readStart = std::chrono::steady_clock::now();
    const UINT bytes = UINT(s_pieceStage.size() * sizeof(scene::Piece));
    if (!s_readback || s_readbackBytes < bytes) {
        if (s_readback)
            s_readback->Release();
        s_readback = nullptr;
        D3D11_BUFFER_DESC rd{};
        rd.ByteWidth = bytes;
        rd.Usage = D3D11_USAGE_STAGING;
        rd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        if (FAILED(Dev()->CreateBuffer(&rd, nullptr, &s_readback)))
            return false;
        s_readbackBytes = bytes;
    }

    D3D11_BOX box{};
    box.right = bytes;
    box.bottom = 1;
    box.back = 1;
    Ctx()->CopySubresourceRegion(s_readback, 0, 0, 0, 0, s_pieces.buffer, 0, &box);

    D3D11_MAPPED_SUBRESOURCE m{};
    if (FAILED(Ctx()->Map(s_readback, 0, D3D11_MAP_READ, 0, &m)))
        return false;
    const auto* out = static_cast<const scene::Piece*>(m.pData);
    for (int i = 0; i < f.count; ++i) {
        for (int a = 0; a < 3; ++a) {
            f.positions[size_t(i) * 3 + size_t(a)] = out[i].posRadius[a];
            f.velocities[size_t(i) * 3 + size_t(a)] = out[i].velMass[a];
            f.angular[size_t(i) * 3 + size_t(a)] = out[i].angGyr[a];
        }
        for (int a = 0; a < 4; ++a)
            f.rotations[size_t(i) * 4 + size_t(a)] = out[i].rot[a];
        f.resting[size_t(i)] = out[i].state[0] ? 1 : 0;
    }
    Ctx()->Unmap(s_readback, 0);

    s_readbackMs = std::chrono::duration<double, std::milli>(
                       std::chrono::steady_clock::now() - readStart).count();
    return true;
#endif
}

void LastTiming(double& outDispatchMs, double& outReadbackMs)
{
    outDispatchMs = s_dispatchMs;
    outReadbackMs = s_readbackMs;
}

void StopForTesting()
{
    std::lock_guard<std::mutex> lock(s_mutex);
    ReleaseAll();
    s_tried = false;
    device::StopForTesting();
}

}
