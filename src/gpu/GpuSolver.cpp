// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "gpu/GpuSolver.h"

#include "gpu/Device.h"
#include "f4kit/Log.h"

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <d3d11.h>

#include <algorithm>
#include <mutex>
#include <vector>

#if FLEXREVIVE_HAVE_GPU
#include "integrate_cs.h"
#endif

using namespace f4kit;

namespace flexrevive::gpu {

namespace {

std::mutex s_mutex;
bool s_tried = false;
bool s_usable = false;
const char* s_notReady = "the collision passes still run on the CPU, so a GPU step would "
                         "cost a readback per substep to save less than it spends";

ID3D11ComputeShader* s_integrate = nullptr;

// Per-piece state as the shader sees it, and it has to be exactly as the shader sees it: the
// stride is declared on the buffer and the shader indexes with it, so a struct that disagrees
// reads every field of every piece from the wrong offset.
//
// Structured buffers pack tightly. That is the opposite of a constant buffer, where a vector
// that would straddle a 16-byte register is pushed to the next one, and assuming the constant
// buffer rule here is what made the first version of this read `vel` four bytes late. The
// authority is `fxc /Fc`, which prints the offset of every member; the sizes asserted below
// are copied from it rather than worked out by hand.
//
//   struct Piece { float3 pos;  // 0
//                  float3 vel;  // 12
//                  float mass;  // 24   } size 28
struct GpuPiece {
    float pos[3];
    float vel[3];
    float mass;
};
static_assert(sizeof(GpuPiece) == 28, "GpuPiece must match struct Piece in integrate.hlsl");

//   struct Blast { float3 pos;          // 0
//                  float radius;        // 12
//                  float strength;      // 16
//                  uint linearFalloff;  // 20   } size 24
struct GpuBlast {
    float pos[3];
    float radius;
    float strength;
    uint32_t linearFalloff;
};
static_assert(sizeof(GpuBlast) == 24, "GpuBlast must match struct Blast in integrate.hlsl");

// A constant buffer, by contrast, really is packed into 16-byte registers: gravity and dt
// share the first one, which is why dt sits at 12 rather than 16.
struct GpuParams {
    float gravity[3];
    float dt;
    float dragBase;
    float maxSpeed;
    uint32_t stepCount;
    uint32_t blastCount;
};
static_assert(sizeof(GpuParams) == 32, "GpuParams must match cbuffer Params in integrate.hlsl");
static_assert(sizeof(GpuParams) % 16 == 0, "constant buffers are sized in 16-byte registers");

// A structured buffer that grows to fit and is never shrunk, so a steady scene stops
// allocating after its first few frames.
struct Buffer {
    ID3D11Buffer* buffer = nullptr;
    ID3D11ShaderResourceView* srv = nullptr;
    ID3D11UnorderedAccessView* uav = nullptr;
    UINT capacity = 0;

    void Release()
    {
        if (uav) { uav->Release(); uav = nullptr; }
        if (srv) { srv->Release(); srv = nullptr; }
        if (buffer) { buffer->Release(); buffer = nullptr; }
        capacity = 0;
    }
};

Buffer s_pieces;
Buffer s_stepList;
Buffer s_blasts;
ID3D11Buffer* s_params = nullptr;
ID3D11Buffer* s_readback = nullptr;
UINT s_readbackCapacity = 0;

std::vector<GpuPiece> s_upload;
std::vector<GpuBlast> s_blastUpload;
std::vector<uint32_t> s_stepUpload;

ID3D11Device* Dev() { return static_cast<ID3D11Device*>(device::Device()); }
ID3D11DeviceContext* Ctx() { return static_cast<ID3D11DeviceContext*>(device::Context()); }

// A dynamic structured buffer, mapped with DISCARD each frame. Dynamic rather than default
// because every one of these is written whole from the CPU every step, which is exactly the
// case the discard path exists for.
bool EnsureBuffer(Buffer& b, UINT stride, UINT count, bool writable)
{
    if (b.buffer && b.capacity >= count)
        return true;
    b.Release();

    // Grown in powers of two, so a scene that creeps upward does not reallocate every frame.
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
    b.capacity = capacity;
    return true;
}

// The piece buffer is written by the shader, so it cannot be dynamic and is filled through a
// staging upload instead.
bool WriteDefaultBuffer(ID3D11Buffer* dst, const void* data, size_t bytes)
{
    D3D11_BOX box{};
    box.left = 0;
    box.right = UINT(bytes);
    box.bottom = 1;
    box.back = 1;
    Ctx()->UpdateSubresource(dst, 0, &box, data, 0, 0);
    return true;
}

bool WriteDynamicBuffer(ID3D11Buffer* dst, const void* data, size_t bytes)
{
    D3D11_MAPPED_SUBRESOURCE m{};
    if (FAILED(Ctx()->Map(dst, 0, D3D11_MAP_WRITE_DISCARD, 0, &m)))
        return false;
    memcpy(m.pData, data, bytes);
    Ctx()->Unmap(dst, 0);
    return true;
}

void ReleaseAll()
{
    s_pieces.Release();
    s_stepList.Release();
    s_blasts.Release();
    if (s_params) { s_params->Release(); s_params = nullptr; }
    if (s_readback) { s_readback->Release(); s_readback = nullptr; }
    s_readbackCapacity = 0;
    if (s_integrate) { s_integrate->Release(); s_integrate = nullptr; }
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

    if (FAILED(Dev()->CreateComputeShader(g_integrateCS, sizeof(g_integrateCS), nullptr,
                                          &s_integrate))) {
        s_notReady = "the integration shader was rejected by the driver";
        log::Write("gpu: %s, staying on the CPU", s_notReady);
        return false;
    }

    D3D11_BUFFER_DESC cb{};
    cb.ByteWidth = sizeof(GpuParams);
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
    log::Write("gpu: compute backend initialised on %s", device::Info().description);
    return true;
#endif
}

bool Ready()
{
    // Deliberately not `s_usable`. The device works and the pass is correct, but the loop is
    // not yet resident on the card, so running it would cost more than it saves. See the note
    // at the top of GpuSolver.h.
    return false;
}

const char* NotReadyReason()
{
    return s_notReady;
}

bool IntegrateStep(float* positions, float* velocities, const float* mass, int count,
                   const int* stepList, int stepCount, const Blast* blasts, int blastCount,
                   const StepParams& params)
{
#if !FLEXREVIVE_HAVE_GPU
    (void)positions; (void)velocities; (void)mass; (void)count;
    (void)stepList; (void)stepCount; (void)blasts; (void)blastCount; (void)params;
    return false;
#else
    std::lock_guard<std::mutex> lock(s_mutex);
    if (!s_usable || !positions || !velocities || !mass || count <= 0 || !stepList ||
        stepCount <= 0)
        return false;

    s_upload.resize(size_t(count));
    for (int i = 0; i < count; ++i) {
        GpuPiece& g = s_upload[size_t(i)];
        for (int a = 0; a < 3; ++a) {
            g.pos[a] = positions[size_t(i) * 3 + size_t(a)];
            g.vel[a] = velocities[size_t(i) * 3 + size_t(a)];
        }
        g.mass = mass[size_t(i)];
    }

    s_stepUpload.resize(size_t(stepCount));
    for (int k = 0; k < stepCount; ++k)
        s_stepUpload[size_t(k)] = uint32_t(stepList[k] >= 0 ? stepList[k] : 0);

    // At least one entry, since a zero-length structured buffer cannot be created and the
    // shader reads none of it when the count is zero.
    const int blastN = std::max(blastCount, 0);
    s_blastUpload.resize(size_t(std::max(blastN, 1)));
    for (int b = 0; b < blastN; ++b) {
        GpuBlast& g = s_blastUpload[size_t(b)];
        for (int a = 0; a < 3; ++a)
            g.pos[a] = blasts[b].pos[a];
        g.radius = blasts[b].radius;
        g.strength = blasts[b].strength;
        g.linearFalloff = blasts[b].linearFalloff;
    }

    if (!EnsureBuffer(s_pieces, sizeof(GpuPiece), UINT(count), true) ||
        !EnsureBuffer(s_stepList, sizeof(uint32_t), UINT(stepCount), false) ||
        !EnsureBuffer(s_blasts, sizeof(GpuBlast), UINT(s_blastUpload.size()), false))
        return false;

    if (!WriteDefaultBuffer(s_pieces.buffer, s_upload.data(),
                            s_upload.size() * sizeof(GpuPiece)) ||
        !WriteDynamicBuffer(s_stepList.buffer, s_stepUpload.data(),
                            s_stepUpload.size() * sizeof(uint32_t)) ||
        !WriteDynamicBuffer(s_blasts.buffer, s_blastUpload.data(),
                            s_blastUpload.size() * sizeof(GpuBlast)))
        return false;

    GpuParams gp{};
    for (int a = 0; a < 3; ++a)
        gp.gravity[a] = params.gravity[a];
    gp.dt = params.dt;
    gp.dragBase = params.dragBase;
    gp.maxSpeed = params.maxSpeed;
    gp.stepCount = uint32_t(stepCount);
    gp.blastCount = uint32_t(blastN);
    if (!WriteDynamicBuffer(s_params, &gp, sizeof(gp)))
        return false;

    ID3D11ShaderResourceView* srvs[] = {s_stepList.srv, s_blasts.srv};
    Ctx()->CSSetShader(s_integrate, nullptr, 0);
    Ctx()->CSSetConstantBuffers(0, 1, &s_params);
    Ctx()->CSSetShaderResources(0, 2, srvs);
    Ctx()->CSSetUnorderedAccessViews(0, 1, &s_pieces.uav, nullptr);
    Ctx()->Dispatch(UINT((stepCount + 63) / 64), 1, 1);

    // Unbound before the readback, since a resource cannot be both a UAV and a copy source.
    ID3D11UnorderedAccessView* none = nullptr;
    Ctx()->CSSetUnorderedAccessViews(0, 1, &none, nullptr);

    const UINT bytes = UINT(s_upload.size() * sizeof(GpuPiece));
    if (!s_readback || s_readbackCapacity < bytes) {
        if (s_readback)
            s_readback->Release();
        s_readback = nullptr;
        D3D11_BUFFER_DESC rd{};
        rd.ByteWidth = bytes;
        rd.Usage = D3D11_USAGE_STAGING;
        rd.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        if (FAILED(Dev()->CreateBuffer(&rd, nullptr, &s_readback)))
            return false;
        s_readbackCapacity = bytes;
    }

    D3D11_BOX box{};
    box.right = bytes;
    box.bottom = 1;
    box.back = 1;
    Ctx()->CopySubresourceRegion(s_readback, 0, 0, 0, 0, s_pieces.buffer, 0, &box);

    D3D11_MAPPED_SUBRESOURCE m{};
    if (FAILED(Ctx()->Map(s_readback, 0, D3D11_MAP_READ, 0, &m)))
        return false;
    const auto* out = static_cast<const GpuPiece*>(m.pData);
    for (int i = 0; i < count; ++i)
        for (int a = 0; a < 3; ++a) {
            positions[size_t(i) * 3 + size_t(a)] = out[i].pos[a];
            velocities[size_t(i) * 3 + size_t(a)] = out[i].vel[a];
        }
    Ctx()->Unmap(s_readback, 0);
    return true;
#endif
}

void StopForTesting()
{
    std::lock_guard<std::mutex> lock(s_mutex);
    ReleaseAll();
    s_tried = false;
    device::StopForTesting();
}

}
