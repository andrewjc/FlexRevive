// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>

// The solver's state, laid out the way the shaders read it.
//
// This is the half of the GPU backend that has nothing to do with Direct3D: it flattens the
// solver's own containers into the flat arrays a shader can index, and it is a pure function of
// its inputs. That is deliberate. Marshalling is where a port like this actually goes wrong,
// far more often than the arithmetic does, and a translation layer that needs a graphics device
// to exercise is a translation layer nobody tests.
//
// Every struct here is float4 and uint4 alone, matching shaders/common.hlsli exactly. See the
// note at the top of that file for why.
namespace flexrevive::gpu::scene {

struct Piece {
    float posRadius[4];   // xyz world position, w collision radius
    float velMass[4];     // xyz velocity, w mass
    float rot[4];         // orientation quaternion
    float angGyr[4];      // xyz angular velocity, w radius of gyration
    uint32_t state[4];    // x resting, y hull index or ~0, z stepping, w spare
};
static_assert(sizeof(Piece) == 80, "must match struct Piece in common.hlsli");

struct Collider {
    float posType[4];
    float rot[4];
    float dimsRadius[4];
    float prevMoved[4];
    float aabbLo[4];
    float aabbHi[4];
    uint32_t refs[4];     // x mesh index or ~0, y plane start, z plane count, w spare
};
static_assert(sizeof(Collider) == 112, "must match struct Collider in common.hlsli");

struct MeshDesc {
    float lower[4];
    float upper[4];
    float gridOrigin[4];   // w non-zero when the grid is usable
    float gridInvCell[4];
    uint32_t gridDim[4];
    uint32_t offsets[4];   // x vertex base, y index base, z grid start base, w grid tri base
    uint32_t counts[4];    // x vertex count, y triangle count
};
static_assert(sizeof(MeshDesc) == 112, "must match struct MeshDesc in common.hlsli");

struct Blast {
    float posRadius[4];
    float strength[4];     // x strength, y non-zero for linear falloff
};
static_assert(sizeof(Blast) == 32, "must match struct Blast in common.hlsli");

// One cell's pieces, as a run in the piece list. The unit of parallel work in the pair pass.
struct CellRun {
    uint32_t range[4];     // x start, y count
};
static_assert(sizeof(CellRun) == 16, "must match struct CellRun in pairs.hlsl");

// The parameter block, matching cbuffer Params.
struct Params {
    float gravityDt[4];
    float dragMaxSpeed[4];
    float friction[4];
    float sleep[4];
    uint32_t counts[4];
    uint32_t more[4];
};
static_assert(sizeof(Params) == 96, "must match cbuffer Params in common.hlsli");
static_assert(sizeof(Params) % 16 == 0, "constant buffers are sized in 16-byte registers");

// Matches grid::kBuckets and grid::kColours.
constexpr int kBuckets = 8192;
constexpr int kColours = 27;
constexpr uint32_t kNoIndex = 0xFFFFFFFFu;

// The spatial hash the pair pass walks, as the shader sees it: a bucket holds the index of its
// first piece and each piece the index of the next in the same bucket, both kNoIndex-terminated.
//
// Built here rather than copied from grid::PieceGrid because the CPU one keeps its links in
// signed ints with -1 as the terminator, and a shader reading that as unsigned would walk off
// the end of the world. Same hash, same cells, so the two agree on which pairs exist.
void BuildHash(const float* positions, int count, float cellSize, uint32_t* outHead,
               uint32_t* outNext, int32_t* outCells);

// The colour of a cell, matching grid::PieceGrid::ColourOfCell. Non-negative for any cell,
// including one left of the origin, where a plain modulo would fold onto a negative bucket.
int ColourOfCell(const int32_t* cell);

}
