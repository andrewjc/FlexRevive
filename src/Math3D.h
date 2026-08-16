// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>

// Vector, quaternion, hashing and sampling primitives used by the solver.
//
// Vectors are bare `float*` of 3 components and quaternions of 4 as (x, y, z, w), matching the
// packed buffers the engine hands over, so nothing here copies to reach them. Matrices are
// row-major 3x3.
namespace flexrevive::math {

// The leaf operations are defined here rather than compiled separately: each is a few
// arithmetic instructions called from the innermost loops of the sweep and the contact
// solver, where the call itself would cost more than the work.

inline float Dot(const float* a, const float* b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

inline void Cross(const float* a, const float* b, float* out)
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

// Multiply a row-major 3x3 by a vector.
inline void Mat3Mul(const float* m, const float* v, float* out)
{
    out[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
    out[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
    out[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

// Rotate a vector by a quaternion (x, y, z, w).
inline void QuatRotate(const float* q, const float* v, float* out)
{
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    const float tx = 2.0f * (y * v[2] - z * v[1]);
    const float ty = 2.0f * (z * v[0] - x * v[2]);
    const float tz = 2.0f * (x * v[1] - y * v[0]);
    out[0] = v[0] + w * tx + (y * tz - z * ty);
    out[1] = v[1] + w * ty + (z * tx - x * tz);
    out[2] = v[2] + w * tz + (x * ty - y * tx);
}

// The inverse rotation, for a unit quaternion.
inline void QuatConjugate(const float* q, float* out)
{
    out[0] = -q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = q[3];
}

// Advance an orientation by an angular velocity in radians/sec for dt seconds, using
// q' = q + 0.5 * (omega as a pure quaternion) * q, renormalised. A quaternion that cannot be
// normalised becomes the identity rather than an arbitrary rotation.
void QuatIntegrate(float* q, const float* omega, float dt);

// A cheap integer hash. Adjacent inputs give unrelated outputs, which is what keeps pieces
// spawned into consecutive slots from tumbling along a regular sweep.
uint32_t Mix(uint32_t x);

// A value in [0, 1), advancing the state so successive draws are independent.
float NextFloat(uint32_t& state);

// The raw bits of a float, for exact comparison.
uint32_t FloatBits(float f);

// An exact fingerprint of a position. The engine returns transforms bit for bit, so the raw
// float bits identify a piece even when it turns up in a different slot.
uint64_t PositionKey(const float* p);

}
