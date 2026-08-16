// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "Math3D.h"

#include <cmath>
#include <cstring>

namespace flexrevive::math {

uint32_t Mix(uint32_t x)
{
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

uint32_t FloatBits(float f)
{
    uint32_t u = 0;
    memcpy(&u, &f, sizeof(u));
    return u;
}

uint64_t PositionKey(const float* p)
{
    uint64_t h = 1469598103934665603ull;
    for (int a = 0; a < 3; ++a) {
        h ^= FloatBits(p[a]);
        h *= 1099511628211ull;
    }
    return h;
}

float NextFloat(uint32_t& state)
{
    state = Mix(state + 0x9e3779b9u);
    return float(state >> 8) * (1.0f / 16777216.0f);
}



void QuatIntegrate(float* q, const float* omega, float dt)
{
    const float wx = omega[0] * 0.5f * dt;
    const float wy = omega[1] * 0.5f * dt;
    const float wz = omega[2] * 0.5f * dt;

    const float x = q[0], y = q[1], z = q[2], w = q[3];
    q[0] = x + (wx * w + wy * z - wz * y);
    q[1] = y + (wy * w + wz * x - wx * z);
    q[2] = z + (wz * w + wx * y - wy * x);
    q[3] = w - (wx * x + wy * y + wz * z);

    const float len = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (len > 1e-8f) {
        const float inv = 1.0f / len;
        q[0] *= inv; q[1] *= inv; q[2] *= inv; q[3] *= inv;
    } else {
        q[0] = q[1] = q[2] = 0.0f;
        q[3] = 1.0f;
    }
}




}
