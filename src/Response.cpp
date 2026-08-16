// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "Response.h"

#include <algorithm>
#include <cmath>

namespace flexrevive::response {

// Below one unit a chunk is degenerate rather than light, so the laws clamp there.
float MassResponse(float mass)
{
    return 1.0f / std::cbrt(std::max(mass, 1.0f));
}

float CarryShare(float mass, float heft)
{
    const float share = MassResponse(mass) / std::max(heft, 0.05f);
    return std::min(1.0f, std::max(0.01f, share));
}

float DragFactor(float mass, float dragBase, float dt, float heft)
{
    const float base = std::max(dragBase, 0.0f) / std::max(heft, 0.05f);
    return 1.0f - std::min(base * MassResponse(mass) * std::max(dt, 0.0f), 1.0f);
}

void SeparationSplit(float massI, float massJ, float& shareI, float& shareJ)
{
    const float mi = std::max(massI, 1.0f);
    const float mj = std::max(massJ, 1.0f);
    const float total = mi + mj;
    shareI = mj / total;   // the lighter piece yields more
    shareJ = mi / total;
}

void ResolveAgainstMovingSurface(float* vel, const float* normal, const float* surfaceVel,
                                 float share, float restitution)
{
    const float s = std::min(1.0f, std::max(0.0f, share));
    const float bounce = std::max(restitution, 0.0f);

    // Scaled once, and used for both the entry into the surface's frame and the return from
    // it, so the transform is symmetric.
    const float carried[3] = {surfaceVel[0] * s, surfaceVel[1] * s, surfaceVel[2] * s};

    float rel[3];
    for (int a = 0; a < 3; ++a)
        rel[a] = vel[a] - carried[a];

    const float vn = rel[0] * normal[0] + rel[1] * normal[1] + rel[2] * normal[2];
    if (vn < 0.0f)
        for (int a = 0; a < 3; ++a)
            rel[a] -= normal[a] * vn * (1.0f + bounce);

    for (int a = 0; a < 3; ++a)
        vel[a] = rel[a] + carried[a];
}

}
