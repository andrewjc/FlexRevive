// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "Shock.h"

#include "Response.h"

#include <algorithm>
#include <cmath>

namespace flexrevive::shock {

void Record(std::vector<Impact>& impacts, const float* pos, float deltaV, float radius)
{
    for (Impact& im : impacts) {
        const float d[3] = {pos[0] - im.pos[0], pos[1] - im.pos[1], pos[2] - im.pos[2]};
        if (d[0]*d[0] + d[1]*d[1] + d[2]*d[2] < kBurstRadius * kBurstRadius) {
            im.deltaV = std::max(im.deltaV, deltaV);
            return;
        }
    }
    if (int(impacts.size()) >= kMaxImpacts)
        return;

    Impact im;
    for (int a = 0; a < 3; ++a)
        im.pos[a] = pos[a];
    im.radius = radius;
    im.deltaV = deltaV;
    impacts.push_back(im);
}

bool PushFor(const std::vector<Impact>& impacts, const float* pos, float mass, float* outPush)
{
    float push[3] = {0, 0, 0};
    bool touched = false;

    for (const Impact& im : impacts) {
        const float d[3] = {pos[0] - im.pos[0], pos[1] - im.pos[1], pos[2] - im.pos[2]};
        const float dist2 = d[0]*d[0] + d[1]*d[1] + d[2]*d[2];
        if (dist2 >= im.radius * im.radius || dist2 < 1e-4f)
            continue;
        const float dist = std::sqrt(dist2);
        const float mag = im.deltaV * (1.0f - dist / im.radius);
        for (int a = 0; a < 3; ++a)
            push[a] += (d[a] / dist) * mag;
        push[2] += mag * kLiftFraction;
        touched = true;
    }
    if (!touched)
        return false;

    // A shock carries momentum, so its effect depends on the chunk: pressure follows cross
    // section while inertia follows volume, leaving the speed picked up falling off as the
    // cube root of mass.
    const float perMass = response::MassResponse(mass);
    for (int a = 0; a < 3; ++a)
        push[a] *= perMass;

    const float mag = std::sqrt(push[0]*push[0] + push[1]*push[1] + push[2]*push[2]);
    if (mag > kMaxShockSpeed) {
        const float k = kMaxShockSpeed / mag;
        for (int a = 0; a < 3; ++a)
            push[a] *= k;
    }

    for (int a = 0; a < 3; ++a)
        outPush[a] = push[a];
    return true;
}

}
