// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "gpu/Scene.h"

#include <cmath>

namespace flexrevive::gpu::scene {

namespace {

int CellOf(float v, float invCell)
{
    return int(std::floor(v * invCell));
}

uint32_t BucketOf(int x, int y, int z)
{
    const uint32_t h = uint32_t(x) * 73856093u ^ uint32_t(y) * 19349663u ^
                       uint32_t(z) * 83492791u;
    return h & uint32_t(kBuckets - 1);
}

} // namespace

int ColourOfCell(const int32_t* cell)
{
    // Modulo that stays non-negative. Cells left of the origin have negative coordinates and
    // the built-in operator would fold them onto negative colours, which indexes out of the
    // bucket array rather than merely giving a different answer.
    auto m3 = [](int32_t v) { return ((v % 3) + 3) % 3; };
    return (m3(cell[0]) * 3 + m3(cell[1])) * 3 + m3(cell[2]);
}

void BuildHash(const float* positions, int count, float cellSize, uint32_t* outHead,
               uint32_t* outNext, int32_t* outCells)
{
    if (!outHead || !outNext || !outCells)
        return;
    for (int b = 0; b < kBuckets; ++b)
        outHead[b] = kNoIndex;
    for (int i = 0; i < count; ++i) {
        outNext[i] = kNoIndex;
        for (int a = 0; a < 4; ++a)
            outCells[size_t(i) * 4 + size_t(a)] = 0;
    }
    if (!positions || count <= 0)
        return;

    const float invCell = 1.0f / std::fmax(cellSize, 1e-3f);
    for (int i = 0; i < count; ++i) {
        const float* p = &positions[size_t(i) * 3];
        // A retired slot can hold anything, and one that is not a finite position must not be
        // indexed: it would land in an arbitrary bucket and be offered as a neighbour to
        // whatever else happens to hash there.
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]))
            continue;

        const int cell[3] = {CellOf(p[0], invCell), CellOf(p[1], invCell), CellOf(p[2], invCell)};
        for (int a = 0; a < 3; ++a)
            outCells[size_t(i) * 4 + size_t(a)] = cell[a];
        outCells[size_t(i) * 4 + 3] = 1;   // indexed

        const uint32_t bucket = BucketOf(cell[0], cell[1], cell[2]);
        outNext[i] = outHead[bucket];
        outHead[bucket] = uint32_t(i);
    }
}

}
