// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "gpu/Scene.h"

#include <algorithm>
#include <array>
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

void BuildCellRuns(const int32_t* cells, const int* stepList, int stepCount,
                   std::vector<uint32_t>& outPieces, std::vector<CellRun>& outRuns,
                   std::vector<uint32_t>& outColourFirst, std::vector<uint32_t>& outColourCount,
                   std::vector<uint32_t>& outLoose)
{
    outPieces.clear();
    outRuns.clear();
    outLoose.clear();
    outColourFirst.assign(size_t(kColours), 0);
    outColourCount.assign(size_t(kColours), 0);
    if (!cells || !stepList || stepCount <= 0)
        return;

    // Sorted on colour, then cell, then step-list position. Sorting by colour first is what
    // makes each colour a contiguous span of runs, so a colour is one dispatch rather than a
    // gather; the rest is what makes the order deterministic.
    struct Entry {
        int colour;
        int32_t cell[3];
        uint32_t piece;
        int order;
    };
    std::vector<Entry> sorted;
    sorted.reserve(size_t(stepCount));

    for (int k = 0; k < stepCount; ++k) {
        const int piece = stepList[k];
        if (piece < 0)
            continue;
        const int32_t* c = &cells[size_t(piece) * 4];
        if (c[3] == 0) {
            outLoose.push_back(uint32_t(piece));   // no cell, so nothing bounds its reach
            continue;
        }
        Entry e;
        e.colour = ColourOfCell(c);
        for (int a = 0; a < 3; ++a)
            e.cell[a] = c[a];
        e.piece = uint32_t(piece);
        e.order = k;
        sorted.push_back(e);
    }

    std::sort(sorted.begin(), sorted.end(), [](const Entry& a, const Entry& b) {
        if (a.colour != b.colour) return a.colour < b.colour;
        for (int i = 0; i < 3; ++i)
            if (a.cell[i] != b.cell[i]) return a.cell[i] < b.cell[i];
        return a.order < b.order;
    });

    for (size_t a = 0; a < sorted.size();) {
        size_t b = a + 1;
        while (b < sorted.size() && sorted[b].colour == sorted[a].colour &&
               sorted[b].cell[0] == sorted[a].cell[0] &&
               sorted[b].cell[1] == sorted[a].cell[1] &&
               sorted[b].cell[2] == sorted[a].cell[2])
            ++b;

        const int colour = sorted[a].colour;
        if (outColourCount[size_t(colour)] == 0)
            outColourFirst[size_t(colour)] = uint32_t(outRuns.size());

        CellRun run{};
        run.range[0] = uint32_t(outPieces.size());
        run.range[1] = uint32_t(b - a);
        outRuns.push_back(run);
        ++outColourCount[size_t(colour)];

        for (size_t k = a; k < b; ++k)
            outPieces.push_back(sorted[k].piece);
        a = b;
    }
}

}
