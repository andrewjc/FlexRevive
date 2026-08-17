// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

// A spatial hash over piece positions, so piece-versus-piece costs a walk of each piece's own
// neighbourhood rather than the square of the pool.
//
// Cells are sized to the largest piece, so a chunk can only reach something in its own cell or
// one of the twenty-six around it. Buckets are shared between distant cells, which is harmless:
// a collision offers a pair that is far apart and the caller rejects it with one distance test.
// What must never happen is the opposite, a pair close enough to touch that is never offered.
namespace flexrevive::grid {

// Power of two, so the hash reduces with a mask.
constexpr int kBuckets = 8192;

// Cells are coloured by their coordinates modulo three on each axis, which is what lets the
// pieces in one colour be resolved at the same time.
//
// A piece reaches only into its own cell and the twenty-six around it, so resolving it touches
// nothing outside that block of three. Two cells of the same colour differ by a multiple of
// three on some axis, so their blocks do not overlap, so no two pieces of one colour can reach
// the same third piece. Three is the smallest stride with that property, and twenty-seven the
// smallest number of colours, because a stride of two puts a cell's block against its own
// neighbour's.
constexpr int kColours = 27;

class PieceGrid {
public:
    // Indexes `count` positions, three floats each. Slots holding anything that is not a finite
    // position are left out, since a retired slot can contain anything.
    void Build(const float* positions, int count, float cellSize);

    // Which cell a piece was indexed into, or false when it was left out.
    bool CellOfPiece(int piece, int* outCell) const
    {
        if (piece < 0 || size_t(piece) >= m_indexed.size() || !m_indexed[size_t(piece)])
            return false;
        for (int a = 0; a < 3; ++a)
            outCell[a] = m_cells[size_t(piece) * 3 + size_t(a)];
        return true;
    }

    // The colour of a piece's cell, or -1 when it was left out. In [0, kColours).
    int ColourOfPiece(int piece) const
    {
        int cell[3];
        if (!CellOfPiece(piece, cell))
            return -1;
        return ColourOfCell(cell);
    }

    static int ColourOfCell(const int* cell)
    {
        // Modulo that stays non-negative, since cells left of the origin have negative
        // coordinates and the built-in operator would fold them onto negative colours.
        auto m3 = [](int v) { return ((v % 3) + 3) % 3; };
        return (m3(cell[0]) * 3 + m3(cell[1])) * 3 + m3(cell[2]);
    }

    // Whether two cells are the same or touching, which is the whole of what a piece can reach.
    // The bucket walk offers pieces from unrelated cells that happen to share a hash bucket,
    // and those are outside the block a colour reserves, so they have to be dropped before
    // anything reads them.
    static bool CellsAdjacent(const int* a, const int* b)
    {
        for (int i = 0; i < 3; ++i) {
            const int d = a[i] - b[i];
            if (d < -1 || d > 1)
                return false;
        }
        return true;
    }

    // Offers every piece that could be within reach of `self` at `p`, by index, and only ever
    // indices above `self` so each pair is offered once.
    template <typename F>
    void ForEachNeighbour(int self, const float* p, F&& fn) const
    {
        ForEachBucket(p, [&](int j) {
            if (j > self)
                fn(j);
        });
    }

    // Every piece that could be within reach of `self`, in both directions.
    //
    // Walking only the pieces that are moving needs this rather than the ordered form: a
    // settled neighbour never appears as `self`, so the pair has to be reachable from the
    // moving side whatever the two indices happen to be. The caller decides how to avoid
    // handling a pair twice when both of them are moving.
    template <typename F>
    void ForEachCandidate(int self, const float* p, F&& fn) const
    {
        ForEachBucket(p, [&](int j) {
            if (j != self)
                fn(j);
        });
    }

private:
    // The distinct buckets a piece's own cell and its twenty-six neighbours occupy.
    template <typename F>
    void ForEachBucket(const float* p, F&& fn) const
    {
        if (m_head.empty())
            return;

        const int base[3] = {CellOf(p[0]), CellOf(p[1]), CellOf(p[2])};
        int buckets[27];
        int used = 0;
        for (int ox = -1; ox <= 1; ++ox)
            for (int oy = -1; oy <= 1; ++oy)
                for (int oz = -1; oz <= 1; ++oz) {
                    const int bucket = BucketOf(base[0] + ox, base[1] + oy, base[2] + oz);
                    bool seen = false;
                    for (int k = 0; k < used && !seen; ++k)
                        seen = buckets[k] == bucket;
                    if (!seen)
                        buckets[used++] = bucket;
                }

        for (int k = 0; k < used; ++k)
            for (int j = m_head[size_t(buckets[k])]; j >= 0; j = m_next[size_t(j)])
                fn(j);
    }

    int CellOf(float v) const { return int(std::floor(v * m_invCell)); }

    static int BucketOf(int x, int y, int z)
    {
        const uint32_t h = uint32_t(x) * 73856093u ^ uint32_t(y) * 19349663u ^
                           uint32_t(z) * 83492791u;
        return int(h & uint32_t(kBuckets - 1));
    }

    std::vector<int> m_head;
    std::vector<int> m_next;
    // Each piece's cell, kept so a candidate can be tested for being genuinely nearby rather
    // than merely sharing a hash bucket. Three ints per piece; only meaningful where
    // m_indexed says the piece was indexed.
    std::vector<int> m_cells;
    std::vector<uint8_t> m_indexed;
    float m_invCell = 1.0f;
};

}
