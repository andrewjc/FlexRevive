#pragma once

#include <cmath>
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

class PieceGrid {
public:
    // Indexes `count` positions, three floats each. Slots holding anything that is not a finite
    // position are left out, since a retired slot can contain anything.
    void Build(const float* positions, int count, float cellSize);

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
    float m_invCell = 1.0f;
};

}
