// The GPU backend's marshalling, against the CPU structures it has to agree with.
//
// This is the half of the port that needs no graphics device, and it is the half that a port
// usually gets wrong. The arithmetic in a kernel is a transcription anyone can read side by
// side; the layouts, the index bases and the sentinel values are where a mistake hides, and it
// hides as debris in the wrong place rather than as anything that fails to build.
//
// The spatial hash is the case that matters most. The pair pass walks it on the card to decide
// which pieces are neighbours, and the colouring decides which of them may run at once. If the
// GPU copy disagrees with grid::PieceGrid about either, the result is not a slightly different
// answer, it is two threads writing the same chunk.

#include "gpu/Scene.h"
#include "PieceGrid.h"
#include "TestHarness.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <vector>

using namespace flexrevive;
using namespace f4kit;
using namespace flexrevive::gpu;

namespace {

std::vector<float> Scatter(int n, float span, uint32_t seed = 12345u)
{
    std::vector<float> p(size_t(n) * 3);
    uint32_t h = seed;
    auto next = [&] {
        h ^= h << 13; h ^= h >> 17; h ^= h << 5;
        return float(h % 100000u) / 100000.0f;
    };
    for (int i = 0; i < n * 3; ++i)
        p[size_t(i)] = (next() - 0.5f) * span;
    return p;
}

// Every piece the GPU hash offers as a neighbour of `self`, filtered on the cell exactly as
// pairs.hlsl filters it.
std::vector<int> GpuNeighbours(const std::vector<uint32_t>& head,
                               const std::vector<uint32_t>& next,
                               const std::vector<int32_t>& cells, int self)
{
    std::vector<int> out;
    const int32_t* ci = &cells[size_t(self) * 4];
    if (ci[3] == 0)
        return out;

    for (int ox = -1; ox <= 1; ++ox)
        for (int oy = -1; oy <= 1; ++oy)
            for (int oz = -1; oz <= 1; ++oz) {
                const int c[3] = {ci[0] + ox, ci[1] + oy, ci[2] + oz};
                const uint32_t h = uint32_t(c[0]) * 73856093u ^ uint32_t(c[1]) * 19349663u ^
                                   uint32_t(c[2]) * 83492791u;
                for (uint32_t j = head[h & uint32_t(scene::kBuckets - 1)];
                     j != scene::kNoIndex; j = next[j]) {
                    if (int(j) == self)
                        continue;
                    const int32_t* cj = &cells[size_t(j) * 4];
                    if (cj[3] != 0 && (std::abs(ci[0] - cj[0]) > 1 || std::abs(ci[1] - cj[1]) > 1 ||
                                       std::abs(ci[2] - cj[2]) > 1))
                        continue;
                    out.push_back(int(j));
                }
            }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

float Distance(const std::vector<float>& p, int a, int b)
{
    float d2 = 0.0f;
    for (int i = 0; i < 3; ++i) {
        const float d = p[size_t(a) * 3 + size_t(i)] - p[size_t(b) * 3 + size_t(i)];
        d2 += d * d;
    }
    return std::sqrt(d2);
}

void TestLayoutsMatchTheShaders()
{
    test::Suite("the structs are the size the shaders read");

    // Sizes taken from fxc /Fc rather than worked out by hand. The static_asserts in Scene.h
    // are the real guard; these restate them so a failure names the struct rather than
    // stopping the build with a line number.
    CHECK_EQ(int(sizeof(scene::Piece)), 80);
    CHECK_EQ(int(sizeof(scene::Collider)), 112);
    CHECK_EQ(int(sizeof(scene::MeshDesc)), 112);
    CHECK_EQ(int(sizeof(scene::Blast)), 32);
    CHECK_EQ(int(sizeof(scene::CellRun)), 16);
    CHECK_EQ(int(sizeof(scene::Params)), 96);

    // Every one is a whole number of 16-byte slots, which is what makes the two packing rules
    // agree. A struct that is not says the float4-only discipline has been broken somewhere.
    CHECK_EQ(int(sizeof(scene::Piece)) % 16, 0);
    CHECK_EQ(int(sizeof(scene::Collider)) % 16, 0);
    CHECK_EQ(int(sizeof(scene::MeshDesc)) % 16, 0);
    CHECK_EQ(int(sizeof(scene::Blast)) % 16, 0);
    CHECK_EQ(int(sizeof(scene::CellRun)) % 16, 0);
    CHECK_EQ(int(sizeof(scene::Params)) % 16, 0);
}

void TestColoursAgreeWithTheCpu()
{
    test::Suite("colours agree with PieceGrid");

    // The pair pass depends on both sides agreeing about which cells may run together. A
    // disagreement here is a data race on the card, not a different answer.
    int mismatches = 0;
    for (int x = -20; x <= 20; ++x)
        for (int y = -20; y <= 20; ++y)
            for (int z = -20; z <= 20; ++z) {
                const int32_t cell[3] = {x, y, z};
                const int mine = scene::ColourOfCell(cell);
                const int theirs = grid::PieceGrid::ColourOfCell(reinterpret_cast<const int*>(cell));
                if (mine != theirs)
                    ++mismatches;
                if (mine < 0 || mine >= scene::kColours)
                    ++mismatches;
            }
    CHECK_EQ(mismatches, 0);
}

void TestHashOffersTheSameNeighbours()
{
    test::Suite("the hash offers what PieceGrid offers");

    const int n = 900;
    const float cell = 40.0f;
    const std::vector<float> p = Scatter(n, 300.0f);

    grid::PieceGrid cpu;
    cpu.Build(p.data(), n, cell);

    std::vector<uint32_t> head(size_t(scene::kBuckets), 0u), next(size_t(n), 0u);
    std::vector<int32_t> cells(size_t(n) * 4);
    scene::BuildHash(p.data(), n, cell, head.data(), next.data(), cells.data());

    int cellMismatch = 0, missing = 0, extra = 0, compared = 0;
    for (int i = 0; i < n; ++i) {
        int c[3];
        const bool indexed = cpu.CellOfPiece(i, c);
        if (indexed != (cells[size_t(i) * 4 + 3] != 0)) {
            ++cellMismatch;
            continue;
        }
        if (!indexed)
            continue;
        for (int a = 0; a < 3; ++a)
            if (c[a] != cells[size_t(i) * 4 + size_t(a)])
                ++cellMismatch;

        // What the CPU pass would consider, filtered the same way.
        std::vector<int> want;
        cpu.ForEachCandidate(i, &p[size_t(i) * 3], [&](int j) {
            int cj[3];
            if (cpu.CellOfPiece(j, cj) && !grid::PieceGrid::CellsAdjacent(c, cj))
                return;
            want.push_back(j);
        });
        std::sort(want.begin(), want.end());
        want.erase(std::unique(want.begin(), want.end()), want.end());

        const std::vector<int> got = GpuNeighbours(head, next, cells, i);
        compared += int(want.size());

        std::vector<int> diff;
        std::set_difference(want.begin(), want.end(), got.begin(), got.end(),
                            std::back_inserter(diff));
        missing += int(diff.size());
        diff.clear();
        std::set_difference(got.begin(), got.end(), want.begin(), want.end(),
                            std::back_inserter(diff));
        extra += int(diff.size());
    }

    CHECK(compared > 500);   // the fixture is exercising the case
    CHECK_EQ(cellMismatch, 0);
    CHECK_EQ(missing, 0);
    CHECK_EQ(extra, 0);
}

void TestNoPairWithinReachIsMissed()
{
    test::Suite("no pair within reach is missed");

    // The property that matters on its own terms, independent of the CPU: a pair close enough
    // to touch that is never offered is debris passing through debris.
    const int n = 700;
    const float cell = 40.0f;
    const std::vector<float> p = Scatter(n, 350.0f, 999u);

    std::vector<uint32_t> head(size_t(scene::kBuckets), 0u), next(size_t(n), 0u);
    std::vector<int32_t> cells(size_t(n) * 4);
    scene::BuildHash(p.data(), n, cell, head.data(), next.data(), cells.data());

    int inReach = 0, missed = 0;
    for (int i = 0; i < n; ++i) {
        const std::vector<int> got = GpuNeighbours(head, next, cells, i);
        for (int j = 0; j < n; ++j) {
            if (j == i || Distance(p, i, j) > cell)
                continue;
            ++inReach;
            if (!std::binary_search(got.begin(), got.end(), j))
                ++missed;
        }
    }
    CHECK(inReach > 200);
    CHECK_EQ(missed, 0);
}

void TestSlotsThatAreNotNumbers()
{
    test::Suite("positions that are not numbers");

    // A retired slot can hold anything. It must not be indexed, or it lands in an arbitrary
    // bucket and is offered as a neighbour to whatever else hashes there.
    std::vector<float> p = {0, 0, 0,
                            std::nanf(""), 0, 0,
                            4.0f, 0, 0,
                            std::numeric_limits<float>::infinity(), 0, 0};
    const int n = 4;
    std::vector<uint32_t> head(size_t(scene::kBuckets), 0u), next(size_t(n), 0u);
    std::vector<int32_t> cells(size_t(n) * 4);
    scene::BuildHash(p.data(), n, 20.0f, head.data(), next.data(), cells.data());

    CHECK(cells[0 * 4 + 3] != 0);
    CHECK_EQ(cells[1 * 4 + 3], 0);
    CHECK(cells[2 * 4 + 3] != 0);
    CHECK_EQ(cells[3 * 4 + 3], 0);

    const std::vector<int> got = GpuNeighbours(head, next, cells, 0);
    CHECK(std::find(got.begin(), got.end(), 2) != got.end());   // the real neighbour
    CHECK(std::find(got.begin(), got.end(), 1) == got.end());   // never a broken slot
    CHECK(std::find(got.begin(), got.end(), 3) == got.end());
}

void TestTerminatorIsUnsigned()
{
    test::Suite("the walk terminates");

    // The CPU grid links with signed ints and -1; a shader reading that as unsigned would walk
    // off the end of the world rather than stop. Every chain must end at kNoIndex within the
    // number of pieces that exist.
    const int n = 400;
    const std::vector<float> p = Scatter(n, 60.0f, 31337u);   // dense, so chains are long
    std::vector<uint32_t> head(size_t(scene::kBuckets), 0u), next(size_t(n), 0u);
    std::vector<int32_t> cells(size_t(n) * 4);
    scene::BuildHash(p.data(), n, 20.0f, head.data(), next.data(), cells.data());

    int walked = 0, overran = 0;
    for (int b = 0; b < scene::kBuckets; ++b) {
        int steps = 0;
        for (uint32_t j = head[size_t(b)]; j != scene::kNoIndex; j = next[j]) {
            if (j >= uint32_t(n)) { ++overran; break; }
            if (++steps > n) { ++overran; break; }
            ++walked;
        }
    }
    CHECK_EQ(overran, 0);
    CHECK(walked > 0);
}

} // namespace

int main()
{
    printf("GpuScene\n");
    TestLayoutsMatchTheShaders();
    TestColoursAgreeWithTheCpu();
    TestHashOffersTheSameNeighbours();
    TestNoPairWithinReachIsMissed();
    TestSlotsThatAreNotNumbers();
    TestTerminatorIsUnsigned();
    return test::Report("GpuScene");
}
