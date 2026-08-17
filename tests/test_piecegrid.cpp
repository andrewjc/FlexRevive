// The spatial hash that piece-versus-piece walks instead of testing every pair.
//
// The property that matters is completeness: every pair close enough to touch must be offered
// to the caller. Offering extra pairs only costs a distance test, but missing one is debris
// passing through debris, and it would show up as an occasional dropped collision rather than
// as anything reproducible.

#include "PieceGrid.h"
#include "TestHarness.h"

#include <algorithm>
#include <array>
#include <iterator>
#include <map>
#include <cmath>
#include <vector>

using namespace flexrevive;
using namespace f4kit;
using namespace flexrevive::grid;

namespace {

float Distance(const std::vector<float>& p, int a, int b)
{
    float d2 = 0.0f;
    for (int i = 0; i < 3; ++i) {
        const float d = p[size_t(a) * 3 + size_t(i)] - p[size_t(b) * 3 + size_t(i)];
        d2 += d * d;
    }
    return std::sqrt(d2);
}

// A repeatable scatter of positions across a volume, without any random source.
std::vector<float> Scatter(int n, float span)
{
    std::vector<float> p(size_t(n) * 3);
    uint32_t h = 12345u;
    auto next = [&] {
        h ^= h << 13; h ^= h >> 17; h ^= h << 5;
        return float(h % 100000u) / 100000.0f;
    };
    for (int i = 0; i < n * 3; ++i)
        p[size_t(i)] = (next() - 0.5f) * span;
    return p;
}

// Every pair the grid offers, as a sorted list.
std::vector<std::pair<int, int>> VisitedPairs(const PieceGrid& g, const std::vector<float>& p,
                                              int n)
{
    std::vector<std::pair<int, int>> seen;
    for (int i = 0; i < n; ++i)
        g.ForEachNeighbour(i, &p[size_t(i) * 3], [&](int j) { seen.emplace_back(i, j); });
    std::sort(seen.begin(), seen.end());
    return seen;
}

} // namespace

static void TestEmptyAndTrivial()
{
    test::Suite("empty and trivial grids");

    PieceGrid g;
    std::vector<float> none;
    g.Build(none.data(), 0, 10.0f);

    const float p[3] = {0, 0, 0};
    int visits = 0;
    g.ForEachNeighbour(0, p, [&](int) { ++visits; });
    CHECK_EQ(visits, 0);

    // A single piece has no neighbour, and never itself.
    std::vector<float> one = {5.0f, 5.0f, 5.0f};
    g.Build(one.data(), 1, 10.0f);
    visits = 0;
    g.ForEachNeighbour(0, one.data(), [&](int) { ++visits; });
    CHECK_EQ(visits, 0);
}

static void TestNeighboursAreFound()
{
    test::Suite("close pieces are offered");

    // Two pieces well within one cell.
    std::vector<float> p = {0, 0, 0, 3.0f, 0, 0};
    PieceGrid g;
    g.Build(p.data(), 2, 20.0f);

    std::vector<int> seen;
    g.ForEachNeighbour(0, &p[0], [&](int j) { seen.push_back(j); });
    CHECK_EQ(int(seen.size()), 1);
    CHECK_EQ(seen[0], 1);

    // Each pair is offered once, from the lower index only, so the caller never resolves the
    // same contact twice in a step.
    seen.clear();
    g.ForEachNeighbour(1, &p[3], [&](int j) { seen.push_back(j); });
    CHECK_EQ(int(seen.size()), 0);
}

static void TestNoPairIsMissed()
{
    test::Suite("no pair within reach is missed");

    // A cloud dense enough that many pairs are in reach and many are not.
    const int n = 600;
    const float span = 400.0f;
    const float cell = 40.0f;
    const std::vector<float> p = Scatter(n, span);

    PieceGrid g;
    g.Build(p.data(), n, cell);
    const auto visited = VisitedPairs(g, p, n);

    // Cell size follows the largest piece, so anything within one cell must be offered.
    int expected = 0, missed = 0;
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j) {
            if (Distance(p, i, j) > cell)
                continue;
            ++expected;
            if (!std::binary_search(visited.begin(), visited.end(), std::make_pair(i, j)))
                ++missed;
        }

    CHECK(expected > 50);   // the fixture is actually exercising the case
    CHECK_EQ(missed, 0);

    // And the walk is genuinely cheaper than testing everything.
    CHECK(int(visited.size()) < n * (n - 1) / 4);
}

static void TestNoDuplicates()
{
    test::Suite("each pair offered once");

    const int n = 300;
    const std::vector<float> p = Scatter(n, 150.0f);
    PieceGrid g;
    g.Build(p.data(), n, 30.0f);
    auto visited = VisitedPairs(g, p, n);

    const size_t before = visited.size();
    visited.erase(std::unique(visited.begin(), visited.end()), visited.end());
    CHECK_EQ(int(visited.size()), int(before));

    // Bucket collisions are allowed to offer distant pairs, but never a piece against itself.
    for (const auto& pair : visited)
        CHECK(pair.first < pair.second);
}

static void TestPositionsThatAreNotNumbers()
{
    test::Suite("positions that are not numbers");

    // A retired slot can hold anything. It must not be indexed, and must not throw off the
    // pieces around it.
    std::vector<float> p = {0, 0, 0,
                            std::nanf(""), 0, 0,
                            4.0f, 0, 0,
                            1e30f, 1e30f, 1e30f};
    PieceGrid g;
    g.Build(p.data(), 4, 20.0f);

    std::vector<int> seen;
    g.ForEachNeighbour(0, &p[0], [&](int j) { seen.push_back(j); });
    CHECK(std::find(seen.begin(), seen.end(), 2) != seen.end());   // the real neighbour
    CHECK(std::find(seen.begin(), seen.end(), 1) == seen.end());   // never the broken slot
}

static void TestNegativeAndLargeCoordinates()
{
    test::Suite("coordinates far from the origin");

    // Fallout 4's world coordinates run to tens of thousands and are frequently negative, so
    // the hash has to behave the same there as it does near zero.
    std::vector<float> p = {-78420.0f, 84530.0f, 7519.0f,
                            -78412.0f, 84528.0f, 7521.0f,
                             52000.0f, -9000.0f, 400.0f};
    PieceGrid g;
    g.Build(p.data(), 3, 30.0f);

    std::vector<int> seen;
    g.ForEachNeighbour(0, &p[0], [&](int j) { seen.push_back(j); });
    CHECK(std::find(seen.begin(), seen.end(), 1) != seen.end());
}


// Every candidate near a piece, in both directions.
static std::vector<std::pair<int,int>> AllCandidates(const PieceGrid& g,
                                                     const std::vector<float>& p, int n)
{
    std::vector<std::pair<int,int>> seen;
    for (int i = 0; i < n; ++i)
        g.ForEachCandidate(i, &p[size_t(i) * 3], [&](int j) { seen.emplace_back(i, j); });
    std::sort(seen.begin(), seen.end());
    return seen;
}

static void TestCandidatesBothWays()
{
    test::Suite("candidates in both directions");

    // Walking only the pieces that are moving means a pair has to be reachable from either
    // side: a moving piece must find a settled neighbour whatever their indices are.
    std::vector<float> p = {0, 0, 0, 3.0f, 0, 0};
    PieceGrid g;
    g.Build(p.data(), 2, 20.0f);

    std::vector<int> from0, from1;
    g.ForEachCandidate(0, &p[0], [&](int j) { from0.push_back(j); });
    g.ForEachCandidate(1, &p[3], [&](int j) { from1.push_back(j); });

    CHECK_EQ(int(from0.size()), 1);
    CHECK_EQ(from0[0], 1);
    CHECK_EQ(int(from1.size()), 1);
    CHECK_EQ(from1[0], 0);
}

static void TestCandidateNeverSelf()
{
    test::Suite("a piece is never its own candidate");

    const int n = 200;
    const std::vector<float> p = Scatter(n, 100.0f);
    PieceGrid g;
    g.Build(p.data(), n, 25.0f);

    for (const auto& pair : AllCandidates(g, p, n))
        CHECK(pair.first != pair.second);
}

static void TestCandidatesAreSymmetric()
{
    test::Suite("candidates are symmetric where it matters");

    // Walking only the moving pieces means a pair is reached from whichever side is moving, so
    // any pair close enough to touch has to be offered from both. Bucket collisions may also
    // offer far-apart pairs, and those need not be symmetric: they are rejected by distance
    // before anything is done with them.
    const int n = 400;
    const float cell = 40.0f;
    const std::vector<float> p = Scatter(n, 300.0f);
    PieceGrid g;
    g.Build(p.data(), n, cell);

    const auto all = AllCandidates(g, p, n);

    int inReach = 0, oneWay = 0, missed = 0;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            if (i == j || Distance(p, i, j) > cell)
                continue;
            ++inReach;
            const bool forward = std::binary_search(all.begin(), all.end(),
                                                    std::make_pair(i, j));
            const bool back = std::binary_search(all.begin(), all.end(),
                                                 std::make_pair(j, i));
            if (!forward)
                ++missed;
            if (forward != back)
                ++oneWay;
        }

    CHECK(inReach > 100);   // the fixture exercises the case
    CHECK_EQ(missed, 0);
    CHECK_EQ(oneWay, 0);
}

// The pieces one moving piece reads and writes when it resolves its neighbourhood: itself,
// plus every candidate the grid offers that survives the cell filter.
static std::vector<int> Touches(const PieceGrid& g, const std::vector<float>& p, int i)
{
    std::vector<int> out{i};
    int cellI[3];
    const bool have = g.CellOfPiece(i, cellI);
    g.ForEachCandidate(i, &p[size_t(i) * 3], [&](int j) {
        int cellJ[3];
        if (have && g.CellOfPiece(j, cellJ) && !PieceGrid::CellsAdjacent(cellI, cellJ))
            return;
        out.push_back(j);
    });
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

static void TestColoursCoverEveryCell()
{
    test::Suite("every cell has exactly one colour, and every colour is used");

    bool used[kColours] = {};
    for (int x = -20; x <= 20; ++x)
        for (int y = -20; y <= 20; ++y)
            for (int z = -20; z <= 20; ++z) {
                const int cell[3] = {x, y, z};
                const int c = PieceGrid::ColourOfCell(cell);
                CHECK(c >= 0 && c < kColours);
                used[c] = true;
            }

    // Negative coordinates are the case a plain % gets wrong: it would fold cells left of the
    // origin onto negative colours and index out of the bucket array.
    for (int c = 0; c < kColours; ++c)
        CHECK(used[c]);
}

static void TestSameColourCellsCannotShareANeighbour()
{
    test::Suite("same-coloured cells reach into blocks that do not overlap");

    // The whole safety argument for resolving a colour across threads. A piece reaches into
    // its own cell and the twenty-six around it, so two pieces may run at once only if those
    // blocks are disjoint, and blocks are disjoint exactly when the cells differ by three or
    // more on some axis.
    std::vector<std::array<int, 3>> cells;
    for (int x = -6; x <= 6; ++x)
        for (int y = -6; y <= 6; ++y)
            for (int z = -6; z <= 6; ++z)
                cells.push_back({x, y, z});

    int sameColourPairs = 0, overlapping = 0;
    for (size_t a = 0; a < cells.size(); ++a)
        for (size_t b = a + 1; b < cells.size(); ++b) {
            if (PieceGrid::ColourOfCell(cells[a].data()) !=
                PieceGrid::ColourOfCell(cells[b].data()))
                continue;
            ++sameColourPairs;
            int spread = 0;
            for (int k = 0; k < 3; ++k) {
                const int d = cells[a][k] - cells[b][k];
                spread = std::max(spread, d < 0 ? -d : d);
            }
            if (spread < 3)
                ++overlapping;
        }

    CHECK(sameColourPairs > 10000);   // the fixture is exercising the case
    CHECK_EQ(overlapping, 0);
}

static void TestOneColourNeverTouchesTheSamePieceTwice()
{
    test::Suite("no two cells of a colour touch the same piece");

    // The same argument as above, but through the grid rather than the arithmetic, so it
    // covers the bucket walk and the cell filter the solver actually runs.
    //
    // The unit here is a cell, which is the point. Colouring separates cells; it does nothing
    // for two pieces sitting in one cell, whose neighbourhoods are identical. Grouping by
    // piece instead of by cell fails this by tens of thousands of shared pieces, which is what
    // it did before.
    const int n = 900;
    const float span = 300.0f;
    const float cell = 40.0f;
    const std::vector<float> p = Scatter(n, span);

    PieceGrid g;
    g.Build(p.data(), n, cell);

    // Pieces grouped by cell, then cells grouped by colour, exactly as the solver groups them.
    std::map<std::array<int, 3>, std::vector<int>> cells;
    for (int i = 0; i < n; ++i) {
        int c[3];
        CHECK(g.CellOfPiece(i, c));   // every finite position is indexed
        cells[{c[0], c[1], c[2]}].push_back(i);
    }

    std::vector<std::vector<std::vector<int>>> byColour(kColours);
    for (const auto& entry : cells) {
        // Everything one cell's worth of work reads and writes.
        std::vector<int> touched;
        for (int i : entry.second) {
            const std::vector<int> t = Touches(g, p, i);
            touched.insert(touched.end(), t.begin(), t.end());
        }
        std::sort(touched.begin(), touched.end());
        touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
        byColour[size_t(PieceGrid::ColourOfCell(entry.first.data()))].push_back(touched);
    }

    int collisions = 0, compared = 0, populated = 0;
    for (const auto& group : byColour) {
        if (group.size() < 2)
            continue;
        ++populated;
        for (size_t a = 0; a < group.size(); ++a)
            for (size_t b = a + 1; b < group.size(); ++b) {
                ++compared;
                std::vector<int> shared;
                std::set_intersection(group[a].begin(), group[a].end(), group[b].begin(),
                                      group[b].end(), std::back_inserter(shared));
                collisions += int(shared.size());
            }
    }

    CHECK(populated > 20);   // the scatter really does spread across colours
    CHECK(compared > 500);
    CHECK_EQ(collisions, 0);
}

static void TestTheCellFilterDropsNoRealPair()
{
    test::Suite("the cell filter drops nothing within reach");

    // Filtering candidates on the cell is what makes a colour safe, but it must not cost a
    // collision. Cells are sized to the largest piece, so anything close enough to touch is in
    // the same cell or one beside it, and the filter can only remove hash collisions.
    const int n = 700;
    const float span = 350.0f;
    const float cell = 40.0f;
    const std::vector<float> p = Scatter(n, span);

    PieceGrid g;
    g.Build(p.data(), n, cell);

    int inReach = 0, dropped = 0;
    for (int i = 0; i < n; ++i) {
        const std::vector<int> kept = Touches(g, p, i);
        for (int j = 0; j < n; ++j) {
            if (j == i || Distance(p, i, j) > cell)
                continue;
            ++inReach;
            if (!std::binary_search(kept.begin(), kept.end(), j))
                ++dropped;
        }
    }

    CHECK(inReach > 200);
    CHECK_EQ(dropped, 0);
}

int main()
{
    printf("PieceGrid\n");
    TestEmptyAndTrivial();
    TestNeighboursAreFound();
    TestNoPairIsMissed();
    TestNoDuplicates();
    TestPositionsThatAreNotNumbers();
    TestNegativeAndLargeCoordinates();
    TestCandidatesBothWays();
    TestCandidateNeverSelf();
    TestCandidatesAreSymmetric();
    TestColoursCoverEveryCell();
    TestSameColourCellsCannotShareANeighbour();
    TestOneColourNeverTouchesTheSamePieceTwice();
    TestTheCellFilterDropsNoRealPair();
    return test::Report("PieceGrid");
}
