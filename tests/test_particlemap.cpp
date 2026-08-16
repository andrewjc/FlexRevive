// The particles each piece owns, and how they follow it.
//
// The engine describes a piece as a run of entries in an index array, and those runs are what
// let the particle buffers be advanced by walking the pieces that moved. The inputs come
// straight from the engine and are not trusted: ranges may be empty, reversed, or point outside
// the container.

#include "ParticleMap.h"
#include "TestHarness.h"

#include <algorithm>

#include <vector>

using namespace flexrevive;
using namespace f4kit;
using namespace flexrevive::particles;

static void TestPieceSlots()
{
    test::Suite("the slots each piece owns");

    const int offsets[] = {0, 2, 5, 6};
    const int indices[] = {10, 11, 40, 41, 42, 7};
    std::vector<int> start, slots;

    BuildPieceSlots(offsets, indices, 3, 64, start, slots);

    // One more entry than pieces, so a piece's run is start[i] to start[i + 1].
    CHECK_EQ(int(start.size()), 4);
    CHECK_EQ(start[1] - start[0], 2);
    CHECK_EQ(start[2] - start[1], 3);
    CHECK_EQ(start[3] - start[2], 1);

    CHECK_EQ(slots[start[0] + 0], 10);
    CHECK_EQ(slots[start[0] + 1], 11);
    CHECK_EQ(slots[start[1] + 2], 42);
    CHECK_EQ(slots[start[2] + 0], 7);

    // Walking the runs visits every slot exactly once, which is what makes iterating pieces a
    // replacement for iterating the container rather than an approximation of it.
    std::vector<int> visited;
    for (int piece = 0; piece < 3; ++piece)
        for (int k = start[size_t(piece)]; k < start[size_t(piece) + 1]; ++k)
            visited.push_back(slots[size_t(k)]);
    std::sort(visited.begin(), visited.end());
    const std::vector<int> expected = {7, 10, 11, 40, 41, 42};
    CHECK_EQ(int(visited.size()), int(expected.size()));
    CHECK(visited == expected);
}

static void TestPieceSlotsMatchTheEngineRuns()
{
    test::Suite("the runs match what the engine described");

    // Checked straight against the arrays the engine passed rather than against a second map
    // built from the same data, so nothing here can agree with a shared mistake.
    const int offsets[] = {0, 3, 4, 7};
    const int indices[] = {2, 9, 4, 0, 15, 6, 1};
    std::vector<int> start, slots;
    BuildPieceSlots(offsets, indices, 3, 32, start, slots);

    for (int piece = 0; piece < 3; ++piece) {
        const int begin = offsets[piece], end = offsets[piece + 1];
        CHECK_EQ(start[size_t(piece) + 1] - start[size_t(piece)], end - begin);
        for (int k = 0; k < end - begin; ++k)
            CHECK_EQ(slots[size_t(start[size_t(piece)] + k)], indices[begin + k]);
    }

    // Every slot the engine named appears exactly once across all the runs.
    CHECK_EQ(int(slots.size()), 7);
}

static void TestPieceSlotsHostileInputs()
{
    test::Suite("piece runs from inputs that cannot be trusted");

    std::vector<int> start, slots;

    // Slots outside the container are dropped, and the runs stay consistent with what is left.
    {
        const int offsets[] = {0, 3};
        const int indices[] = {1, 999, -2};
        BuildPieceSlots(offsets, indices, 1, 8, start, slots);
        CHECK_EQ(int(start.size()), 2);
        CHECK_EQ(start[1] - start[0], 1);
        CHECK_EQ(slots[start[0]], 1);
    }

    // A piece with no particles has an empty run rather than a missing one.
    {
        const int offsets[] = {0, 0, 1};
        const int indices[] = {3};
        BuildPieceSlots(offsets, indices, 2, 8, start, slots);
        CHECK_EQ(int(start.size()), 3);
        CHECK_EQ(start[1] - start[0], 0);
        CHECK_EQ(start[2] - start[1], 1);
    }

    // A reversed run is ignored, and null inputs give empty runs for every piece.
    {
        const int offsets[] = {5, 2};
        const int indices[] = {0, 1, 2, 3, 4, 5};
        BuildPieceSlots(offsets, indices, 1, 8, start, slots);
        CHECK_EQ(start[1] - start[0], 0);

        BuildPieceSlots(nullptr, nullptr, 3, 8, start, slots);
        CHECK_EQ(int(start.size()), 4);
        CHECK_EQ(int(slots.size()), 0);
    }
}

static void TestCarryParticle()
{
    test::Suite("a particle moving with its piece");

    // The particle follows the piece exactly and takes the piece's velocity, so the two can
    // never drift apart however long the piece is simulated.
    {
        float pos[4] = {10.0f, 20.0f, 30.0f, 0.5f};
        float vel[3] = {-900.0f, 0.0f, -4000.0f};
        const float delta[3] = {1.0f, -2.0f, 3.0f};
        const float pieceVel[3] = {5.0f, 6.0f, 7.0f};
        CarryParticle(delta, pieceVel, pos, vel);
        CHECK_NEAR(pos[0], 11.0, 1e-6);
        CHECK_NEAR(pos[1], 18.0, 1e-6);
        CHECK_NEAR(pos[2], 33.0, 1e-6);
        CHECK_VEC3(vel, 5.0f, 6.0f, 7.0f, 1e-6);
    }

    // The fourth component is the particle's inverse mass, not a coordinate. Writing it would
    // change the piece's weight, and a zero there is what marks a particle as pinned.
    {
        float pos[4] = {0.0f, 0.0f, 0.0f, 0.25f};
        float vel[3] = {0.0f, 0.0f, 0.0f};
        const float delta[3] = {100.0f, 100.0f, 100.0f};
        const float pieceVel[3] = {1.0f, 1.0f, 1.0f};
        CarryParticle(delta, pieceVel, pos, vel);
        CHECK_NEAR(pos[3], 0.25, 1e-9);
    }

    // A piece that has not moved leaves its particles where they are.
    {
        float pos[4] = {7.0f, 8.0f, 9.0f, 1.0f};
        float vel[3] = {0.0f, 0.0f, 0.0f};
        const float none[3] = {0.0f, 0.0f, 0.0f};
        CarryParticle(none, none, pos, vel);
        CHECK_NEAR(pos[0], 7.0, 1e-9);
        CHECK_NEAR(pos[1], 8.0, 1e-9);
        CHECK_NEAR(pos[2], 9.0, 1e-9);
    }

    // Repeated over many steps the particle stays exactly on its piece, which is the property
    // that stops the two describing different places to the engine.
    {
        float pos[4] = {0.0f, 0.0f, 0.0f, 1.0f};
        float vel[3] = {0.0f, 0.0f, 0.0f};
        float piece[3] = {0.0f, 0.0f, 0.0f};
        for (int step = 0; step < 500; ++step) {
            const float delta[3] = {0.3f, -0.1f, -1.7f};
            const float pieceVel[3] = {18.0f, -6.0f, -102.0f};
            for (int a = 0; a < 3; ++a)
                piece[a] += delta[a];
            CarryParticle(delta, pieceVel, pos, vel);
        }
        CHECK_NEAR(pos[0], piece[0], 1e-2);
        CHECK_NEAR(pos[1], piece[1], 1e-2);
        CHECK_NEAR(pos[2], piece[2], 1e-2);
    }
}

int main()
{
    printf("ParticleMap\n");
    TestPieceSlots();
    TestPieceSlotsMatchTheEngineRuns();
    TestPieceSlotsHostileInputs();
    TestCarryParticle();
    return test::Report("ParticleMap");
}
