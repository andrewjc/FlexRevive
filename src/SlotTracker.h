// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <vector>

// Working out what the engine did to each debris slot between one call and the next.
//
// The engine hands back the transforms it was given, bit for bit, and recycles a fixed pool of
// slots. A slot arriving with a position other than the one last reported for it has therefore
// either been re-seeded with a new fragment, or been handed a piece that was living in a
// different slot when the pool was compacted.
//
// The two are told apart by looking for the incoming position among everything previously
// reported: a piece that changed seats carries its old position exactly, a new fragment does
// not. Getting this wrong in either direction is visible on screen. Calling a moved piece new
// relaunches it from wherever it had settled; calling a new fragment moved leaves it inheriting
// another piece's motion.
namespace flexrevive::slots {

enum SlotChange : uint8_t {
    kUnchanged = 0,   // the engine handed back what it was given
    kFresh = 1,       // a new fragment, to be seeded from the engine's transform
    kMigrated = 2,    // a known piece, arriving in a different slot
};

struct Classification {
    std::vector<uint8_t> change;
    std::vector<int> source;   // for kMigrated, the slot it came from; -1 otherwise
};

// Classifies every slot in `incoming` (three floats each).
//
// `lastReported` and `reported` describe what was handed back last time, `previousCount` how
// many slots existed then, and `respawnDistance` how far a position must differ before the slot
// is examined at all. Below that it is a round trip, since the engine nudges a settled piece by
// a unit or so from time to time.
void Classify(const float* incoming, int count, const std::vector<float>& lastReported,
              const std::vector<uint8_t>& reported, int previousCount, float respawnDistance,
              Classification& out);

// How far a settled piece may be handed back from where it lay and still count as the same
// piece, undisturbed. The engine nudges a resting chunk by a unit or so between calls.
constexpr float kSettledJump = 32.0f;

// What to do with a slot the classifier called new.
struct Reseed {
    bool asleep;       // the resting flag the piece should carry afterwards
    bool clearMotion;  // drop its velocity and spin
    bool thrown;       // give it a spawn burst, a spin, and let it shock the pile
};

// Decides how a slot reported as new should be treated, given whether the piece already in it
// was at rest and how far the incoming position is from the one last reported for it.
//
// A piece that was moving is a genuine spawn and is thrown. A piece that was at rest and comes
// back where it lay has not been touched and stays asleep. A piece that was at rest and comes
// back somewhere else is a different fragment in a recycled slot: it keeps none of the old
// motion, and must be awake, or it hangs at the position it was handed.
Reseed OnReseed(bool wasResting, float jump);

}
