#pragma once

#include <vector>

// A shot landing in a pile.
//
// Debris is not in the collision world a projectile tests against, so a burst of fresh chunks
// is the only evidence of an impact available: where it appeared and how fast the engine
// launched it. Each burst becomes a brief outward shove on the settled debris around it.
namespace flexrevive::shock {

// However hard a burst lands, and however many land at once, no single step may throw a chunk
// faster than this.
constexpr float kMaxShockSpeed = 90.0f;

// How far apart two fragments can be and still count as one impact. About 1.7 m at this game's
// scale: wide enough for a single burst, narrow enough not to merge two hits.
constexpr float kBurstRadius = 120.0f;

// A fraction of the shove is added upward, so a heap scatters rather than sliding flat.
constexpr float kLiftFraction = 0.3f;

// The fraction of a burst's launch speed that becomes a shove on the rubble around it.
constexpr float kBurstStrength = 0.22f;

// Bounds the list, so a pathological frame cannot grow it without limit.
constexpr int kMaxImpacts = 64;

struct Impact {
    float pos[3] = {0, 0, 0};
    float radius = 0.0f;
    float deltaV = 0.0f;
};

// Adds a burst, merging it into an existing one within kBurstRadius and keeping the stronger
// of the two. Fragments from one impact arrive as a cluster, so shoves close together are one
// event; two hits a weapon's spread apart are not.
void Record(std::vector<Impact>& impacts, const float* pos, float deltaV,
            float radius = kBurstRadius);

// The velocity change a piece at `pos` of relative `mass` receives from every impact reaching
// it, summed and capped together. Returns false, leaving `outPush` untouched, when none does.
//
// Everything landing on one chunk is combined before the cap because sustained fire into one
// spot lands impacts inside a heap many times a second, and applied one at a time they would
// compound until the rubble was thrown clear.
bool PushFor(const std::vector<Impact>& impacts, const float* pos, float mass, float* outPush);

}
