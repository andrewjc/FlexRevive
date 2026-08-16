#pragma once

// Two chunks resolving against each other.
//
// A pair that overlaps is separated along the axis between them, sheds the motion it has
// relative to its neighbour, and exchanges whatever closing velocity is left. Separation is
// split by mass and the impulse is shared by mass, so neither step moves the pair's centre of
// mass or changes its momentum: both redistribute rather than add.
namespace flexrevive::pairs {

// Which of the pair is held up by the other, if either.
constexpr int kSupportedNeither = 0;
constexpr int kSupportedA = 1;
constexpr int kSupportedB = 2;

// One chunk as this solver sees it. `ang` may be null for a piece with no angular storage.
struct Body {
    float* pos;
    float* vel;
    float* ang;
    float mass;
    float extent;    // how far this chunk reaches along the separation axis
    bool resting;
};

// Rates per second at which contact between two chunks sheds spin and relative motion. A
// settled stack exchanges almost no impulse, so these are rates rather than anything
// proportional to one.
constexpr float kDefaultSpinDamp = 36.0f;
constexpr float kDefaultLinearDamp = 14.0f;

// How much of an overlap one pass takes out. Contacts accumulate within a pass, so this is
// what keeps a dense cluster from over-correcting.
constexpr float kDefaultRelaxation = 0.5f;

struct Settings {
    float contactSkin = 0.5f;   // margin held between surfaces
    float restitution = 0.0f;   // already scaled for the feel wanted
    float mu = 0.0f;            // friction between chunks, scaled by the settle rate
    float spinDamp = kDefaultSpinDamp;
    float linearDamp = kDefaultLinearDamp;
    float sleepSpeed = 0.0f;    // below this a piece counts as stationary
    float dt = 0.0f;

    // The share of an overlap taken out in one pass.
    //
    // A piece in a heap is touched once per overlapping neighbour, and every correction lands
    // in the same pass. Resolving each contact in full would move it as many times further as
    // it has neighbours, which packs a cluster with stored displacement and fires it apart
    // when the pile stops being fed. Taking a share converges over the substeps instead.
    float relaxation = kDefaultRelaxation;
};

struct Result {
    float overlap = 0.0f;
    bool wakeA = false;
    bool wakeB = false;
    int supported = kSupportedNeither;
};

// Resolves `a` against `b`, where `sep` is the unit axis running from b toward a and `dist` is
// the distance between their centres. `gravity` decides which of them is holding up the other.
//
// Returns false, touching nothing, when the two do not overlap.
//
// Separation always runs when they do. Waking is a separate decision: a contact only wakes a
// sleeping neighbour when one of the pair is genuinely moving, or when the overlap is severe
// enough that the piece has been displaced rather than merely settled against its neighbour.
// A heap comes to rest holding a little residual overlap everywhere, so waking on overlap
// alone would spread from neighbour to neighbour with nothing having pushed the pile.
bool Resolve(Body& a, Body& b, const float* sep, float dist, const float* gravity,
             const Settings& s, Result& out);

}
