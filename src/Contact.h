// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

#include "Fragment.h"

// Rigid body contact against a chunk's own silhouette.
//
// The contact sits at the hull's support point in the contact direction, which is off to one
// side of the centre of mass, so the normal impulse generates a torque and the piece tips down
// onto a face. A sphere touches directly beneath its centre, where the impulse passes through
// the centre of mass and produces no torque at all, leaving the chunk at whatever attitude it
// arrived in.
//
// Mass is taken as 1, so the inverse inertia tensor is per unit mass and the impulse
// denominators lose their 1/m term.
namespace flexrevive::contact {

// Contact response: a little bounce, most tangential speed scrubbed off, and a small margin
// held off the surface so a resting piece does not re-collide.
constexpr float kContactSkin = 0.5f;   // margin on top of the piece's own radius
constexpr float kRestitution = 0.25f;
constexpr float kFriction = 0.6f;

// Gravity adds about 6.9 units/s per 0.01 s step, which sets the floor for meaningful motion.
// Below the bounce threshold a contact stops rebounding; below the sleep threshold the piece
// is parked.
constexpr float kNoBounceSpeed = 45.0f;       // units/s of normal approach
constexpr float kSleepSpeed = 25.0f;          // units/s total
constexpr float kStaticFrictionSpeed = 60.0f; // below this a resting piece grips the surface
constexpr float kRollBlend = 0.35f;           // how fast contact converts sliding to rolling

// How strongly a surface scrubs spin off a piece resting on it, expressed against the normal
// impulse so a light touch scrubs lightly. A piece that keeps spinning also keeps lifting
// itself, since its silhouette turns with it and sweeps the support point around.
constexpr float kRollingResistance = 0.5f;

// The furthest one contact may move a piece in a single step, as a multiple of the chunk's own
// bounding radius.
//
// A piece that has ended up deep inside geometry reports a penetration of whatever that depth
// is, and lifting it clear in one go would fling it the entire distance. Debris ends up
// scattered across the sky rather than on the ground. Climbing out over several steps looks
// like a piece pushing itself free; teleporting looks like a bug, and is one.
constexpr float kMaxSeparationRadii = 8.0f;

struct HullContact {
    float r[3];      // contact point relative to the centre of mass
    float depth;     // how far the support point is past the surface, positive when touching
    bool valid;
};

// How far a chunk reaches from its centre along one direction, with its silhouette turned by
// its current orientation `q`.
float SupportExtent(const fragment::Hull& shp, const float* q, const float* dir);

// Locates the contact between a piece at `pos` with orientation `q` and a surface point with
// the given outward normal. `valid` is false when the hull cannot produce one.
HullContact FindHullContact(const fragment::Hull& shp, const float* q, const float* pos,
                            const float* surface, const float* normal);

// Applies one contact: separates along the normal, then a normal and a friction impulse at the
// support point, both feeding angular velocity `w` through the inverse inertia tensor.
// `spinFromFriction` scales the spin the tangential impulse imparts. An impulse straight
// through the centre of mass cannot rotate a piece, so it is the sideways part of a contact
// that makes a chunk cartwheel or roll down a slope; zero leaves a piece sliding flat.
void ResolveHullContact(const fragment::Hull& shp, const HullContact& c, float* pos, float* vel,
                        float* w, const float* normal, float restitution, float friction,
                        float skin, float spinFromFriction = 1.0f);

}
