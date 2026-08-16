#pragma once

// How a piece's mass modulates what a contact does to it, and how a contact against a surface
// that is itself moving is resolved.
//
// Each law here answers the same question in a different place: a contact delivers roughly the
// same impulse whatever it lands on, and an impulse changes a heavy chunk's velocity less than
// a light one's. Force follows the contact's cross section while inertia follows the volume, so
// the response falls off as the cube root of mass. They live together so they stay consistent
// with each other.
namespace flexrevive::response {

// The fraction of a moving collider's motion a chunk of this mass responds to, in (0, 1].
// `heft` scales the whole effect.
float CarryShare(float mass, float heft);

// The bare law the others are built from: how much a chunk of this mass responds to a given
// impulse, relative to a one-unit chunk. Falls off as the cube root of mass.
float MassResponse(float mass);

// Per-step velocity retention under air resistance, in [0, 1]. Drag follows surface area while
// mass follows volume, so a larger chunk keeps more of its speed.
float DragFactor(float mass, float dragBase, float dt, float heft);

// How two touching chunks divide the job of separating: the lighter one yields more. The two
// shares always sum to one.
void SeparationSplit(float massI, float massJ, float& shareI, float& shareJ);

// Resolves `vel` against a surface with outward `normal` that is itself moving at
// `surfaceVel`, of which the piece responds to `share`.
//
// The contact is resolved in the surface's own frame, which is what carries a piece along
// rather than pushing it clear and letting it drop back. Entering and leaving that frame use
// one velocity, scaled once: the transform is only meaningful if it is symmetric.
void ResolveAgainstMovingSurface(float* vel, const float* normal, const float* surfaceVel,
                                 float share, float restitution);

}
