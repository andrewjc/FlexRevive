// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#pragma once

// Builds the description of a single debris chunk.
//
// The engine hands a fragment's triangle mesh to flexExtCreateRigidFromMesh and keeps the
// FragmentModel it gets back. That model states how many particles the chunk has and where
// they sit, which is what the engine's own particle budget is spent against, and what the
// solver measures a piece's size, mass and inertia from.
//
// FragmentModel's layout is fixed by the engine, which reads it directly. See Fragment.cpp.
namespace flexrevive::fragment {

// Builds a fragment model from a triangle mesh. Returns null if the mesh is unusable.
// `radius` is the solver's particle radius, which sets the voxel spacing.
// `populate` fills in the particles, cluster indices and centre of mass; with it false the
// model describes an empty chunk.
void* BuildFromMesh(const float* vertices, int numVertices, const int* indices,
                    int numTriangleIndices, float radius, float expand, bool populate);

// Releases a model built above. Safe to call with null.
void Destroy(void* model);

// Measures a model for the solver's per-piece sizing. `radius` is the mean half extent of the
// particle cloud; `gyration` is the radius a uniform sphere of the same moment would have,
// which is what the chunk's tumbling responds to.
bool Describe(const void* model, float* outCentre, float& outRadius, float& outGyration,
              int& outParticles);

// The most points a chunk's silhouette is reduced to. Contacts resolve against these, so the
// count bounds the per-contact cost.
constexpr int kMaxHullPoints = 26;

// A chunk's silhouette: extreme points relative to its centre of mass, plus the inverse
// inertia tensor of the whole particle cloud per unit mass.
//
// Resolving a contact against the support point in the contact direction produces a torque,
// which is what rolls a piece down onto a flat side. A chunk modelled as a sphere has no
// support direction to speak of and comes to rest at whatever angle it arrived at.
struct Hull {
    float points[kMaxHullPoints * 3] = {};
    int numPoints = 0;
    float invInertia[9] = {};     // row major, symmetric, per unit mass
    float radius = 0.0f;          // bounding radius of the hull points
    float collisionRadius = 0.0f; // the sphere the piece sweeps as, for broad phase
    float gyration = 0.0f;        // radius a sphere of the same moment would have
    // Each point is a particle of this radius rather than a mathematical point, so the chunk's
    // surface is the hull swept by a sphere of it. A one-particle chunk therefore behaves as
    // the sphere it is instead of collapsing to a point.
    float inflate = 0.0f;
};

bool GetHull(const void* model, Hull& out);

// Builds a silhouette and inertia tensor from a particle cloud directly, for pieces whose
// rest positions the engine supplies with no model to look up. Points are float3 or float4 at
// `stride` floats each, in any frame; the centre of mass is taken from them.
bool BuildFromPoints(const float* points, int count, int stride, float particleRadius,
                     Hull& out);

// True if the pointer is a model this module built.
bool IsOurs(const void* model);

}
