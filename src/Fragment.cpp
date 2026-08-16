// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "Fragment.h"
#include "f4kit/Log.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace flexrevive::fragment {

using namespace f4kit;

namespace {

// The chunk description the engine reads directly, so its layout is fixed: 128 bytes, with
// every field at the offset asserted below. A rigid fragment uses the particle and cluster
// fields; the spring, triangle and inflatable fields stay null and zero.
struct FragmentModel {
    float* particles;           // 0x00  one float4 per particle: x, y, z, inverse mass
    int numParticles;           // 0x08
    int maxParticles;           // 0x0c  zero for a rigid
    int* springIndices;         // 0x10
    float* springCoefficients;  // 0x18
    float* springRestLengths;   // 0x20
    int numSprings;             // 0x28
    int pad0;
    int* clusterIndices;        // 0x30  which particles belong to each cluster
    int numClusterIndices;      // 0x38
    int pad1;
    int* clusterOffsets;        // 0x40  one end offset per cluster
    float* clusterStiffness;    // 0x48
    float* clusterCentres;      // 0x50  float4 each
    int numClusters;            // 0x58  one for a rigid
    int pad2;
    int* triangleIndices;       // 0x60
    int numTriangles;           // 0x68
    int inflatable;             // 0x6c
    float inflatableVolume;     // 0x70
    float inflatablePressure;   // 0x74
    float inflatableStiffness;  // 0x78
    int pad3;
};

static_assert(sizeof(FragmentModel) == 0x80, "FragmentModel must be 128 bytes");
static_assert(offsetof(FragmentModel, particles) == 0x00, "");
static_assert(offsetof(FragmentModel, numParticles) == 0x08, "");
static_assert(offsetof(FragmentModel, maxParticles) == 0x0c, "");
static_assert(offsetof(FragmentModel, springIndices) == 0x10, "");
static_assert(offsetof(FragmentModel, springCoefficients) == 0x18, "");
static_assert(offsetof(FragmentModel, springRestLengths) == 0x20, "");
static_assert(offsetof(FragmentModel, numSprings) == 0x28, "");
static_assert(offsetof(FragmentModel, clusterIndices) == 0x30, "");
static_assert(offsetof(FragmentModel, numClusterIndices) == 0x38, "");
static_assert(offsetof(FragmentModel, clusterOffsets) == 0x40, "");
static_assert(offsetof(FragmentModel, clusterStiffness) == 0x48, "");
static_assert(offsetof(FragmentModel, clusterCentres) == 0x50, "");
static_assert(offsetof(FragmentModel, numClusters) == 0x58, "");
static_assert(offsetof(FragmentModel, triangleIndices) == 0x60, "");
static_assert(offsetof(FragmentModel, numTriangles) == 0x68, "");

// Voxel grid sizing. Sample spacing sits a fraction under the particle radius, so an edge
// that is an exact multiple of it does not land a row of samples on the boundary. The grid
// carries two cells of padding per side so surface features are sampled rather than clipped.
constexpr float kSpacingEpsScale = 0.999899983f; // 1 - 1e-4
constexpr int kMaxDim = 64;
constexpr int kMaxParticles = 4096;

// Everything measured while building a model, held beside it: FragmentModel's layout is
// fixed by the engine and has nowhere to put this.
struct Measured {
    float centre[3] = {0, 0, 0};
    float radius = 0.0f;
    float gyration = 0.0f;
    int particles = 0;
    Hull hull;
};

// Inverts a symmetric 3x3. Returns false when singular, which is the case for a single
// particle or a collinear cloud.
bool InvertSymmetric3x3(const float* m, float* out)
{
    const float a = m[0], b = m[1], c = m[2];
    const float d = m[4], e = m[5], f = m[8];
    const float c0 = d * f - e * e;
    const float c1 = c * e - b * f;
    const float c2 = b * e - c * d;
    const float det = a * c0 + b * c1 + c * c2;
    if (!(std::fabs(det) > 1e-12f) || !std::isfinite(det))
        return false;
    const float inv = 1.0f / det;
    out[0] = c0 * inv;
    out[1] = out[3] = c1 * inv;
    out[2] = out[6] = c2 * inv;
    out[4] = (a * f - c * c) * inv;
    out[5] = out[7] = (b * c - a * e) * inv;
    out[8] = (a * d - b * b) * inv;
    for (int i = 0; i < 9; ++i)
        if (!std::isfinite(out[i]))
            return false;
    return true;
}

// Reduces the particle cloud to the points that bound it: the furthest particle along each of
// the 26 directions of a 3x3x3 neighbourhood, which keeps corners and face centres. The point
// count is capped regardless of how finely the chunk was voxelized.
void BuildHull(const float* particles, int count, const float* centre, Hull& hull)
{
    int chosen[kMaxHullPoints];
    int numChosen = 0;

    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0)
                    continue;
                const float dir[3] = {float(dx), float(dy), float(dz)};

                int best = -1;
                float bestDot = -1e30f;
                for (int i = 0; i < count; ++i) {
                    const float* q = &particles[size_t(i) * 4];
                    const float r[3] = {q[0] - centre[0], q[1] - centre[1], q[2] - centre[2]};
                    const float dp = r[0] * dir[0] + r[1] * dir[1] + r[2] * dir[2];
                    if (dp > bestDot) {
                        bestDot = dp;
                        best = i;
                    }
                }
                if (best < 0)
                    continue;

                bool seen = false;
                for (int k = 0; k < numChosen && !seen; ++k)
                    seen = (chosen[k] == best);
                if (!seen && numChosen < kMaxHullPoints)
                    chosen[numChosen++] = best;
            }
        }
    }

    hull.numPoints = numChosen;
    hull.radius = 0.0f;
    for (int k = 0; k < numChosen; ++k) {
        const float* q = &particles[size_t(chosen[k]) * 4];
        float r2 = 0.0f;
        for (int a = 0; a < 3; ++a) {
            const float r = q[a] - centre[a];
            hull.points[size_t(k) * 3 + size_t(a)] = r;
            r2 += r * r;
        }
        hull.radius = std::max(hull.radius, std::sqrt(r2));
    }
}

std::mutex s_mutex;
std::unordered_map<const void*, Measured> s_ours;

// Solid voxelization by ray parity. For each column of cells along x, every triangle the
// column's centre line passes through contributes one crossing; sorting the crossings and
// filling between consecutive pairs marks the interior. Exact for a closed mesh.
void FillInterior(const float* verts, int numVerts, const int* indices, int numTris,
                  const float* lower, float spacing, int dim, std::vector<uint8_t>& solid)
{
    std::vector<std::vector<float>> columns(size_t(dim) * size_t(dim));

    for (int t = 0; t < numTris; ++t) {
        const int i0 = indices[size_t(t) * 3 + 0];
        const int i1 = indices[size_t(t) * 3 + 1];
        const int i2 = indices[size_t(t) * 3 + 2];
        if (i0 < 0 || i1 < 0 || i2 < 0 || i0 >= numVerts || i1 >= numVerts || i2 >= numVerts)
            continue;

        const float* a = verts + size_t(i0) * 3;
        const float* b = verts + size_t(i1) * 3;
        const float* c = verts + size_t(i2) * 3;

        // Projected onto yz, the triangle is the footprint of the columns it can cross, and
        // its barycentric coordinates there give the crossing's x.
        const float det = (b[1] - a[1]) * (c[2] - a[2]) - (c[1] - a[1]) * (b[2] - a[2]);
        if (std::fabs(det) < 1e-12f)
            continue; // edge-on to the sweep, bounding no interior along x

        float loY = std::min(a[1], std::min(b[1], c[1]));
        float hiY = std::max(a[1], std::max(b[1], c[1]));
        float loZ = std::min(a[2], std::min(b[2], c[2]));
        float hiZ = std::max(a[2], std::max(b[2], c[2]));

        int y0 = int((loY - lower[1]) / spacing - 0.5f);
        int y1 = int((hiY - lower[1]) / spacing + 0.5f) + 1;
        int z0 = int((loZ - lower[2]) / spacing - 0.5f);
        int z1 = int((hiZ - lower[2]) / spacing + 0.5f) + 1;
        y0 = std::max(y0, 0); z0 = std::max(z0, 0);
        y1 = std::min(y1, dim - 1); z1 = std::min(z1, dim - 1);

        const float invDet = 1.0f / det;
        for (int z = z0; z <= z1; ++z) {
            const float cz = lower[2] + (float(z) + 0.5f) * spacing;
            for (int y = y0; y <= y1; ++y) {
                const float cy = lower[1] + (float(y) + 0.5f) * spacing;

                const float l1 = ((cy - a[1]) * (c[2] - a[2]) -
                                  (c[1] - a[1]) * (cz - a[2])) * invDet;
                const float l2 = ((b[1] - a[1]) * (cz - a[2]) -
                                  (cy - a[1]) * (b[2] - a[2])) * invDet;
                if (l1 < 0.0f || l2 < 0.0f || l1 + l2 > 1.0f)
                    continue;

                const float x = a[0] + l1 * (b[0] - a[0]) + l2 * (c[0] - a[0]);
                if (std::isfinite(x))
                    columns[size_t(z) * size_t(dim) + size_t(y)].push_back(x);
            }
        }
    }

    for (int z = 0; z < dim; ++z) {
        for (int y = 0; y < dim; ++y) {
            auto& hits = columns[size_t(z) * size_t(dim) + size_t(y)];
            if (hits.size() < 2)
                continue;
            std::sort(hits.begin(), hits.end());

            // An odd tally means the mesh is open along this column. The unpaired tail is
            // dropped rather than filled from.
            for (size_t k = 0; k + 1 < hits.size(); k += 2) {
                const float xa = hits[k], xb = hits[k + 1];
                int ix0 = int((xa - lower[0]) / spacing - 0.5f);
                int ix1 = int((xb - lower[0]) / spacing + 0.5f) + 1;
                ix0 = std::max(ix0, 0);
                ix1 = std::min(ix1, dim - 1);
                for (int x = ix0; x <= ix1; ++x) {
                    const float cx = lower[0] + (float(x) + 0.5f) * spacing;
                    if (cx >= xa && cx <= xb)
                        solid[(size_t(z) * size_t(dim) + size_t(y)) * size_t(dim) +
                              size_t(x)] = 1;
                }
            }
        }
    }
}

} // namespace

void* BuildFromMesh(const float* vertices, int numVertices, const int* indices,
                    int numTriangleIndices, float radius, float expand, bool populate)
{
    if (!vertices || numVertices <= 0 || numVertices > 1'000'000 || !(radius > 0.0f) ||
        !std::isfinite(radius))
        return nullptr;

    // Vertices are tightly packed float3.
    float meshLower[3] = {1e30f, 1e30f, 1e30f};
    float meshUpper[3] = {-1e30f, -1e30f, -1e30f};
    for (int i = 0; i < numVertices; ++i) {
        const float* v = vertices + size_t(i) * 3;
        for (int a = 0; a < 3; ++a) {
            if (!std::isfinite(v[a]))
                return nullptr;
            meshLower[a] = std::min(meshLower[a], v[a]);
            meshUpper[a] = std::max(meshUpper[a], v[a]);
        }
    }

    const float spacing = radius;
    const float spacingEps = spacing * kSpacingEpsScale;

    float edges[3];
    int d[3];
    for (int a = 0; a < 3; ++a) {
        edges[a] = meshUpper[a] - meshLower[a];
        // At least one sample per axis, however thin the chunk.
        d[a] = (spacing > edges[a]) ? 1 : int(edges[a] / spacingEps);
        d[a] = std::max(d[a], 1);
    }

    int dim = std::max(d[0], std::max(d[1], d[2])) + 4;
    if (dim > kMaxDim)
        return nullptr;

    // Two cells of margin, plus a half-cell shift centring the sample lattice on the chunk
    // rather than its lower corner, so opposite faces sample alike.
    float lower[3];
    for (int a = 0; a < 3; ++a) {
        lower[a] = meshLower[a] - 2.0f * spacing -
                   0.5f * (spacing - (edges[a] - float(d[a] - 1) * spacing));
    }

    const int numTris = (indices && numTriangleIndices >= 3) ? numTriangleIndices / 3 : 0;
    std::vector<uint8_t> solid(size_t(dim) * size_t(dim) * size_t(dim), 0);
    if (numTris > 0)
        FillInterior(vertices, numVertices, indices, numTris, lower, spacing, dim, solid);

    std::vector<float> particles;
    float centre[3] = {0, 0, 0};
    for (int z = 0; z < dim && int(particles.size() / 4) < kMaxParticles; ++z) {
        for (int y = 0; y < dim && int(particles.size() / 4) < kMaxParticles; ++y) {
            for (int x = 0; x < dim && int(particles.size() / 4) < kMaxParticles; ++x) {
                if (!solid[(size_t(z) * size_t(dim) + size_t(y)) * size_t(dim) + size_t(x)])
                    continue;
                const float p[3] = {lower[0] + (float(x) + 0.5f) * spacing,
                                    lower[1] + (float(y) + 0.5f) * spacing,
                                    lower[2] + (float(z) + 0.5f) * spacing};
                for (int a = 0; a < 3; ++a) {
                    particles.push_back(p[a]);
                    centre[a] += p[a];
                }
                particles.push_back(1.0f); // inverse mass
            }
        }
    }

    // A sliver thinner than the sample spacing encloses no cell centre. It is still a chunk
    // the engine draws, so it becomes one particle at its middle.
    if (particles.empty()) {
        for (int a = 0; a < 3; ++a) {
            centre[a] = (meshLower[a] + meshUpper[a]) * 0.5f;
            particles.push_back(centre[a]);
        }
        particles.push_back(1.0f);
    } else {
        const int n = int(particles.size() / 4);
        for (int a = 0; a < 3; ++a)
            centre[a] /= float(n);
    }
    (void)expand; // offsets samples along a distance field, which is not built here

    const int count = int(particles.size() / 4);

    auto* model = static_cast<FragmentModel*>(calloc(1, sizeof(FragmentModel)));
    if (!model)
        return nullptr;

    // What the engine reads. These particles are drawn from its own budget, and it hands the
    // rest positions back through flexSetRigids.
    if (populate) {
        model->particles = static_cast<float*>(malloc(size_t(count) * 4 * sizeof(float)));
        model->clusterIndices = static_cast<int*>(malloc(size_t(count) * sizeof(int)));
        model->clusterOffsets = static_cast<int*>(malloc(sizeof(int)));
        model->clusterStiffness = static_cast<float*>(malloc(sizeof(float)));
        model->clusterCentres = static_cast<float*>(malloc(4 * sizeof(float)));

        if (!model->particles || !model->clusterIndices || !model->clusterOffsets ||
            !model->clusterStiffness || !model->clusterCentres) {
            Destroy(model);
            return nullptr;
        }

        memcpy(model->particles, particles.data(), size_t(count) * 4 * sizeof(float));
        model->numParticles = count;
        model->maxParticles = 0;

        // One cluster spanning every particle, which is what makes the chunk rigid.
        for (int i = 0; i < count; ++i)
            model->clusterIndices[i] = i;
        model->numClusterIndices = count;
        model->clusterOffsets[0] = count;
        model->clusterStiffness[0] = 1.0f;
        model->clusterCentres[0] = centre[0];
        model->clusterCentres[1] = centre[1];
        model->clusterCentres[2] = centre[2];
        model->clusterCentres[3] = 0.0f;
        model->numClusters = 1;
    }

    // Measured here, where the sample spacing is still in hand.
    Measured m;
    for (int i = 0; i < 3; ++i)
        m.centre[i] = centre[i];
    m.particles = count;
    {
        float lo[3] = {1e30f, 1e30f, 1e30f};
        float hi[3] = {-1e30f, -1e30f, -1e30f};
        float sumDist2 = 0.0f;
        for (int i = 0; i < count; ++i) {
            const float* q = &particles[size_t(i) * 4];
            float d2 = 0.0f;
            for (int a = 0; a < 3; ++a) {
                lo[a] = std::min(lo[a], q[a]);
                hi[a] = std::max(hi[a], q[a]);
                const float off = q[a] - centre[a];
                d2 += off * off;
            }
            sumDist2 += d2;
        }

        // Collision radius is the mean half extent of the cloud plus the half cell the
        // outermost samples stand in for, not the distance to the furthest particle: a sphere
        // through a chunk's corner is wider than the chunk is anywhere else, and holds the
        // piece off the ground by the difference.
        float meanHalf = 0.0f;
        for (int a = 0; a < 3; ++a)
            meanHalf += (hi[a] - lo[a]) * 0.5f;
        m.hull.inflate = spacing * 0.5f;
        m.hull.radius += spacing * 0.5f;
        m.radius = meanHalf / 3.0f + spacing * 0.5f;

        // Gyration measures how far the mass is spread rather than how wide the silhouette
        // is. A solid sphere has mean square radius 0.6 r^2, so dividing that out gives the
        // radius of the sphere with the same moment.
        m.gyration = std::sqrt(sumDist2 / float(count) / 0.6f);
        if (!(m.gyration > 0.0f))
            m.gyration = m.radius;

        BuildHull(particles.data(), count, centre, m.hull);
        // The hull carries everything a piece needs, so it can be handed over on its own.
        m.hull.collisionRadius = m.radius;
        m.hull.gyration = m.gyration;

        // Inertia about the centre of mass, per unit mass: sum of |r|^2 * identity minus the
        // outer product of r with itself.
        float tensor[9] = {};
        for (int i = 0; i < count; ++i) {
            const float* q = &particles[size_t(i) * 4];
            const float r[3] = {q[0] - centre[0], q[1] - centre[1], q[2] - centre[2]};
            const float r2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
            for (int a = 0; a < 3; ++a) {
                for (int b = 0; b < 3; ++b)
                    tensor[a * 3 + b] -= r[a] * r[b];
                tensor[a * 3 + a] += r2;
            }
        }
        for (int i = 0; i < 9; ++i)
            tensor[i] /= float(count);

        if (!InvertSymmetric3x3(tensor, m.hull.invInertia)) {
            // No invertible tensor: fall back to a uniform sphere of the same gyration.
            const float sphere = 0.4f * std::max(m.gyration, 1e-3f) * std::max(m.gyration, 1e-3f);
            for (int i = 0; i < 9; ++i)
                m.hull.invInertia[i] = 0.0f;
            m.hull.invInertia[0] = m.hull.invInertia[4] = m.hull.invInertia[8] = 1.0f / sphere;
        }
    }

    {
        std::lock_guard<std::mutex> lock(s_mutex);
        s_ours[model] = m;
    }

    static int logged = 0;
    if (logged < 5) {
        ++logged;
        log::Write("built fragment: %d verts / %d tris -> %d particles on a %dx%dx%d grid "
                   "at spacing %.2f, radius %.2f, gyration %.2f, centre (%.1f, %.1f, %.1f), "
                   "%s to the engine",
                   numVertices, numTris, count, dim, dim, dim, spacing, m.radius, m.gyration,
                   centre[0], centre[1], centre[2], populate ? "handed over" : "withheld");
    }
    return model;
}

void Destroy(void* model)
{
    if (!model)
        return;

    {
        std::lock_guard<std::mutex> lock(s_mutex);
        if (s_ours.erase(model) == 0)
            return; // not ours
    }

    auto* m = static_cast<FragmentModel*>(model);
    free(m->particles);
    free(m->springIndices);
    free(m->springCoefficients);
    free(m->springRestLengths);
    free(m->clusterIndices);
    free(m->clusterOffsets);
    free(m->clusterStiffness);
    free(m->clusterCentres);
    free(m->triangleIndices);
    free(m);
}

bool BuildFromPoints(const float* points, int count, int stride, float particleRadius,
                     Hull& out)
{
    if (!points || count <= 0 || (stride != 3 && stride != 4))
        return false;

    // The cloud arrives in whatever frame the engine keeps it in, so the centre of mass comes
    // from the points rather than being assumed to be the origin.
    float centre[3] = {0, 0, 0};
    for (int i = 0; i < count; ++i) {
        const float* q = points + size_t(i) * size_t(stride);
        for (int a = 0; a < 3; ++a) {
            if (!std::isfinite(q[a]))
                return false;
            centre[a] += q[a];
        }
    }
    for (int a = 0; a < 3; ++a)
        centre[a] /= float(count);

    // Repacked to float4, the stride the hull builder walks. The fourth component is unused.
    std::vector<float> packed(size_t(count) * 4, 0.0f);
    for (int i = 0; i < count; ++i)
        for (int a = 0; a < 3; ++a)
            packed[size_t(i) * 4 + size_t(a)] = points[size_t(i) * size_t(stride) + size_t(a)];

    out = Hull();
    BuildHull(packed.data(), count, centre, out);
    if (out.numPoints <= 0)
        return false;

    float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
    float sumDist2 = 0.0f;
    for (int i = 0; i < count; ++i) {
        const float* q = &packed[size_t(i) * 4];
        float d2 = 0.0f;
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], q[a]);
            hi[a] = std::max(hi[a], q[a]);
            const float off = q[a] - centre[a];
            d2 += off * off;
        }
        sumDist2 += d2;
    }
    float meanHalf = 0.0f;
    for (int a = 0; a < 3; ++a)
        meanHalf += (hi[a] - lo[a]) * 0.5f;
    // Each point is a particle of its own radius, so the chunk reaches that much further than
    // the cloud. A single-particle chunk becomes a sphere of exactly that radius.
    const float inflate = std::max(particleRadius, 1e-3f);
    out.inflate = inflate;
    out.radius += inflate;
    out.collisionRadius = meanHalf / 3.0f + inflate;
    out.gyration = std::max(std::sqrt(sumDist2 / float(count) / 0.6f), inflate);

    float tensor[9] = {};
    for (int i = 0; i < count; ++i) {
        const float* q = &packed[size_t(i) * 4];
        const float r[3] = {q[0] - centre[0], q[1] - centre[1], q[2] - centre[2]};
        const float r2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
        for (int a = 0; a < 3; ++a) {
            for (int b = 0; b < 3; ++b)
                tensor[a * 3 + b] -= r[a] * r[b];
            tensor[a * 3 + a] += r2;
        }
    }
    for (int i = 0; i < 9; ++i)
        tensor[i] /= float(count);

    if (!InvertSymmetric3x3(tensor, out.invInertia)) {
        const float sphere = 0.4f * std::max(out.gyration, 1e-3f) * std::max(out.gyration, 1e-3f);
        for (int i = 0; i < 9; ++i)
            out.invInertia[i] = 0.0f;
        out.invInertia[0] = out.invInertia[4] = out.invInertia[8] = 1.0f / sphere;
    }
    return true;
}

bool GetHull(const void* model, Hull& out)
{
    std::lock_guard<std::mutex> lock(s_mutex);
    auto it = s_ours.find(model);
    if (it == s_ours.end() || it->second.hull.numPoints <= 0)
        return false;
    out = it->second.hull;
    return true;
}

bool IsOurs(const void* model)
{
    if (!model)
        return false;
    std::lock_guard<std::mutex> lock(s_mutex);
    return s_ours.find(model) != s_ours.end();
}

bool Describe(const void* model, float* outCentre, float& outRadius, float& outGyration,
              int& outParticles)
{
    std::lock_guard<std::mutex> lock(s_mutex);
    auto it = s_ours.find(model);
    if (it == s_ours.end() || it->second.particles <= 0)
        return false;

    const Measured& m = it->second;
    for (int a = 0; a < 3; ++a)
        outCentre[a] = m.centre[a];
    outRadius = m.radius;
    outGyration = m.gyration;
    outParticles = m.particles;
    return true;
}

}
