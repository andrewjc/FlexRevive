#include "Collision.h"

#include <algorithm>
#include <cmath>

namespace flexrevive::collision {

int GridCell(const TriGrid& g, int x, int y, int z)
{
    return (z * g.dim[1] + y) * g.dim[0] + x;
}

void BuildTriGrid(TriMesh& m)
{
    TriGrid& g = m.grid;
    g = TriGrid();

    const int triCount = int(m.indices.size() / 3);
    const size_t vertCount = m.verts.size() / 3;
    if (triCount < kGridMinTriangles || vertCount == 0)
        return;

    float extent[3];
    for (int a = 0; a < 3; ++a) {
        g.origin[a] = m.lower[a];
        extent[a] = m.upper[a] - m.lower[a];
        if (!std::isfinite(extent[a]) || extent[a] < 0.0f)
            return;
    }

    // Roughly two triangles per cell, with cells sized from the mesh volume rather than per
    // axis so they stay cube-ish. Flat geometry collapses one axis to a single cell.
    const float volume = std::max(extent[0], 1.0f) * std::max(extent[1], 1.0f) *
                         std::max(extent[2], 1.0f);
    const int targetCells = std::min(std::max(triCount / 2, 1), kGridMaxCells);
    const float cellSize = std::cbrt(volume / float(targetCells));
    if (!(cellSize > 0.0f) || !std::isfinite(cellSize))
        return;

    auto sizeAxes = [&]() {
        int cells = 1;
        for (int a = 0; a < 3; ++a)
            cells *= g.dim[a];
        for (int a = 0; a < 3; ++a)
            g.invCell[a] = float(g.dim[a]) / std::max(extent[a], 1e-3f);
        return cells;
    };

    for (int a = 0; a < 3; ++a)
        g.dim[a] = std::min(std::max(int(extent[a] / cellSize) + 1, 1), 256);
    int cells = sizeAxes();

    // A very anisotropic mesh can still overshoot the cap; halve its longest axis until it
    // fits rather than abandoning the index.
    while (cells > kGridMaxCells) {
        int a = 0;
        if (g.dim[1] > g.dim[a]) a = 1;
        if (g.dim[2] > g.dim[a]) a = 2;
        if (g.dim[a] <= 1)
            return;
        g.dim[a] = std::max(g.dim[a] / 2, 1);
        cells = sizeAxes();
    }
    if (cells <= 1)
        return;

    // The cell range a triangle's own bounding box covers.
    auto triCells = [&](int t, int lo[3], int hi[3]) {
        const int i0 = m.indices[size_t(t) * 3 + 0];
        const int i1 = m.indices[size_t(t) * 3 + 1];
        const int i2 = m.indices[size_t(t) * 3 + 2];
        if (i0 < 0 || i1 < 0 || i2 < 0 || size_t(i0) >= vertCount || size_t(i1) >= vertCount ||
            size_t(i2) >= vertCount)
            return false;
        const float* v0 = &m.verts[size_t(i0) * 3];
        const float* v1 = &m.verts[size_t(i1) * 3];
        const float* v2 = &m.verts[size_t(i2) * 3];
        for (int a = 0; a < 3; ++a) {
            const float mn = std::min(v0[a], std::min(v1[a], v2[a]));
            const float mx = std::max(v0[a], std::max(v1[a], v2[a]));
            if (!std::isfinite(mn) || !std::isfinite(mx))
                return false;
            lo[a] = std::min(std::max(int((mn - g.origin[a]) * g.invCell[a]), 0), g.dim[a] - 1);
            hi[a] = std::min(std::max(int((mx - g.origin[a]) * g.invCell[a]), 0), g.dim[a] - 1);
        }
        return true;
    };

    // Count per cell, prefix-sum into offsets, then fill. Triangles spanning the whole mesh
    // inflate the entry list, so the build gives up past a cap and leaves the linear path.
    const size_t entryCap = size_t(triCount) * 8 + 1024;
    g.start.assign(size_t(cells) + 1, 0);
    size_t entries = 0;

    for (int t = 0; t < triCount; ++t) {
        int lo[3], hi[3];
        if (!triCells(t, lo, hi))
            continue;
        for (int z = lo[2]; z <= hi[2]; ++z)
            for (int y = lo[1]; y <= hi[1]; ++y)
                for (int x = lo[0]; x <= hi[0]; ++x)
                    ++g.start[size_t(GridCell(g, x, y, z)) + 1];
        entries += size_t(hi[0] - lo[0] + 1) * size_t(hi[1] - lo[1] + 1) *
                   size_t(hi[2] - lo[2] + 1);
        if (entries > entryCap) {
            g = TriGrid();
            return;
        }
    }

    for (size_t c = 0; c < size_t(cells); ++c)
        g.start[c + 1] += g.start[c];

    std::vector<int> cursor(g.start.begin(), g.start.end() - 1);
    g.tris.resize(entries);
    for (int t = 0; t < triCount; ++t) {
        int lo[3], hi[3];
        if (!triCells(t, lo, hi))
            continue;
        for (int z = lo[2]; z <= hi[2]; ++z)
            for (int y = lo[1]; y <= hi[1]; ++y)
                for (int x = lo[0]; x <= hi[0]; ++x)
                    g.tris[size_t(cursor[size_t(GridCell(g, x, y, z))]++)] = t;
    }

    g.valid = true;
}

bool SegmentHitsTriangle(const float* from, const float* dir, float maxDist, const float* v0,
                         const float* v1, const float* v2, float& outDist, float* outNormal)
{
    float e1[3], e2[3];
    for (int i = 0; i < 3; ++i) {
        e1[i] = v1[i] - v0[i];
        e2[i] = v2[i] - v0[i];
    }

    const float pv[3] = {dir[1] * e2[2] - dir[2] * e2[1], dir[2] * e2[0] - dir[0] * e2[2],
                         dir[0] * e2[1] - dir[1] * e2[0]};
    const float det = e1[0] * pv[0] + e1[1] * pv[1] + e1[2] * pv[2];
    if (std::fabs(det) < 1e-8f)
        return false;

    // The barycentric tests run unscaled, against det rather than against 1, so the division
    // is paid only by a triangle that is actually hit. Nearly every triangle a sweep visits is
    // rejected, and a float divide costs more than the comparisons that reject it.
    const float tv[3] = {from[0] - v0[0], from[1] - v0[1], from[2] - v0[2]};
    const float u = tv[0] * pv[0] + tv[1] * pv[1] + tv[2] * pv[2];
    if (det > 0.0f) { if (u < 0.0f || u > det) return false; }
    else            { if (u > 0.0f || u < det) return false; }

    const float qv[3] = {tv[1] * e1[2] - tv[2] * e1[1], tv[2] * e1[0] - tv[0] * e1[2],
                         tv[0] * e1[1] - tv[1] * e1[0]};
    const float v = dir[0] * qv[0] + dir[1] * qv[1] + dir[2] * qv[2];
    if (det > 0.0f) { if (v < 0.0f || u + v > det) return false; }
    else            { if (v > 0.0f || u + v < det) return false; }

    const float invDet = 1.0f / det;
    const float t = (e2[0] * qv[0] + e2[1] * qv[1] + e2[2] * qv[2]) * invDet;
    if (t < 0.0f || t > maxDist)
        return false;

    // Face normal, oriented against the direction of travel. Squared length answers the
    // degeneracy test on its own, so a caller that wants no normal pays no square root.
    const float n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
                        e1[0] * e2[1] - e1[1] * e2[0]};
    const float len2 = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];
    if (len2 < 1e-24f)
        return false;

    outDist = t;
    if (outNormal) {
        const float sign = (n[0]*dir[0] + n[1]*dir[1] + n[2]*dir[2]) > 0.0f ? -1.0f : 1.0f;
        const float inv = sign / std::sqrt(len2);
        for (int i = 0; i < 3; ++i)
            outNormal[i] = n[i] * inv;
    }
    return true;
}

bool ClosestOnPrimitive(const Collider& sh, const float* local, float* closest, bool& inside)
{
    inside = false;
    switch (sh.type) {
    case kColliderSphere: {
        closest[0] = closest[1] = closest[2] = 0.0f;
        return true;
    }
    case kColliderCapsule: {
        // Capsules run along local x, from -halfHeight to +halfHeight.
        const float h = sh.dims[1];
        closest[0] = std::min(std::max(local[0], -h), h);
        closest[1] = 0.0f;
        closest[2] = 0.0f;
        return true;
    }
    case kColliderBox: {
        for (int a = 0; a < 3; ++a)
            closest[a] = std::min(std::max(local[a], -sh.dims[a]), sh.dims[a]);
        // A point strictly within every extent has no nearest surface point in this sense, so
        // it leaves along whichever face it is closest to.
        inside = (std::fabs(local[0]) < sh.dims[0] && std::fabs(local[1]) < sh.dims[1] &&
                  std::fabs(local[2]) < sh.dims[2]);
        if (inside) {
            int axis = 0;
            float best = sh.dims[0] - std::fabs(local[0]);
            for (int a = 1; a < 3; ++a) {
                const float d = sh.dims[a] - std::fabs(local[a]);
                if (d < best) {
                    best = d;
                    axis = a;
                }
            }
            closest[axis] = (local[axis] >= 0.0f) ? sh.dims[axis] : -sh.dims[axis];
        }
        return true;
    }
    default:
        return false;
    }
}

bool SweepMesh(const TriMesh& m, const float* from, const float* dir, float maxDist,
               float skin, float& outDist, float* outNormal)
{
    const int triCount = int(m.indices.size() / 3);
    if (triCount <= 0)
        return false;

    // Cheap reject against the mesh's own bounds, before any triangle is touched.
    const float to[3] = {from[0] + dir[0] * maxDist, from[1] + dir[1] * maxDist,
                         from[2] + dir[2] * maxDist};
    for (int a = 0; a < 3; ++a) {
        const float lo = std::min(from[a], to[a]) - skin;
        const float hi = std::max(from[a], to[a]) + skin;
        if (hi < m.lower[a] || lo > m.upper[a])
            return false;
    }

    const size_t maxIndex = m.verts.size() / 3;
    float best = maxDist;
    bool hit = false;

    auto testTriangle = [&](int t) {
        const int i0 = m.indices[size_t(t) * 3 + 0];
        const int i1 = m.indices[size_t(t) * 3 + 1];
        const int i2 = m.indices[size_t(t) * 3 + 2];
        if (i0 < 0 || i1 < 0 || i2 < 0 || size_t(i0) >= maxIndex || size_t(i1) >= maxIndex ||
            size_t(i2) >= maxIndex)
            return;

        float hitT = 0.0f, n[3];
        if (SegmentHitsTriangle(from, dir, best, &m.verts[size_t(i0) * 3],
                                &m.verts[size_t(i1) * 3], &m.verts[size_t(i2) * 3], hitT, n)) {
            best = hitT;
            hit = true;
            if (outNormal)
                for (int a = 0; a < 3; ++a)
                    outNormal[a] = n[a];
        }
    };

    const TriGrid& g = m.grid;
    if (!g.valid) {
        for (int t = 0; t < triCount; ++t)
            testTriangle(t);
    } else {
        int lo[3], hi[3];
        for (int a = 0; a < 3; ++a) {
            const float mn = std::min(from[a], to[a]) - skin;
            const float mx = std::max(from[a], to[a]) + skin;
            lo[a] = std::min(std::max(int((mn - g.origin[a]) * g.invCell[a]), 0), g.dim[a] - 1);
            hi[a] = std::min(std::max(int((mx - g.origin[a]) * g.invCell[a]), 0), g.dim[a] - 1);
        }
        for (int z = lo[2]; z <= hi[2]; ++z)
            for (int y = lo[1]; y <= hi[1]; ++y)
                for (int x = lo[0]; x <= hi[0]; ++x) {
                    const size_t cell = size_t(GridCell(g, x, y, z));
                    for (int e = g.start[cell]; e < g.start[cell + 1]; ++e)
                        testTriangle(g.tris[size_t(e)]);
                }
    }

    if (hit)
        outDist = best;
    return hit;
}

float ConvexDistance(const float* planes, int planeCount, const float* local, int& outPlane)
{
    outPlane = -1;
    if (!planes || planeCount <= 0)
        return 1e30f;

    float best = -1e30f;
    for (int k = 0; k < planeCount; ++k) {
        const float* pl = &planes[size_t(k) * 4];
        const float d = pl[0] * local[0] + pl[1] * local[1] + pl[2] * local[2] + pl[3];
        if (d > best) {
            best = d;
            outPlane = k;
        }
    }
    return best;
}

bool SphereTouchesAabb(const float* lo, const float* hi, const float* point, float reach)
{
    for (int a = 0; a < 3; ++a)
        if (point[a] < lo[a] - reach || point[a] > hi[a] + reach)
            return false;
    return true;
}

float PrimitiveRadius(const Collider& sh)
{
    switch (sh.type) {
    case kColliderSphere:  return sh.dims[0];
    case kColliderCapsule: return sh.dims[0];
    default:            return 0.0f;
    }
}

}
