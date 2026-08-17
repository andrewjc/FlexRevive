// Copyright (c) 2026 AndyR007
// SPDX-License-Identifier: MIT

#include "PieceGrid.h"

#include <algorithm>

namespace flexrevive::grid {

void PieceGrid::Build(const float* positions, int count, float cellSize)
{
    m_head.assign(kBuckets, -1);
    m_next.assign(count > 0 ? size_t(count) : 0, -1);
    m_cells.assign(count > 0 ? size_t(count) * 3 : 0, 0);
    m_indexed.assign(count > 0 ? size_t(count) : 0, 0);
    m_invCell = 1.0f / std::max(cellSize, 1e-3f);
    if (!positions || count <= 0)
        return;

    for (int i = 0; i < count; ++i) {
        const float* p = &positions[size_t(i) * 3];
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]))
            continue;
        const int cell[3] = {CellOf(p[0]), CellOf(p[1]), CellOf(p[2])};
        for (int a = 0; a < 3; ++a)
            m_cells[size_t(i) * 3 + size_t(a)] = cell[a];
        m_indexed[size_t(i)] = 1;

        const int bucket = BucketOf(cell[0], cell[1], cell[2]);
        m_next[size_t(i)] = m_head[size_t(bucket)];
        m_head[size_t(bucket)] = i;
    }
}

}
