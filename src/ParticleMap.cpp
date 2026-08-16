#include "ParticleMap.h"

namespace flexrevive::particles {

void BuildPieceSlots(const int* offsets, const int* indices, int numPieces, int maxParticles,
                     std::vector<int>& start, std::vector<int>& slots)
{
    start.assign(numPieces > 0 ? size_t(numPieces) + 1 : 1, 0);
    slots.clear();
    if (!offsets || !indices || numPieces <= 0 || maxParticles <= 0)
        return;

    for (int piece = 0; piece < numPieces; ++piece) {
        start[size_t(piece)] = int(slots.size());
        const int begin = offsets[piece];
        const int end = offsets[piece + 1];
        if (begin < 0 || end <= begin)
            continue;
        for (int k = begin; k < end; ++k) {
            const int slot = indices[k];
            if (slot >= 0 && slot < maxParticles)
                slots.push_back(slot);
        }
    }
    start[size_t(numPieces)] = int(slots.size());
}

void CarryParticle(const float* delta, const float* pieceVel, float* pos, float* vel)
{
    for (int a = 0; a < 3; ++a) {
        pos[a] += delta[a];
        vel[a] = pieceVel[a];
    }
}

}
