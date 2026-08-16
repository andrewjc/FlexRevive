#pragma once

#include <cstdint>
#include <vector>

// The particles each debris piece owns, and how they follow it.
//
// The engine describes a piece as a run of entries in an index array, naming the container
// slots that piece holds. Reading that run per piece is what lets the particle buffers be
// advanced by walking the pieces that are moving, rather than the whole container.
namespace flexrevive::particles {

// The container slots each piece owns, as runs: piece i owns slots[start[i] .. start[i + 1]).
// `start` always has one more entry than there are pieces, so a piece with no particles has an
// empty run rather than a missing one.
void BuildPieceSlots(const int* offsets, const int* indices, int numPieces, int maxParticles,
                     std::vector<int>& start, std::vector<int>& slots);

// Moves one particle with the piece that owns it.
//
// The particles of a rigid piece are attached to it. Advancing them under their own gravity
// instead lets them drift away from the chunk they belong to without bound, since nothing stops
// a particle against the world: the piece lands and settles while its particles carry on
// falling. Anything the engine reads back out of those buffers then describes a position
// nowhere near the piece.
//
// `pos` is four floats, the fourth being the particle's inverse mass, which is left alone.
void CarryParticle(const float* delta, const float* pieceVel, float* pos, float* vel);

}
