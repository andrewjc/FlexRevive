#include "PairContact.h"

#include "Response.h"

#include <algorithm>
#include <cmath>

namespace flexrevive::pairs {

bool Resolve(Body& a, Body& b, const float* sep, float dist, const float* gravity,
             const Settings& s, Result& out)
{
    out = Result();

    const float minDist = a.extent + b.extent + s.contactSkin;
    if (!(dist < minDist))
        return false;

    const float overlap = minDist - dist;
    out.overlap = overlap;

    // How much overlap counts as settled rather than as a collision. A piece parks as soon as
    // its correction drops below this rather than at exactly zero separation.
    const float settleGap = std::max(s.contactSkin, 0.25f * std::min(a.extent, b.extent));

    const float wake2 = s.sleepSpeed * s.sleepSpeed;
    const float spA = a.vel[0]*a.vel[0] + a.vel[1]*a.vel[1] + a.vel[2]*a.vel[2];
    const float spB = b.vel[0]*b.vel[0] + b.vel[1]*b.vel[1] + b.vel[2]*b.vel[2];
    const bool inMotion = spA > wake2 || spB > wake2;

    // Split by mass: chunks of one material share a density but not a size.
    float shareA = 0.0f, shareB = 0.0f;
    response::SeparationSplit(a.mass, b.mass, shareA, shareB);
    const float relax = std::min(1.0f, std::max(0.0f, s.relaxation));
    for (int i = 0; i < 3; ++i) {
        a.pos[i] += sep[i] * overlap * shareA * relax;
        b.pos[i] -= sep[i] * overlap * shareB * relax;
    }

    // Chunk on chunk loses spin and relative motion just as it would to the ground. A settled
    // stack exchanges almost no impulse, so these are rates rather than anything proportional
    // to one.
    const float mu = std::max(s.mu, 0.0f);
    const float keep = std::max(0.0f, 1.0f - mu * s.spinDamp * s.dt);
    if (a.ang)
        for (int i = 0; i < 3; ++i) a.ang[i] *= keep;
    if (b.ang)
        for (int i = 0; i < 3; ++i) b.ang[i] *= keep;

    // Pulled toward the velocity of their shared centre of mass, so their common motion is
    // untouched and only the grinding between them is removed. Weighting by mass is what makes
    // this conserve momentum: blending toward the plain average of the two instead would let a
    // light chunk drag a heavy one along with it.
    const float ma = std::max(a.mass, 1.0f);
    const float mb = std::max(b.mass, 1.0f);
    const float total = ma + mb;
    const float blend = std::min(0.5f, mu * s.linearDamp * s.dt);
    for (int i = 0; i < 3; ++i) {
        const float vcm = (ma * a.vel[i] + mb * b.vel[i]) / total;
        a.vel[i] += (vcm - a.vel[i]) * blend;
        b.vel[i] += (vcm - b.vel[i]) * blend;
    }

    // The separation runs from b toward a, so pointing against gravity puts a on top.
    //
    // Only a piece at rest holds another one up. A chunk lying on one that is itself falling is
    // not standing on anything, and treating it as though it were lets a cluster of falling
    // debris support itself: every piece is held up by a neighbour that is also on its way down,
    // so the whole group settles in mid-air and stays there.
    const float gl = std::sqrt(gravity[0]*gravity[0] + gravity[1]*gravity[1] +
                               gravity[2]*gravity[2]);
    if (gl > 1e-3f) {
        const float up = -(sep[0]*gravity[0] + sep[1]*gravity[1] + sep[2]*gravity[2]) / gl;
        if (up > 0.5f && b.resting)
            out.supported = kSupportedA;
        else if (up < -0.5f && a.resting)
            out.supported = kSupportedB;
    }

    if (overlap > settleGap && (inMotion || overlap > 2.0f * settleGap)) {
        out.wakeA = true;
        out.wakeB = true;
    }

    const float approach = (a.vel[0] - b.vel[0]) * sep[0] + (a.vel[1] - b.vel[1]) * sep[1] +
                           (a.vel[2] - b.vel[2]) * sep[2];
    if (approach < 0.0f) {
        // Exchange the closing velocity, damped by restitution and shared by mass, so the
        // heavier piece barely notices.
        const float impulse = -approach * (1.0f + std::max(s.restitution, 0.0f)) / total;
        for (int i = 0; i < 3; ++i) {
            a.vel[i] += sep[i] * impulse * mb;
            b.vel[i] -= sep[i] * impulse * ma;
        }
        if (std::fabs(approach) > s.sleepSpeed) {
            out.wakeA = true;
            out.wakeB = true;
        }
    }
    return true;
}

}
