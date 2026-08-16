#include "Contact.h"

#include "Math3D.h"

#include <algorithm>
#include <cmath>

namespace flexrevive::contact {

using namespace math;

float SupportExtent(const fragment::Hull& shp, const float* q, const float* dir)
{
    // A rotation preserves the dot product, so dot(R*p, dir) equals dot(p, inverse(R)*dir).
    // Turning the direction into the hull's own frame once therefore gives the same answer as
    // turning all 26 points into the world, for one rotation instead of 26.
    float inv[4], local[3];
    QuatConjugate(q, inv);
    QuatRotate(inv, dir, local);

    float best = 0.0f;
    for (int k = 0; k < shp.numPoints; ++k) {
        const float d = Dot(&shp.points[size_t(k) * 3], local);
        if (d > best)
            best = d;
    }
    return best + shp.inflate;   // each point is a particle, not a point
}

HullContact FindHullContact(const fragment::Hull& shp, const float* q, const float* pos,
                         const float* surface, const float* normal)
{
    HullContact c;
    c.valid = false;
    c.depth = 0.0f;
    c.r[0] = c.r[1] = c.r[2] = 0.0f;

    // Which point is deepest does not depend on the constant offset from the centre to the
    // surface, so the search runs on dot(p, inverse(R)*normal) in the hull's own frame and only
    // the winner is rotated into the world. Two rotations instead of one per point.
    float inv[4], localN[3];
    QuatConjugate(q, inv);
    QuatRotate(inv, normal, localN);

    const float offset[3] = {pos[0] - surface[0], pos[1] - surface[1], pos[2] - surface[2]};
    const float base = Dot(offset, normal);

    float best = 1e30f;
    int winner = -1;
    for (int k = 0; k < shp.numPoints; ++k) {
        const float d = Dot(&shp.points[size_t(k) * 3], localN);
        if (d < best) {
            best = d;
            winner = k;
        }
    }
    if (winner >= 0) {
        QuatRotate(q, &shp.points[size_t(winner) * 3], c.r);
        best += base;
    }
    if (shp.numPoints > 0) {
        // The surface is a particle radius short of the deepest centre.
        c.depth = shp.inflate - best;
        c.valid = true;
    }
    return c;
}

void ResolveHullContact(const fragment::Hull& shp, const HullContact& c, float* pos,
                         float* vel, float* w, const float* normal, float restitution,
                         float friction, float skin, float spinFromFriction)
{
    // Lift the piece until its deepest point clears the surface by the contact skin, but never
    // by more than a piece of this size could plausibly need in one step. A depth far larger
    // than the chunk means it is somewhere it should not be, and answering that literally
    // throws it into the sky.
    const float maxPush = std::max(shp.radius, 1.0f) * kMaxSeparationRadii;
    const float push = std::min(c.depth + skin, maxPush);
    if (push > 0.0f)
        for (int a = 0; a < 3; ++a)
            pos[a] += normal[a] * push;

    // Velocity of the material at the contact: body velocity plus the spin carried around by
    // the lever arm.
    float wxr[3];
    Cross(w, c.r, wxr);
    const float vc[3] = {vel[0] + wxr[0], vel[1] + wxr[1], vel[2] + wxr[2]};
    const float vn = Dot(vc, normal);
    if (vn >= 0.0f)
        return; // already separating

    // Effective mass along the normal: 1/m plus how much the lever arm resists rotation.
    float rn[3], irn[3], irnxr[3];
    Cross(c.r, normal, rn);
    Mat3Mul(shp.invInertia, rn, irn);
    Cross(irn, c.r, irnxr);
    const float denom = 1.0f + Dot(irnxr, normal);
    if (!(denom > 1e-6f))
        return;

    // A surface stops answering a slow approach with a bounce.
    //
    // Gravity adds a few units/s of closing speed to a resting piece every substep, and giving
    // a share of it back each time is a heap that never quite stops moving. Below the threshold
    // the contact is inelastic, which is what lets debris settle.
    const float bounce = (-vn < kNoBounceSpeed) ? 0.0f : restitution;

    const float jn = -(1.0f + bounce) * vn / denom;
    for (int a = 0; a < 3; ++a)
        vel[a] += normal[a] * jn;
    {
        float torque[3], dw[3], impulse[3] = {normal[0] * jn, normal[1] * jn, normal[2] * jn};
        Cross(c.r, impulse, torque);
        Mat3Mul(shp.invInertia, torque, dw);
        for (int a = 0; a < 3; ++a)
            w[a] += dw[a];
    }

    // Rolling resistance, against the normal impulse just delivered and capped by the spin
    // itself, so it can only slow the piece and never reverse it.
    {
        const float wLen = std::sqrt(Dot(w, w));
        if (wLen > 1e-4f) {
            const float resist = friction * std::fabs(jn) * kRollingResistance /
                                 std::max(shp.radius, 1e-3f);
            const float damp = std::min(wLen, resist);
            for (int a = 0; a < 3; ++a)
                w[a] -= (w[a] / wLen) * damp;
        }
    }

    // Coulomb friction along the remaining tangential motion, capped by the normal impulse.
    // This is what turns a skid into a tumble rather than merely slowing it.
    float tangent[3] = {vc[0] - normal[0] * vn, vc[1] - normal[1] * vn, vc[2] - normal[2] * vn};
    const float tLen = std::sqrt(Dot(tangent, tangent));
    if (tLen < 1e-4f)
        return;
    for (int a = 0; a < 3; ++a)
        tangent[a] /= tLen;

    float rt[3], irt[3], irtxr[3];
    Cross(c.r, tangent, rt);
    Mat3Mul(shp.invInertia, rt, irt);
    Cross(irt, c.r, irtxr);
    const float denomT = 1.0f + Dot(irtxr, tangent);
    if (!(denomT > 1e-6f))
        return;

    // Coulomb friction, capped against the normal impulse, is what turns a skid into a tumble.
    //
    // The cap is proportional to how hard the piece presses, so a chunk resting under its own
    // weight is barely held and creeps across the surface indefinitely. Once the drift is slow
    // enough to be drift rather than travel, the surface grips instead and the whole of the
    // remaining tangential motion is taken out.
    float jt = -tLen / denomT;
    if (tLen >= kStaticFrictionSpeed) {
        const float limit = friction * std::fabs(jn);
        jt = std::max(-limit, std::min(limit, jt));
    }

    for (int a = 0; a < 3; ++a)
        vel[a] += tangent[a] * jt;
    {
        float torque[3], dw[3];
        const float impulse[3] = {tangent[0] * jt, tangent[1] * jt, tangent[2] * jt};
        Cross(c.r, impulse, torque);
        Mat3Mul(shp.invInertia, torque, dw);
        const float spin = std::max(spinFromFriction, 0.0f);
        for (int a = 0; a < 3; ++a)
            w[a] += dw[a] * spin;
    }
}

}
