#include "SlotTracker.h"

#include "Math3D.h"

#include <algorithm>
#include <unordered_map>

namespace flexrevive::slots {

void Classify(const float* incoming, int count, const std::vector<float>& lastReported,
              const std::vector<uint8_t>& reported, int previousCount, float respawnDistance,
              Classification& out)
{
    out.change.assign(count > 0 ? size_t(count) : 0, kUnchanged);
    out.source.assign(count > 0 ? size_t(count) : 0, -1);
    if (!incoming || count <= 0)
        return;

    const int known = std::min({previousCount, int(reported.size()),
                                int(lastReported.size() / 3)});

    // Every position reported last time, keyed exactly. The engine returns floats untouched,
    // so the raw bits identify a piece wherever it turns up.
    std::unordered_map<uint64_t, int> reportedAt;
    reportedAt.reserve(size_t(std::max(known, 0)) * 2);
    for (int k = 0; k < known; ++k)
        if (reported[size_t(k)])
            reportedAt[math::PositionKey(&lastReported[size_t(k) * 3])] = k;

    const float threshold2 = respawnDistance * respawnDistance;

    for (int i = 0; i < count; ++i) {
        if (i >= known) {
            out.change[size_t(i)] = kFresh;
            continue;
        }
        if (!reported[size_t(i)]) {
            out.change[size_t(i)] = kFresh;
            continue;
        }

        const float* mine = &lastReported[size_t(i) * 3];
        const float* theirs = &incoming[size_t(i) * 3];
        const float d[3] = {theirs[0] - mine[0], theirs[1] - mine[1], theirs[2] - mine[2]};
        if (d[0]*d[0] + d[1]*d[1] + d[2]*d[2] <= threshold2)
            continue;   // a round trip, with the engine's own nudging on top

        auto it = reportedAt.find(math::PositionKey(theirs));
        if (it != reportedAt.end() && it->second != i && it->second < known) {
            const float* was = &lastReported[size_t(it->second) * 3];
            if (was[0] == theirs[0] && was[1] == theirs[1] && was[2] == theirs[2]) {
                out.change[size_t(i)] = kMigrated;
                out.source[size_t(i)] = it->second;
                continue;
            }
        }
        out.change[size_t(i)] = kFresh;
    }
}

Reseed OnReseed(bool wasResting, float jump)
{
    if (!wasResting)
        return Reseed{false, false, true};

    // Displaced pieces keep no motion but must be awake, or they hang where they were put.
    return Reseed{jump < kSettledJump, true, false};
}

}
