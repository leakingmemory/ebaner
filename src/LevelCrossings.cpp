// ebaner - a Vulkan viewer for terrainmapper rail/terrain exports.
// Copyright (C) 2026 Jan-Espen Oversand <sigsegv@radiotube.org>
//
// This file is part of ebaner. ebaner is free software: you can redistribute it
// and/or modify it under the terms of version 3 of the GNU General Public License
// as published by the Free Software Foundation.
//
// ebaner is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE. See the GNU General Public License for more details. You
// should have received a copy of the license along with ebaner; if not, see
// <https://www.gnu.org/licenses/>.

#include "LevelCrossings.h"

#include "TrackPath.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace {

std::string crossingsFile(const std::string& root) {
    return root + "/overlay/level-crossings.txt";
}

// `<trackHex>:<frac>`, as the rest of the overlay anchors things.
bool parseAt(const std::string& tok, std::uint32_t& track, double& frac) {
    const auto c = tok.find(':');
    if (c == std::string::npos) return false;
    track = static_cast<std::uint32_t>(
        std::strtoul(tok.substr(0, c).c_str(), nullptr, 16));
    frac = std::atof(tok.substr(c + 1).c_str());
    return true;
}

} // namespace

std::vector<LevelCrossing> loadLevelCrossings(const std::string& datasetRoot) {
    std::vector<LevelCrossing> out;
    std::ifstream f(crossingsFile(datasetRoot));
    if (!f) return out;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream is(line);
        std::string kind, atTok;
        LevelCrossing x;
        is >> kind >> x.id;
        if (kind != "crossing") continue;
        readName(is, x.name);
        is >> atTok;
        if (atTok.empty() || !parseAt(atTok, x.trackId, x.frac)) continue;
        double over = 0.0;
        if (is >> over) x.outerM = over; // optional
        out.push_back(std::move(x));
    }
    return out;
}

bool writeLevelCrossings(const std::string& datasetRoot,
                         const std::vector<LevelCrossing>& xs) {
    std::error_code ec;
    fs::create_directories(datasetRoot + "/overlay", ec);
    std::ofstream f(crossingsFile(datasetRoot), std::ios::trunc);
    if (!f) return false;
    f << "# ebaner level crossings, secured by lights alone (no barriers). Four heads,\n"
         "# red over white, flashing; three detection circuits of the crossing's own.\n"
         "# crossing <id> \"<name>\" <trackHex>:<frac> [<outerM>]\n"
         "# outerM is optional: left out, the approach distance is derived from the line\n"
         "# speed here, which is what nearly every crossing should do.\n";
    for (const LevelCrossing& x : xs) {
        f << "crossing " << x.id << ' ' << quoteName(x.name) << ' ' << std::hex
          << x.trackId << std::dec << ':' << x.frac;
        if (x.outerM > 0.0) f << ' ' << x.outerM;
        f << '\n';
    }
    return static_cast<bool>(f);
}

double approachDistance(double lineSpeedKmh) {
    // An unknown limit is not a licence to arm late: fall back to the slowest sensible
    // line speed rather than to zero, which would put the circuit at the minimum.
    const double kmh = lineSpeedKmh > 1.0 ? lineSpeedKmh : 40.0;
    const double v = kmh / 3.6;
    const double d = v * v / (2.0 * kCrossingBrakeDecel) + kSequenceS * v;
    return std::clamp(d, kOuterMinM, kOuterMaxM);
}

std::vector<CrossingSite> resolveCrossings(const std::vector<LevelCrossing>& xs,
                                           const std::vector<TrackPath>& paths,
                                           const std::vector<TrackPoly>& polys,
                                           const glm::dvec3& origin) {
    std::vector<CrossingSite> out(xs.size());
    for (std::size_t i = 0; i < xs.size(); ++i) {
        const LevelCrossing& x = xs[i];
        const glm::dvec3 w = fracToWorld(polys, x.trackId, x.frac);
        if (w.x == 0.0 && w.y == 0.0) continue; // stale/missing track

        // Which path carries this point, and where along it. A path chains several
        // trackIds, so the crossing's own track id is not enough to find it - the
        // position is. Bounding box first, as everything else here does.
        const glm::vec2 target(static_cast<float>(w.x - origin.x),
                               static_cast<float>(w.y - origin.y));
        double best = 25.0; // m; beyond this it is not on that path at all
        for (std::size_t pi = 0; pi < paths.size(); ++pi) {
            const TrackPath& p = paths[pi];
            if (!p.nearXY(target, static_cast<float>(best))) continue;
            for (float s = 0.0f; s <= p.length(); s += 2.0f) {
                const glm::vec3 q = p.poseAt(std::min(s, p.length())).pos;
                const double d = std::hypot(q.x - target.x, q.y - target.y);
                if (d < best) {
                    best = d;
                    out[i].path = static_cast<int>(pi);
                    out[i].s = std::min(s, p.length());
                }
            }
        }
        if (out[i].path < 0) continue;

        const TrackPath& p = paths[out[i].path];
        out[i].lineSpeedKmh = p.speedLimitAt(out[i].s);
        out[i].innerM = static_cast<float>(innerHalfM());
        out[i].outerM = static_cast<float>(
            x.outerM > 0.0 ? x.outerM : approachDistance(out[i].lineSpeedKmh));
    }
    return out;
}

void stepCrossing(CrossingState& st, const CrossingOccupancy& occ, double now) {
    const bool allClear = !occ.outerA && !occ.inner && !occ.outerB;
    st.allClearSince = allClear ? (st.allClearSince == 0.0 ? now : st.allClearSince) : 0.0;
    if (occ.inner) st.innerSeen = true;

    // The approach circuits arm on their edge - clear to occupied - and nowhere else.
    // That is what keeps a departing train from re-arming the crossing out of the far
    // approach circuit the moment it gets there.
    const bool outerEdge = (occ.outerA && !st.prevOuterA) || (occ.outerB && !st.prevOuterB);
    st.prevOuterA = occ.outerA;
    st.prevOuterB = occ.outerB;

    const auto enter = [&](CrossingPhase p) {
        st.phase = p;
        st.phaseSince = now;
    };

    switch (st.phase) {
        case CrossingPhase::Idle:
            // Either the approach edge, or the inner circuit simply being occupied. The
            // inner one is not edge-gated on purpose: it is the fallback for an approach
            // circuit that has failed, and a fallback that needs a clean edge is no
            // fallback at all.
            if (outerEdge || occ.inner) enter(CrossingPhase::Closing);
            break;

        case CrossingPhase::Closing:
            if (now - st.phaseSince >= kTrainDelayS) enter(CrossingPhase::Secured);
            break;

        case CrossingPhase::Secured:
            if (st.innerSeen && !occ.inner) {
                // The train has been over the crossing and is off it again. Released by
                // that and by nothing else - what the approach circuits read at this
                // moment is unknown and beside the point.
                enter(CrossingPhase::Opening);
            } else if (!st.innerSeen && st.allClearSince > 0.0 &&
                       now - st.allClearSince >= kStuckTimeoutS) {
                // Armed, and then nothing arrived: a movement that turned back. Without
                // this the crossing stays shut for good.
                enter(CrossingPhase::Opening);
            }
            break;

        case CrossingPhase::Opening:
            if (now - st.phaseSince >= kTrainDelayS) {
                st.innerSeen = false; // the cycle is over
                enter(CrossingPhase::Idle);
            }
            break;
    }
}

CrossingLights crossingLights(CrossingPhase phase) {
    CrossingLights l;
    switch (phase) {
        case CrossingPhase::Idle:
            l.roadWhite = true;
            l.trainRed = true;
            l.fast = false;
            break;
        case CrossingPhase::Closing:
        case CrossingPhase::Opening:
            l.roadRed = true;
            l.trainRed = true;
            l.fast = true;
            break;
        case CrossingPhase::Secured:
            l.roadRed = true;
            l.trainWhite = true;
            l.fast = true;
            break;
    }
    return l;
}
