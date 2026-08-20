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

#include "SignalPaths.h" // walkAhead: one copy of "which leg are the points set to"
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
        CrossingTrack first;
        if (atTok.empty() || !parseAt(atTok, first.trackId, first.frac)) continue;
        x.tracks.push_back(first);
        // The tail is read as keywords rather than by position. `outerM` used to be a
        // typed extraction, which kills the stream the moment it meets a non-number - so
        // a bare `barriers` would have eaten the line before anything could read it, and
        // the two fields would have been locked into one order. Read as tokens they stay
        // optional, stay in either order, and every file written before this still loads.
        std::string tok;
        while (is >> tok) {
            if (tok == "barriers") { x.barriers = true; continue; }
            if (tok == "also") { // another track of the same crossing
                std::string more;
                CrossingTrack t;
                if ((is >> more) && parseAt(more, t.trackId, t.frac)) x.tracks.push_back(t);
                continue;
            }
            x.outerM = std::atof(tok.c_str());
        }
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
    f << "# ebaner level crossings. Heads red over white, flashing, a pair facing the\n"
         "# trains on each track it spans and a pair facing the road; three detection\n"
         "# circuits per track, of the crossing's own. Secured by lights alone unless\n"
         "# `barriers` says otherwise, in which case a boom each side covers the lane its\n"
         "# traffic arrives on. Both variants light and ring the same.\n"
         "# crossing <id> \"<name>\" <trackHex>:<frac> [also <trackHex>:<frac>]... "
         "[<outerM>] [barriers]\n"
         "# Every trailing field is optional and they may come in any order. `also` names\n"
         "# another track of the *same* crossing - one road shut and one bell, with its\n"
         "# own circuits per track - which is what a crossing inside a station needs.\n"
         "# outerM left out means the approach distance is derived from the line speed\n"
         "# here, which is what nearly every crossing should do.\n";
    for (const LevelCrossing& x : xs) {
        if (x.tracks.empty()) continue; // a crossing on no track is not a record
        f << "crossing " << x.id << ' ' << quoteName(x.name) << ' ' << std::hex
          << x.tracks.front().trackId << std::dec << ':' << x.tracks.front().frac;
        for (std::size_t t = 1; t < x.tracks.size(); ++t)
            f << " also " << std::hex << x.tracks[t].trackId << std::dec << ':'
              << x.tracks[t].frac;
        if (x.outerM > 0.0) f << ' ' << x.outerM;
        if (x.barriers) f << " barriers";
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

double distantDistance(double lineSpeedKmh, double outerM) {
    const double kmh = lineSpeedKmh > 1.0 ? lineSpeedKmh : 40.0; // as above
    const double v = kmh / 3.6;
    const double braking = v * v / (2.0 * kCrossingBrakeDecel);
    // Inside the approach circuit by a margin, whatever the braking figure says.
    double d = std::min(kDistantOfBraking * braking, kDistantOfApproach * outerM);
    // And never so close that it lands on the crossing's own head, or inside the inner
    // circuit where a train standing at it would be holding the crossing shut.
    return std::max(d, innerHalfM() + kSignalOffsetM);
}

std::vector<CrossingSite> resolveCrossings(const std::vector<LevelCrossing>& xs,
                                           const std::vector<TrackPath>& paths,
                                           const std::vector<TrackPoly>& polys,
                                           const glm::dvec3& origin) {
    std::vector<CrossingSite> out(xs.size());
    for (std::size_t i = 0; i < xs.size(); ++i) {
        const LevelCrossing& x = xs[i];
        out[i].tracks.assign(x.tracks.size(), CrossingSite::On{});
        for (std::size_t t = 0; t < x.tracks.size(); ++t) {
            const glm::dvec3 w = fracToWorld(polys, x.tracks[t].trackId, x.tracks[t].frac);
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
                        out[i].tracks[t].path = static_cast<int>(pi);
                        out[i].tracks[t].s = std::min(s, p.length());
                    }
                }
            }
        }
        if (!out[i].resolved()) continue;

        // The distances are the crossing's, not a track's, and come off the fastest track
        // it spans: the warning time a crossing needs is set by the fastest train that can
        // reach it, and two tracks arming one road at two distances would be two crossings.
        // A loop whose limit the export does not carry reads 0 here and falls back inside
        // approachDistance, which is exactly why the *highest* is taken rather than the
        // first.
        for (const CrossingSite::On& o : out[i].tracks) {
            if (o.path < 0) continue;
            out[i].lineSpeedKmh =
                std::max(out[i].lineSpeedKmh, paths[o.path].speedLimitAt(o.s));
        }
        out[i].innerM = static_cast<float>(innerHalfM());
        out[i].outerM = static_cast<float>(
            x.outerM > 0.0 ? x.outerM : approachDistance(out[i].lineSpeedKmh));
        // Off the resolved approach, not the derived one: a crossing whose circuits were
        // shortened by hand must bring its repeats in with them.
        out[i].distantM =
            static_cast<float>(distantDistance(out[i].lineSpeedKmh, out[i].outerM));
    }
    return out;
}

int crossingTrackUnder(const CrossingSite& site, const std::vector<TrackPath>& paths,
                       const glm::vec2& at, float& s) {
    constexpr double kOnIt = 4.0; // m; beyond this the point is on none of these roads
    int on = -1;
    double onD = kOnIt;
    for (std::size_t t = 0; t < site.tracks.size(); ++t) {
        const int pi = site.tracks[t].path;
        if (pi < 0 || pi >= static_cast<int>(paths.size())) continue;
        const TrackPath& p = paths[pi];
        const float sAt = site.tracks[t].s;
        // Search near the crossing rather than along the whole path: a path can be tens of
        // kilometres long, and nothing outside the circuits can be on this crossing.
        const float lo = std::max(0.0f, sAt - site.outerM - 200.0f);
        const float hi = std::min(p.length(), sAt + site.outerM + 200.0f);
        // Is the point anywhere near this crossing at all? Asked once, against the
        // crossing itself, before walking the approach five metres at a time.
        //
        // A point that comes out on this road sits within kOnIt of some place in the
        // window above, and that place is at most (outerM + 200) of *track* from the
        // crossing - which is at least as far as the straight line between them. So
        // anything further off than that in plan cannot be on this road, and the
        // several hundred samples that would say so need not be taken. Nearly every
        // ask is this case: a train is near one crossing and kilometres from the rest,
        // and this is asked of every crossing for every axle, every frame.
        const glm::vec3 c = p.poseAt(sAt).pos;
        if (std::hypot(c.x - at.x, c.y - at.y) > site.outerM + 200.0 + kOnIt) continue;
        float bestS = lo;
        double bestD = 1e30;
        for (float u = lo; u <= hi; u += 5.0f) {
            const glm::vec3 q = p.poseAt(u).pos;
            const double d = std::hypot(q.x - at.x, q.y - at.y);
            if (d < bestD) { bestD = d; bestS = u; }
        }
        // Refine: the coarse step is wider than the gap between two roads at a turnout,
        // and which of them is nearer is the whole question here.
        for (float u = std::max(lo, bestS - 5.0f); u <= std::min(hi, bestS + 5.0f);
             u += 0.5f) {
            const glm::vec3 q = p.poseAt(u).pos;
            const double d = std::hypot(q.x - at.x, q.y - at.y);
            if (d < bestD) { bestD = d; bestS = u; }
        }
        if (bestD < onD) { onD = bestD; on = static_cast<int>(t); s = bestS; }
    }
    return on;
}

std::vector<float> crossingReach(const CrossingSite& site,
                                 const std::vector<CrossingGuard>& guards,
                                 const std::vector<char>& open) {
    std::vector<float> reach(2 * site.tracks.size(), site.outerM);
    for (const CrossingGuard& g : guards) {
        if (g.track < 0) continue;
        if (g.placement >= 0 && g.placement < static_cast<int>(open.size()) &&
            open[g.placement])
            continue; // it is giving an authority: the circuit runs through it
        const std::size_t slot =
            2 * static_cast<std::size_t>(g.track) + (g.atM < 0.0f ? 0u : 1u);
        if (slot >= reach.size()) continue;
        // The nearest one wins: it is the first a train meets coming in, and nothing
        // beyond it is on the road to here at all.
        reach[slot] = std::min(reach[slot], std::abs(g.atM));
    }
    // Never inside the inner circuit, which is at the crossing itself.
    for (float& r : reach) r = std::max(r, site.innerM);
    return reach;
}

int crossingRoadAtPoints(const CrossingSite& site, const SwitchNetwork& net, int on,
                         float s) {
    if (on < 0 || on >= static_cast<int>(site.tracks.size())) return on;
    const int path = site.tracks[on].path;
    if (path < 0) return on;
    const float sx = site.tracks[on].s;
    if (std::abs(sx - s) < 1e-3f) return on; // standing on the crossing itself
    const int dir = s < sx ? +1 : -1;        // the way it must run to reach the crossing

    // The nearest turnout between the train and the crossing that is *facing* it and set
    // away from this road. Nearest, because that is the one it meets first: anything beyond
    // it is on a road this train will not be taking.
    int road = on;
    float nearest = 1e30f;
    const std::vector<Turnout>& tos = net.turnouts();
    for (std::size_t i = 0; i < tos.size(); ++i) {
        if (tos[i].mainPath != path) continue;
        const float ts = tos[i].sMain;
        if (dir * (ts - s) <= 0.0f || dir * (sx - ts) <= 0.0f) continue; // not in the way
        // Trailing points do not divert anything: a train running into the heel takes the
        // road it is already on (and forces the switch), whichever way they are set.
        if (tos[i].facingS != dir) continue;
        if (net.state(static_cast<int>(i)) != SwitchState::Diverging) continue;
        for (std::size_t t = 0; t < site.tracks.size(); ++t) {
            if (static_cast<int>(t) == on) continue;
            if (site.tracks[t].path != tos[i].sidingPath) continue;
            if (dir * (ts - s) < nearest) { nearest = dir * (ts - s); road = static_cast<int>(t); }
        }
    }
    return road;
}

namespace {

// One track's sequence. Everything here is about that track alone: its own circuits, its
// own timers, its own phase. Whether the road is shut, when the bell started and where the
// booms are belong to the crossing and are settled by the caller once all the tracks have
// been stepped.
void stepTrack(CrossingTrackState& st, const CrossingOccupancy& occ, double now) {
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

} // namespace

void stepCrossing(const LevelCrossing& x, CrossingState& st,
                  const std::vector<CrossingOccupancy>& occ, double now) {
    st.tracks.resize(std::max<std::size_t>(x.tracks.size(), 1));
    const bool wasShut = st.shut();
    for (std::size_t t = 0; t < st.tracks.size(); ++t)
        stepTrack(st.tracks[t], t < occ.size() ? occ[t] : CrossingOccupancy{}, now);

    // The crossing activating is the *first* of its tracks leaving Idle, and the bell is
    // timed from that. The second track arming ten seconds later does not restart it: one
    // crossing, one bell, however many roads it carries.
    if (!wasShut && st.shut()) st.activeSince = now;

    // --- The barriers -----------------------------------------------------------
    // Moved at a rate toward where they ought to be, rather than computed from a
    // timestamp. That is what makes an interrupted movement come out right for nothing: a
    // train arriving while the booms are still coming up sends them back down from
    // wherever they had got to, instead of snapping to the start of a fresh fall.
    //
    // Down while the crossing is shut, once the road has had kBarrierDelayS of flashing
    // first. Up from the moment the train is off the inner circuit - Opening is entered on
    // that and nothing else, so the booms start lifting exactly as the red delay begins.
    const double dt = st.lastStepS > 0.0 ? now - st.lastStepS : 0.0;
    st.lastStepS = now;
    if (x.barriers) {
        // Down while *any* track is still closing or secured, and starting up only once
        // every one of them has released: one road, one pair of booms.
        bool closing = false;
        for (const CrossingTrackState& t : st.tracks)
            closing |= t.phase == CrossingPhase::Closing || t.phase == CrossingPhase::Secured;
        const float target =
            (closing && now - st.activeSince >= kBarrierDelayS) ? 1.0f : 0.0f;
        const float step = static_cast<float>(dt / kBarrierTravelS);
        if (st.barrier < target) st.barrier = std::min(target, st.barrier + step);
        else if (st.barrier > target) st.barrier = std::max(target, st.barrier - step);
    } else {
        st.barrier = 0.0f; // no booms to move
    }
}

CrossingLights crossingLights(CrossingPhase phase, bool fast) {
    CrossingLights l;
    l.fast = fast;
    switch (phase) {
        case CrossingPhase::Idle:
            l.roadWhite = true;
            l.trainRed = true;
            break;
        case CrossingPhase::Closing:
        case CrossingPhase::Opening:
            l.roadRed = true;
            l.trainRed = true;
            break;
        case CrossingPhase::Secured:
            l.roadRed = true;
            l.trainWhite = true;
            break;
    }
    return l;
}

bool crossingBell(const CrossingState& st, double now) {
    if (!st.shut()) return false;
    return now - st.activeSince < kBellS;
}

int crossingTrackAhead(const LevelCrossing& x, const std::vector<TrackPoly>& polys,
                       const TrackJunctions& junctions, const SwitchNetwork& net,
                       std::uint32_t trackId, double frac, int dir, double maxM) {
    int found = -1;
    // The walk itself - and the awkward business of which leg of a turnout the points are
    // set to - is the one in SignalPaths.h that the distant signals read the line with.
    // All this asks of it is a different question about each stretch: is the crossing on
    // it, and if so which of its tracks.
    walkAhead(polys, junctions, net, trackId, frac, dir, maxM,
              [&](std::uint32_t track, double from, double to, int d) {
                  for (std::size_t t = 0; t < x.tracks.size(); ++t) {
                      if (x.tracks[t].trackId != track) continue;
                      const double f = x.tracks[t].frac;
                      if (d * (f - from) < -1e-9 || d * (to - f) < -1e-9) continue;
                      found = static_cast<int>(t);
                      return true;
                  }
                  return false;
              });
    return found;
}
