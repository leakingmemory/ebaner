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

#include "LineBlock.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

// Do two routes share a positive length of rail? Meeting end to end at a border is not
// sharing: that is the join two routes set in sequence are made of. The same rule
// routesOppose() reads, and it has to be the same or the two would disagree about which
// pairs are even comparable.
bool overlaps(const SignalPath& a, const SignalPath& b) {
    for (const SectionInterval& x : a.parts) {
        for (const SectionInterval& y : b.parts) {
            if (x.trackId != y.trackId) continue;
            const double xlo = std::min(x.from, x.to), xhi = std::max(x.from, x.to);
            const double ylo = std::min(y.from, y.to), yhi = std::max(y.from, y.to);
            if (std::min(xhi, yhi) - std::max(xlo, ylo) > 1e-6) return true;
        }
    }
    return false;
}

bool sameBorder(const Border& a, const Border& b, const std::vector<TrackPoly>& polys) {
    return a.trackId == b.trackId &&
           std::abs(a.frac - b.frac) <= sameFracTol(polys, a.trackId);
}

// The road a block signal governs: the path leaving its border the way it faces.
int roadFor(const BlockSignal& bs, const std::vector<SignalPath>& signalPaths,
            const std::vector<TrackPoly>& polys, int& found) {
    const glm::dvec2 facing = trackTangent(polys, bs.at.trackId, bs.at.frac, bs.dir);
    int road = -1;
    found = 0;
    for (std::size_t pi = 0; pi < signalPaths.size(); ++pi) {
        const SignalPath& p = signalPaths[pi];
        if (!sameBorder(p.start, bs.at, polys)) continue;
        glm::dvec3 w(0.0);
        glm::dvec2 fwd(0.0);
        if (!routeStartPose(p, polys, w, fwd)) continue;
        if (glm::dot(fwd, facing) <= 0.0) continue; // leaves the other way
        ++found;
        if (road < 0) road = static_cast<int>(pi);
    }
    return road;
}

} // namespace

LineBlocks resolveLineBlocks(const std::vector<BlockSignal>& blockSignals,
                             const std::vector<SignalPath>& signalPaths,
                             const std::vector<TrackPoly>& polys,
                             const TrackCircuits& circuits) {
    LineBlocks lb;
    if (blockSignals.empty()) return lb; // nothing authored: the whole thing is inert
    lb.roads.resize(blockSignals.size());

    char note[256];
    for (std::size_t i = 0; i < blockSignals.size(); ++i) {
        const BlockSignal& bs = blockSignals[i];
        int found = 0;
        const int road = roadFor(bs, signalPaths, polys, found);
        lb.roads[i].road = road;
        if (road < 0) {
            // Fails to danger, and says so. The other way round - a block signal carrying
            // its own copy of the road - would fail silently: it would go on clearing over
            // a road nothing else in the overlay believed in.
            std::snprintf(note, sizeof(note),
                          "block signal \"%s\" stands at %x:%g but no signal path leaves "
                          "that border facing that way - it will stay at danger",
                          bs.name.c_str(), bs.at.trackId, bs.at.frac);
            lb.notes.push_back(note);
            continue;
        }
        if (found > 1) {
            std::snprintf(note, sizeof(note),
                          "block signal \"%s\" has %d roads leaving its border; taking "
                          "\"%s\"", bs.name.c_str(), found, signalPaths[road].name.c_str());
            lb.notes.push_back(note);
        }
        lb.roads[i].sections = pathSections(signalPaths[road], circuits);
    }

    // Group the signals into lines. Two belong together when they stand at one border -
    // the pair facing opposite ways, whose two roads are the two halves of the line - or
    // when their roads run over rails in common, which is what chains a line cut by more
    // than one border.
    std::vector<int> parent(blockSignals.size());
    for (std::size_t i = 0; i < parent.size(); ++i) parent[i] = static_cast<int>(i);
    std::function<int(int)> find = [&](int a) {
        while (parent[a] != a) { parent[a] = parent[parent[a]]; a = parent[a]; }
        return a;
    };
    auto join = [&](int a, int b) {
        a = find(a); b = find(b);
        if (a != b) parent[a] = b;
    };
    for (std::size_t i = 0; i < blockSignals.size(); ++i) {
        if (lb.roads[i].road < 0) continue;
        for (std::size_t j = i + 1; j < blockSignals.size(); ++j) {
            if (lb.roads[j].road < 0) continue;
            if (sameBorder(blockSignals[i].at, blockSignals[j].at, polys) ||
                overlaps(signalPaths[lb.roads[i].road], signalPaths[lb.roads[j].road]))
                join(static_cast<int>(i), static_cast<int>(j));
        }
    }

    // One LineBlock per group, in first-appearance order so the numbering is stable.
    std::vector<int> lineOf(blockSignals.size(), -1);
    for (std::size_t i = 0; i < blockSignals.size(); ++i) {
        if (lb.roads[i].road < 0) continue;
        const int root = find(static_cast<int>(i));
        if (lineOf[root] < 0) {
            lineOf[root] = static_cast<int>(lb.lines.size());
            lb.lines.push_back({});
        }
        const int li = lineOf[root];
        lb.roads[i].line = li;
        lb.lines[li].signals.push_back(static_cast<int>(i));
    }

    for (std::size_t li = 0; li < lb.lines.size(); ++li) {
        LineBlock& L = lb.lines[li];
        for (const int i : L.signals)
            for (const int id : lb.roads[i].sections)
                if (std::find(L.sections.begin(), L.sections.end(), id) == L.sections.end())
                    L.sections.push_back(id);
        std::sort(L.sections.begin(), L.sections.end());

        // The line's two ends: where its roads finish, less every border a block signal of
        // this line stands on. What is left is the two station borders.
        for (const int i : L.signals) {
            const Border& e = signalPaths[lb.roads[i].road].end;
            bool atSignal = false;
            for (const int j : L.signals)
                if (sameBorder(e, blockSignals[j].at, polys)) atSignal = true;
            if (atSignal) continue;
            bool have = false;
            for (const Border& b : L.ends)
                if (sameBorder(e, b, polys)) have = true;
            if (!have) L.ends.push_back(e);
        }

        // Two-colour the signals: which of the line's two senses each one faces. Seed the
        // first at +1, then walk outward - two signals whose roads oppose face opposite
        // ways, and two at one border always do, since each takes the road the other does
        // not.
        lb.roads[L.signals.front()].align = 1;
        for (int pass = 0; pass < static_cast<int>(L.signals.size()); ++pass) {
            for (const int i : L.signals) {
                if (lb.roads[i].align == 0) continue;
                for (const int j : L.signals) {
                    if (i == j || lb.roads[j].align != 0) continue;
                    const SignalPath& ri = signalPaths[lb.roads[i].road];
                    const SignalPath& rj = signalPaths[lb.roads[j].road];
                    if (sameBorder(blockSignals[i].at, blockSignals[j].at, polys))
                        lb.roads[j].align = -lb.roads[i].align;
                    else if (overlaps(ri, rj))
                        lb.roads[j].align = routesOppose(ri, rj) ? -lb.roads[i].align
                                                                 : lb.roads[i].align;
                }
            }
        }

        // Sanity. A line that is not two-ended, or whose colouring did not reach every
        // signal or contradicts itself, is a shape this does not understand - so it holds
        // everything on it at danger rather than guessing.
        for (const int i : L.signals) {
            if (lb.roads[i].align != 0) continue;
            L.sound = false;
            std::snprintf(note, sizeof(note),
                          "block signal \"%s\" could not be given a direction on its line",
                          blockSignals[i].name.c_str());
            lb.notes.push_back(note);
        }
        for (const int i : L.signals)
            for (const int j : L.signals) {
                if (i >= j) continue;
                const SignalPath& ri = signalPaths[lb.roads[i].road];
                const SignalPath& rj = signalPaths[lb.roads[j].road];
                if (!overlaps(ri, rj)) continue;
                const int want = routesOppose(ri, rj) ? -lb.roads[i].align : lb.roads[i].align;
                if (lb.roads[j].align == want) continue;
                L.sound = false;
                std::snprintf(note, sizeof(note),
                              "line block %zu: \"%s\" and \"%s\" disagree about which way "
                              "the line runs", li, blockSignals[i].name.c_str(),
                              blockSignals[j].name.c_str());
                lb.notes.push_back(note);
            }
        if (L.ends.size() != 2) {
            L.sound = false;
            std::snprintf(note, sizeof(note),
                          "line block %zu has %zu end(s), not 2 - holding its %zu signal(s) "
                          "at danger", li, L.ends.size(), L.signals.size());
            lb.notes.push_back(note);
        }

        // Named after the roads that make it up, which already carry the two stations'
        // codes: "MAJ-SGD N" and "SGD-MAJ S" give "MAJ-SGD".
        const std::string& first = signalPaths[lb.roads[L.signals.front()].road].name;
        const std::size_t sp = first.find(' ');
        L.name = sp == std::string::npos ? first : first.substr(0, sp);
    }
    return lb;
}

int lineBlockDirection(const LineBlocks& lb, int line, const SignalPath& movement,
                       const std::vector<SignalPath>& signalPaths) {
    if (line < 0 || line >= static_cast<int>(lb.lines.size())) return 0;
    for (const int i : lb.lines[line].signals) {
        const BlockRoad& br = lb.roads[i];
        if (br.road < 0 || br.align == 0) continue;
        const SignalPath& road = signalPaths[br.road];
        if (!overlaps(movement, road)) continue;
        // Running the other way over this road means facing its signal, so the movement
        // is going the way that signal's opposite number faces.
        return routesOppose(movement, road) ? -br.align : br.align;
    }
    return 0; // shares no rails with this line
}

void claimLineBlocks(const LineBlocks& lb, std::vector<LineBlockState>& state,
                     const SignalPath& movement,
                     const std::vector<SignalPath>& signalPaths, const std::string& by) {
    state.resize(lb.lines.size());
    for (std::size_t li = 0; li < lb.lines.size(); ++li) {
        if (!lb.lines[li].sound) continue;
        const int dir = lineBlockDirection(lb, static_cast<int>(li), movement, signalPaths);
        if (dir == 0) continue;
        if (state[li].claimed == dir) continue; // already ours: a following move
        state[li].claimed = dir;
        state[li].by = by;
    }
}

std::string lineBlockRefusal(const LineBlocks& lb,
                             const std::vector<LineBlockState>& state,
                             const SignalPath& movement,
                             const std::vector<SignalPath>& signalPaths,
                             const std::function<bool(int)>& occupied,
                             std::string* brief) {
    for (std::size_t li = 0; li < lb.lines.size() && li < state.size(); ++li) {
        const LineBlock& L = lb.lines[li];
        const int dir = lineBlockDirection(lb, static_cast<int>(li), movement, signalPaths);
        if (dir == 0) continue;
        if (!L.sound) {
            if (brief) *brief = "line unclear";
            return "the line block at " + L.name + " does not make sense - see the log";
        }
        if (state[li].claimed != 0 && state[li].claimed != dir) {
            if (brief) *brief = "line set";
            return "the line " + L.name + " is set the other way, by " + state[li].by;
        }
        if (state[li].claimed == 0) {
            // Nothing claimed it, so nothing knows which way whatever is standing there is
            // pointing. Refuse rather than clear a train toward it.
            for (const int id : L.sections) {
                if (!occupied(id)) continue;
                if (brief) *brief = "line occupied";
                return "a train stands on the line " + L.name + " and no route claimed it";
            }
        }
    }
    return {};
}

bool stepLineBlocks(const LineBlocks& lb, std::vector<LineBlockState>& state,
                    const std::vector<SignalPath>& signalPaths, const SwitchNetwork& net,
                    const std::vector<TrackPoly>& polys,
                    const std::function<bool(int)>& occupied,
                    const std::function<bool(int)>& routeInto, BlockOutcome& out) {
    // Nothing authored: leave the outcome untouched rather than sizing it up to a vector of
    // zeros, which would read as a change on the first step and dirty the signal buffer on
    // every line that has no block signals at all.
    if (lb.roads.empty()) return false;
    state.resize(lb.lines.size());
    const std::vector<char> wasSet = out.roadSet;
    const std::vector<SignalAspect> wasShown = out.aspect;
    out.roadSet.assign(signalPaths.size(), 0);
    out.aspect.assign(lb.roads.size(), SignalAspect::Stop);

    // Release first, so a signal is never decided against a claim that is already spent.
    // A claim outlives its route on purpose: the route is erased as the train enters its
    // last circuit, and the train is then out on the line with nothing else recording
    // which way it went.
    for (std::size_t li = 0; li < lb.lines.size(); ++li) {
        if (state[li].claimed == 0) continue;
        if (routeInto(static_cast<int>(li))) continue;
        bool anyOccupied = false;
        for (const int id : lb.lines[li].sections)
            if (occupied(id)) anyOccupied = true;
        if (anyOccupied) continue;
        state[li] = {};
    }

    for (std::size_t i = 0; i < lb.roads.size(); ++i) {
        const BlockRoad& br = lb.roads[i];
        if (br.road < 0 || br.line < 0) continue;
        const LineBlock& L = lb.lines[br.line];
        if (!L.sound) continue;
        if (state[br.line].claimed != br.align) continue; // not set our way
        bool clear = true;
        for (const int id : br.sections)
            if (occupied(id)) clear = false;
        if (!clear) continue;
        const SignalPath& road = signalPaths[br.road];
        if (!pathSwitchesAligned(road, net, polys)) continue;
        // The road is opened before the head is given its green, so there is no step, not
        // even one, at which a block signal shows clear over a road that is not set.
        out.roadSet[br.road] = 1;
        out.aspect[i] = defaultRouteType(road, net, polys) == RouteType::C2
                            ? SignalAspect::ClearReduced
                            : SignalAspect::Clear;
    }
    return out.roadSet != wasSet || out.aspect != wasShown;
}

void dropBlockRoads(std::vector<SignalPlacement>& dwarfs,
                    const std::vector<char>& blockRoad) {
    for (SignalPlacement& sp : dwarfs) {
        std::vector<int> keep;
        for (const int pi : sp.paths)
            if (pi < 0 || pi >= static_cast<int>(blockRoad.size()) || !blockRoad[pi])
                keep.push_back(pi);
        sp.paths = std::move(keep);
    }
    dwarfs.erase(std::remove_if(dwarfs.begin(), dwarfs.end(),
                                [](const SignalPlacement& sp) { return sp.paths.empty(); }),
                 dwarfs.end());
}
