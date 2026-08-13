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

#include "TxpGraph.h"

#include "TrackCircuits.h" // TrackPoly, fracToWorld
#include "TrackPath.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>

namespace {

// How far from a path a TXP position may be and still be taken to stand on it. The
// position is resolved from its own track, so this only has to cover the sampling step
// and the spline pulling away from the surveyed points.
constexpr double kOnPathM = 25.0;
constexpr double kPathStepM = 5.0;

} // namespace

void TxpGraph::build(const std::vector<TxpPosition>& ps,
                     const std::vector<SignalStation>& attached,
                     const std::vector<Station>& stations,
                     const std::vector<TrackPoly>& polys,
                     const std::vector<TrackPath>& paths, const glm::dvec3& origin) {
    nodes_.clear();
    edges_.clear();
    (void)stations; // the name is all that is needed; attachStations resolved it already

    // Each position, placed on the line it stands on. Resolved from the position's own
    // track rather than from the station node, which can sit some way off the rails.
    struct Placed { int path; float s; };
    std::map<std::string, std::vector<Placed>> byStation;
    for (std::size_t i = 0; i < ps.size(); ++i) {
        if (i >= attached.size() || attached[i].name.empty()) continue;
        const glm::dvec3 w = fracToWorld(polys, ps[i].trackId, ps[i].frac);
        if (w.x == 0.0 && w.y == 0.0) continue; // stale/missing track
        const glm::vec2 target(static_cast<float>(w.x - origin.x),
                               static_cast<float>(w.y - origin.y));
        double best = kOnPathM;
        int bp = -1;
        float bs = 0.0f;
        for (std::size_t pi = 0; pi < paths.size(); ++pi) {
            const TrackPath& p = paths[pi];
            if (!p.nearXY(target, static_cast<float>(best))) continue;
            for (float s = 0.0f; s <= p.length(); s += static_cast<float>(kPathStepM)) {
                const glm::vec3 q = p.poseAt(std::min(s, p.length())).pos;
                const double d = std::hypot(q.x - target.x, q.y - target.y);
                if (d < best) { best = d; bp = static_cast<int>(pi); bs = s; }
            }
        }
        if (bp >= 0) byStation[attached[i].name].push_back({bp, bs});
    }

    // One node per station, on the longest line any of its positions stands on. A station
    // has positions on its sidings as well as on the running line, and it is the running
    // line the next station is reached along - a siding would order it by nothing.
    for (const auto& [name, placed] : byStation) {
        int bestPath = -1;
        float bestLen = -1.0f;
        for (const Placed& q : placed) {
            const float len = paths[q.path].length();
            if (len > bestLen) { bestLen = len; bestPath = q.path; }
        }
        if (bestPath < 0) continue;
        // Centred among whatever it has on that line, so a station reads as one place
        // rather than as whichever of its positions happened to be found first.
        double sum = 0.0;
        int n = 0;
        for (const Placed& q : placed)
            if (q.path == bestPath) { sum += q.s; ++n; }
        nodes_.push_back({name, bestPath, static_cast<float>(sum / std::max(n, 1))});
    }

    // Ordered along the line, and chained. Grouping by path first is what keeps two
    // stations either side of a break from being called neighbours: they are on different
    // lines as far as the network is concerned, and nothing can be offered across.
    std::sort(nodes_.begin(), nodes_.end(), [](const TxpStationNode& a,
                                               const TxpStationNode& b) {
        return a.path != b.path ? a.path < b.path : a.s < b.s;
    });
    for (std::size_t i = 0; i + 1 < nodes_.size(); ++i)
        if (nodes_[i].path == nodes_[i + 1].path)
            edges_.push_back({static_cast<int>(i), static_cast<int>(i + 1)});

    if (!nodes_.empty())
        std::printf("[TxpGraph] %zu TXP station(s), %zu link(s)\n", nodes_.size(),
                    edges_.size());
}

int TxpGraph::indexOf(const std::string& station) const {
    for (std::size_t i = 0; i < nodes_.size(); ++i)
        if (nodes_[i].name == station) return static_cast<int>(i);
    return -1;
}

std::vector<std::string> TxpGraph::neighbours(const std::string& station) const {
    std::vector<std::string> out;
    const int i = indexOf(station);
    if (i < 0) return out;
    for (const auto& [a, b] : edges_) {
        if (a == i) out.push_back(nodes_[b].name);
        else if (b == i) out.push_back(nodes_[a].name);
    }
    return out;
}

bool TxpGraph::linked(const std::string& a, const std::string& b) const {
    const int ia = indexOf(a), ib = indexOf(b);
    if (ia < 0 || ib < 0) return false;
    for (const auto& [x, y] : edges_)
        if ((x == ia && y == ib) || (x == ib && y == ia)) return true;
    return false;
}

std::string TxpGraph::next(const std::string& station, bool forward) const {
    const int i = indexOf(station);
    if (i < 0) return {};
    const int j = forward ? i + 1 : i - 1;
    if (j < 0 || j >= static_cast<int>(nodes_.size())) return {};
    // Only along the same line: the node beyond the end of one chain is the start of
    // another and there is no way to reach it.
    if (nodes_[j].path != nodes_[i].path) return {};
    return nodes_[j].name;
}
