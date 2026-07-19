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

#include "TrackGraph.h"

#include "TerrainData.h"

#include <unordered_set>

namespace {
// A small lift above the rails so the markers/lines float clear of the rail
// heads instead of z-fighting them.
constexpr float kLift = 0.3f; // metres

// Overlay colour by track type (0 = main line, 1 = siding, 2 = yard). Points are
// drawn a touch brighter than the connecting lines.
glm::vec3 lineColor(std::uint8_t trackType) {
    switch (trackType) {
        case 1:  return {0.25f, 0.85f, 1.00f}; // siding: cyan
        case 2:  return {1.00f, 0.45f, 0.90f}; // yard: magenta
        default: return {1.00f, 0.80f, 0.15f}; // main line: amber
    }
}
glm::vec3 pointColor(std::uint8_t trackType) {
    return glm::min(lineColor(trackType) + 0.25f, glm::vec3(1.0f));
}
} // namespace

TrackGraph buildTrackGraph(const TerrainData& data) {
    const glm::dvec3 origin = data.sceneOrigin();
    std::unordered_set<std::uint32_t> seen;
    TrackGraph g;

    for (const Tile& t : data.tiles()) {
        for (const TrackSegment& seg : t.tracks) {
            if (!seen.insert(seg.trackId).second) continue; // one per through-track
            if (seg.pts.size() < 2) continue;

            // Convert to scene-relative and drop coincident points (mirrors
            // buildTrackPaths so the overlay lines up with the rendered rails).
            std::vector<glm::vec3> pts;
            pts.reserve(seg.pts.size());
            for (const glm::dvec3& w : seg.pts) {
                const glm::vec3 p(static_cast<float>(w.x - origin.x),
                                  static_cast<float>(w.y - origin.y),
                                  static_cast<float>(w.z - origin.z) + kLift);
                if (pts.empty() || glm::distance(pts.back(), p) > 1e-3f)
                    pts.push_back(p);
            }
            if (pts.size() < 2) continue;
            ++g.trackCount;

            const glm::vec3 pc = pointColor(seg.trackType);
            const glm::vec3 lc = lineColor(seg.trackType);
            for (const glm::vec3& p : pts) g.points.push_back({p, pc});
            for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
                g.lines.push_back({pts[k], lc});
                g.lines.push_back({pts[k + 1], lc});
            }
        }
    }
    return g;
}
