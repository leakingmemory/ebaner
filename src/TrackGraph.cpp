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

#include "SpatialGrid.h"

#include <unordered_map>

#include "TerrainData.h"

#include <cmath>
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
    TrackGraph g;

    // Endpoints (scene + world) and all edges of every segment, to detect dead
    // ends afterwards (an endpoint is a dead end only if it neither meets another
    // endpoint nor lies on another track's line).
    struct End { glm::vec3 scene; glm::dvec3 world; std::uint32_t track; };
    std::vector<End> ends;
    struct Edge { glm::vec2 a, b; std::uint32_t track; };
    std::vector<Edge> edges;

    {
        for (const TrackSegment& seg : data.networkTracks()) {
            if (seg.pts.size() < 2) continue;

            // Convert to scene-relative and drop coincident points (mirrors
            // buildTrackPaths so the overlay lines up with the rendered rails).
            // Keep each kept point's world coord for selection / future edits.
            std::vector<glm::vec3> pts;
            std::vector<glm::dvec3> ptsW;
            pts.reserve(seg.pts.size());
            ptsW.reserve(seg.pts.size());
            for (const glm::dvec3& w : seg.pts) {
                const glm::vec3 p(static_cast<float>(w.x - origin.x),
                                  static_cast<float>(w.y - origin.y),
                                  static_cast<float>(w.z - origin.z) + kLift);
                if (pts.empty() || glm::distance(pts.back(), p) > 1e-3f) {
                    pts.push_back(p);
                    ptsW.push_back(w);
                }
            }
            if (pts.size() < 2) continue;
            ++g.trackCount;

            const glm::vec3 pc = pointColor(seg.trackType);
            const glm::vec3 lc = lineColor(seg.trackType);
            for (std::size_t k = 0; k < pts.size(); ++k) {
                g.points.push_back({pts[k], pc});
                g.pointWorld.push_back(ptsW[k]);
                g.pointTrack.push_back(seg.trackId);
            }
            for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
                g.lines.push_back({pts[k], lc});
                g.lines.push_back({pts[k + 1], lc});
                edges.push_back({glm::vec2(pts[k]), glm::vec2(pts[k + 1]), seg.trackId});
            }
            ends.push_back({pts.front(), seg.pts.front(), seg.trackId});
            ends.push_back({pts.back(), seg.pts.back(), seg.trackId});
        }
    }

    // Distance from point p to segment a-b (2-D).
    auto distToSeg = [](const glm::vec2& p, const glm::vec2& a, const glm::vec2& b) {
        const glm::vec2 ab = b - a;
        const float l2 = glm::dot(ab, ab);
        const float t = l2 > 1e-9f ? glm::clamp(glm::dot(p - a, ab) / l2, 0.0f, 1.0f) : 0.0f;
        return glm::length(p - (a + ab * t));
    };

    // Dead ends: an endpoint that neither meets another endpoint (~1 m) nor lies on a
    // *different* track's line (~1.5 m). The loose ends of broken links; the editor
    // lets you connect two of them, or snap one onto the track it crosses.
    constexpr float kNodeTol = 1.0f, kTouchTol = 0.6f;
    // Both questions are asked once per endpoint against every endpoint and every edge
    // in the network, which over the whole line is ~150M plus ~1.2G tests. Index them.
    std::unordered_map<std::int64_t, std::vector<std::size_t>> endGrid, edgeGrid;
    for (std::size_t i = 0; i < ends.size(); ++i)
        endGrid[grid::key(ends[i].scene.x, ends[i].scene.y)].push_back(i);
    for (std::size_t e = 0; e < edges.size(); ++e)
        grid::forCellsAlong(edges[e].a.x, edges[e].a.y, edges[e].b.x, edges[e].b.y,
                            [&](std::int64_t c) { edgeGrid[c].push_back(e); });

    for (std::size_t i = 0; i < ends.size(); ++i) {
        bool joined = false;
        grid::forCellsNear(ends[i].scene.x, ends[i].scene.y, kNodeTol,
                           [&](std::int64_t c) {
                               const auto it = endGrid.find(c);
                               if (it == endGrid.end()) return;
                               for (const std::size_t j : it->second) {
                                   if (j == i) continue;
                                   if (std::hypot(ends[i].scene.x - ends[j].scene.x,
                                                  ends[i].scene.y - ends[j].scene.y) <=
                                       kNodeTol)
                                       joined = true;
                               }
                           });
        const glm::vec2 ep(ends[i].scene);
        // A cell of reach: forCellsAlong can miss a cell an edge only clips.
        if (!joined)
            grid::forCellsNear(ep.x, ep.y, grid::kCell, [&](std::int64_t c) {
                const auto it = edgeGrid.find(c);
                if (it == edgeGrid.end()) return;
                for (const std::size_t e : it->second)
                    if (edges[e].track != ends[i].track &&
                        distToSeg(ep, edges[e].a, edges[e].b) <= kTouchTol)
                        joined = true;
            });
        if (!joined) {
            g.deadEnds.push_back({ends[i].scene, glm::vec3(1.0f, 0.15f, 0.12f)});
            g.deadEndWorld.push_back(ends[i].world);
        }
    }
    return g;
}
