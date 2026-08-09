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

#include "TerrainCarve.h"

#include "TerrainData.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {
constexpr int P = TerrainData::PIXELS; // 256
inline bool isNodata(float h) { return h <= TerrainData::NODATA + 1.0f; }

// Ballast cross-section (mirrors TrackMesh.cpp): base half-width, and base offset
// below the export z (the rail head, which already carries the +0.6 m bed offset).
constexpr float kBallastBotHalf = 2.8f;
constexpr float kBallastBotZ = -0.10f;

constexpr float kFloorHalf = kBallastBotHalf + 0.8f;  // 3.6 m (ballast + drainage shoulder)
constexpr float kFloorBelowTrack = 0.2f;              // floor just under the ballast base
constexpr float kSlopeTan = 0.36397f;                 // tan(20 deg)
constexpr float kSearchRadius = 80.0f;                // m; caps the trench half-width / influence
constexpr double kBucket = 64.0;                      // spatial-hash cell size (m)

// Media that are underground (get a portal wall) and that carve (surface).
inline bool isUnderground(std::uint8_t m) { return m == 0x55 || m == 0x54; }
inline bool isSurface(std::uint8_t m) {
    return m != 0x55 && m != 0x54 && m != 0x4C && m != 0x42; // not tunnel/tube/bridge
}

// One surface polyline edge in world xy, with rail-head elevation at each end and a
// flag marking a terminal vertex that coincides with a tunnel portal.
struct Edge {
    glm::dvec2 a, b;
    float za, zb;
    bool aPortal, bPortal;
};

inline std::int64_t bucketKey(int bx, int by) {
    return (std::int64_t(bx) << 32) ^ std::int64_t(std::uint32_t(by));
}
inline std::int64_t quantKey(double x, double y) { // ~1 m quantisation for portals
    return (std::int64_t(std::llround(x)) << 32) ^
           std::int64_t(std::uint32_t(std::llround(y)));
}
} // namespace

void carveTrackCuttings(std::vector<Tile*>& tiles,
                        const std::vector<TrackSegment>& tracks,
                        const glm::dvec3& sceneOrigin) {
    // --- Gather portal points (endpoints of underground segments) and dedup the
    // surface segments across tiles (a through-segment appears in every tile). ---
    std::unordered_set<std::int64_t> portals;
    std::unordered_set<std::string> seenSeg;
    std::vector<const TrackSegment*> surface;
    std::unordered_map<int, int> mediumHist;

    {
        for (const TrackSegment& s : tracks) {
            mediumHist[s.medium]++;
            if (s.pts.size() < 2) continue;
            if (isUnderground(s.medium)) {
                portals.insert(quantKey(s.pts.front().x, s.pts.front().y));
                portals.insert(quantKey(s.pts.back().x, s.pts.back().y));
            }
            if (!isSurface(s.medium)) continue;
            char key[96];
            std::snprintf(key, sizeof(key), "%u:%u:%zu:%lld:%lld:%lld:%lld",
                          s.trackId, s.medium, s.pts.size(),
                          (long long)std::llround(s.pts.front().x),
                          (long long)std::llround(s.pts.front().y),
                          (long long)std::llround(s.pts.back().x),
                          (long long)std::llround(s.pts.back().y));
            if (seenSeg.insert(key).second) surface.push_back(&s);
        }
    }

    // --- Build the edge list + spatial hash (each edge in every bucket its bbox,
    // expanded by the search radius, touches). ---
    std::vector<Edge> edges;
    std::unordered_map<std::int64_t, std::vector<int>> grid;
    for (const TrackSegment* s : surface) {
        const std::size_t n = s->pts.size();
        const bool firstPortal = portals.count(quantKey(s->pts[0].x, s->pts[0].y)) > 0;
        const bool lastPortal =
            portals.count(quantKey(s->pts[n - 1].x, s->pts[n - 1].y)) > 0;
        for (std::size_t k = 0; k + 1 < n; ++k) {
            // A few imported vertices carry a NODATA sentinel for z instead of a height.
            // Carving down to one digs a crater kilometres deep, which went unnoticed
            // while only the ground around Bodo was ever loaded.
            if (isNodata(static_cast<float>(s->pts[k].z)) ||
                isNodata(static_cast<float>(s->pts[k + 1].z)))
                continue;
            Edge e;
            e.a = {s->pts[k].x, s->pts[k].y};
            e.b = {s->pts[k + 1].x, s->pts[k + 1].y};
            e.za = static_cast<float>(s->pts[k].z);
            e.zb = static_cast<float>(s->pts[k + 1].z);
            e.aPortal = (k == 0) && firstPortal;
            e.bPortal = (k + 2 == n) && lastPortal;
            const int ei = static_cast<int>(edges.size());
            edges.push_back(e);

            const double minx = std::min(e.a.x, e.b.x) - kSearchRadius;
            const double maxx = std::max(e.a.x, e.b.x) + kSearchRadius;
            const double miny = std::min(e.a.y, e.b.y) - kSearchRadius;
            const double maxy = std::max(e.a.y, e.b.y) + kSearchRadius;
            for (int bx = int(std::floor(minx / kBucket)); bx <= int(std::floor(maxx / kBucket)); ++bx)
                for (int by = int(std::floor(miny / kBucket)); by <= int(std::floor(maxy / kBucket)); ++by)
                    grid[bucketKey(bx, by)].push_back(ei);
        }
    }

    // --- Carve each tile's height grid ---
    std::vector<std::uint32_t> epochOf(edges.size(), 0);
    std::uint32_t epoch = 0;
    std::size_t carvedCells = 0;
    float deepest = 0.0f;
    glm::dvec3 deepestAt{0.0};

    for (Tile* tp : tiles) {
        Tile& t = *tp;
        if (t.heights.empty()) continue;
        const double res = t.resolution, ext = t.extent, ox = t.originX, oy = t.originY;
        for (int row = 0; row < P; ++row) {
            const double wy = oy + ext - (row + 0.5) * res;
            for (int col = 0; col < P; ++col) {
                const double wx = ox + (col + 0.5) * res;
                float& h = t.heights[static_cast<std::size_t>(row) * P + col];
                if (isNodata(h)) continue;

                // Visit candidate edges in the surrounding buckets (deduped by epoch).
                ++epoch;
                float bestD = std::numeric_limits<float>::infinity();
                float ztBest = 0.0f;
                float minProfile = std::numeric_limits<float>::infinity();
                const int bx0 = int(std::floor((wx - kSearchRadius) / kBucket));
                const int bx1 = int(std::floor((wx + kSearchRadius) / kBucket));
                const int by0 = int(std::floor((wy - kSearchRadius) / kBucket));
                const int by1 = int(std::floor((wy + kSearchRadius) / kBucket));
                for (int bx = bx0; bx <= bx1; ++bx) {
                    for (int by = by0; by <= by1; ++by) {
                        auto it = grid.find(bucketKey(bx, by));
                        if (it == grid.end()) continue;
                        for (int ei : it->second) {
                            if (epochOf[ei] == epoch) continue;
                            epochOf[ei] = epoch;
                            const Edge& e = edges[ei];
                            const glm::dvec2 ab = e.b - e.a;
                            const double len2 = glm::dot(ab, ab);
                            if (len2 < 1e-9) continue;
                            double u = glm::dot(glm::dvec2(wx, wy) - e.a, ab) / len2;
                            u = std::clamp(u, 0.0, 1.0);
                            // Leave a vertical wall at a tunnel portal: don't let the
                            // carve wrap past a portal endpoint.
                            if ((u <= 0.0 && e.aPortal) || (u >= 1.0 && e.bPortal))
                                continue;
                            const glm::dvec2 foot = e.a + ab * u;
                            const float d =
                                static_cast<float>(glm::length(glm::dvec2(wx, wy) - foot));
                            if (d > kSearchRadius) continue;
                            const float zt =
                                e.za + (e.zb - e.za) * static_cast<float>(u);
                            const float floorZ = zt - kFloorBelowTrack;
                            const float profileZ =
                                floorZ + std::max(0.0f, d - kFloorHalf) * kSlopeTan;
                            minProfile = std::min(minProfile, profileZ);
                            if (d < bestD) { bestD = d; ztBest = zt; }
                        }
                    }
                }

                if (!std::isfinite(bestD)) continue;         // no track nearby
                // Carve only where terrain rises above the trench profile. On flat
                // ground the track sits ~0.6 m above grade (bed offset), so the floor
                // (track_z - 0.2) is already above the ground and nothing is carved;
                // no separate "is this a cutting" gate is needed (a gate left low
                // mounds poking up around the ballast near tunnel approaches).
                const float nh = std::min(h, minProfile);
                if (nh < h) {
                    if (h - ztBest > deepest) {
                        deepest = h - ztBest;
                        deepestAt = glm::dvec3(wx - sceneOrigin.x, wy - sceneOrigin.y,
                                               nh - sceneOrigin.z);
                    }
                    h = nh;
                    ++carvedCells;
                }
            }
        }
    }

    std::string mh;
    for (const auto& [m, c] : mediumHist) {
        char b[32];
        std::snprintf(b, sizeof(b), " 0x%02X:%d", m, c);
        mh += b;
    }
    std::printf("[TerrainCarve] media{%s }, %zu surface segments, %zu edges, "
                "%zu portals; carved %zu cells; deepest cut %.1f m at scene "
                "(%.0f, %.0f, %.0f)\n",
                mh.c_str(), surface.size(), edges.size(), portals.size(),
                carvedCells, deepest, deepestAt.x, deepestAt.y, deepestAt.z);
}
