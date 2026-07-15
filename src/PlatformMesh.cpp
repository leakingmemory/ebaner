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

#include "PlatformMesh.h"

#include "EarClip.h"
#include "TerrainData.h"
#include "TrackPath.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <unordered_set>
#include <vector>

namespace {

// Concrete greys: walls a touch darker than the top surface.
const glm::vec3 kWallColor(0.60f, 0.60f, 0.62f);
const glm::vec3 kTopColor(0.72f, 0.72f, 0.74f);

// The rail head sits this far above the track centreline z in the rendered
// track cross-section (mirrors kRailTopZ in TrackMesh.cpp).
constexpr float kRailHeadAboveCentreline = 0.76f;
// Norwegian platforms are built to 0.76 m above top-of-rail (modern standard).
constexpr float kPlatformAboveRail = 0.76f;
// A platform only references a track whose centreline passes within this
// horizontal distance of the platform end (edge-to-centreline is a few metres).
constexpr float kTrackSearchRadius = 12.0f;

// Rail-head elevation (scene-relative z) of the nearest track centreline to
// `pt` (scene-relative x,y), searching all paths. Returns false if none is
// within kTrackSearchRadius.
bool nearestRailHeadZ(const glm::vec2& pt, const std::vector<TrackPath>& paths,
                      float& railZ) {
    float bestD2 = kTrackSearchRadius * kTrackSearchRadius;
    bool found = false;
    for (const TrackPath& path : paths) {
        for (float s = 0.0f; s <= path.length(); s += 2.0f) {
            const glm::vec3 c = path.poseAt(s).pos;
            const float dx = c.x - pt.x, dy = c.y - pt.y;
            const float d2 = dx * dx + dy * dy;
            if (d2 < bestD2) {
                bestD2 = d2;
                railZ = c.z + kRailHeadAboveCentreline;
                found = true;
            }
        }
    }
    return found;
}

std::uint64_t hashPlatform(const PlatformSegment& p) {
    auto q = [](double v) { return static_cast<std::int64_t>(std::llround(v * 10.0)); };
    std::uint64_t h = 1469598103934665603ull;
    auto mix = [&](std::uint64_t x) { h ^= x; h *= 1099511628211ull; };
    mix(p.footprint.size());
    double cx = 0, cy = 0;
    for (const glm::dvec2& v : p.footprint) { cx += v.x; cy += v.y; }
    const double inv = p.footprint.empty() ? 0.0 : 1.0 / p.footprint.size();
    mix(static_cast<std::uint64_t>(q(cx * inv)));
    mix(static_cast<std::uint64_t>(q(cy * inv)));
    mix(static_cast<std::uint64_t>(q(p.footprint.front().x)));
    mix(static_cast<std::uint64_t>(q(p.footprint.front().y)));
    return h;
}

} // namespace

void PlatformMesh::build(const TerrainData& data,
                         const std::vector<TrackPath>& paths) {
    const glm::dvec3 origin = data.sceneOrigin();

    std::unordered_set<std::uint64_t> seen;
    std::vector<const PlatformSegment*> uniq;
    for (const Tile& t : data.tiles())
        for (const PlatformSegment& p : t.platforms) {
            if (p.footprint.size() < 3) continue;
            if (seen.insert(hashPlatform(p)).second) uniq.push_back(&p);
        }

    // Quad with an outward normal (flipped to face away from `inside`).
    auto emitQuad = [&](const glm::vec3& p0, const glm::vec3& p1,
                        const glm::vec3& p2, const glm::vec3& p3,
                        const glm::vec3& inside, const glm::vec3& color) {
        glm::vec3 nrm = glm::cross(p1 - p0, p3 - p0);
        const float nl = glm::length(nrm);
        nrm = (nl > 1e-9f) ? nrm / nl : glm::vec3(0.0f, 0.0f, 1.0f);
        const glm::vec3 cen = (p0 + p1 + p2 + p3) * 0.25f;
        if (glm::dot(nrm, cen - inside) < 0.0f) nrm = -nrm;
        const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
        const glm::vec2 uv(0.0f);
        vertices_.push_back({p0, nrm, color, uv, -1.0f});
        vertices_.push_back({p1, nrm, color, uv, -1.0f});
        vertices_.push_back({p2, nrm, color, uv, -1.0f});
        vertices_.push_back({p3, nrm, color, uv, -1.0f});
        indices_.push_back(base + 0);
        indices_.push_back(base + 1);
        indices_.push_back(base + 2);
        indices_.push_back(base + 0);
        indices_.push_back(base + 2);
        indices_.push_back(base + 3);
    };
    auto emitTopTri = [&](const glm::vec3& p0, const glm::vec3& p1,
                          const glm::vec3& p2) {
        // Top surface faces straight up.
        const glm::vec3 nrm(0.0f, 0.0f, 1.0f);
        const glm::vec2 uv(0.0f);
        const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
        vertices_.push_back({p0, nrm, kTopColor, uv, -1.0f});
        vertices_.push_back({p1, nrm, kTopColor, uv, -1.0f});
        vertices_.push_back({p2, nrm, kTopColor, uv, -1.0f});
        indices_.push_back(base + 0);
        indices_.push_back(base + 1);
        indices_.push_back(base + 2);
    };

    for (const PlatformSegment* pp : uniq) {
        const PlatformSegment& p = *pp;
        const int n = static_cast<int>(p.footprint.size());

        // Scene-relative footprint + heights.
        std::vector<glm::vec2> fp(n);
        glm::vec2 centroid(0.0f);
        for (int i = 0; i < n; ++i) {
            fp[i] = glm::vec2(static_cast<float>(p.footprint[i].x - origin.x),
                              static_cast<float>(p.footprint[i].y - origin.y));
            centroid += fp[i];
        }
        centroid /= static_cast<float>(n);

        // Ground under the footprint (scene-relative z): the walls reach down to
        // the lowest so nothing floats, and the top must clear the highest so it
        // never sinks into rising terrain. Sample the loaded terrain densely
        // along the edges (ring vertices alone miss the long-edge maxima).
        float gmin = 1e30f, gmax = -1e30f;
        bool anyGround = false;
        for (int i = 0; i < n; ++i) {
            const glm::dvec2& wa = p.footprint[i];
            const glm::dvec2& wc = p.footprint[(i + 1) % n];
            const double len = glm::length(wc - wa);
            const int steps = std::max(1, static_cast<int>(len / 5.0));
            for (int s = 0; s <= steps; ++s) {
                const glm::dvec2 w = wa + (wc - wa) * (double(s) / steps);
                float e;
                if (data.sampleGround(w.x, w.y, e)) {
                    gmin = std::min(gmin, e);
                    gmax = std::max(gmax, e);
                    anyGround = true;
                }
            }
        }
        const float gminS = static_cast<float>(
            (anyGround ? gmin : p.baseZ) - origin.z);
        const float gmaxS = static_cast<float>(
            (anyGround ? gmax : p.baseZ) - origin.z);

        // The correct datum for a platform is the adjacent *rail*, not the
        // terrain: it is built a fixed step height above top-of-rail. Find the
        // two ends (the farthest-apart footprint vertices) and use the lowest
        // neighbouring rail head; the flat top sits kPlatformAboveRail above it.
        int e0 = 0, e1 = 0;
        float far2 = -1.0f;
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j) {
                const float d2 = glm::dot(fp[i] - fp[j], fp[i] - fp[j]);
                if (d2 > far2) { far2 = d2; e0 = i; e1 = j; }
            }
        float railZ = std::numeric_limits<float>::max();
        bool haveRail = false;
        for (int end : {e0, e1}) {
            float rz;
            if (nearestRailHeadZ(fp[end], paths, rz)) {
                railZ = std::min(railZ, rz);
                haveRail = true;
            }
        }

        const float zb = gminS;
        // Never let the top sink under the terrain (keep a little clearance).
        const float zt = haveRail
            ? std::max(railZ + kPlatformAboveRail, gmaxS + 0.05f)
            : gmaxS + p.height; // isolated platform: fall back to terrain
        const glm::vec3 inside(centroid.x, centroid.y, (zb + zt) * 0.5f);

        // Walls: one quad per footprint edge.
        for (int i = 0; i < n; ++i) {
            const glm::vec2& a = fp[i];
            const glm::vec2& c = fp[(i + 1) % n];
            emitQuad(glm::vec3(a.x, a.y, zb), glm::vec3(c.x, c.y, zb),
                     glm::vec3(c.x, c.y, zt), glm::vec3(a.x, a.y, zt), inside,
                     kWallColor);
        }

        // Flat top slab.
        const std::vector<int> tris = earClipTriangulate(fp);
        for (std::size_t k = 0; k + 3 <= tris.size(); k += 3)
            emitTopTri(glm::vec3(fp[tris[k]], zt), glm::vec3(fp[tris[k + 1]], zt),
                       glm::vec3(fp[tris[k + 2]], zt));
    }

    std::printf("[PlatformMesh] %zu platforms, %zu vertices, %zu triangles\n",
                uniq.size(), vertices_.size(), indices_.size() / 3);
}
