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

#include "RoadMesh.h"

#include "TerrainData.h"

#include <cmath>
#include <cstdio>
#include <unordered_set>
#include <vector>

namespace {

struct RoadStyle {
    float width;      // ribbon width, metres
    glm::vec3 color;  // asphalt tint
    float lift;       // metres above the DTM (avoids z-fighting)
    bool priv;        // private (de-emphasised) road
};

// Realistic asphalt: darker/wider for the important classes, muted tan-grey and
// thin for private tracks (de-emphasised).
RoadStyle styleFor(std::uint8_t kategori) {
    switch (kategori) {
        case 'E': return {11.0f, {0.26f, 0.26f, 0.28f}, 0.35f, false};
        case 'R': return {9.0f, {0.30f, 0.30f, 0.32f}, 0.35f, false};
        case 'F': return {7.0f, {0.34f, 0.34f, 0.35f}, 0.32f, false};
        case 'K': return {5.5f, {0.38f, 0.38f, 0.39f}, 0.30f, false};
        case 'P': return {2.5f, {0.46f, 0.44f, 0.40f}, 0.15f, true};
        default:  return {5.0f, {0.38f, 0.38f, 0.39f}, 0.30f, false};
    }
}

// Stable geometry hash for dedup (roads have no unique id; a through-road appears
// in full in every tile it touches).
std::uint64_t hashRoad(const RoadSegment& r) {
    auto q = [](double v) { return static_cast<std::int64_t>(std::llround(v * 10.0)); };
    std::uint64_t h = 1469598103934665603ull;
    auto mix = [&](std::uint64_t x) { h ^= x; h *= 1099511628211ull; };
    mix(r.kategori);
    mix(r.nummer);
    mix(r.pts.size());
    mix(static_cast<std::uint64_t>(q(r.pts.front().x)));
    mix(static_cast<std::uint64_t>(q(r.pts.front().y)));
    mix(static_cast<std::uint64_t>(q(r.pts.back().x)));
    mix(static_cast<std::uint64_t>(q(r.pts.back().y)));
    return h;
}

} // namespace

void RoadMesh::build(const TerrainData& data) {
    const glm::dvec3 origin = data.sceneOrigin();

    // Collect unique roads across tiles (dedup by geometry).
    std::unordered_set<std::uint64_t> seen;
    std::vector<const RoadSegment*> uniq;
    for (const Tile& t : data.tiles())
        for (const RoadSegment& r : t.roads) {
            if (r.pts.size() < 2) continue;
            if (seen.insert(hashRoad(r)).second) uniq.push_back(&r);
        }

    // Flat mitred ribbon for one road centreline (scene-relative, given lift).
    auto emitRibbon = [&](const RoadSegment& r, const RoadStyle& st) {
        const float hw = st.width * 0.5f;

        // Sanitise z: some GML vertices carry a NODATA sentinel (~-1e6) that would
        // make the ribbon plunge far below the terrain. Replace any implausible z
        // with the nearest valid one (forward-fill, seeded by the first valid).
        auto validZ = [](double zz) { return zz > -500.0 && zz < 9000.0; };
        const int np = static_cast<int>(r.pts.size());
        int firstValid = -1;
        for (int i = 0; i < np; ++i)
            if (validZ(r.pts[i].z)) { firstValid = i; break; }
        if (firstValid < 0) return; // road has no usable elevation

        std::vector<glm::vec3> c;
        c.reserve(np);
        double carry = r.pts[firstValid].z;
        for (int i = 0; i < np; ++i) {
            if (validZ(r.pts[i].z)) carry = r.pts[i].z;
            c.emplace_back(static_cast<float>(r.pts[i].x - origin.x),
                           static_cast<float>(r.pts[i].y - origin.y),
                           static_cast<float>(carry - origin.z) + st.lift);
        }

        const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
        const int n = static_cast<int>(c.size());
        for (int i = 0; i < n; ++i) {
            glm::vec2 tang(0.0f);
            if (i > 0) tang += glm::vec2(c[i].x - c[i - 1].x, c[i].y - c[i - 1].y);
            if (i < n - 1)
                tang += glm::vec2(c[i + 1].x - c[i].x, c[i + 1].y - c[i].y);
            const float tl = glm::length(tang);
            const glm::vec2 perp =
                (tl > 1e-6f) ? glm::vec2(-tang.y, tang.x) / tl : glm::vec2(1.0f, 0.0f);
            const glm::vec3 off(perp.x * hw, perp.y * hw, 0.0f);
            const glm::vec3 up(0.0f, 0.0f, 1.0f);
            const glm::vec2 uv(0.0f);
            vertices_.push_back({c[i] + off, up, st.color, uv, -1.0f}); // left
            vertices_.push_back({c[i] - off, up, st.color, uv, -1.0f}); // right
        }
        for (int i = 0; i + 1 < n; ++i) {
            const std::uint32_t l0 = base + 2 * i, r0 = l0 + 1;
            const std::uint32_t l1 = base + 2 * (i + 1), r1 = l1 + 1;
            indices_.push_back(l0);
            indices_.push_back(r0);
            indices_.push_back(r1);
            indices_.push_back(l0);
            indices_.push_back(r1);
            indices_.push_back(l1);
        }
    };

    // Emit private first, then public, so the public network draws on top at
    // junctions (reinforced by the higher public lift).
    for (int pass = 0; pass < 2; ++pass) {
        const bool wantPrivate = (pass == 0);
        for (const RoadSegment* r : uniq) {
            const RoadStyle st = styleFor(r->kategori);
            if (st.priv == wantPrivate) emitRibbon(*r, st);
        }
    }

    std::printf("[RoadMesh] %zu roads, %zu vertices, %zu triangles\n",
                uniq.size(), vertices_.size(), indices_.size() / 3);
}
