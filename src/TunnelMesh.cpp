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

#include "TunnelMesh.h"

#include "TerrainData.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace {
// A tight older single-track blasted profile, as the Nordland Line has: near-vertical walls
// with an arch over, and no lining.
constexpr float kHalfW = 3.5f;      // m, wall to centreline
constexpr float kWallZ = 2.5f;      // m above rail level where the arch springs
constexpr float kCrownZ = 6.0f;     // m above rail level at the crown
constexpr float kFloorZ = -0.30f;   // just under the ballast base (which sits at -0.10)
constexpr int kArch = 8;            // arch segments
constexpr float kStepM = 6.0f;      // resample the centreline this finely
constexpr float kWobble = 0.28f;    // m; blasted rock is not a swept pipe
constexpr float kAmbientOnlyLayer = -5.0f; // texLayer: no sun term (see track.frag)

const glm::vec3 kRock{0.46f, 0.43f, 0.40f};

// Deterministic hash -> [-1, 1], so the wobble is stable across runs and rebuilds.
float wob(int a, int b) {
    std::uint32_t h = static_cast<std::uint32_t>(a) * 0x9E3779B9u ^
                      static_cast<std::uint32_t>(b) * 0x85EBCA6Bu;
    h ^= h >> 15;
    h *= 0x2545F491u;
    h ^= h >> 13;
    return static_cast<float>(h & 0xFFFFu) / 32767.5f - 1.0f;
}

// The bore's cross-section, as (across, up) offsets from the rail-level centre. Ordered round
// the ring: up the left wall, over the arch, down the right wall, and the floor closes it.
std::vector<glm::vec2> profile() {
    std::vector<glm::vec2> r;
    r.push_back({-kHalfW, kFloorZ});
    for (int i = 0; i <= kArch; ++i) { // t from pi to 0: left springing, crown, right
        const float t = 3.14159265f * (1.0f - static_cast<float>(i) / kArch);
        r.push_back({kHalfW * std::cos(t), kWallZ + (kCrownZ - kWallZ) * std::sin(t)});
    }
    r.push_back({kHalfW, kFloorZ});
    return r;
}

constexpr float kCell = 32.0f; // m, the span-lookup grid
std::int64_t cellKey(float x, float y) {
    const std::int64_t cx = static_cast<std::int64_t>(std::floor(x / kCell));
    const std::int64_t cy = static_cast<std::int64_t>(std::floor(y / kCell));
    return (cx << 32) ^ (cy & 0xFFFFFFFF);
}
} // namespace

void TunnelMesh::build(const TerrainData& data) {
    vertices_.clear();
    indices_.clear();
    spans_.clear();
    grid_.clear();
    bores_ = 0;
    lengthM_ = 0.0f;

    const glm::dvec3 origin = data.sceneOrigin();
    const std::vector<glm::vec2> prof = profile();

    // A segment crossing a tile boundary is included in full in every tile it touches, so
    // take one copy per trackId or every tunnel gets built two or three times over.
    std::unordered_set<std::uint32_t> seen;
    for (const Tile& t : data.tiles()) {
        for (const TrackSegment& seg : t.tracks) {
            if (seg.medium != 0x55 && seg.medium != 0x54) continue; // not underground
            if (!seen.insert(seg.trackId).second) continue;
            if (seg.pts.size() < 2) continue;

            // Resample the centreline: the import gives a handful of points over hundreds of
            // metres, and a bore swept that coarsely visibly cuts corners.
            std::vector<glm::vec3> line;
            for (const glm::dvec3& w : seg.pts) {
                const glm::vec3 p(static_cast<float>(w.x - origin.x),
                                  static_cast<float>(w.y - origin.y),
                                  static_cast<float>(w.z - origin.z));
                if (line.empty()) { line.push_back(p); continue; }
                const float d = glm::distance(line.back(), p);
                if (d < 1e-3f) continue;
                const int n = std::max(1, static_cast<int>(d / kStepM));
                for (int k = 1; k <= n; ++k)
                    line.push_back(glm::mix(line.back(), p, static_cast<float>(k) / n));
            }
            if (line.size() < 2) continue;
            ++bores_;

            // Centreline spans, for insideBore and its grid.
            for (std::size_t i = 1; i < line.size(); ++i) {
                lengthM_ += glm::length(glm::vec2(line[i] - line[i - 1]));
                const int idx = static_cast<int>(spans_.size());
                spans_.push_back({line[i - 1], line[i]});
                const float x0 = std::min(line[i - 1].x, line[i].x) - kHalfW;
                const float x1 = std::max(line[i - 1].x, line[i].x) + kHalfW;
                const float y0 = std::min(line[i - 1].y, line[i].y) - kHalfW;
                const float y1 = std::max(line[i - 1].y, line[i].y) + kHalfW;
                for (float x = x0; x <= x1 + kCell; x += kCell)
                    for (float y = y0; y <= y1 + kCell; y += kCell)
                        grid_[cellKey(x, y)].push_back(idx);
            }

            // Sweep the profile. The frame keeps world up rather than following the rail's
            // own up: a tunnel is near level, and a twisting frame would only wobble the
            // crown for no gain.
            const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
            const int ring = static_cast<int>(prof.size());
            for (std::size_t i = 0; i < line.size(); ++i) {
                const glm::vec3 fwd = glm::normalize(
                    line[std::min(i + 1, line.size() - 1)] - line[i > 0 ? i - 1 : 0]);
                glm::vec3 right(fwd.y, -fwd.x, 0.0f);
                if (const float L = glm::length(right); L > 1e-6f) right /= L;
                const glm::vec3 up(0.0f, 0.0f, 1.0f);
                for (int k = 0; k < ring; ++k) {
                    // Blasted rock is not a swept pipe: push each point off the profile by a
                    // little, varying along the bore as well as around it. The floor is left
                    // alone so the ballast still sits flat on it.
                    // The floor stays flat so the ballast sits on it, and the two end rings
                    // stay true to the profile: the terrain cut is made to the profile, so a
                    // mouth ring pulled inward by the wobble would leave daylight around it.
                    const bool fixed = k == 0 || k == ring - 1 || i == 0 ||
                                       i + 1 == line.size();
                    const float j = fixed ? 0.0f : kWobble * wob(static_cast<int>(i), k);
                    const glm::vec2 c = prof[k];
                    const glm::vec2 rad =
                        glm::normalize(c - glm::vec2(0.0f, (kWallZ + kCrownZ) * 0.5f));
                    const glm::vec2 q = c + rad * j;
                    TrackVertex v{};
                    v.pos = line[i] + right * q.x + up * q.y;
                    // Facing in: a bore is only ever seen from inside it.
                    v.normal = glm::normalize(right * -rad.x + up * -rad.y);
                    v.color = kRock;
                    v.uv = {0.0f, 0.0f};
                    v.texLayer = kAmbientOnlyLayer;
                    vertices_.push_back(v);
                }
            }
            // Inward-facing quads between consecutive rings, and the floor closing the loop.
            for (std::size_t i = 0; i + 1 < line.size(); ++i) {
                for (int k = 0; k < ring; ++k) {
                    const int k2 = (k + 1) % ring;
                    const std::uint32_t a = base + static_cast<std::uint32_t>(i * ring + k);
                    const std::uint32_t b = base + static_cast<std::uint32_t>(i * ring + k2);
                    const std::uint32_t c = base + static_cast<std::uint32_t>((i + 1) * ring + k2);
                    const std::uint32_t d = base + static_cast<std::uint32_t>((i + 1) * ring + k);
                    indices_.push_back(a); indices_.push_back(c); indices_.push_back(b);
                    indices_.push_back(a); indices_.push_back(d); indices_.push_back(c);
                }
            }
        }
    }

}

bool TunnelMesh::nearBore(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) const {
    if (grid_.empty()) return false;
    const float x0 = std::min({a.x, b.x, c.x}), x1 = std::max({a.x, b.x, c.x});
    const float y0 = std::min({a.y, b.y, c.y}), y1 = std::max({a.y, b.y, c.y});
    const float z0 = std::min({a.z, b.z, c.z});
    // Cheap in plan, then the part that actually matters: a triangle only needs subdividing
    // where it dips into the bore's own height. Over the body of a tunnel the hillside is
    // tens of metres above the crown, and subdividing all of that would cost hundreds of
    // thousands of vertices to cut nothing.
    for (float x = x0; x <= x1 + kCell; x += kCell)
        for (float y = y0; y <= y1 + kCell; y += kCell) {
            const auto it = grid_.find(cellKey(x, y));
            if (it == grid_.end()) continue;
            for (const int si : it->second) {
                const Span& s = spans_[si];
                if (z0 > std::max(s.a.z, s.b.z) + kCrownZ + 1.0f) continue;
                const float sx0 = std::min(s.a.x, s.b.x) - kHalfW - 1.0f;
                const float sx1 = std::max(s.a.x, s.b.x) + kHalfW + 1.0f;
                const float sy0 = std::min(s.a.y, s.b.y) - kHalfW - 1.0f;
                const float sy1 = std::max(s.a.y, s.b.y) + kHalfW + 1.0f;
                if (x1 < sx0 || x0 > sx1 || y1 < sy0 || y0 > sy1) continue;
                return true;
            }
        }
    return false;
}

bool TunnelMesh::insideBore(const glm::vec3& p) const {
    const auto it = grid_.find(cellKey(p.x, p.y));
    if (it == grid_.end()) return false;
    // Negative on purpose: the cut has to fall *inside* the tube, not outside it. Cut even
    // slightly wide and there is a seam where the terrain has gone but the wall does not
    // reach, and daylight comes through the hillside. Only slightly, though - undercut hard
    // and the rock face closes over the bottom of the mouth. The end rings are unwobbled, so
    // a few centimetres is enough.
    constexpr float kMargin = -0.15f;
    for (const int si : it->second) {
        const Span& s = spans_[si];
        const glm::vec2 a(s.a), b(s.b), q(p);
        const glm::vec2 ab = b - a;
        const float L2 = glm::dot(ab, ab);
        const float t = L2 > 1e-9f ? std::clamp(glm::dot(q - a, ab) / L2, 0.0f, 1.0f) : 0.0f;
        // Clamping to the span is also the length bound: a point past the portal projects to
        // the end and is only accepted if it is within half a bore-width of it. Without that
        // the trench in front of every tunnel - at rail level, on the centreline - would be
        // punched out along with the mouth.
        if (glm::length(q - (a + ab * t)) > kHalfW + kMargin) continue;
        // And below the crown, which is what leaves the hillside whole over the tunnel body.
        const float railZ = glm::mix(s.a.z, s.b.z, t);
        if (p.z <= railZ + kCrownZ + kMargin) return true;
    }
    return false;
}
