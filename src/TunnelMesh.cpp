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
#include <cstdio>
#include <cstdlib>
#include <unordered_set>

namespace {
// A tight older single-track blasted profile, as the Nordland Line has: near-vertical walls
// with an arch over, and no lining.
constexpr float kHalfW = 3.5f;      // m, wall to centreline
constexpr float kWallZ = 2.5f;      // m above rail level where the arch springs
constexpr float kCrownZ = 6.0f;     // m above rail level at the crown
constexpr float kFloorZ = -0.30f;   // just under the ballast base (which sits at -0.10)
constexpr int kArch = 8;            // arch segments
// The bore is a shell, not a surface: an outer skin this far out through the rock, closed off
// at each portal by an annulus. That thickness is where the terrain cut is allowed to land -
// clear of the inner wall, so nothing bites into the opening, and still covered from behind,
// so nothing shows daylight. It is what a blasted mouth looks like anyway: rock with an edge.
constexpr float kShell = 2.0f;
constexpr float kStepM = 6.0f;      // resample the centreline this finely
// An imported tunnel segment often starts *inside* the rock rather than at the face - by a
// couple of metres usually, six at worst here - so a bore built only over the segment stops
// short of daylight. The tube is extended outward at each end until the ground comes down to
// the rail, which is where the face is, and the terrain cut follows the tube exactly. Cut
// and tube have to be the same set: cut further than the tube reaches and the hillside opens
// onto nothing, which is a hole in the mountain around the mouth.
constexpr float kMaxExtendM = 40.0f;   // give up looking for daylight past this
constexpr float kFaceAboveRail = 2.0f; // ground this far over the rail still counts as rock
constexpr float kThroughM = 1.5f;      // and push the tube this bit further, through the face
// The mirror case: a segment can also run *past* the hillside, out over a drop, where the
// alignment carries on as tunnel but the ground has already fallen away. A tube built out
// there hangs in the air. Trim back to the rock instead.
constexpr float kMaxTrimM = 60.0f;
constexpr float kJoinM = 2.0f;         // segment ends this close are the same joint
// Rock and air get their own thresholds, with the face in the dead band between them: a
// portal in a shallow bank has ground only a metre over the rail, and must be left alone
// rather than read as either. Air means the ground has fallen below the bore floor.
constexpr float kAirBelowRail = -2.0f;
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
std::vector<glm::vec2> profile(float halfW, float crown, float floor) {
    std::vector<glm::vec2> r;
    r.push_back({-halfW, floor});
    for (int i = 0; i <= kArch; ++i) { // t from pi to 0: left springing, crown, right
        const float t = 3.14159265f * (1.0f - static_cast<float>(i) / kArch);
        r.push_back({halfW * std::cos(t), kWallZ + (crown - kWallZ) * std::sin(t)});
    }
    r.push_back({halfW, floor});
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
    const std::vector<glm::vec2> prof = profile(kHalfW, kCrownZ, kFloorZ);
    const std::vector<glm::vec2> profOut =
        profile(kHalfW + kShell, kCrownZ + kShell, kFloorZ - kShell);

    // A segment crossing a tile boundary is included in full in every tile it touches, so
    // take one copy per trackId or every tunnel gets built two or three times over.
    std::vector<std::vector<glm::vec3>> lines;
    std::unordered_set<std::uint32_t> seen;
    for (const Tile& t : data.tiles()) {
        for (const TrackSegment& seg : t.tracks) {
            if (seg.medium != 0x55 && seg.medium != 0x54) continue; // not underground
            if (!seen.insert(seg.trackId).second) continue;
            if (seg.pts.size() < 2) continue;
            std::vector<glm::vec3> line;
            for (const glm::dvec3& w : seg.pts)
                line.emplace_back(static_cast<float>(w.x - origin.x),
                                  static_cast<float>(w.y - origin.y),
                                  static_cast<float>(w.z - origin.z));
            lines.push_back(std::move(line));
        }
    }

    // One tunnel often arrives as several segments laid end to end - the alignment is cut
    // wherever the import's own bookkeeping changes, not where the rock does. Joining them
    // first is what makes the ends below *portals*: left apart, the shared joint deep inside
    // the mountain looks like two ends that need a face found for them, and there is none.
    for (bool merged = true; merged;) {
        merged = false;
        for (std::size_t i = 0; i < lines.size() && !merged; ++i)
            for (std::size_t j = i + 1; j < lines.size() && !merged; ++j) {
                auto& a = lines[i];
                auto& b = lines[j];
                // Neither is oriented, so try b both ways round against either end of a,
                // and leave it as found if it does not fit.
                for (int flipB = 0; flipB < 2 && !merged; ++flipB) {
                    if (flipB) std::reverse(b.begin(), b.end());
                    if (glm::distance(a.back(), b.front()) < kJoinM)
                        a.insert(a.end(), b.begin() + 1, b.end());
                    else if (glm::distance(a.front(), b.back()) < kJoinM)
                        a.insert(a.begin(), b.begin(), b.end() - 1);
                    else {
                        if (flipB) std::reverse(b.begin(), b.end());
                        continue;
                    }
                    lines.erase(lines.begin() + static_cast<long>(j));
                    merged = true;
                }
            }
    }

    for (const std::vector<glm::vec3>& chain : lines) {
        {
            // Resample the centreline: the import gives a handful of points over hundreds of
            // metres, and a bore swept that coarsely visibly cuts corners.
            std::vector<glm::vec3> line;
            for (const glm::vec3& p : chain) {
                if (line.empty()) { line.push_back(p); continue; }
                const float d = glm::distance(line.back(), p);
                if (d < 1e-3f) continue;
                const int n = std::max(1, static_cast<int>(d / kStepM));
                for (int k = 1; k <= n; ++k)
                    line.push_back(glm::mix(line.back(), p, static_cast<float>(k) / n));
            }
            if (line.size() < 2) continue;
            // Line each end up with the rock face. The imported segment rarely ends there:
            // usually it stops a few metres inside the hill, sometimes it runs on past the
            // hillside altogether. Ground above the rail means rock, below it means air.
            auto groundOver = [&](const glm::vec3& w, float railZ, float& over) {
                float g = 0.0f;
                if (!data.sampleGround(w.x + origin.x, w.y + origin.y, g)) return false;
                over = g - static_cast<float>(origin.z) - railZ;
                return true;
            };
            auto inRock = [&](const glm::vec3& w, float railZ) {
                float over = 0.0f;
                return groundOver(w, railZ, over) && over >= kFaceAboveRail;
            };
            auto inAir = [&](const glm::vec3& w, float railZ) {
                float over = 0.0f;
                return groundOver(w, railZ, over) && over <= kAirBelowRail;
            };
            for (const int end : {0, 1}) {
                const glm::vec3 tip = end ? line.back() : line.front();
                const glm::vec3 nb = end ? line[line.size() - 2] : line[1];
                const glm::vec3 dir = glm::normalize(tip - nb); // outward
                if (inRock(tip, tip.z)) {
                    // Buried: reach out to daylight, or the tube stops short of the face and
                    // the cut opens hillside it cannot fill.
                    float out = 0.0f;
                    for (float m = 1.0f; m <= kMaxExtendM; m += 1.0f) {
                        if (!inRock(tip + dir * m, tip.z)) break;
                        out = m;
                    }
                    out += kThroughM;
                    for (float m = kStepM; m < out; m += kStepM) {
                        const glm::vec3 w = tip + dir * m;
                        if (end) line.push_back(w); else line.insert(line.begin(), w);
                    }
                    const glm::vec3 w = tip + dir * out;
                    if (end) line.push_back(w); else line.insert(line.begin(), w);
                } else if (line.size() > 4 && inAir(tip, tip.z)) {
                    // Out in the air: trim back until the rock starts, leaving the same
                    // little overshoot through the face.
                    float in = 0.0f;
                    for (float m = 1.0f; m <= kMaxTrimM; m += 1.0f) {
                        in = m;
                        if (!inAir(tip - dir * m, tip.z)) break;
                    }
                    in = std::max(0.0f, in - kThroughM);
                    while (line.size() > 4) {
                        const glm::vec3& t2 = end ? line.back() : line.front();
                        if (glm::distance(t2, tip) >= in) break;
                        if (end) line.pop_back(); else line.erase(line.begin());
                    }
                }
            }
            ++bores_;
            // Centreline spans, for insideBore and its grid.
            for (std::size_t i = 1; i < line.size(); ++i) {
                lengthM_ += glm::length(glm::vec2(line[i] - line[i - 1]));
                const int idx = static_cast<int>(spans_.size());
                spans_.push_back({line[i - 1], line[i]});
                const float x0 = std::min(line[i - 1].x, line[i].x) - kHalfW - kShell;
                const float x1 = std::max(line[i - 1].x, line[i].x) + kHalfW + kShell;
                const float y0 = std::min(line[i - 1].y, line[i].y) - kHalfW - kShell;
                const float y1 = std::max(line[i - 1].y, line[i].y) + kHalfW + kShell;
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
                for (int side = 0; side < 2; ++side)
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
                    const glm::vec2 c = side ? profOut[k] : prof[k];
                    // Both skins take the same displacement, so the shell keeps its thickness
                    // and the outer one carries the same blasted shape as the wall it backs.
                    const glm::vec2 rad =
                        glm::normalize(prof[k] - glm::vec2(0.0f, (kWallZ + kCrownZ) * 0.5f));
                    const glm::vec2 q = c + rad * j;
                    TrackVertex v{};
                    v.pos = line[i] + right * q.x + up * q.y;
                    // The inner skin faces in, the outer one out; each is seen from its side.
                    const float sgn = side ? 1.0f : -1.0f;
                    v.normal = glm::normalize(right * (sgn * rad.x) + up * (sgn * rad.y));
                    v.color = kRock;
                    v.uv = {0.0f, 0.0f};
                    v.texLayer = kAmbientOnlyLayer;
                    vertices_.push_back(v);
                }
            }
            // Quads between consecutive rings, and the floor closing the loop. Wound both
            // ways: the wall is normally seen from inside, but wherever the terrain has been
            // cut it can also be seen from without, and a one-sided wall is see-through from
            // that side - a hole in the mountain looking straight out the other side of it.
            // The rock is shaded flat, so the second face costs nothing but its triangles.
            const int stride = 2 * ring;
            auto vtx = [&](std::size_t i, int side, int k) {
                return base + static_cast<std::uint32_t>(i * stride + side * ring + k);
            };
            auto quad = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c,
                            std::uint32_t d) {
                indices_.push_back(a); indices_.push_back(c); indices_.push_back(b);
                indices_.push_back(a); indices_.push_back(d); indices_.push_back(c);
                indices_.push_back(a); indices_.push_back(b); indices_.push_back(c);
                indices_.push_back(a); indices_.push_back(c); indices_.push_back(d);
            };
            for (std::size_t i = 0; i + 1 < line.size(); ++i)
                for (int side = 0; side < 2; ++side)
                    for (int k = 0; k < ring; ++k) {
                        const int k2 = (k + 1) % ring;
                        quad(vtx(i, side, k), vtx(i, side, k2), vtx(i + 1, side, k2),
                             vtx(i + 1, side, k));
                    }
            // Close the shell off at the portals, so the mouth reads as an edge of rock
            // rather than as a paper-thin wall seen end on.
            for (const std::size_t i : {std::size_t{0}, line.size() - 1})
                for (int k = 0; k < ring; ++k) {
                    const int k2 = (k + 1) % ring;
                    quad(vtx(i, 0, k), vtx(i, 0, k2), vtx(i, 1, k2), vtx(i, 1, k));
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
                const float sx0 = std::min(s.a.x, s.b.x) - kHalfW - kShell;
                const float sx1 = std::max(s.a.x, s.b.x) + kHalfW + kShell;
                const float sy0 = std::min(s.a.y, s.b.y) - kHalfW - kShell;
                const float sy1 = std::max(s.a.y, s.b.y) + kHalfW + kShell;
                if (x1 < sx0 || x0 > sx1 || y1 < sy0 || y0 > sy1) continue;
                return true;
            }
        }
    return false;
}

bool TunnelMesh::insideBore(const glm::vec3& p) const {
    const auto it = grid_.find(cellKey(p.x, p.y));
    if (it == grid_.end()) return false;
    // The cut lands in the shell: outside the inner wall, so no terrain is left biting into
    // the opening, and inside the outer skin, so whatever it opens is backed by rock rather
    // than by daylight. The terrain only drops a sub-triangle with *every* corner inside, so
    // the edge it actually leaves falls up to one of those short of this - which is why the
    // margin sits nearer the inner wall than the outer.
    constexpr float kMargin = 1.2f;
    for (const int si : it->second) {
        const Span& s = spans_[si];
        const glm::vec2 a(s.a), b(s.b), q(p);
        const glm::vec2 ab = b - a;
        const float L2 = glm::dot(ab, ab);
        if (L2 < 1e-9f) continue;
        const float tu = glm::dot(q - a, ab) / L2; // unclamped: negative or >1 is past an end
        const float t = std::clamp(tu, 0.0f, 1.0f);
        // Across the bore, measured to the axis rather than to the segment, so reaching past
        // a portal stays a straight continuation of the tube instead of curling round its end.
        // Clamped to the span, so the cut stops where the tube does. The tube already
        // reaches the face, so there is nothing to gain by reaching further - and plenty to
        // lose: the approach trench lies at rail level on this same centreline.
        if (glm::length(q - (a + ab * t)) > kHalfW + kMargin) continue;
        const float railZ = glm::mix(s.a.z, s.b.z, t);
        // Between floor and crown. Below the crown is what leaves the hillside whole over
        // the body of a tunnel; above the floor keeps a bore that runs out over a valley
        // from cutting the ground far beneath it.
        if (p.z <= railZ + kCrownZ + kMargin && p.z >= railZ + kFloorZ - 2.0f) return true;
    }
    return false;
}
