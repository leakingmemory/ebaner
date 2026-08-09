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

#include "TerrainMesh.h"

#include "TunnelMesh.h"

#include "TerrainData.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <unordered_map>

namespace {
constexpr int P = TerrainData::PIXELS; // 256
constexpr double kRes[4] = {10.0, 20.0, 40.0, 80.0};

inline bool isNodata(float h) { return h <= TerrainData::NODATA + 1.0f; }

// Packs (lod,col,row) into a 64-bit key for the tile index.
inline std::uint64_t keyOf(int lod, int col, int row) {
    return (std::uint64_t(std::uint32_t(lod)) << 40) |
           (std::uint64_t(std::uint32_t(col) & 0xFFFFF) << 20) |
           std::uint64_t(std::uint32_t(row) & 0xFFFFF);
}
} // namespace

void TerrainMesh::build(const TerrainData& data, const TunnelMesh* bores) {
    // Reset accumulators so build() is idempotent (the editor rebuilds to re-preview).
    vertices_.clear();
    indices_.clear();
    dropped_ = 0;
    for (const auto& [key, t] : data.tiles()) appendTile(data, t.get(), bores);
    std::printf("[TerrainMesh] %zu vertices, %zu indices; %zu sub-triangles cut for "
                "tunnel mouths\n",
                vertices_.size(), indices_.size(), dropped_);
}

void TerrainMesh::buildTile(const TerrainData& data, const Tile& tile,
                            const TunnelMesh* bores) {
    vertices_.clear();
    indices_.clear();
    dropped_ = 0;
    appendTile(data, &tile, bores);
}

// One tile's contribution: its own interior, plus the seams and corners it owns.
//
// Every triangle in the terrain belongs to exactly one tile. The interior is obviously
// the tile's own; a seam is built by whichever of the two neighbours owns it (the finer,
// or N/E on a same-LOD tie), and a corner by the finest of its four quadrants. What makes
// that enough to build a tile alone is that the seam and corner passes emit their own
// vertices for the *neighbour* side rather than indexing into the neighbour's - so a
// tile's triangles only ever reference vertices this call appended.
void TerrainMesh::appendTile(const TerrainData& data, const Tile* a,
                             const TunnelMesh* bores) {
    const glm::dvec3 origin = data.sceneOrigin();
    // Every triangle goes through here so the tunnel test is applied in one place: a
    // triangle whose centre stands inside a bore is what would otherwise seal the mouth.

    // A terrain cell is 10 m and a bore is 7 m across, so dropping whole triangles would take
    // away far more hillside than the mouth and leave a gap nothing can plausibly fill. The
    // few triangles that actually stand at a portal are subdivided first, and only the parts
    // inside the bore are dropped - which cuts the opening to the shape of the tunnel.
    // Sub-triangles of about a metre, whatever the cell size: the edge of the opening is
    // only ever as smooth as this, and a portal is worth a metre.
    constexpr float kSubTargetM = 1.0f;
    auto emitTri = [&](std::uint32_t i0, std::uint32_t i1, std::uint32_t i2) {
        auto keep = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
            indices_.push_back(a);
            indices_.push_back(b);
            indices_.push_back(c);
        };
        if (!bores || !bores->nearBore(vertices_[i0].pos, vertices_[i1].pos,
                                       vertices_[i2].pos)) {
            keep(i0, i1, i2);
            return;
        }
        const Vertex v0 = vertices_[i0], v1 = vertices_[i1], v2 = vertices_[i2];
        const float edge = std::max({glm::distance(v0.pos, v1.pos),
                                     glm::distance(v1.pos, v2.pos),
                                     glm::distance(v2.pos, v0.pos)});
        const int kSub = std::clamp(static_cast<int>(edge / kSubTargetM), 8, 24);
        auto lerp3 = [&](float u, float v) { // barycentric (u along v0->v1, v along v0->v2)
            const float w = 1.0f - u - v;
            Vertex r;
            r.pos = v0.pos * w + v1.pos * u + v2.pos * v;
            r.normal = glm::normalize(v0.normal * w + v1.normal * u + v2.normal * v);
            r.elevation = v0.elevation * w + v1.elevation * u + v2.elevation * v;
            // Not interpolated: the terrain shader takes landcover flat, from the
            // provoking vertex, so the whole original triangle draws as one class. Giving
            // every sub-vertex that same class is what makes a subdivided triangle shade
            // identically to the one it replaced - blending it here would hand the pieces
            // classes of their own and fringe the mouth with whatever they landed on.
            r.landcover = v0.landcover;
            return r;
        };
        // Barycentric grid of (kSub+1)(kSub+2)/2 points, rows u+v <= 1.
        std::vector<std::uint32_t> gi;
        gi.reserve((kSub + 1) * (kSub + 2) / 2);
        for (int r = 0; r <= kSub; ++r)
            for (int q = 0; q + r <= kSub; ++q) {
                gi.push_back(static_cast<std::uint32_t>(vertices_.size()));
                vertices_.push_back(lerp3(static_cast<float>(q) / kSub,
                                          static_cast<float>(r) / kSub));
            }
        auto at = [&](int q, int r) { // index into the triangular grid
            const int off = r * (kSub + 1) - (r * (r - 1)) / 2;
            return gi[off + q];
        };
        // All three corners, not the centre. A sub-triangle is still a metre across, and one
        // whose centre is just inside the bore reaches a good part of that metre outside it -
        // dropping it takes away terrain the tube does not reach, and daylight comes through
        // the seam. Requiring the whole piece to be inside leaves the rock overlapping the
        // wall by up to a sub-triangle instead, which is invisible.
        auto allIn = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c) {
            return bores->insideBore(vertices_[a].pos) && bores->insideBore(vertices_[b].pos) &&
                   bores->insideBore(vertices_[c].pos);
        };
        for (int r = 0; r < kSub; ++r)
            for (int q = 0; q + r < kSub; ++q) {
                const std::uint32_t a = at(q, r), b = at(q + 1, r), c = at(q, r + 1);
                if (allIn(a, b, c)) ++dropped_;
                else keep(a, b, c);
                if (q + r + 1 >= kSub) continue;
                const std::uint32_t d = at(q + 1, r + 1);
                if (allIn(b, d, c)) ++dropped_;
                else keep(b, d, c);
            }
    };

    auto heightOf = [&](const Tile* t, int r, int c) -> float {
        return t->heights[static_cast<std::size_t>(r) * P + c];
    };
    auto lcOf = [&](const Tile* t, int r, int c) -> float {
        if (t->landcover.empty()) return 0.0f;
        return static_cast<float>(t->landcover[static_cast<std::size_t>(r) * P + c]);
    };
    // Finest loaded tile covering a world point (LODs form an overlapping pyramid).
    auto ownerAt = [&](double wx, double wy) -> const Tile* {
        for (int lod = 0; lod < 4; ++lod) {
            const double E = P * kRes[lod];
            const Tile* t = data.tileAt(lod, static_cast<int>(std::floor(wx / E)),
                                        static_cast<int>(std::floor(wy / E)));
            if (t) return t;
        }
        return nullptr;
    };
    auto tileNormal = [&](const Tile* t, int row, int col) -> glm::vec3 {
        const double res = t->resolution;
        const float z = heightOf(t, row, col);
        auto s = [&](int r, int c) -> float {
            r = std::clamp(r, 0, P - 1);
            c = std::clamp(c, 0, P - 1);
            const float v = heightOf(t, r, c);
            return isNodata(v) ? z : v;
        };
        const float dzdx = (s(row, col + 1) - s(row, col - 1)) / float(2.0 * res);
        const float dzdy = (s(row - 1, col) - s(row + 1, col)) / float(2.0 * res);
        return glm::normalize(glm::vec3(-dzdx, -dzdy, 1.0f));
    };
    // Bilinear surface sample of a tile at a world point; false if any nodata.
    auto sampleAt = [&](const Tile* t, double x, double y, float& z,
                        glm::vec3& n) -> bool {
        const double fc = (x - t->originX) / t->resolution - 0.5;
        const double fr = (t->originY + t->extent - y) / t->resolution - 0.5;
        const int c0 = static_cast<int>(std::floor(fc));
        const int r0 = static_cast<int>(std::floor(fr));
        const double tx = fc - c0, ty = fr - r0;
        const int c0c = std::clamp(c0, 0, P - 1), c1c = std::clamp(c0 + 1, 0, P - 1);
        const int r0c = std::clamp(r0, 0, P - 1), r1c = std::clamp(r0 + 1, 0, P - 1);
        const float h00 = heightOf(t, r0c, c0c), h01 = heightOf(t, r0c, c1c);
        const float h10 = heightOf(t, r1c, c0c), h11 = heightOf(t, r1c, c1c);
        if (isNodata(h00) || isNodata(h01) || isNodata(h10) || isNodata(h11))
            return false;
        const float top = h00 + (h01 - h00) * float(tx);
        const float bot = h10 + (h11 - h10) * float(tx);
        z = top + (bot - top) * float(ty);
        n = tileNormal(t, r0c, c0c);
        return true;
    };
    auto worldX = [&](const Tile* t, int col) {
        return t->originX + (col + 0.5) * t->resolution;
    };
    auto worldY = [&](const Tile* t, int row) {
        return t->originY + t->extent - (row + 0.5) * t->resolution;
    };
    auto emit = [&](double wx, double wy, float z, glm::vec3 n,
                    float lc) -> std::uint32_t {
        Vertex v;
        v.pos = glm::vec3(static_cast<float>(wx - origin.x),
                          static_cast<float>(wy - origin.y),
                          static_cast<float>(z - origin.z));
        v.normal = n;
        v.elevation = z;
        v.landcover = lc;
        const std::uint32_t idx = static_cast<std::uint32_t>(vertices_.size());
        vertices_.push_back(v);
        return idx;
    };

    // --- Pass 1: interior vertices + de-overlapped quads ---
    const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
    for (int row = 0; row < P; ++row)
        for (int col = 0; col < P; ++col)
            emit(worldX(a, col), worldY(a, row), heightOf(a, row, col),
                 tileNormal(a, row, col), lcOf(a, row, col));

    for (int row = 0; row < P - 1; ++row) {
        for (int col = 0; col < P - 1; ++col) {
            const float h00 = heightOf(a, row, col);
            const float h01 = heightOf(a, row, col + 1);
            const float h10 = heightOf(a, row + 1, col);
            const float h11 = heightOf(a, row + 1, col + 1);
            if (isNodata(h00) || isNodata(h01) || isNodata(h10) ||
                isNodata(h11))
                continue;
            // De-overlap: keep the quad only if this tile is the finest one
            // covering the quad centre (else a finer tile renders it).
            const double cx = a->originX + (col + 1) * a->resolution;
            const double cy = a->originY + a->extent - (row + 1) * a->resolution;
            if (ownerAt(cx, cy) != a) continue;

            const std::uint32_t v00 = base + row * P + col;
            const std::uint32_t v01 = v00 + 1;
            const std::uint32_t v10 = v00 + P;
            const std::uint32_t v11 = v10 + 1;
            emitTri(v00, v10, v01);
            emitTri(v01, v10, v11);
        }
    }

    // --- Pass 2: edge bridges (stitch seams between neighbouring tiles) ---
    // edge: 0=N (row 0), 1=E (col 255), 2=S (row 255), 3=W (col 0)
    const bool stitch = std::getenv("EBANER_NOSTITCH") == nullptr;
    if (stitch) {
        const std::uint32_t abase = base;

        for (int edge = 0; edge < 4; ++edge) {
            const bool ownOnTie = (edge == 0 || edge == 1); // N,E win same-LOD tie
            std::array<std::uint32_t, P> aIdx{};
            std::array<long, P> outIdx{};
            std::array<const Tile*, P> nb{};

            for (int i = 0; i < P; ++i) {
                int row, col;
                if (edge == 0) { row = 0; col = i; }
                else if (edge == 2) { row = P - 1; col = i; }
                else if (edge == 1) { col = P - 1; row = i; }
                else { col = 0; row = i; }

                aIdx[i] = abase + row * P + col;
                outIdx[i] = -1;
                nb[i] = nullptr;

                const float az = heightOf(a, row, col);
                if (isNodata(az)) continue;

                const double sx = worldX(a, col), sy = worldY(a, row);
                // A must own its own edge sample (not sitting under a finer tile).
                if (ownerAt(sx, sy) != a) continue;

                // Neighbour just outside the footprint at this sample.
                const double eps = 0.001 * a->resolution + 1e-3;
                double nx = sx, ny = sy;
                if (edge == 0) ny = a->originY + a->extent + eps;
                else if (edge == 2) ny = a->originY - eps;
                else if (edge == 1) nx = a->originX + a->extent + eps;
                else nx = a->originX - eps;

                const Tile* b = ownerAt(nx, ny);
                if (!b || b == a) continue;
                const bool build =
                    (b->lod > a->lod) || (b->lod == a->lod && ownOnTie);
                if (!build) continue;

                // Outer bridge vertex: a point on B's border edge adjacent to A.
                const double bres = b->resolution;
                double ox, oy;
                float oz, olc;
                glm::vec3 on;
                bool ok = true;
                if (edge == 1 || edge == 3) { // vertical border, vary along y
                    const double Xb =
                        (edge == 1) ? a->originX + a->extent : a->originX;
                    int cB = static_cast<int>(std::floor((Xb - b->originX) / bres)) +
                             (edge == 3 ? -1 : 0);
                    cB = std::clamp(cB, 0, P - 1);
                    const double t = sy;
                    const double fr = (b->originY + b->extent - t) / bres - 0.5;
                    const int r0 = static_cast<int>(std::floor(fr));
                    const double frac = fr - r0;
                    const int r0c = std::clamp(r0, 0, P - 1);
                    const int r1c = std::clamp(r0 + 1, 0, P - 1);
                    const float z0 = heightOf(b, r0c, cB), z1 = heightOf(b, r1c, cB);
                    if (isNodata(z0) || isNodata(z1)) ok = false;
                    oz = static_cast<float>(z0 + (z1 - z0) * frac);
                    ox = b->originX + (cB + 0.5) * bres;
                    oy = t;
                    on = tileNormal(b, r0c, cB);
                    olc = lcOf(b, r0c, cB);
                } else { // horizontal border, vary along x
                    const double Yb =
                        (edge == 0) ? a->originY + a->extent : a->originY;
                    int rB = static_cast<int>(
                                 std::floor((b->originY + b->extent - Yb) / bres)) +
                             (edge == 0 ? -1 : 0);
                    rB = std::clamp(rB, 0, P - 1);
                    const double t = sx;
                    const double fc = (t - b->originX) / bres - 0.5;
                    const int c0 = static_cast<int>(std::floor(fc));
                    const double frac = fc - c0;
                    const int c0c = std::clamp(c0, 0, P - 1);
                    const int c1c = std::clamp(c0 + 1, 0, P - 1);
                    const float z0 = heightOf(b, rB, c0c), z1 = heightOf(b, rB, c1c);
                    if (isNodata(z0) || isNodata(z1)) ok = false;
                    oz = static_cast<float>(z0 + (z1 - z0) * frac);
                    ox = t;
                    oy = b->originY + b->extent - (rB + 0.5) * bres;
                    on = tileNormal(b, rB, c0c);
                    olc = lcOf(b, rB, c0c);
                }
                if (!ok) continue;

                outIdx[i] = static_cast<long>(emit(ox, oy, oz, on, olc));
                nb[i] = b;
            }

            // Bridge quads between consecutive built samples with the same neighbour.
            for (int i = 0; i < P - 1; ++i) {
                if (outIdx[i] < 0 || outIdx[i + 1] < 0) continue;
                if (nb[i] != nb[i + 1]) continue;
                const std::uint32_t A0 = aIdx[i], A1 = aIdx[i + 1];
                const std::uint32_t O0 = static_cast<std::uint32_t>(outIdx[i]);
                const std::uint32_t O1 = static_cast<std::uint32_t>(outIdx[i + 1]);
                emitTri(A0, A1, O1);
                emitTri(A0, O1, O0);
            }
        }
    }

    // --- Pass 3: corner fills (close the pinholes where tiles meet) ---
    // Each geometric corner is owned by the finest quadrant tile (SW tie-break),
    // so it is built exactly once. Fill the small quad joining the up-to-four
    // participating tiles' nearest-corner surface points.
    auto quadrantOwners = [&](double cx, double cy, const Tile* q[4]) {
        const double e = 0.05;
        q[0] = ownerAt(cx - e, cy - e); // SW
        q[1] = ownerAt(cx + e, cy - e); // SE
        q[2] = ownerAt(cx + e, cy + e); // NE
        q[3] = ownerAt(cx - e, cy + e); // NW
    };
    auto cornerBuilder = [&](const Tile* q[4]) -> const Tile* {
        const Tile* best = nullptr;
        for (int k = 0; k < 4; ++k)
            if (q[k] && (!best || q[k]->lod < best->lod)) best = q[k];
        if (best && q[0] && q[0]->lod == best->lod) best = q[0]; // SW tie-break
        return best;
    };
    if (stitch) {
        const double oX = a->originX, oY = a->originY, E = a->extent;
        const double cornersXY[4][2] = {
            {oX + E, oY + E}, {oX, oY + E}, {oX + E, oY}, {oX, oY}};
        for (auto& c : cornersXY) {
            const double cx = c[0], cy = c[1];
            const Tile* q[4];
            quadrantOwners(cx, cy, q);
            if (cornerBuilder(q) != a) continue;

            // Sample each present quadrant's nearest-corner surface point.
            const double off[4][2] = {
                {-0.5, -0.5}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.5}};
            long idx[4] = {-1, -1, -1, -1};
            for (int k = 0; k < 4; ++k) {
                if (!q[k]) continue;
                const double res = q[k]->resolution;
                const double px = cx + off[k][0] * res;
                const double py = cy + off[k][1] * res;
                float z;
                glm::vec3 n;
                if (sampleAt(q[k], px, py, z, n)) {
                    const int lcC = std::clamp(
                        static_cast<int>(std::floor((px - q[k]->originX) / res)),
                        0, P - 1);
                    const int lcR = std::clamp(
                        static_cast<int>(std::floor(
                            (q[k]->originY + q[k]->extent - py) / res)),
                        0, P - 1);
                    idx[k] = static_cast<long>(
                        emit(px, py, z, n, lcOf(q[k], lcR, lcC)));
                }
            }
            // Fan around the corner (CCW: SW,SE,NE,NW), skipping missing ones.
            auto tri = [&](int i, int j, int k) {
                if (idx[i] < 0 || idx[j] < 0 || idx[k] < 0) return;
                emitTri(static_cast<std::uint32_t>(idx[i]),
                        static_cast<std::uint32_t>(idx[j]),
                        static_cast<std::uint32_t>(idx[k]));
            };
            tri(0, 1, 2);
            tri(0, 2, 3);
        }
    }

}
