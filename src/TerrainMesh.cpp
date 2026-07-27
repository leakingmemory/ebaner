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

#include "TerrainData.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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

void TerrainMesh::build(const TerrainData& data) {
    // Reset accumulators so build() is idempotent (the editor rebuilds to re-preview).
    vertices_.clear();
    indices_.clear();
    const glm::dvec3 origin = data.sceneOrigin();
    const auto& tiles = data.tiles();

    // --- Tile index + per-tile vertex base ---
    std::unordered_map<std::uint64_t, const Tile*> byKey;
    std::unordered_map<const Tile*, std::uint32_t> baseIdx;
    byKey.reserve(tiles.size() * 2);
    for (const Tile& t : tiles) byKey[keyOf(t.lod, t.col, t.row)] = &t;

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
            const int col = static_cast<int>(std::floor(wx / E));
            const int row = static_cast<int>(std::floor(wy / E));
            auto it = byKey.find(keyOf(lod, col, row));
            if (it != byKey.end()) return it->second;
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
    for (const Tile& t : tiles) {
        const Tile* a = &t;
        const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
        baseIdx[a] = base;
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
                indices_.push_back(v00);
                indices_.push_back(v10);
                indices_.push_back(v01);
                indices_.push_back(v01);
                indices_.push_back(v10);
                indices_.push_back(v11);
            }
        }
    }
    const std::size_t interiorIdx = indices_.size();

    // --- Pass 2: edge bridges (stitch seams between neighbouring tiles) ---
    // edge: 0=N (row 0), 1=E (col 255), 2=S (row 255), 3=W (col 0)
    const bool stitch = std::getenv("EBANER_NOSTITCH") == nullptr;
    for (const Tile& t : tiles) {
        if (!stitch) break;
        const Tile* a = &t;
        const std::uint32_t abase = baseIdx[a];

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
                indices_.push_back(A0);
                indices_.push_back(A1);
                indices_.push_back(O1);
                indices_.push_back(A0);
                indices_.push_back(O1);
                indices_.push_back(O0);
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
        for (const Tile& t : tiles) {
            const Tile* a = &t;
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
                    indices_.push_back(static_cast<std::uint32_t>(idx[i]));
                    indices_.push_back(static_cast<std::uint32_t>(idx[j]));
                    indices_.push_back(static_cast<std::uint32_t>(idx[k]));
                };
                tri(0, 1, 2);
                tri(0, 2, 3);
            }
        }
    }

    const std::size_t stitchIdx = indices_.size() - interiorIdx;
    std::printf("[TerrainMesh] %zu vertices, %zu indices "
                "(%zu interior + %zu stitch triangles)\n",
                vertices_.size(), indices_.size(), interiorIdx / 3,
                stitchIdx / 3);
}
