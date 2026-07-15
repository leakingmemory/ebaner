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

#include "BuildingMesh.h"

#include "TerrainData.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_set>
#include <vector>

namespace {

struct BuildingStyle {
    glm::vec3 wall;
    glm::vec3 roof;
};

// Neutral tints by building kind (0 other, 1 residential, 2 commercial,
// 3 industrial); roofs a little darker.
BuildingStyle styleFor(std::uint8_t kind) {
    switch (kind) {
        case 1: return {{0.80f, 0.72f, 0.60f}, {0.45f, 0.28f, 0.24f}}; // resid.
        case 2: return {{0.70f, 0.74f, 0.78f}, {0.40f, 0.42f, 0.46f}}; // commerc.
        case 3: return {{0.62f, 0.60f, 0.56f}, {0.34f, 0.35f, 0.38f}}; // industr.
        default: return {{0.74f, 0.72f, 0.68f}, {0.42f, 0.42f, 0.44f}};
    }
}

std::uint64_t hashBuilding(const BuildingSegment& b) {
    auto q = [](double v) { return static_cast<std::int64_t>(std::llround(v * 10.0)); };
    std::uint64_t h = 1469598103934665603ull;
    auto mix = [&](std::uint64_t x) { h ^= x; h *= 1099511628211ull; };
    mix(b.kind);
    mix(b.footprint.size());
    double cx = 0, cy = 0;
    for (const glm::dvec2& p : b.footprint) { cx += p.x; cy += p.y; }
    const double inv = b.footprint.empty() ? 0.0 : 1.0 / b.footprint.size();
    mix(static_cast<std::uint64_t>(q(cx * inv)));
    mix(static_cast<std::uint64_t>(q(cy * inv)));
    mix(static_cast<std::uint64_t>(q(b.footprint.front().x)));
    mix(static_cast<std::uint64_t>(q(b.footprint.front().y)));
    return h;
}

bool pointInTri(const glm::vec2& p, const glm::vec2& a, const glm::vec2& b,
                const glm::vec2& c) {
    const float d1 = (p.x - b.x) * (a.y - b.y) - (a.x - b.x) * (p.y - b.y);
    const float d2 = (p.x - c.x) * (b.y - c.y) - (b.x - c.x) * (p.y - c.y);
    const float d3 = (p.x - a.x) * (c.y - a.y) - (c.x - a.x) * (p.y - a.y);
    const bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    const bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    return !(neg && pos);
}

// Ear-clipping triangulation of a simple polygon; returns triangle index triples
// into `poly`. Empty if it can't be triangulated (degenerate).
std::vector<int> triangulate(const std::vector<glm::vec2>& poly) {
    const int n = static_cast<int>(poly.size());
    std::vector<int> tris;
    if (n < 3) return tris;

    double area = 0.0;
    for (int i = 0, j = n - 1; i < n; j = i++)
        area += static_cast<double>(poly[j].x) * poly[i].y -
                static_cast<double>(poly[i].x) * poly[j].y;
    const bool ccw = area > 0.0;

    std::vector<int> v(n);
    for (int i = 0; i < n; ++i) v[i] = ccw ? i : (n - 1 - i); // work CCW

    int m = n, guard = 0;
    while (m > 3 && guard++ < 4 * n * n) {
        bool clipped = false;
        for (int i = 0; i < m; ++i) {
            const int i0 = v[(i + m - 1) % m], i1 = v[i], i2 = v[(i + 1) % m];
            const glm::vec2 a = poly[i0], b = poly[i1], c = poly[i2];
            const float cross =
                (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
            if (cross <= 0.0f) continue; // reflex vertex
            bool ear = true;
            for (int k = 0; k < m; ++k) {
                const int vk = v[k];
                if (vk == i0 || vk == i1 || vk == i2) continue;
                if (pointInTri(poly[vk], a, b, c)) { ear = false; break; }
            }
            if (!ear) continue;
            tris.push_back(i0);
            tris.push_back(i1);
            tris.push_back(i2);
            v.erase(v.begin() + i);
            --m;
            clipped = true;
            break;
        }
        if (!clipped) break; // degenerate polygon; give up on the remainder
    }
    if (m == 3) {
        tris.push_back(v[0]);
        tris.push_back(v[1]);
        tris.push_back(v[2]);
    }
    return tris;
}

} // namespace

void BuildingMesh::build(const TerrainData& data) {
    const glm::dvec3 origin = data.sceneOrigin();

    std::unordered_set<std::uint64_t> seen;
    std::vector<const BuildingSegment*> uniq;
    for (const Tile& t : data.tiles())
        for (const BuildingSegment& b : t.buildings) {
            if (b.footprint.size() < 3) continue;
            if (seen.insert(hashBuilding(b)).second) uniq.push_back(&b);
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
    // Roof triangle: outward normal; near-vertical faces (gable ends) get the
    // wall colour, sloped/flat faces the roof colour.
    auto emitRoofTri = [&](const glm::vec3& p0, const glm::vec3& p1,
                           const glm::vec3& p2, const glm::vec3& inside,
                           const BuildingStyle& st) {
        glm::vec3 nrm = glm::cross(p1 - p0, p2 - p0);
        const float nl = glm::length(nrm);
        if (nl < 1e-9f) return; // degenerate
        nrm /= nl;
        const glm::vec3 cen = (p0 + p1 + p2) * (1.0f / 3.0f);
        if (glm::dot(nrm, cen - inside) < 0.0f) nrm = -nrm;
        const glm::vec3 color = (std::abs(nrm.z) < 0.35f) ? st.wall : st.roof;
        const glm::vec2 uv(0.0f);
        const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
        vertices_.push_back({p0, nrm, color, uv, -1.0f});
        vertices_.push_back({p1, nrm, color, uv, -1.0f});
        vertices_.push_back({p2, nrm, color, uv, -1.0f});
        indices_.push_back(base + 0);
        indices_.push_back(base + 1);
        indices_.push_back(base + 2);
    };

    for (const BuildingSegment* bp : uniq) {
        const BuildingSegment& b = *bp;
        const BuildingStyle st = styleFor(b.kind);
        const int n = static_cast<int>(b.footprint.size());

        // Scene-relative footprint + heights.
        std::vector<glm::vec2> fp(n);
        glm::vec2 centroid(0.0f);
        for (int i = 0; i < n; ++i) {
            fp[i] = glm::vec2(static_cast<float>(b.footprint[i].x - origin.x),
                              static_cast<float>(b.footprint[i].y - origin.y));
            centroid += fp[i];
        }
        centroid /= static_cast<float>(n);
        const float zb = static_cast<float>(b.baseZ - origin.z);
        const float zt = static_cast<float>(b.baseZ + b.height - origin.z);
        const glm::vec3 inside(centroid.x, centroid.y, (zb + zt) * 0.5f);

        // Walls: one quad per footprint edge.
        for (int i = 0; i < n; ++i) {
            const glm::vec2& a = fp[i];
            const glm::vec2& c = fp[(i + 1) % n];
            emitQuad(glm::vec3(a.x, a.y, zb), glm::vec3(c.x, c.y, zb),
                     glm::vec3(c.x, c.y, zt), glm::vec3(a.x, a.y, zt), inside,
                     st.wall);
        }

        // Roof, driven by OSM roof_shape.
        // Pitch height scales with footprint size; ridge axis = longest edge.
        glm::vec2 bmin = fp[0], bmax = fp[0];
        for (const glm::vec2& q : fp) { bmin = glm::min(bmin, q); bmax = glm::max(bmax, q); }
        const float ext = std::min(bmax.x - bmin.x, bmax.y - bmin.y);
        const float pitch = std::clamp(0.35f * ext, 1.5f, 8.0f);
        glm::vec2 axis(1.0f, 0.0f);
        float bestLen = -1.0f;
        for (int i = 0; i < n; ++i) {
            const glm::vec2 e = fp[(i + 1) % n] - fp[i];
            const float L = glm::length(e);
            if (L > bestLen) { bestLen = L; if (L > 1e-4f) axis = e / L; }
        }
        std::vector<float> tproj(n);
        float tmin = 1e30f, tmax = -1e30f;
        for (int i = 0; i < n; ++i) {
            tproj[i] = glm::dot(fp[i] - centroid, axis);
            tmin = std::min(tmin, tproj[i]);
            tmax = std::max(tmax, tproj[i]);
        }
        const glm::vec3 rInside(centroid.x, centroid.y, zt + pitch * 0.5f);

        if (b.roofShape == 0) { // flat cap
            const std::vector<int> tris = triangulate(fp);
            for (std::size_t k = 0; k + 3 <= tris.size(); k += 3)
                emitRoofTri(glm::vec3(fp[tris[k]], zt), glm::vec3(fp[tris[k + 1]], zt),
                            glm::vec3(fp[tris[k + 2]], zt), rInside, st);
        } else if (b.roofShape == 4) { // skillion (single slope)
            const float span = std::max(tmax - tmin, 1e-3f);
            std::vector<float> zTop(n);
            for (int i = 0; i < n; ++i)
                zTop[i] = zt + pitch * ((tproj[i] - tmin) / span);
            const std::vector<int> tris = triangulate(fp);
            for (std::size_t k = 0; k + 3 <= tris.size(); k += 3)
                emitRoofTri(glm::vec3(fp[tris[k]], zTop[tris[k]]),
                            glm::vec3(fp[tris[k + 1]], zTop[tris[k + 1]]),
                            glm::vec3(fp[tris[k + 2]], zTop[tris[k + 2]]), rInside, st);
        } else { // ridge roof: gabled=1, hipped=2, pyramidal=3
            const float rmid = (tmin + tmax) * 0.5f, half = (tmax - tmin) * 0.5f;
            const float inset = (b.roofShape == 2) ? 0.28f
                                : (b.roofShape == 3) ? 1.0f
                                                     : 0.0f;
            const float ridgeHalf = half * (1.0f - inset);
            const float ridgeMin = rmid - ridgeHalf, ridgeMax = rmid + ridgeHalf;
            auto ridgePt = [&](float t) {
                return centroid + axis * std::clamp(t, ridgeMin, ridgeMax);
            };
            const float ztop = zt + pitch;
            for (int i = 0; i < n; ++i) {
                const int j = (i + 1) % n;
                const glm::vec3 a(fp[i].x, fp[i].y, zt), c(fp[j].x, fp[j].y, zt);
                const glm::vec2 r0 = ridgePt(tproj[i]), r1 = ridgePt(tproj[j]);
                const glm::vec3 R0(r0.x, r0.y, ztop), R1(r1.x, r1.y, ztop);
                if (glm::distance(r0, r1) < 0.05f) {
                    emitRoofTri(a, c, R0, rInside, st);
                } else {
                    emitRoofTri(a, c, R1, rInside, st);
                    emitRoofTri(a, R1, R0, rInside, st);
                }
            }
        }
    }

    std::printf("[BuildingMesh] %zu buildings, %zu vertices, %zu triangles\n",
                uniq.size(), vertices_.size(), indices_.size() / 3);
}
