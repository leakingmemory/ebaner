#include "BuildingMesh.h"

#include "TerrainData.h"

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

        // Roof: triangulated footprint at the top, facing up.
        const std::vector<int> tris = triangulate(fp);
        const glm::vec3 up(0.0f, 0.0f, 1.0f);
        const glm::vec2 uv(0.0f);
        for (std::size_t k = 0; k + 3 <= tris.size(); k += 3) {
            const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
            for (int j = 0; j < 3; ++j) {
                const glm::vec2& q = fp[tris[k + j]];
                vertices_.push_back({glm::vec3(q.x, q.y, zt), up, st.roof, uv, -1.0f});
            }
            indices_.push_back(base + 0);
            indices_.push_back(base + 1);
            indices_.push_back(base + 2);
        }
    }

    std::printf("[BuildingMesh] %zu buildings, %zu vertices, %zu triangles\n",
                uniq.size(), vertices_.size(), indices_.size() / 3);
}
