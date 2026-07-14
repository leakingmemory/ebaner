#include "TrackMesh.h"

#include "TerrainData.h"

#include <cstdio>
#include <unordered_set>

namespace {
constexpr float kTrackWidth = 5.0f; // ribbon width, metres
constexpr float kTrackLift = 1.5f;  // lift above terrain, metres (avoids z-fight)
} // namespace

void TrackMesh::build(const TerrainData& data) {
    const glm::dvec3 origin = data.sceneOrigin();
    const float hw = kTrackWidth * 0.5f;
    std::unordered_set<std::uint32_t> seen;

    for (const Tile& t : data.tiles()) {
        for (const TrackSegment& seg : t.tracks) {
            // A through-track appears in full in every tile it crosses; draw once.
            if (!seen.insert(seg.trackId).second) continue;
            if (seg.pts.size() < 2) continue;

            // Scene-relative centreline, lifted slightly above the terrain.
            std::vector<glm::vec3> c;
            c.reserve(seg.pts.size());
            for (const glm::dvec3& w : seg.pts) {
                c.emplace_back(static_cast<float>(w.x - origin.x),
                               static_cast<float>(w.y - origin.y),
                               static_cast<float>(w.z - origin.z) + kTrackLift);
            }

            // Two edge vertices per centreline vertex, offset in the horizontal
            // plane along the mitred perpendicular (average of adjacent tangents).
            const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
            const int n = static_cast<int>(c.size());
            for (int i = 0; i < n; ++i) {
                glm::vec2 tang(0.0f);
                if (i > 0)
                    tang += glm::vec2(c[i].x - c[i - 1].x, c[i].y - c[i - 1].y);
                if (i < n - 1)
                    tang += glm::vec2(c[i + 1].x - c[i].x, c[i + 1].y - c[i].y);
                const float tl = glm::length(tang);
                const glm::vec2 perp = (tl > 1e-6f)
                                           ? glm::vec2(-tang.y, tang.x) / tl
                                           : glm::vec2(1.0f, 0.0f);
                const glm::vec3 off(perp.x * hw, perp.y * hw, 0.0f);
                vertices_.push_back({c[i] + off}); // left edge
                vertices_.push_back({c[i] - off}); // right edge
            }

            // Two triangles per span between consecutive edge-vertex pairs.
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
        }
    }

    std::printf("[TrackMesh] %zu tracks, %zu vertices, %zu triangles\n",
                seen.size(), vertices_.size(), indices_.size() / 3);
}
