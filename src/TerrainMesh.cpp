#include "TerrainMesh.h"

#include "TerrainData.h"

#include <cstdio>

namespace {
constexpr int P = TerrainData::PIXELS; // 256

inline bool isNodata(float h) { return h <= TerrainData::NODATA + 1.0f; }
} // namespace

void TerrainMesh::build(const TerrainData& data) {
    const glm::dvec3 origin = data.sceneOrigin();

    for (const Tile& t : data.tiles()) {
        const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
        const auto& h = t.heights;
        const double res = t.resolution;

        // heights[row * P + col], row 0 = north edge.
        auto height = [&](int row, int col) -> float {
            return h[static_cast<std::size_t>(row) * P + col];
        };

        // --- Vertices ---
        for (int row = 0; row < P; ++row) {
            for (int col = 0; col < P; ++col) {
                const float z = height(row, col);
                Vertex vtx;

                const double wx = t.originX + (col + 0.5) * res;
                const double wy = t.originY + t.extent - (row + 0.5) * res;
                vtx.pos = glm::vec3(static_cast<float>(wx - origin.x),
                                    static_cast<float>(wy - origin.y),
                                    static_cast<float>(z - origin.z));
                vtx.elevation = z;

                // Central-difference normal; fall back to centre height where a
                // neighbour is off-tile or nodata (keeps the gradient finite).
                auto sample = [&](int r, int c) -> float {
                    r = r < 0 ? 0 : (r >= P ? P - 1 : r);
                    c = c < 0 ? 0 : (c >= P ? P - 1 : c);
                    float v = height(r, c);
                    return isNodata(v) ? z : v;
                };
                const float hL = sample(row, col - 1);
                const float hR = sample(row, col + 1);
                const float hN = sample(row - 1, col); // north = smaller row
                const float hS = sample(row + 1, col);
                // dz/dx east, dz/dy north.
                const float dzdx = (hR - hL) / static_cast<float>(2.0 * res);
                const float dzdy = (hN - hS) / static_cast<float>(2.0 * res);
                vtx.normal = glm::normalize(glm::vec3(-dzdx, -dzdy, 1.0f));

                vertices_.push_back(vtx);
            }
        }

        // --- Indices (skip quads touching nodata) ---
        for (int row = 0; row < P - 1; ++row) {
            for (int col = 0; col < P - 1; ++col) {
                const float h00 = height(row, col);
                const float h01 = height(row, col + 1);
                const float h10 = height(row + 1, col);
                const float h11 = height(row + 1, col + 1);
                if (isNodata(h00) || isNodata(h01) || isNodata(h10) ||
                    isNodata(h11)) {
                    continue;
                }
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

    std::printf("[TerrainMesh] %zu vertices, %zu indices (%zu triangles)\n",
                vertices_.size(), indices_.size(), indices_.size() / 3);
}
