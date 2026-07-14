#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

class TerrainData;

// Vertex for the railway ribbons (position only — tracks are a single colour).
struct TrackVertex {
    glm::vec3 pos; // scene-origin-relative metres (z up)
};

// Builds flat ribbon geometry (triangles) for all railway tracks in the loaded
// tiles, deduped by trackId.
class TrackMesh {
public:
    void build(const TerrainData& data);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
