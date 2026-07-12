#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

class TerrainData;

// Interleaved vertex used by the terrain pipeline.
struct Vertex {
    glm::vec3 pos;      // scene-origin-relative metres (z up)
    glm::vec3 normal;
    float elevation;    // metres above sea level
    float landcover;    // AR50 artype code (0 = none)
};

// Builds a single indexed triangle mesh from all loaded tiles.
class TerrainMesh {
public:
    void build(const TerrainData& data);

    const std::vector<Vertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    std::vector<Vertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
