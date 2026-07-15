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
