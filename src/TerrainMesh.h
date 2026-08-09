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
struct Tile;
class TunnelMesh;

// Interleaved vertex used by the terrain pipeline.
struct Vertex {
    glm::vec3 pos;      // scene-origin-relative metres (z up)
    glm::vec3 normal;
    float elevation;    // metres above sea level
    float landcover;    // AR50 artype code (0 = none)
};

// Builds a single indexed triangle mesh from all loaded tiles.
//
// `bores` (optional) is the tunnels: a heightfield cannot have a hole in it, so without this
// the surface seals every tunnel mouth. Triangles standing inside a bore are dropped, which
// is what opens the hole in the rock face - see TunnelMesh::insideBore for why that lands at
// the portals and nowhere else.
class TerrainMesh {
public:
    void build(const TerrainData& data, const TunnelMesh* bores = nullptr);

    // One tile's chunk: its own interior plus the seams and corners it owns. Adding a
    // tile therefore also dirties the neighbours that own a seam with it - they built no
    // seam while it was absent.
    void buildTile(const TerrainData& data, const Tile& tile,
                   const TunnelMesh* bores = nullptr);

    const std::vector<Vertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    void appendTile(const TerrainData& data, const Tile* a, const TunnelMesh* bores);
    std::size_t dropped_ = 0; // sub-triangles cut for tunnel mouths

    std::vector<Vertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
