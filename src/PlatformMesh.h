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

#include "TrackMesh.h" // TrackVertex (shared solid-lit vertex)

#include <cstdint>
#include <vector>

class TerrainData;
class TrackPath;

// Extrudes OSM station-platform footprints into low lit concrete slabs (walls +
// flat top). Reuses TrackVertex and the track pipeline; deduped by geometry
// (platforms carry no id). The slab top is placed a standard step height above
// the nearest rail head (falling back to the terrain where no track is near).
class PlatformMesh {
public:
    void build(const TerrainData& data, const std::vector<TrackPath>& paths);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
