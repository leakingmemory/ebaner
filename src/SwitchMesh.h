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

// Places a classic Norwegian manual switch stand (sporveksel) at every turnout in
// the network: a weighted lever (a bar ending in a low cylindrical counterweight)
// driving a throw rod to the movable rail, plus a tall post carrying a rotating
// indicator target. A turnout is detected where one track's endpoint lies on
// another track's line. Every switch is drawn in the straight-through setting for
// now, so each target shows the vertical-line indication and the lever points along
// the through track (opposite the direction a train would be diverted).
//
// Reuses TrackVertex and the track/building pipeline (solid-lit, texLayer < 0);
// merged into the building buffers by the caller, like PlatformMesh.
class SwitchMesh {
public:
    void build(const TerrainData& data);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
