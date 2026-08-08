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

#include <cstdint>
#include <vector>

#include "SpeedLimits.h"
#include "TrackMesh.h" // TrackVertex

// The lineside sign for a speed increase: a post carrying an upward-pointing amber triangle
// with the limit divided by ten in black (40 -> 4, 110 -> 11). Static geometry - a sign never
// changes - so it belongs in the struct buffer beside the buildings and platforms, not in the
// dynamic one the signals use.
class SpeedSignMesh {
public:
    void build(const std::vector<SpeedSign>& signs);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
