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

struct VehicleFrame;

// Shared wheelset dimensions (metres). kAxleCentreAboveBed is the height of the
// axle centre above the rail-bed centreline (pose.pos) — rail top + wheel radius.
namespace wheelset {
constexpr float kRailTopZ = 0.76f;    // matches TrackMesh rail top (scene offset)
constexpr float kWheelRadius = 0.46f; // 0.92 m wheel
constexpr float kAxleCentreAboveBed = kRailTopZ + kWheelRadius;
} // namespace wheelset

// Builds a single railway wheelset (one axle + two wheels, nothing attached),
// seated on the rails at the given track pose. Emits TrackVertex + indices in
// scene coords, drawn by the track pipeline.
class WheelsetMesh {
public:
    void build(const VehicleFrame& frame);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
