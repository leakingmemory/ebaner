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

#include <glm/glm.hpp>

#include <cstdint>
#include <vector>

class Vehicle;
class Consist;

// Shared wheelset dimensions (metres). kAxleCentreAboveBed is the height of the
// axle centre above the rail-bed centreline (pose.pos) — rail top + wheel radius.
namespace wheelset {
constexpr float kRailTopZ = 0.76f;    // matches TrackMesh rail top (scene offset)
constexpr float kWheelRadius = 0.46f; // 0.92 m wheel
constexpr float kAxleCentreAboveBed = kRailTopZ + kWheelRadius;
} // namespace wheelset

// Builds the geometry for a rail vehicle: one wheelset (axle + two wheels) per
// axle, plus a bogie frame box when the vehicle has a wheelbase. Emits TrackVertex
// + indices in scene coords, drawn by the track pipeline.
class VehicleMesh {
public:
    // The whole train, set by set, into one buffer.
    void build(const Consist& consist);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }
    // Index at which the transparent (glass) triangles begin; indices are ordered
    // opaque-first, then glass, so the two can be drawn in separate passes.
    std::uint32_t glassFirstIndex() const { return glassFirstIndex_; }

private:
    // One set's geometry, appended to the buffer.
    void emitUnit(const Vehicle& vehicle);

    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
    std::uint32_t glassFirstIndex_ = 0;
};

// Driver's-eye camera. `count` is the number of driver positions on the train (2 per
// Class 93 set, one per cab; 0 for vehicles with no cab) - every cab can be sat in and
// looked out of, including the shut-down ones at a coupler. `eyePose` gives the eye
// point and forward direction (world space) for cab `position` at the train's current
// pose; false when unavailable.
namespace drivercam {
int count(const Consist& c);
bool eyePose(const Consist& c, int position, glm::vec3& eye, glm::vec3& forward);
} // namespace drivercam
