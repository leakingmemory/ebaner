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

#include "SignalPaths.h" // SignalPlacement
#include "TrackMesh.h"   // TrackVertex (shared solid-lit vertex)

#include <cstdint>
#include <vector>

// Norwegian mini ground signal ("dvergsignal"): a black box on a short (~1 m) pole beside
// the track with two warm-white lamps. A fixed reference lamp sits at the lower-left; a second lamp lights on a
// quarter-arc around it - horizontal (lower-right) = Stop, 45 deg = train ahead on an
// otherwise-clear line, vertical (above the reference) = line clear. Only the Stop aspect is
// drawn for now (reference + horizontal lit; the other arc positions are unlit spots).
//
// Solid-lit TrackVertex geometry (texLayer < 0), drawn with the track/building pipeline;
// static, so it is merged into the static struct buffer like the switch stands.
class SignalMesh {
public:
    void build(const std::vector<SignalPlacement>& signals, glm::dvec3 sceneOrigin);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
