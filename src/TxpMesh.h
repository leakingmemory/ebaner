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

#include "TrackMesh.h" // TrackVertex
#include "TxpPositions.h"

#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

struct TrackPoly;

// The TXP, standing at a position and holding up the departure sign.
//
// The first human figure in the scene, and blocky like everything else in it. Nothing
// here is a lamp - a person and a painted sign in daylight - so it is all ordinary
// sun-lit solids with none of the emissive or blink tagging.
class TxpMesh {
public:
    // `showing` is parallel to `ps`: a position with nothing showing draws nothing at
    // all, which is the point - a figure standing at every authored position for the
    // whole session would be worse than none.
    void build(const std::vector<TxpPosition>& ps, const std::vector<char>& showing,
               const std::vector<TrackPoly>& polys, const glm::dvec3& origin);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
