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

#include "FlagPosts.h"
#include "TrackMesh.h" // TrackVertex

#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

class TrackPath;
struct TrackPoly;

// The flag posts: ironwork always, cloth only when a flag is out.
//
// Nothing here is a lamp - a flag is cloth in daylight - so it is all ordinary sun-lit
// solids and none of the emissive or blink tagging applies. Built into the same buffer
// the signals and the crossings use, which already has an update path for geometry that
// changes while the sim runs.
class FlagMesh {
public:
    // `shown` is parallel to `posts`: what each is displaying this moment.
    void build(const std::vector<FlagPost>& posts,
               const std::vector<FlagColour>& shown,
               const std::vector<TrackPoly>& polys, const glm::dvec3& origin);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
