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

#include "LevelCrossings.h"
#include "TrackMesh.h" // TrackVertex

#include <glm/glm.hpp>
#include <cstdint>
#include <vector>

class TrackPath;

// The four heads of every level crossing: two facing the train, one each way along the
// track, and two facing the road across it. Each is red over white, and only one lamp of
// a head is ever lit.
//
// Built into the same buffer the signals use (merged in main.cpp), so it inherits the
// update path that already exists for geometry that changes while the sim runs.
class CrossingMesh {
public:
    // `states` is parallel to `xs`; a crossing whose site did not resolve is skipped.
    void build(const std::vector<LevelCrossing>& xs,
               const std::vector<CrossingSite>& sites,
               const std::vector<CrossingState>& states,
               const std::vector<TrackPath>& paths, const glm::dvec3& origin);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
};

// The blink phase a crossing runs on, in seconds, from its id. Deterministic, so a
// crossing flashes the same on every run, and spread so that two crossings within sight
// of one another are visibly out of step - which is what real ones do, each having its
// own oscillator rather than a line-wide one.
float crossingPhase(int id);
