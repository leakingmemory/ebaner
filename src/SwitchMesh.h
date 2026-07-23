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

class SwitchNetwork;

// Builds the classic Norwegian manual switch stand (sporveksel) at every turnout in a
// SwitchNetwork: a weighted lever (a near-horizontal bar at right angles to the track,
// ending in a low cylindrical counterweight) driving a throw rod to the movable rail,
// plus a tall post carrying a rotating indicator target. The lever position and the
// two-faced target (arrow one side, filled circle the other, vertical line edge-on)
// are drawn for each switch's current state, so a re-build after a throw animates it.
//
// Solid-lit TrackVertex geometry (texLayer < 0), drawn with the track/building
// pipeline. The viewer keeps it in a dynamic buffer (re-built on a throw); the editor
// merges it into the static building buffers.
class SwitchMesh {
public:
    void build(const SwitchNetwork& net);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
