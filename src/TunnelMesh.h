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

#include <glm/glm.hpp>

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "TrackMesh.h" // TrackVertex

class TerrainData;

// The rock bore around an underground track. Tunnels in this area are blasted rock with no
// portal structure at all - no concrete, no headwall, just the hole in the rock face - so
// there is nothing to model but the tube itself.
//
// The bore also has to get the terrain out of its own way. A heightfield cannot have a hole
// in it, so it would happily seal the mouth; `insideBore` is what the terrain mesh asks to
// know which of its triangles are standing in the way of one.
class TunnelMesh {
public:
    void build(const TerrainData& data);

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }
    std::size_t boreCount() const { return bores_; }
    float totalLength() const { return lengthM_; }

    // Is this scene-relative point inside a bore, with a little margin? True only within a
    // tunnel's own length, within half-width of its centreline, and below its crown - all
    // three matter, see the comment on the implementation. `intoHill`, when given, gets the
    // unit direction along the tunnel pointing away from the nearer portal.
    bool insideBore(const glm::vec3& p) const;

    // Might this triangle meet a bore? A cheap bounding-box reject, so the terrain only pays
    // for the handful of triangles that actually stand at a portal.
    bool nearBore(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) const;

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
    std::size_t bores_ = 0;
    float lengthM_ = 0.0f;

    // One centreline segment, for the point-in-bore test.
    struct Span {
        glm::vec3 a, b;
    };
    std::vector<Span> spans_;
    // Spans bucketed by a coarse (x,y) cell. The terrain asks once per triangle - millions of
    // times - so the test cannot afford to look at every span.
    std::unordered_map<std::int64_t, std::vector<int>> grid_;
};
