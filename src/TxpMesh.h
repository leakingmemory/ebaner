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
class TerrainData;
class TrackPath;

// How far above the track each position stands, parallel to `ps`.
//
// A position on a platform has to stand on the slab: the track z is the rail head, and a
// Norwegian platform is a step above that, so a figure placed at track level is buried to
// the waist in it. Measured at the spot the TXP actually stands, not at the track point -
// they are a few metres to the side, which is often exactly the difference between the
// ballast and the platform.
//
// Worked out once at load rather than per rebuild: the answer cannot change while the sim
// runs, and finding the platform means searching every platform and path.
std::vector<float> txpStandLift(const std::vector<TxpPosition>& ps,
                                const std::vector<TrackPoly>& polys,
                                const TerrainData& data,
                                const std::vector<TrackPath>& paths);

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
    // `lift` comes from txpStandLift and may be empty, which stands everyone at track
    // level - the right answer where no platform is involved.
    void build(const std::vector<TxpPosition>& ps, const std::vector<char>& showing,
               const std::vector<TrackPoly>& polys, const glm::dvec3& origin,
               const std::vector<float>& lift = {});

    const std::vector<TrackVertex>& vertices() const { return vertices_; }
    const std::vector<std::uint32_t>& indices() const { return indices_; }

private:
    std::vector<TrackVertex> vertices_;
    std::vector<std::uint32_t> indices_;
};
