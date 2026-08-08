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

#include <vector>

#include "TrackPath.h"

// Line speeds come in gappy: the import knows a limit at most surveyed points and nothing at
// the rest. Two rules fill the gaps, and between them they make the limit *directional* even
// though the data is not - the same undefined stretch inherits from whichever side it is
// approached:
//
//   - a path that begins undefined in the direction travelled starts at kDefaultKmh;
//   - a defined limit carries forward through undefined track until something redefines it.
//
// Resolution runs along one TrackPath, which is already a chained road rather than a single
// imported segment. It deliberately does not cross junctions: a limit does not survive into
// the next path, which resets to the default if it too begins undefined.
inline constexpr int kDefaultKmh = 40;

// A resolved limit taking effect at `s` and holding until the next one. `s` is measured along
// the path's own parameterisation whichever way it is being read, so for dir = -1 the values
// descend.
struct SpeedStretch {
    float s = 0.0f;
    int kmh = 0;
};

// Resolve one path in one direction: dir = +1 along the path, -1 against it. Stretches come
// back in travel order, the first one starting where the path does in that direction.
std::vector<SpeedStretch> resolveSpeeds(const TrackPath& p, int dir);

// A sign standing where the limit goes up. `tangent` is the direction it faces (the direction
// of travel it applies to) and `right` that direction's cross-track right, which is the side
// it stands on.
struct SpeedSign {
    glm::vec3 pos{0.0f};
    glm::vec3 tangent{1.0f, 0.0f, 0.0f};
    glm::vec3 right{0.0f, 1.0f, 0.0f};
    int kmh = 0;
};

// Every increase over every path, in both directions. The first stretch of a path is never an
// increase: nothing preceded it to rise from, and a path boundary is an artefact of how the
// import was chained rather than a place a driver would expect a sign.
std::vector<SpeedSign> speedIncreaseSigns(const std::vector<TrackPath>& paths);
