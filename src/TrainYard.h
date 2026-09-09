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

#include "Stations.h"

#include <glm/glm.hpp>

#include <deque>
#include <vector>

class Consist;
class TrackPath;
struct VehicleSpec;

// Putting a train on the line somewhere other than where the session started.
//
// One train used to be the whole of it: the start screen spawned it and nothing else could
// make another. Testing a block signal needs a train in the section in front of the one you
// are driving, so trains have to be placeable - and a train placed carelessly is worse than
// no train at all. A path shorter than the train pins the sets past its end at the arc-length
// clamp, and what the driver sees is a heap of triangles that derails on the first step with
// nothing to say why. So a placement is something that can be refused, with a reason.

// Where a train of this shape will fit, nearest a point. `path` null means nowhere would
// take it and `why` says what was wrong.
struct Placement {
    const TrackPath* path = nullptr;
    int pathIdx = -1; // index of `path` in the vector it came from
    float s = 0.0f;   // arc length along it, already inside the margins
    float half = 0.0f; // half the train over its outermost axles
    const char* why = nullptr;
};

// The nearest main line to `targetXY` (scene coordinates) with room for `spec`, sampled the
// way the start-up spawn samples: main line only, bounding box first, 5 m steps.
//
// `reach` is how far to look before falling back to the whole network; 0 searches
// everything. `maxDist` is how far away the answer may be and still count as an answer -
// without it, asking for a station on a line this export does not contain quietly puts the
// train on the nearest point of a different line, hundreds of kilometres from the name that
// was asked for, and nothing says so.
Placement placeTrainNear(const std::vector<TrackPath>& paths, glm::vec2 targetXY,
                         const VehicleSpec& spec, float reach = 6000.0f,
                         float maxDist = 5000.0f);

// Whether that spot is clear of every train already out there. Two trains dropped on the
// same stretch of road interpenetrate and both derail, and the track circuits would read the
// pair as one - so this refuses before it happens rather than explaining afterwards.
bool placementClear(const std::deque<Consist>& trains, const Placement& p);

// The nearest station to a world position, for saying where a train is when it is not
// standing on an authored track circuit. Null when there are no stations at all.
const Station* nearestStation(const std::vector<Station>& all, glm::dvec3 world);
