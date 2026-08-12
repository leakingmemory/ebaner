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
#include <string>
#include <vector>

struct TrackSegment;

// Connecting rails added by the editor's `rail`/slip tools get trackIds at or above
// this base. They are intentional switch connectors, so the switch detector skips its
// shallow-angle stitch filter for them (a slip at a diamond meets the crossed tracks
// very shallowly, yet each end is a real switch the user asked for).
constexpr std::uint32_t kRailIdBase = 0xE0000000;

// Manual track edits kept as a drop-in overlay, separate from the generated tiles
// so a regenerated base dataset can be dropped in without losing the edits. Stored
// as lines in `<datasetRoot>/overlay/track-edits.txt` (world coords, EPSG:25833):
//   link ax ay az bx by bz [tunnel|surface]
//                            - connect two track ends across a broken gap. The
//                              connector takes the medium of what it joins unless a
//                              keyword says otherwise, which is needed where a hole
//                              runs through a hill between two *surface* ends: the
//                              piece the export dropped was a tunnel, and carving it
//                              instead cuts a trench the depth of the hill.
//   elev x y z [trackid [fromz]]
//                            - override the elevation of the nearest track vertex.
//                              trackid picks between coincident points of different
//                              tracks; fromz picks between coincident points of the
//                              *same* track by the height each has now, which is the
//                              only thing left to tell them apart. The export does
//                              produce those - a spike is two points at one spot.
//   move ax ay az bx by bz   - move the nearest track vertex from a to b
//   rail ax ay az bx by bz   - add a new connecting rail (a-b); its ends land on the
//                              tracks there, so a switch forms at each (build a
//                              crossover/slip the export omitted)
struct TrackEdit {
    enum Kind { Link, Elev, Move, Rail } kind = Link;
    // Link/Rail: the two endpoints. Elev: a = {x, y, newZ}. Move: a = old, b = new pos.
    glm::dvec3 a{0.0}, b{0.0};
    // Elev/Move: restrict the vertex match to this track (0 = any track). Lets
    // coincident points from different sidings be edited independently.
    std::uint32_t track = 0;
    // Link: force the connector's medium (0 = take it from the ends it joins).
    std::uint8_t medium = 0;
    // Elev: which of several vertices at one spot on one track is meant, by the height
    // it has before the edit. Without it the first found wins and the others can never
    // be reached.
    bool hasFromZ = false;
    double fromZ = 0.0;
};

// Read the overlay file (empty vector if absent).
std::vector<TrackEdit> loadTrackOverlay(const std::string& datasetRoot);

// Apply edits to the loaded tiles: Elev edits override the nearest track vertex's z;
// each Link snaps a/b to the nearest track-segment endpoints and appends a synthetic
// connector segment so the two routes join into one continuous line (buildTrackPaths).
// Apply edits to the rail network in place. There is one store of track geometry -
// everything that draws or follows the rails reads it - so an edit lands once, here.
void applyTrackOverlay(std::vector<TrackSegment>& segs,
                       const std::vector<TrackEdit>& edits);

// Append edits to the overlay file (batch; opens once), creating `overlay/` if
// needed. Returns false on write failure.
bool appendTrackEdits(const std::string& datasetRoot,
                      const std::vector<TrackEdit>& edits);

// Rewrite the whole overlay file to exactly `edits` (truncating), creating
// `overlay/` if needed. Used to drop edits (e.g. ambiguous overrides) rather than
// only append. Returns false on write failure.
bool writeTrackOverlay(const std::string& datasetRoot,
                       const std::vector<TrackEdit>& edits);
