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

#include <string>
#include <vector>

struct Tile;

// Manual track edits kept as a drop-in overlay, separate from the generated tiles
// so a regenerated base dataset can be dropped in without losing the edits. Stored
// as lines in `<datasetRoot>/overlay/track-edits.txt` (world coords, EPSG:25833):
//   link ax ay az bx by bz   - connect two track ends across a broken gap
//   elev x y z               - override the elevation of the nearest track vertex
//   move ax ay az bx by bz   - move the nearest track vertex from a to b
struct TrackEdit {
    enum Kind { Link, Elev, Move } kind = Link;
    // Link: the two endpoints. Elev: a = {x, y, newZ}. Move: a = old pos, b = new pos.
    glm::dvec3 a{0.0}, b{0.0};
};

// Read the overlay file (empty vector if absent).
std::vector<TrackEdit> loadTrackOverlay(const std::string& datasetRoot);

// Apply edits to the loaded tiles: Elev edits override the nearest track vertex's z;
// each Link snaps a/b to the nearest track-segment endpoints and appends a synthetic
// connector segment so the two routes join into one continuous line (buildTrackPaths).
void applyTrackOverlay(std::vector<Tile>& tiles, const std::vector<TrackEdit>& edits);

// Append edits to the overlay file (batch; opens once), creating `overlay/` if
// needed. Returns false on write failure.
bool appendTrackEdits(const std::string& datasetRoot,
                      const std::vector<TrackEdit>& edits);
