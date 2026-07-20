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
// so a regenerated base dataset can be dropped in without losing the edits. v1 has
// a single edit kind: a "link" connecting two track ends across a gap the exporter
// left broken. Stored as lines in `<datasetRoot>/overlay/track-edits.txt`:
//   link ax ay az bx by bz     (world coordinates, EPSG:25833)
struct TrackEdit {
    glm::dvec3 a{0.0}, b{0.0}; // world endpoints to connect
};

// Read the overlay file (empty vector if absent).
std::vector<TrackEdit> loadTrackOverlay(const std::string& datasetRoot);

// Apply edits to the loaded tiles: for each link, snap a and b to the nearest
// existing track-segment endpoints and append a synthetic connector segment so the
// two routes join into one continuous line (see buildTrackPaths).
void applyTrackOverlay(std::vector<Tile>& tiles, const std::vector<TrackEdit>& edits);

// Append one edit to the overlay file, creating `overlay/` if needed.
// Returns false on write failure.
bool appendTrackEdit(const std::string& datasetRoot, const TrackEdit& edit);
