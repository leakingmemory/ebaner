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

#include <cstddef>
#include <vector>

class TerrainData;

// A minimal coloured vertex for the editor overlay pipelines (points / lines).
struct LineVertex {
    glm::vec3 pos;   // scene-origin-relative metres (z up)
    glm::vec3 color; // solid colour
};

// The raw rail network as loaded from the export, ready for the overlay
// pipelines: one marker vertex per surveyed geo-point (POINT_LIST) and a pair of
// vertices per link between consecutive points within a track (LINE_LIST). This
// is the *unsmoothed* geometry (unlike buildTrackPaths), coloured by track type,
// lifted slightly above the rails so it doesn't z-fight them.
struct TrackGraph {
    std::vector<LineVertex> points; // draw as VK_PRIMITIVE_TOPOLOGY_POINT_LIST
    std::vector<LineVertex> lines;  // draw as VK_PRIMITIVE_TOPOLOGY_LINE_LIST
    std::size_t trackCount = 0;     // distinct tracks (by trackId)
};

// Builds the overlay graph from the loaded tiles: dedup tracks by trackId,
// convert points to scene-relative, drop coincident points.
TrackGraph buildTrackGraph(const TerrainData& data);
