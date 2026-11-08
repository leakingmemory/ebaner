// ebaner - a Norwegian railway simulator.
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

struct Tile;

// A stretch of open water with a known surface level, drawn along its shore.
//
// A laser survey gets no return off water, so the terrain model fills a river or a
// harbour basin in from whatever surrounds it - quays, and the deck of a bridge across
// it. The result can stand metres above the real water, above the rails of the bridge.
// An area puts the surface back: every terrain cell whose centre lies inside the shore
// becomes water, flat at `level`.
//
// File `<datasetRoot>/overlay/water-areas.txt`:
//   water <sea|lake> <level> <x> <y> <x> <y> <x> <y> ...
// The points are the shore in order (EPSG:25833), at least three, the ring closing on
// its own. `sea` is salt or tidal water, `lake` fresh.
struct WaterArea {
    std::uint8_t cover = 82;          // AR50 artype: 82 sea, 81 fresh water
    float level = 0.0f;               // the surface, metres above sea level
    std::vector<glm::dvec2> shore;    // the ring, not repeating its first point
    glm::dvec2 lo{0.0}, hi{0.0};      // its bounds
};

std::vector<WaterArea> loadWaterAreas(const std::string& datasetRoot);

// True if `p` lies inside the area's shore.
bool insideWaterArea(const WaterArea& a, const glm::dvec2& p);

// Sets every cell of `t` whose centre an area covers to that area's level and water
// class. Returns how many cells it set.
std::size_t applyWaterAreas(Tile& t, const std::vector<WaterArea>& areas);
