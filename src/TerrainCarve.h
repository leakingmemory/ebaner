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

struct Tile;

// Carve cuttings into the terrain height grids where a *surface* railway track
// falls below the terrain, so the rails and ballast sit in a trench instead of
// being buried: a flat floor at the ballast base with side walls sloping up at
// ~20 degrees until they meet the original ground. Tunnels/tubes/bridges are left
// alone, and where a surface track meets a tunnel portal in a cutting the trench
// stops at the portal, leaving a vertical (90 degree) wall of terrain.
//
// Applied as a deterministic function of world (x,y) to every tile/LOD, so the
// terrain mesh (which reads Tile::heights for surface, normals and seam stitching)
// stays watertight. Modifies Tile::heights in place. `sceneOrigin` is only used to
// log the deepest cutting's scene-relative location.
void carveTrackCuttings(std::vector<Tile>& tiles, const glm::dvec3& sceneOrigin);
