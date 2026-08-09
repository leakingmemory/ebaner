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

// A place on the railway you can start at.
//
// The export already knows them: every tile's meta.json carries the stations inside it,
// with the name, the node position and what kind of place it is. Nothing here is
// authored by us.
struct Station {
    std::string name;    // UTF-8, as the data spells it ("Bodø", "Oteråga")
    glm::dvec3 world{0}; // EPSG:25833, the station node
    char type = 'S';     // 'S' a station (sidings, signals), 'I' a stop
    std::string line;    // "Nordlandsbanen", ...

    bool isStop() const { return type == 'I'; }
};

// Every station in the dataset, sorted by line then name, deduplicated: one near a tile
// boundary is written into both tiles. Reads every tile's meta.json, which sounds worse
// than it is - the whole sweep is well under a second.
std::vector<Station> loadStations(const std::string& datasetRoot);

// Look one up by name, ignoring case and the difference between the Norwegian letters
// and their ASCII shapes, so "fauske", "Bodo" and "Bodø" all work from a shell that may
// not make the accented forms easy to type. Null if there is no match.
const Station* findStation(const std::vector<Station>& all, const std::string& name);

// Resolve the station named on the command line, or Bodø when `name` is empty. On a name
// that matches nothing this prints what was meant to be near it and returns null - the
// list is 720 long, so neither silence nor printing all of it is any help.
const Station* pickStation(const std::vector<Station>& all, const std::string& name);

// The few names closest to `name`, for telling someone what they might have meant.
std::vector<std::string> nearestNames(const std::vector<Station>& all,
                                      const std::string& name, std::size_t count = 6);
