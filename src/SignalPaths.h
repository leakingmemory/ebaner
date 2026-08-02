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

#include <string>
#include <vector>

#include "TrackCircuits.h" // Border, SectionInterval

// A mini signal path: a directional route from one track-circuit border to another,
// running through one or more circuits and switches. Stored as its two endpoint borders
// plus the ordered directed track intervals of the route (from -> to is the travel
// direction, so `from` may exceed `to`); the switch legs it takes and the circuits it
// spans are derivable from these intervals. Anchored by trackId + arc-length like the
// rest of the overlay, so it survives re-imports.
//
// File `<datasetRoot>/overlay/signal-paths.txt`:
//   path <id> <name> <startTrackHex>:<startFrac> <endTrackHex>:<endFrac> \
//        <trackHex>:<from>:<to> <trackHex>:<from>:<to> ...

struct SignalPath {
    int id = 0;
    std::string name;
    Border start;
    Border end;
    std::vector<SectionInterval> parts; // ordered, directed route intervals
};

// --- File IO (mirrors loadTrackCircuits/writeTrackCircuits) ---
std::vector<SignalPath> loadSignalPaths(const std::string& datasetRoot);
bool writeSignalPaths(const std::string& datasetRoot,
                      const std::vector<SignalPath>& paths);
