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

#include <cstdint>
#include <string>
#include <vector>

// A post the station's TXP hangs a hand signal on.
//
// The flag is not carried: it is on a stick that goes horizontally into a fixture near
// the top of the post, so it hangs down toward the track and stays there. The post and
// its fixture stand whether or not anything is in them - an empty fixture is itself an
// indication, meaning the station is unmanned or has nothing to say.
//
// Red is stop, green is pass through, nothing is neither. The flag hangs, so it reads
// from either direction and there is no facing to author; `side` says which side of the
// track the post stands on, and that is all the geometry it needs.
//
// File `<datasetRoot>/overlay/flag-posts.txt`:
//   flag <id> "<name>" <trackHex>:<frac> <R|L> ["<station>"]
struct FlagPost {
    int id = 0;
    std::string name;
    std::uint32_t trackId = 0;
    double frac = 0.0;
    int side = 1; // +1 right of increasing frac, -1 left
    // Which station's TXP works it. Empty means whichever is nearest, which is what
    // attachStations (SimpleEntrySignals.h) works out and cannot go stale; set it only to
    // overrule that where two stations sit close enough together to confuse it.
    std::string station;
};

// What a post is displaying. One of these per post, set and taken down on its own: the
// posts do not constrain each other, and none of them is tied to whether the station is
// manned - a manned station has no flag out most of the time. Runtime only, as a switch
// position is.
enum class FlagColour { None, Red, Green };

std::vector<FlagPost> loadFlagPosts(const std::string& datasetRoot);
bool writeFlagPosts(const std::string& datasetRoot, const std::vector<FlagPost>& posts);
