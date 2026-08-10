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

// A spot beside the track where the station's TXP stands to give a stopped train
// permission to leave, holding up a white sign with a green circle.
//
// The one hand signal that hangs on nothing: it is a person, and the position is only
// where they stand to give it. Nothing is drawn here at all unless the signal is being
// given - an authored position with a figure permanently in it would be worse than no
// figure - except in the editor, where it has to be visible to be placed and aimed.
//
// File `<datasetRoot>/overlay/txp-positions.txt`:
//   txp <id> "<name>" <trackHex>:<frac> <+|-> <R|L> ["<station>"]
struct TxpPosition {
    int id = 0;
    std::string name;
    std::uint32_t trackId = 0;
    double frac = 0.0;
    // Where the train is departing to. The driver is looking that way, so the TXP faces
    // back down `-dir` to be seen - the convention the level-crossing train heads use.
    int dir = 1;
    // Which side of the track the TXP stands on, +1 right of increasing frac. Absolute
    // on purpose, and not relative to `dir`: turning the signal round should not walk
    // the TXP across the track, which is what tying the two together would do.
    int side = 1;
    // Whose station it belongs to, and so which other positions it stands in for. Empty
    // means the nearest, which is what attachStations (SimpleEntrySignals.h) works out.
    std::string station;
};

std::vector<TxpPosition> loadTxpPositions(const std::string& datasetRoot);
bool writeTxpPositions(const std::string& datasetRoot,
                       const std::vector<TxpPosition>& ps);
