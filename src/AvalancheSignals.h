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

// An avalanche warning signal: a post carrying three lamps in a column, red over white
// over red, protecting a stretch of line where the mountain can come down on it.
//
// At rest the white flashes. It is not saying "proceed" - the line ahead is nobody's
// business here - it is saying that the watch is being kept and has nothing to report.
// A driver who sees nothing at all has learned something too, which is why the resting
// aspect is a flash and not a dark head.
//
// It governs nothing. There is no route through it, no interlocking behind it and no
// aspect logic reads it; it stands where the hillside is and says what the hillside is
// doing. That is why it is its own small thing rather than another SignalKind - a
// SignalPlacement is anchored to a border and belongs to routes, and this belongs to
// neither.
//
// File `<datasetRoot>/overlay/avalanche-signals.txt`:
//   avalanche <id> "<name>" <trackHex>:<frac> <+|-> <R|L>
struct AvalancheSignal {
    int id = 0;
    std::string name;
    std::uint32_t trackId = 0;
    double frac = 0.0;
    // Which way the head looks: +1 reads toward increasing frac, -1 the other way. A
    // stretch is protected by a post at each end, each facing its own approaching
    // traffic - so one direction alone can be protected, which a head readable from both
    // sides could not do.
    int dir = 1;
    // Which side of the track the post stands on, +1 right of `dir` and -1 left. Kept
    // apart from the facing on purpose: turning the head round should not walk the post
    // across the line.
    int side = 1;
};

// What a head is showing.
//
// Nothing sets Warning yet. It is here so that the day something does - a detector, a
// dispatcher, a script - it is a value to assign rather than a shape to invent, and the
// geometry for it is already written and drawn.
enum class AvalancheAspect {
    Clear,   // the white flashes, both reds dark: watched, nothing to report
    Warning, // both reds flash together, the white out: stop, the hillside has moved
};

std::vector<AvalancheSignal> loadAvalancheSignals(const std::string& datasetRoot);
bool writeAvalancheSignals(const std::string& datasetRoot,
                           const std::vector<AvalancheSignal>& signals);
