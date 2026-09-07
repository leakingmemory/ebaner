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

// The line block (linjeblokk): the stretch of plain line between two stations, cut into
// block sections by block signals, and the rule that keeps trains off each other on it.
//
// Two things are going on here, and they are worth separating.
//
// The *block signals* are automatic. Each governs the section ahead of it and shows a
// green when a movement has been authorised its way and that section is empty. That is
// what lets a second train follow a first onto the line: it is held at the block signal
// until the one in front has left the section beyond, and then it goes. One train per
// section, and no dispatcher touches any of it.
//
// The *direction lock* is what stops the two stations clearing departures toward each
// other. Two trains cleared into the same line block from opposite ends do not collide -
// the block signals stop them - but they come to a stand nose to nose with no way out,
// which is not a state normal working should be able to reach at all. So the line takes a
// direction when the first movement claims it, and an opposing departure is refused for
// as long as the claim stands.
//
// The claim has to outlive the route that made it. A MainRoute is erased as its last
// circuit is entered, by which time the train is out on the line and every trace of which
// way it was going has gone with the route. So the claim is held here, and released only
// when the line is genuinely clear: no route into it, and no circuit in it occupied.
//
// Everything below is pure. Occupancy and route-existence arrive as callables so this can
// be driven from a test without a running simulator - the interlocking in main.cpp is
// lambdas over live state and cannot be tested at all, which is exactly why this is not
// there.

#include "SignalPaths.h"
#include "SwitchNetwork.h"
#include "TrackCircuits.h"

#include <functional>
#include <string>
#include <vector>

// One block signal, resolved against the geometry and the paths it stands among.
struct BlockRoad {
    int road = -1;  // index into signalPaths: the road this signal governs, -1 if none
    int line = -1;  // index into LineBlocks::lines, -1 if unresolved
    int align = 0;  // +1 or -1: which of the line's two senses this signal faces
    std::vector<int> sections; // section ids of `road` - the block ahead of this signal
};

// One line block: a stretch of plain line between two stations, and everything on it.
struct LineBlock {
    std::string name;           // from the ends, for messages: "MAJ-SGD"
    std::vector<int> sections;  // every section id on the line, both blocks
    std::vector<int> signals;   // indices into the BlockSignal list
    std::vector<Border> ends;   // the two borders bounding it (the stations' outer ones)
    // False when the shape could not be made sense of - a line whose ends do not come to
    // two, or whose signals cannot be two-coloured. Every signal on an unsound line is
    // held at danger and no claim is granted: loud and safe beats clever.
    bool sound = true;
};

struct LineBlocks {
    std::vector<LineBlock> lines;
    std::vector<BlockRoad> roads;    // parallel to the BlockSignal list
    std::vector<std::string> notes;  // what the resolution could not do; the caller prints
    bool empty() const { return lines.empty(); }
};

// Resolve block signals against the roads authored around them.
//
// A block signal's road is the signal path leaving its border the way it faces. Both of a
// border's two signals resolve this way, one per direction, and their two roads together
// are the line: the section ahead of the northbound signal plus the section ahead of the
// southbound one is the whole of the plain line between the stations. That is where the
// grouping comes from - the line block is not authored, it is what the roads already say.
//
// Deliberately not derived by walking outward to the next main signal. That walk follows
// the points as they currently sit, which would make the extent of a line block a
// function of switch state; and it finds nothing at all until both stations have their
// exit signals, so a line could not be blocked before it was signalled at both ends.
LineBlocks resolveLineBlocks(const std::vector<BlockSignal>& blockSignals,
                             const std::vector<SignalPath>& signalPaths,
                             const std::vector<TrackPoly>& polys,
                             const TrackCircuits& circuits);

// Which way a movement runs over line `line`: +1 or -1 against the line's own sense, or 0
// if it does not enter that line at all.
//
// Measured against whichever of the line's roads the movement actually shares rails with,
// not against one nominated road: on a line cut into three or more blocks a departure may
// overlap only the road nearest the station it leaves.
int lineBlockDirection(const LineBlocks& lb, int line, const SignalPath& movement,
                       const std::vector<SignalPath>& signalPaths);

// What the interlocking is holding for each line. Runtime state, one per LineBlock.
struct LineBlockState {
    int claimed = 0;    // 0 free, else the sense the line is set for
    std::string by;     // the route that claimed it, for the refusal message
};

// Claim every line `movement` enters, in the direction it runs. Silently leaves alone a
// line already claimed the same way - that is the second train following the first, which
// is the whole point of putting a signal in the middle of the line.
void claimLineBlocks(const LineBlocks& lb, std::vector<LineBlockState>& state,
                     const SignalPath& movement,
                     const std::vector<SignalPath>& signalPaths, const std::string& by);

// Why `movement` may not be granted, or "" if it may. `brief` gets a word or two for the
// route picker, which has a hard width budget.
//
// Two ways to be refused: the line is claimed the other way, or a train stands on a line
// nothing has claimed. The second is the conservative case - a train that reached the line
// without a route (spawned there, or shunted out on dwarfs) has no direction on record, so
// nothing is let out against it.
std::string lineBlockRefusal(const LineBlocks& lb,
                             const std::vector<LineBlockState>& state,
                             const SignalPath& movement,
                             const std::vector<SignalPath>& signalPaths,
                             const std::function<bool(int)>& occupied,
                             std::string* brief);

// What one step leaves the block signals showing, and which roads it wants open.
struct BlockOutcome {
    std::vector<char> roadSet;          // parallel to signalPaths: 1 = this road is open
    std::vector<SignalAspect> aspect;   // parallel to the BlockSignal list
};

// Advance every line block one step: release the claims that are spent, then decide what
// each signal shows and which road it has open.
//
// A block signal clears when the line is claimed the way it faces and the section ahead of
// it is empty. Because that is re-read every step rather than latched, both cases fall out
// of the one rule: it clears at once if the block ahead is already empty when the movement
// is authorised, and it clears later, by itself, when the train in front leaves that block.
//
// `occupied(sectionId)` and `routeInto(lineIndex)` are the two questions this cannot
// answer for itself. Returns true if anything changed.
bool stepLineBlocks(const LineBlocks& lb, std::vector<LineBlockState>& state,
                    const std::vector<SignalPath>& signalPaths, const SwitchNetwork& net,
                    const std::vector<TrackPoly>& polys,
                    const std::function<bool(int)>& occupied,
                    const std::function<bool(int)>& routeInto, BlockOutcome& out);

// Take the block signals' roads away from the dwarf placements standing on them.
//
// Both signals at a block border already have a dwarf placement built from the same mini
// path, because signalPlacements() runs over every path there is. Left alone, opening that
// road to clear the block signal would light a shunting signal beside it, put it in the
// crossings' approach logic through signalGivesAuthority, and offer it on the map as
// something the operator could set by hand - which is the one thing an automatic signal
// must not be. Called on the dwarf list before mergeSignals.
void dropBlockRoads(std::vector<SignalPlacement>& dwarfs, const std::vector<char>& blockRoad);
