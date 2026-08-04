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

#include "SwitchNetwork.h" // SwitchNetwork, SwitchState (path alignment)
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

// --- Moving a border (see canMoveBorder/moveBorderFrac in TrackCircuits.h) ---
// Would moving the border at (trackId, oldFrac) to `newFrac` break a route? A path may
// legitimately end at a non-border point (a junction or a track end), which the
// border-neighbour check knows nothing about, so this catches both a route interval
// collapsing to nothing and one *reversing* - the latter happens when the border is
// dragged past a junction the route uses, which would silently turn the route (and its
// signal) around. `why` gets a human-readable reason.
bool borderMoveBreaksPath(const std::vector<SignalPath>& paths,
                          const std::vector<TrackPoly>& polys, std::uint32_t trackId,
                          double oldFrac, double newFrac, std::string& why);
// Rewrite every path endpoint and route-interval endpoint anchored to that border. Matches
// on track id *and* fraction. Returns how many values changed.
int moveBorderFrac(std::vector<SignalPath>& paths, const std::vector<TrackPoly>& polys,
                   std::uint32_t trackId, double oldFrac, double newFrac);

// --- File IO (mirrors loadTrackCircuits/writeTrackCircuits) ---
std::vector<SignalPath> loadSignalPaths(const std::string& datasetRoot);
bool writeSignalPaths(const std::string& datasetRoot,
                      const std::vector<SignalPath>& paths);

// What a mini ground signal displays: the fixed reference lamp plus one lamp on the arc.
// Stop = horizontal pair, TrainOnTrack = 45 deg (a train stands in the route's circuits),
// Clear = vertical (route set and clear; not driven yet).
enum class SignalAspect { Stop, TrainOnTrack, Clear };

// Where a ground signal sits: the on-track start point of a path and the path's initial
// travel direction (a signal governs movements leaving that point in that direction).
struct SignalPlacement {
    glm::dvec3 world{0.0};   // start-border world position (on the track)
    glm::dvec2 forward{0.0}; // unit travel direction leaving the border
    std::vector<int> paths;  // indices of the signal paths this signal governs
    SignalAspect aspect = SignalAspect::Stop;
};

// One placement per distinct (start border, travel direction) over all paths - so paths
// sharing a start collapse to a single signal (governing all of them).
std::vector<SignalPlacement> signalPlacements(const std::vector<SignalPath>& paths,
                                              const std::vector<TrackPoly>& polys);

// A turnout the path traverses and the position it needs there: straight where the path
// runs through the turnout, diverging where it crosses to or from the branch.
struct PathSwitch {
    int turnout = -1;
    SwitchState need = SwitchState::Straight;
};

// Every turnout the path traverses, with the position that path needs.
std::vector<PathSwitch> pathSwitchRequirements(const SignalPath& p, const SwitchNetwork& net,
                                               const std::vector<TrackPoly>& polys);

// True if every turnout the path traverses currently sits in the position that path needs.
bool pathSwitchesAligned(const SignalPath& p, const SwitchNetwork& net,
                         const std::vector<TrackPoly>& polys);

// The ids of the track-circuit sections the path actually runs through (a shared end
// point with a neighbouring section does not count as running through it).
std::vector<int> pathSections(const SignalPath& p, const TrackCircuits& circuits);

// Recompute each signal's aspect: Clear when one of its paths has a route set, else
// TrainOnTrack when one of its paths has its switches aligned and a train standing in that
// path's circuits, else Stop. `secOccupied` is indexed like `circuits.sections` and
// `routeSet` like `paths`. Returns true if any aspect changed.
bool updateSignalAspects(std::vector<SignalPlacement>& placements,
                         const std::vector<SignalPath>& paths, const SwitchNetwork& net,
                         const std::vector<TrackPoly>& polys,
                         const TrackCircuits& circuits,
                         const std::vector<char>& secOccupied,
                         const std::vector<char>& routeSet);
