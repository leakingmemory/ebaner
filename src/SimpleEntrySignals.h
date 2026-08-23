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

#include "Stations.h"
#include "TrackCircuits.h" // TrackPoly, fracToWorld

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

// An entry signal that knows almost nothing.
//
// The entry signals in SignalPaths.h are the full article: a route from border to border,
// a C1/C2 authority, interlocked against circuit occupancy and against other routes. All
// of which needs track circuits drawn through the station, which is why they exist at
// Bodo and nowhere else on the line.
//
// This is the other end of that: red or green, placed anywhere on a track in a moment,
// with no circuits, no route and nothing to reason about. The only rule is that a station
// shows one green at a time, which is what stops both ends of it inviting a train in at
// once. A station usually has two, one at each end.
//
// File `<datasetRoot>/overlay/simple-entry-signals.txt`, shaped like the distant signals
// it is placed the same way as:
//   entry <id> "<name>" <trackHex>:<frac> <+|-> ["<station>"]
struct SimpleEntrySignal {
    int id = 0;
    std::string name;
    std::uint32_t trackId = 0;
    double frac = 0.0;
    int dir = 1; // +1 governs movements toward increasing frac, -1 the other way
    int side = 1; // +1 right of that direction, -1 left; independent of it
    // Which station it belongs to, and so which other signals it is interlocked with.
    // Normally empty, meaning "whichever is nearest" - that is what attachStations works
    // out, and it cannot go stale the way a written-down name could. Set only to overrule
    // the nearest-station rule where it picks the wrong one of two close together.
    std::string station;
};

std::vector<SimpleEntrySignal> loadSimpleEntrySignals(const std::string& datasetRoot);
bool writeSimpleEntrySignals(const std::string& datasetRoot,
                             const std::vector<SimpleEntrySignal>& sigs);

// Which station each signal belongs to, resolved. One entry per signal, parallel to
// `sigs`: the station's name, and how far the signal stands from its node.
//
// A signal much further out than `kFarM` is very unlikely to serve the station it came
// out nearest to, so `far` is set and the editor says so. It is still attached: a signal
// interlocked with nothing at all is worse than one attached to the wrong place, because
// the wrong place is at least visible.
struct SignalStation {
    std::string name; // empty only when the dataset has no stations at all
    double distanceM = 0.0;
    bool far = false;
};
constexpr double kFarM = 4000.0;

// Anything anchored the way these are - a `trackId`, a `frac` and a `station` that is
// empty unless it overrules the nearest - can be attached. That is the entry signals and
// the flag posts, and a template keeps it one implementation rather than two that agree
// only until one of them is edited.
template <typename Anchored>
std::vector<SignalStation> attachStations(const std::vector<Anchored>& items,
                                          const std::vector<Station>& stations,
                                          const std::vector<TrackPoly>& polys) {
    std::vector<SignalStation> out(items.size());
    for (std::size_t i = 0; i < items.size(); ++i) {
        const Anchored& s = items[i];
        const glm::dvec3 w = fracToWorld(polys, s.trackId, s.frac);
        if (w.x == 0.0 && w.y == 0.0) continue; // stale/missing track

        // An override names its station outright; otherwise take the nearest.
        if (!s.station.empty()) {
            out[i].name = s.station;
            if (const Station* st = findStation(stations, s.station))
                out[i].distanceM = std::hypot(st->world.x - w.x, st->world.y - w.y);
            out[i].far = out[i].distanceM > kFarM;
            continue;
        }
        double best = 1e30;
        for (const Station& st : stations) {
            const double d = std::hypot(st.world.x - w.x, st.world.y - w.y);
            if (d < best) {
                best = d;
                out[i].name = st.name;
            }
        }
        if (!out[i].name.empty()) {
            out[i].distanceM = best;
            out[i].far = best > kFarM;
        }
    }
    return out;
}
