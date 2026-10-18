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

#include <string>
#include <vector>

// A place on the railway you can start at.
//
// The export already knows them: every tile's meta.json carries the stations inside it,
// with the name, the node position and what kind of place it is. Only the position can
// be overruled, by the overlay (applyStationOverrides).
struct Station {
    std::string name;    // UTF-8, as the data spells it ("Bodø", "Oteråga")
    glm::dvec3 world{0}; // EPSG:25833, the station node
    char type = 'S';     // 'S' a station (sidings, signals), 'I' a stop
    std::string line;    // "Nordlandsbanen", ...
    bool production = false; // open for play; false while it is still being built

    bool isStop() const { return type == 'I'; }
};

// Every station in the dataset, sorted by line then name in Norwegian alphabetical order
// (norwegianLess), deduplicated: one near a tile
// boundary is written into both tiles. Reads every tile's meta.json, which sounds worse
// than it is - the whole sweep is well under a second.
std::vector<Station> loadStations(const std::string& datasetRoot);

// Move stations to where the overlay says they are. The export's node is sometimes not
// at the station: Stod's sits 3.5 km up the line from its passing loop. Where a run
// starts and which station a flag post, a TXP or a simple entry signal belongs to all
// follow the position, so it is corrected here, once, as the stations are loaded. Lines in `<datasetRoot>/overlay/stations.txt`:
//   station "<name>" <x> <y> <z>
// in world coordinates (EPSG:25833), z the rail height there. World coordinates rather
// than `<trackHex>:<frac>` as the other overlay files anchor things, because the
// stations are loaded before the track. Called by loadStations.
void applyStationOverrides(const std::string& datasetRoot, std::vector<Station>& all);

// Mark the stations that are open for play: finished enough - track, signalling, the
// station's own working - to be offered as a place to start. The rest are still being built
// and the pickers keep them out of the way, under one last row. Lines in
// `<datasetRoot>/overlay/production-stations.txt`:
//   station "<name>"
// Matched by the name exactly as the data spells it. Called by loadStations.
void applyProductionStations(const std::string& datasetRoot, std::vector<Station>& all);

// How a station reads in a picker: "Bodø  Nordlandsbanen", "Vudu  (stop)  Nordlandsbanen".
std::string stationLabel(const Station& s);

// Whether `a` comes before `b` in the Norwegian alphabet: Æ, Ø and Å after Z, in that
// order, and upper and lower case together. Ä, Ö and Ü file as Æ, Ø and Y, É as E.
bool norwegianLess(const std::string& a, const std::string& b);

// What the station pickers show. The top level is the stations open for play, grouped by
// line and alphabetical inside each (the order loadStations leaves them in), and then one
// row that opens everything still being built, in the same order.
// Without a production list there is no second level: every station is on the top one.
struct StationMenu {
    std::vector<std::string> top;
    std::vector<int> topRow; // -> index into the stations, or -1 for the row opening `dev`
    std::vector<std::string> dev;
    std::vector<int> devRow; // -> index into the stations

    static constexpr const char* kDevRow = "Under development (not finished)";
    // Where station `idx` is listed: whether on the top level, and its row on that level.
    bool onTop(int idx) const;
    int row(int idx) const;
    // The top-level row that opens `dev`, or -1 when there is none.
    int devRowIndex() const {
        return !topRow.empty() && topRow.back() < 0 ? static_cast<int>(topRow.size()) - 1
                                                    : -1;
    }
};
StationMenu stationMenu(const std::vector<Station>& all);

// A line as the traffic manager steps through it: its stations in the order the line
// runs, from `<datasetRoot>/overlay/traffic-manager-lines.txt`. A dispatcher works the
// stations with something to work - signalling or a TXP - so the list leaves the rest
// out. A line the file does not list offers every station the data puts on it, in the
// order they are loaded (`listed` false). Lines in the file:
//   line "<line>"
//   station "<name>"
// each station belonging to the line above it. A station may be on more than one line,
// as a junction is.
struct LineStations {
    std::string line;
    std::vector<int> stations; // -> index into the stations
    bool listed = false;
};
// Every line, in Norwegian alphabetical order.
std::vector<LineStations> trafficManagerLines(const std::string& datasetRoot,
                                              const std::vector<Station>& all);

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
