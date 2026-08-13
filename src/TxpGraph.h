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

#include "SimpleEntrySignals.h" // SignalStation
#include "Stations.h"
#include "TxpPositions.h"

#include <glm/glm.hpp>
#include <string>
#include <vector>

struct TrackPoly;
class TrackPath;

// Which TXP stations can talk to each other.
//
// On a line worked by train orders a station deals with the one either side of it and
// with nobody else: a train may not leave until the next station along has accepted it.
// That is the graph, and it is derived rather than authored.
//
// Both halves of it fall out of what is already there. A station is a TXP station because
// a TXP position was placed at it - there is no separate way to be one - and two stations
// are neighbours because nothing lies between them on the running line. So there is no
// file to write, nothing to keep in step as positions are added, and no way for the graph
// to disagree with the positions it describes.
//
// What it cannot express, should either ever arise:
//  - a junction. Chaining stations along one line gives each at most two neighbours, so a
//    branch where a station deals with three would need the edges stating outright.
//  - a break in the line. Stations on either side of one land on different paths and are
//    chained separately, which is arguably right - a severed line cannot pass a train
//    order - but it shows up as an edge quietly missing rather than as a fault.
// Both want an authored override when they turn up. Neither does yet, on a line that runs
// Bodø to Rognan without a junction, so neither is built.
struct TxpStationNode {
    std::string name;
    int path = -1;   // the running line it stands on (-1 = could not be placed)
    float s = 0.0f;  // how far along that line, which is what orders them
};

class TxpGraph {
public:
    // `attached` is attachStations(ps, stations, polys), parallel to `ps`.
    void build(const std::vector<TxpPosition>& ps,
               const std::vector<SignalStation>& attached,
               const std::vector<Station>& stations,
               const std::vector<TrackPoly>& polys,
               const std::vector<TrackPath>& paths, const glm::dvec3& origin);

    // In order along the line, so walking them walks the railway.
    const std::vector<TxpStationNode>& nodes() const { return nodes_; }

    // The stations this one deals with: the one either side, so at most two, and one at a
    // terminus. Empty for a station that is not a TXP station at all.
    std::vector<std::string> neighbours(const std::string& station) const;

    // Whether the two deal with each other. Symmetric, as train-order working is: the
    // section between them belongs to both.
    bool linked(const std::string& a, const std::string& b) const;

    // The next station along from `station`, further down the line when `forward` (that
    // is, toward increasing arc length) and back up it otherwise. Empty at the end of the
    // chain, which is what a train has to be told when there is nobody to offer it to.
    std::string next(const std::string& station, bool forward) const;

private:
    int indexOf(const std::string& station) const;

    std::vector<TxpStationNode> nodes_;
    // Index pairs into `nodes_`, each an undirected edge. Kept as well as the ordering
    // because a chain broken into two by a severed line is not the same as one chain.
    std::vector<std::pair<int, int>> edges_;
};
