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

#include "TxpGraph.h"

#include <functional>
#include <set>
#include <string>
#include <utility>
#include <vector>

// The train-order network as it actually stands, which is not the same as the graph.
//
// TxpGraph says which stations *could* deal with each other. This says which ones do: a
// station takes part only while it is manned, so the working chain is the manned stations
// and the sections between them, and it is rebuilt every time one opens or closes.
//
// Opening is not a local act. The station between two others takes over a section that
// already belonged to them, so it has to ask, and they have to agree. That is the whole
// of the interlocking here: a station cannot appear in the middle of a section with a
// train in it, because it would then be holding a road it was never told about.
enum class TxpMsgKind { Connect, Accept, Reject };

struct TxpMessage {
    TxpMsgKind kind = TxpMsgKind::Connect;
    std::string from, to;
    std::string reason; // why, on a Reject
};

struct TxpOpenResult {
    bool accepted = false;
    std::vector<TxpMessage> exchange; // in the order it happened, for the log
};

class TxpNetwork {
public:
    // Whether the line between two stations holds no train. Supplied by the caller: the
    // network knows the topology and deliberately nothing about trains.
    using ClearFn = std::function<bool(const std::string&, const std::string&)>;

    // Man a station, asking its neighbours first. Accepted with nothing sent when there
    // is nobody to ask - the first station on a line has no one to agree with.
    TxpOpenResult open(const TxpGraph& g, const std::string& station, const ClearFn& clear);

    // Unman it. The section it held falls back to its neighbours, so the two either side
    // are joined again - which needs nobody's agreement: a longer section under fewer
    // stations takes nothing away from anyone.
    void close(const TxpGraph& g, const std::string& station);

    bool isOpen(const std::string& station) const { return open_.count(station) > 0; }
    std::size_t openCount() const { return open_.size(); }

    // Who this station is working with right now.
    std::vector<std::string> linksOf(const std::string& station) const;
    bool linked(const std::string& a, const std::string& b) const;
    const std::vector<std::pair<std::string, std::string>>& links() const { return links_; }

private:
    // The nearest manned station either side along the line, skipping the closed ones.
    // Empty where there is none, which is what makes the first station a special case
    // without it being written down as one.
    std::string nearestOpen(const TxpGraph& g, const std::string& from, bool forward) const;

    void addLink(const std::string& a, const std::string& b);
    void dropLink(const std::string& a, const std::string& b);

    std::set<std::string> open_;
    std::vector<std::pair<std::string, std::string>> links_;
};
