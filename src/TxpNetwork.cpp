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

#include "TxpNetwork.h"

#include <algorithm>

std::string TxpNetwork::nearestOpen(const TxpGraph& g, const std::string& from,
                                    bool forward) const {
    const std::vector<TxpStationNode>& ns = g.nodes();
    int i = -1;
    for (std::size_t k = 0; k < ns.size(); ++k)
        if (ns[k].name == from) i = static_cast<int>(k);
    if (i < 0) return {};
    const int step = forward ? 1 : -1;
    for (int k = i + step; k >= 0 && k < static_cast<int>(ns.size()); k += step) {
        // Never past the end of the line this station is on: a station beyond a break is
        // not reachable, so there is nothing to agree with it about.
        if (ns[k].path != ns[i].path) return {};
        if (isOpen(ns[k].name)) return ns[k].name;
    }
    return {};
}

void TxpNetwork::addLink(const std::string& a, const std::string& b) {
    if (a.empty() || b.empty() || a == b || linked(a, b)) return;
    links_.push_back({a, b});
}

void TxpNetwork::dropLink(const std::string& a, const std::string& b) {
    links_.erase(std::remove_if(links_.begin(), links_.end(),
                                [&](const std::pair<std::string, std::string>& l) {
                                    return (l.first == a && l.second == b) ||
                                           (l.first == b && l.second == a);
                                }),
                 links_.end());
}

bool TxpNetwork::linked(const std::string& a, const std::string& b) const {
    for (const auto& l : links_)
        if ((l.first == a && l.second == b) || (l.first == b && l.second == a)) return true;
    return false;
}

std::vector<std::string> TxpNetwork::linksOf(const std::string& station) const {
    std::vector<std::string> out;
    for (const auto& l : links_) {
        if (l.first == station) out.push_back(l.second);
        else if (l.second == station) out.push_back(l.first);
    }
    return out;
}

TxpOpenResult TxpNetwork::open(const TxpGraph& g, const std::string& station,
                               const ClearFn& clear) {
    TxpOpenResult r;
    if (station.empty() || isOpen(station)) {
        r.accepted = true;
        return r;
    }
    // A station with no TXP position is not part of this at all. It can still be manned -
    // it has its own signals - but it works no sections and asks nobody.
    bool known = false;
    for (const TxpStationNode& n : g.nodes())
        if (n.name == station) known = true;
    if (!known) {
        r.accepted = true;
        return r;
    }

    const std::string down = nearestOpen(g, station, false);
    const std::string up = nearestOpen(g, station, true);

    // Nobody manned within reach: the first station on the line opens on its own, with
    // nothing to send and nobody to refuse it.
    if (down.empty() && up.empty()) {
        open_.insert(station);
        r.accepted = true;
        return r;
    }

    // What has to be clear is the section this station is about to take over. Between two
    // manned stations that is the one they hold between them, which is about to become
    // two; at the end of the chain it is the stretch out to the one neighbour. Either way
    // a train inside it is one the new station would not know it had.
    const bool splitting = !down.empty() && !up.empty();
    std::string secA, secB;
    if (splitting) { secA = down; secB = up; }        // the one about to become two
    else if (!down.empty()) { secA = down; secB = station; } // out to the one neighbour
    else { secA = station; secB = up; }

    for (const std::string& peer : {down, up}) {
        if (peer.empty()) continue;
        r.exchange.push_back({TxpMsgKind::Connect, station, peer, {}});
    }
    const bool ok = !clear || clear(secA, secB);
    for (const std::string& peer : {down, up}) {
        if (peer.empty()) continue;
        if (ok) {
            r.exchange.push_back({TxpMsgKind::Accept, peer, station, {}});
        } else {
            r.exchange.push_back({TxpMsgKind::Reject, peer, station,
                                  "the line " + secA + " - " + secB + " is occupied"});
        }
    }
    if (!ok) return r; // refused: nothing opens and no link moves

    // Agreed. The station is manned, and the section it stood in the middle of becomes
    // two - so the link that spanned it goes, which is the point of asking both.
    open_.insert(station);
    if (splitting) dropLink(down, up);
    addLink(down, station);
    addLink(station, up);
    r.accepted = true;
    return r;
}

void TxpNetwork::close(const TxpGraph& g, const std::string& station) {
    if (!isOpen(station)) return;
    const std::string down = nearestOpen(g, station, false);
    const std::string up = nearestOpen(g, station, true);
    dropLink(down, station);
    dropLink(station, up);
    open_.erase(station);
    // The two either side inherit the whole of what it held between them.
    if (!down.empty() && !up.empty()) addLink(down, up);
}
