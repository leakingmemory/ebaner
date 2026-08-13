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

const char* txpTrainTypeName(TxpTrainType t) {
    switch (t) {
        case TxpTrainType::Passenger: return "passenger";
        case TxpTrainType::Cargo: return "cargo";
        case TxpTrainType::Other: return "other";
    }
    return "?";
}

namespace {
// What a held section is called when refusing something else on it.
std::string heldAs(const TxpSection& s) {
    return s.state == TxpLineState::Prepared
               ? "already given to a " + std::string(txpTrainTypeName(s.type)) +
                     " train from " + s.from
               : "a " + std::string(txpTrainTypeName(s.type)) + " train from " + s.from +
                     " is on it";
}
} // namespace

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

TxpSection* TxpNetwork::find(const std::string& a, const std::string& b) {
    for (TxpSection& s : links_)
        if (s.joins(a, b)) return &s;
    return nullptr;
}

const TxpSection* TxpNetwork::section(const std::string& a, const std::string& b) const {
    for (const TxpSection& s : links_)
        if (s.joins(a, b)) return &s;
    return nullptr;
}

TxpLineState TxpNetwork::state(const std::string& a, const std::string& b) const {
    const TxpSection* s = section(a, b);
    return s ? s->state : TxpLineState::Clear;
}

std::vector<const TxpSection*> TxpNetwork::sectionsAt(const std::string& station) const {
    std::vector<const TxpSection*> out;
    for (const TxpSection& s : links_)
        if (s.a == station || s.b == station) out.push_back(&s);
    return out;
}

void TxpNetwork::addLink(const std::string& a, const std::string& b) {
    if (a.empty() || b.empty() || a == b || linked(a, b)) return;
    links_.push_back(TxpSection{a, b, TxpLineState::Clear, {}, TxpTrainType::Passenger});
}

void TxpNetwork::dropLink(const std::string& a, const std::string& b) {
    links_.erase(std::remove_if(links_.begin(), links_.end(),
                                [&](const TxpSection& s) { return s.joins(a, b); }),
                 links_.end());
}

bool TxpNetwork::linked(const std::string& a, const std::string& b) const {
    return section(a, b) != nullptr;
}

std::vector<std::string> TxpNetwork::linksOf(const std::string& station) const {
    std::vector<std::string> out;
    for (const TxpSection& s : links_) {
        if (s.a == station) out.push_back(s.b);
        else if (s.b == station) out.push_back(s.a);
    }
    return out;
}

TxpExchange TxpNetwork::open(const TxpGraph& g, const std::string& station,
                             const ClearFn& clear) {
    TxpExchange r;
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

    // Two reasons to refuse, and they are different things. A train order on the section
    // is a refusal by the books: the station cannot take over a line that is spoken for,
    // because the order was made with the far end and it would now be in the middle of it.
    // A train standing in it is a refusal by the ground, which is what `clear` is for.
    std::string why;
    if (const TxpSection* held = splitting ? section(down, up) : nullptr;
        held && held->held()) {
        why = "the line " + secA + " - " + secB + " is " + heldAs(*held);
    } else if (clear && !clear(secA, secB)) {
        why = "the line " + secA + " - " + secB + " is occupied";
    }
    for (const std::string& peer : {down, up}) {
        if (peer.empty()) continue;
        if (why.empty()) r.exchange.push_back({TxpMsgKind::Accept, peer, station, {}});
        else r.exchange.push_back({TxpMsgKind::Reject, peer, station, why});
    }
    if (!why.empty()) return r; // refused: nothing opens and no link moves

    // Agreed. The station is manned, and the section it stood in the middle of becomes
    // two - so the link that spanned it goes, which is the point of asking both.
    open_.insert(station);
    if (splitting) dropLink(down, up);
    addLink(down, station);
    addLink(station, up);
    r.accepted = true;
    return r;
}

TxpExchange TxpNetwork::close(const TxpGraph& g, const std::string& station) {
    TxpExchange r;
    if (!isOpen(station)) {
        r.accepted = true;
        return r;
    }
    // A station holding a train order cannot walk away from it: the two sections either
    // side are about to become one, and one line cannot carry two sets of books - nor
    // would anyone be left to report the train arrived.
    for (const TxpSection* s : sectionsAt(station)) {
        if (!s->held()) continue;
        const std::string other = s->a == station ? s->b : s->a;
        r.exchange.push_back({TxpMsgKind::Reject, station, station,
                              "the line to " + other + " is " + heldAs(*s)});
        return r;
    }
    const std::string down = nearestOpen(g, station, false);
    const std::string up = nearestOpen(g, station, true);
    dropLink(down, station);
    dropLink(station, up);
    open_.erase(station);
    // The two either side inherit the whole of what it held between them.
    if (!down.empty() && !up.empty()) addLink(down, up);
    r.accepted = true;
    return r;
}

TxpExchange TxpNetwork::requestDispatch(const std::string& from, const std::string& to,
                                        TxpTrainType type) {
    TxpExchange r;
    TxpSection* s = find(from, to);
    if (!s) {
        r.exchange.push_back({TxpMsgKind::Reject, to, from,
                              "no line is worked between " + from + " and " + to});
        return r;
    }
    r.exchange.push_back({TxpMsgKind::Request, from, to, txpTrainTypeName(type)});
    // Answered from the books and from nothing else. A section already given away is
    // given away whoever asks and whichever end they ask from.
    if (s->held()) {
        r.exchange.push_back({TxpMsgKind::Reject, to, from, "the line is " + heldAs(*s)});
        return r;
    }
    s->state = TxpLineState::Prepared;
    s->from = from;
    s->type = type;
    r.exchange.push_back({TxpMsgKind::LineClear, to, from, {}});
    r.accepted = true;
    return r;
}

TxpExchange TxpNetwork::trainOnTrack(const std::string& from, const std::string& to) {
    TxpExchange r;
    TxpSection* s = find(from, to);
    if (!s || s->state != TxpLineState::Prepared || s->from != from) {
        r.exchange.push_back(
            {TxpMsgKind::Reject, to, from,
             !s ? "no line is worked there"
                : s->state == TxpLineState::Occupied ? "that train is already on the line"
                : s->state == TxpLineState::Clear    ? "no train has been given that line"
                                                     : "that line was given to " + s->from});
        return r;
    }
    s->state = TxpLineState::Occupied;
    r.exchange.push_back({TxpMsgKind::OnTrack, from, to, {}});
    r.accepted = true;
    return r;
}

TxpExchange TxpNetwork::trainArrived(const std::string& at, const std::string& from) {
    TxpExchange r;
    TxpSection* s = find(at, from);
    // Only the far end reports an arrival. A station reporting the arrival of a train it
    // dispatched itself would be clearing a line the train is still on.
    if (!s || s->state != TxpLineState::Occupied || s->from != from || s->from == at) {
        r.exchange.push_back(
            {TxpMsgKind::Reject, at, at,
             !s ? "no line is worked there"
                : s->state != TxpLineState::Occupied ? "no train is on that line"
                                                     : "that train is not running to here"});
        return r;
    }
    s->state = TxpLineState::Clear;
    s->from.clear();
    r.exchange.push_back({TxpMsgKind::Arrived, at, from, {}});
    r.accepted = true;
    return r;
}

TxpExchange TxpNetwork::cancelDispatch(const std::string& from, const std::string& to) {
    TxpExchange r;
    TxpSection* s = find(from, to);
    // Only while it has not left. Afterwards there is a train on the line and saying it
    // is not coming does not make it so - the arrival is the only thing that clears it.
    if (!s || s->state != TxpLineState::Prepared || s->from != from) {
        r.exchange.push_back(
            {TxpMsgKind::Reject, to, from,
             !s ? "no line is worked there"
                : s->state == TxpLineState::Occupied ? "the train has already left"
                : s->state == TxpLineState::Clear    ? "nothing is booked on that line"
                                                     : "that line was given to " + s->from});
        return r;
    }
    s->state = TxpLineState::Clear;
    s->from.clear();
    r.exchange.push_back({TxpMsgKind::Cancel, from, to, {}});
    r.accepted = true;
    return r;
}
