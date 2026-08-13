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
#include <vector>

// The train-order network as it actually stands, which is not the same as the graph.
//
// TxpGraph says which stations *could* deal with each other. This says which ones do, and
// what is booked on the line between them. A station takes part only while it is manned,
// so the working chain is the manned stations and the sections between them, and it is
// rebuilt every time one opens or closes.
//
// Two things happen here, and both are refusals as much as they are acts. Opening is not
// local: the station between two others takes over a section that already belonged to
// them, so it has to ask. And a train is not simply driven away: the line is asked for,
// granted, held while the train is on it, and released on arrival - and while it is held
// nothing else may have it.
enum class TxpMsgKind {
    Connect,   // may I join the chain
    Accept,    // you may
    Reject,    // you may not, and here is why
    Request,   // may I send a train
    LineClear, // you may - the line is yours
    OnTrack,   // the train has left and is on the line
    Arrived,   // it is here; the line is clear again
    Cancel,    // it is not coming after all
};

// What kind of train an order is for. Carried through the exchange because it is what the
// far end writes in its book, not because anything in the sim behaves differently.
enum class TxpTrainType { Passenger, Cargo, Other };
const char* txpTrainTypeName(TxpTrainType t);

struct TxpMessage {
    TxpMsgKind kind = TxpMsgKind::Connect;
    std::string from, to;
    std::string reason; // why, on a Reject
};

// What came of an act, and everything said doing it. Named for the exchange rather than
// for any one act because opening, requesting, dispatching and releasing all produce one.
struct TxpExchange {
    bool accepted = false;
    std::vector<TxpMessage> exchange; // in the order it happened, for the log
};

// One worked line, between two manned stations.
//
// The pair is undirected - the section belongs to both ends equally - but an order on it
// runs one way, which is what `from` records. That is what decides which end may report
// the train onto the line and which may report it arrived: the two are not
// interchangeable, and a station reporting the arrival of a train it dispatched itself
// would clear a line with a train still on it.
enum class TxpLineState {
    Clear,    // nothing booked
    Prepared, // given to a train that has not left yet
    Occupied, // the train is on it
};

struct TxpSection {
    std::string a, b;
    TxpLineState state = TxpLineState::Clear;
    std::string from; // the end the train runs from, while it is booked
    TxpTrainType type = TxpTrainType::Passenger;

    bool held() const { return state != TxpLineState::Clear; }
    bool joins(const std::string& x, const std::string& y) const {
        return (a == x && b == y) || (a == y && b == x);
    }
};

class TxpNetwork {
public:
    // Whether the line between two stations holds no train. Supplied by the caller: the
    // network knows the topology and deliberately nothing about trains.
    using ClearFn = std::function<bool(const std::string&, const std::string&)>;

    // --- Manning ------------------------------------------------------------------
    // Man a station, asking its neighbours first. Accepted with nothing sent when there
    // is nobody to ask - the first station on a line has no one to agree with.
    TxpExchange open(const TxpGraph& g, const std::string& station, const ClearFn& clear);

    // Unman it. The section it held falls back to its neighbours, so the two either side
    // are joined again - which needs nobody's agreement: a longer section under fewer
    // stations takes nothing away from anyone.
    //
    // Refused while it has a train booked on either side of it. Merging two sections into
    // one would leave a single line carrying two sets of books, and the order would be
    // stranded with nobody left to report it arrived.
    TxpExchange close(const TxpGraph& g, const std::string& station);

    // --- Train orders -------------------------------------------------------------
    // Ask the far end for the line. Answered from the books alone: refused because the
    // section is already booked, not because something is standing in it. Granted, both
    // ends hold it - there is one section, not one record per station.
    TxpExchange requestDispatch(const std::string& from, const std::string& to,
                                TxpTrainType type);
    // The train has left. Only the end it was granted to may say so.
    TxpExchange trainOnTrack(const std::string& from, const std::string& to);
    // It is here, and the line is clear again. Only the far end may say so: a station
    // cannot report the arrival of a train it sent.
    TxpExchange trainArrived(const std::string& at, const std::string& from);
    // It is not coming after all. Only until it has left - after that there is a train
    // out there and nothing but its arrival releases the line.
    TxpExchange cancelDispatch(const std::string& from, const std::string& to);

    // --- Reading it ---------------------------------------------------------------
    bool isOpen(const std::string& station) const { return open_.count(station) > 0; }
    std::size_t openCount() const { return open_.size(); }

    std::vector<std::string> linksOf(const std::string& station) const;
    bool linked(const std::string& a, const std::string& b) const;
    const std::vector<TxpSection>& links() const { return links_; }

    // The section between two stations, or null if they do not work one together.
    const TxpSection* section(const std::string& a, const std::string& b) const;
    TxpLineState state(const std::string& a, const std::string& b) const;
    // Every section this station is an end of, held or not.
    std::vector<const TxpSection*> sectionsAt(const std::string& station) const;

private:
    // The nearest manned station either side along the line, skipping the closed ones.
    // Empty where there is none, which is what makes the first station a special case
    // without it being written down as one.
    std::string nearestOpen(const TxpGraph& g, const std::string& from, bool forward) const;

    TxpSection* find(const std::string& a, const std::string& b);
    void addLink(const std::string& a, const std::string& b);
    void dropLink(const std::string& a, const std::string& b);

    std::set<std::string> open_;
    std::vector<TxpSection> links_;
};
