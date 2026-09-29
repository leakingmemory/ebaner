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

#include <vector>

class Consist;

// A train that drives itself forward: within the speed limits, up to the signals, and to
// a stand where a stand is called for.
//
// Every train in the simulator was driven by hand, which is workable for one and
// impossible for a scenario - the level crossings, the block signals and the train-order
// network are all about trains meeting each other, and there was no way to have a second
// one moving. This is the first mode of the answer, and it is deliberately the simple
// one: forward along whatever road the switches are set for, and no shunting, no
// reversing and no timetable.
//
// The judgement is kept apart from the world. What the road holds is gathered by the
// caller, which is the only part that needs the dataset, the signals and the stations;
// what to DO about it is planDrive below, which is arithmetic on a list of numbers and can
// be tested without any of that. See tests/AutoDriveTest.cpp.

// What stands on the road ahead, in metres from the nose, nearest first.
struct RoadAhead {
    // A ceiling taking effect `d` metres ahead and holding until the next one.
    struct Limit {
        float d = 0.0f;
        int kmh = 0;
    };
    // Something to stop at: a signal that is not offering a road, or the flag post of a
    // manned station that is not waving the train through.
    enum class StopKind { Signal, FlagPost };
    struct Stop {
        float d = 0.0f;
        StopKind kind = StopKind::Signal;
        int ref = -1; // which signal placement / flag post, for the caller's own bookkeeping
    };

    std::vector<Limit> limits;
    std::vector<Stop> stops;
    int here = 40;         // the ceiling the train is under where it stands
    bool roadRunsOut = false; // the walk hit the end of the track within the lookahead
    // Whether this movement is a shunt - a dwarf is what is letting it go, rather than a
    // main signal. Not a property of the road alone, since it depends on what the train
    // has already passed, but it comes out of the same scan and the caller keeps it.
    bool shunting = false;
};

// What the driver has decided: a speed to hold, and whether it is coming to a stand.
struct DriverDemand {
    float targetMs = 0.0f;  // hold this
    bool stopping = false;  // ...and it is a stop, not a limit
    float stopIn = 0.0f;    // metres to the mark it is stopping at (only if `stopping`)
    RoadAhead::StopKind stopKind = RoadAhead::StopKind::Signal;
    int stopRef = -1;
    // The deceleration the road is actually asking for at the speed the train is doing:
    // the hardest of (v^2 - v_limit^2) / 2d over everything ahead. This, rather than how
    // far over a target the train is, is what picks the brake notch - which is how a
    // driver does it, and the only way to arrive AT a restriction rather than well under
    // it or well past it.
    float needDecel = 0.0f; // m/s^2
};

// The rate the driver plans his braking at. Deliberately gentler than the brake can
// manage - full service on a Class 93 is 1.3 m/s^2 - because a driver who plans to the
// limit of the brake arrives at every restriction having used all of it, and has nothing
// left for the thing he did not plan for.
inline constexpr float kAutoDecel = 0.5f; // m/s^2
// How far short of the mark to come to a stand. A signal is passed at danger by a
// centimetre as surely as by a metre.
inline constexpr float kStopShortM = 5.0f;
// And how far before a speed restriction to be down to it. The controller allows itself
// a little over the target before it brakes, so aiming AT the board puts the train a few
// km/h over as it crosses - which is exactly the thing the board is there to prevent.
// A driver is at the limit before the limit starts.
inline constexpr float kLimitEarlyM = 60.0f;
// How far ahead to read the road. Enough to lose 130 km/h at kAutoDecel (1.3 km) with
// room to see the next thing beyond it.
inline constexpr float kLookAheadM = 2000.0f;

// The whole policy: the fastest this train may be going *now* such that every limit and
// every stop ahead can still be met by braking at kAutoDecel.
//
// v = sqrt(v_limit^2 + 2*a*d) over each of them, and the smallest wins. A stop is the same
// with a limit of zero, placed kStopShortM before the mark.
DriverDemand planDrive(const RoadAhead& road, float speedMs);

// Work the handles toward a demand, `dt` seconds since the last time this was called.
//
// Two things keep it from hunting, and both are what a driver does rather than what a
// controller does. There is a wide band around the target inside which NOTHING moves -
// a train two km/h under the limit is a train at the limit, not a train to be corrected.
// And a handle that has just moved is left alone for a moment before it moves again, so
// the train answers one notch before being given another. Only an increase in braking
// ignores that, because a brake that has to wait is not a brake.
//
// `cab` is the driving position; the caller has already put it in gear.
void applyDrive(Consist& train, int cab, const DriverDemand& demand, float dt);

// Whether the train is standing at the mark it was stopping for: stopped, and the mark is
// no further than the margin the plan aims for. What tells the caller a station stop is
// complete and the driver should hand back.
bool arrivedAtStop(const DriverDemand& demand, float speedMs);
