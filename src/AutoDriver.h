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
    // Something to stop at: a signal that is not offering a road, a manned station that
    // is not waving the train through, or the end of the rails.
    enum class StopKind { Signal, Station, EndOfTrack };
    struct Stop {
        float d = 0.0f;
        StopKind kind = StopKind::Signal;
        int ref = -1; // which signal placement / flag post, for the caller's own bookkeeping
    };

    std::vector<Limit> limits;
    std::vector<Stop> stops;
    int here = 40;         // the ceiling the train is under where it stands
    // The walk hit the end of the road inside the lookahead - the rails stop, or a
    // facing switch is set against the train and there is nowhere to go. It arrives as a
    // stop like any other rather than as a state of its own, so it is planned for with
    // the same braking curve as a signal.
    bool roadRunsOut = false;
    // The train is already past the place it was told to stand, and is still moving.
    // The stop is raised at zero rather than dropped, so the driver puts the brake on
    // and comes to a stand wherever it can - but the caller wants to know, because an
    // overrun is the end of the movement and not a stop that was served properly.
    bool overshotStop = false;
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

// The rate the driver plans his braking at, far out from whatever he is braking for.
// Deliberately gentler than the brake can manage - full service on a Class 93 is
// 1.3 m/s^2 - because a driver who plans to the limit of the brake arrives at every
// restriction having used all of it, and has nothing left for what he did not plan for.
inline constexpr float kAutoDecel = 0.55f; // m/s^2
// ...and the rate he plans for the last stretch of it. Shallower, so the brake is coming
// OFF as the mark arrives rather than still going on: a curve planned at one rate all the
// way is steepest exactly where it matters, and a train following it is still shedding
// speed hard at the moment it should be settling. Easing the plan at the end means the
// speed is already low when the distance runs out, which is both how it is driven and
// where the margin against overshooting comes from.
// Below what the lightest notch is worth (B1 gives 0.35), which is deliberate: a rate
// between notches is held by cycling the handle, and the average is what the train
// feels. A light set answers quickly enough for that to be smooth, and a long heavy one
// averages it along its own length anyway - it is slow to apply and slow to let go, so
// asking it for a rate it cannot hold steadily is exactly what a light cycled
// application avoids.
inline constexpr float kEndDecel = 0.25f; // m/s^2, at the mark
// How far out the easing starts, and it is generous on purpose. The transition has to be
// early enough that the train is ALREADY braking lightly well before the mark: come down
// the steep part too long and it arrives under the curve with the brake still hard on,
// stops short, and has to be driven up again.
inline constexpr float kShallowM = 400.0f;
// What the brake costs before it does anything: the seconds between the handle moving
// and the shoes being on, which the train runs through at whatever speed it had. Counted
// out of the distance available rather than hidden in a fudge factor, so it scales with
// speed - and with the train, because a long one is slower to apply and slower to let go.
inline constexpr float kBuildUpS = 3.0f;
// How far short of the mark to come to a stand.
inline constexpr float kStopShortM = 20.0f;
// And how close is close enough to stop planning and simply hold it. Inside this the
// brake goes on and stays on: there is nothing to be gained by easing a train the last
// twenty metres up to a signal at danger, and a good deal to be lost.
inline constexpr float kStopHoldM = 25.0f;
// And how far short of the END of the rails, which is a different kind of mark: there is
// no signal there to stop at, only a buffer stop or a broken rail, and nothing is gained
// by going near it. On top of the stopping margin, so the train stands 40 m off.
inline constexpr float kEndOfTrackM = 20.0f;
// And how far before a speed restriction to be down to it. The controller allows itself
// a little over the target before it brakes, so aiming AT the board puts the train a few
// km/h over as it crosses - which is exactly the thing the board is there to prevent.
// A driver is at the limit before the limit starts.
inline constexpr float kLimitEarlyM = 60.0f;
// How far ahead to read the road. Enough to lose 130 km/h at kAutoDecel (1.3 km) with
// room to see the next thing beyond it.
inline constexpr float kLookAheadM = 2000.0f;

// Where a train stops at a station worked by hand signals, in metres ahead of its nose.
//
// The flag post is not it. The post marks where the TXP stands, which is a good thing to
// be near and a poor thing to stop at: what a train crossing another one has to do is
// stand clear of the switches at BOTH ends, so the other train can get past it. That is
// the band, and everything else is a preference inside it.
//
// Filled in by the caller, which is the only part that knows what a turnout or a platform
// is; the choosing is here so it can be tested to the metre with no dataset at all.
struct StationStop {
    // The crossing area: the outermost switches on the road this train is on. On a loop
    // that is where its road leaves the main and rejoins it, on the main the outermost
    // pair. Without it there is nothing to reason about and `fallback` is used.
    float bandFrom = 0.0f, bandTo = 0.0f;
    bool haveBand = false;
    // A platform over this road, if any, and where the TXP stands for it.
    float platformFrom = 0.0f, platformTo = 0.0f;
    bool havePlatform = false;
    float txpAt = 0.0f;
    bool haveTxp = false;
    // The station node put on the road, for a station with no switches on this road at
    // all - a dead-end siding, or a stopping place that is not a crossing station.
    float fallback = 0.0f;
    bool passenger = false; // a platform is only of interest to a train carrying people
    // The two things that are the TRAIN's and not the station's, and without which the
    // band cannot be used properly: how much road the train occupies, and which way it
    // is running along it. A 600 m freight at Rognan, whose switches are 465 m apart,
    // cannot stand clear at both ends however the point is chosen - but it can still
    // choose to hang out of the end it came in by rather than the end it is going to.
    float trainLength = 0.0f;
    bool towardHi = true; // travelling toward the larger coordinate along this road
};

// How far clear of a switch a train stands. Not a fudge: the band's ends ARE the
// switches, and a stop computed exactly on one puts the train on the thing the whole
// rule exists to keep it off.
inline constexpr float kSwitchClearM = 25.0f;

// The band, and inside it: a platform for a passenger train, positioned as near the TXP
// as the platform allows; else the TXP itself; else the middle of the band. A platform
// lying outside the band is ignored rather than clamped to its edge - crossing comes
// first, and the edge of the band is exactly where a train must not stand.
float stationStopPoint(const StationStop& s);

// The hardest brake notch worth using at this speed. Published so the rule can be tested
// for what it is: a cap on the SPEED, not on how far over a limit the train happens to
// be. Five km/h over a five km/h limit is a hundred per cent and is still five km/h.
int notchCapAt(float speedMs);

// The whole policy: the fastest this train may be going *now* such that every limit and
// every stop ahead can still be met.
//
// v = sqrt(v_limit^2 + 2*a*d) over each of them, and the smallest wins - but neither `a`
// nor `d` is quite what it looks like. `d` has the build-up run taken out of it, because
// the first seconds of a brake application stop nothing. And `a` is not one number: it
// eases from kAutoDecel far out to kEndDecel at the mark, so the plan asks for less
// braking exactly where a curve of one rate asks for most.
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
