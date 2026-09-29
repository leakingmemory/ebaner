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

#include "AutoDriver.h"

#include "Consist.h"
#include "Vehicle.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr float kKmhToMs = 1.0f / 3.6f;

// The bands around the target. Between kSettleBandMs and kUnderBandMs under it nothing
// moves at all: a train two km/h below the limit is a train at the limit, and correcting
// that is what hunting IS. Above kSettleBandMs of the target the power eases back so the
// train settles on to the limit instead of running through it and being braked, and only
// past kOverBandMs over it does a brake go on for the limit alone.
//
// The first version had these at 0.6 and 0.4 m/s with no settle band at all, and eased
// the power back on every tick it was over target - so it powered, overshot, braked, fell
// under, powered, five times a second, which is audible from outside the train.
constexpr float kUnderBandMs = 1.0f;  // m/s under target: take another notch
constexpr float kSettleBandMs = 0.3f; // m/s under target: start easing off
constexpr float kOverBandMs = 1.0f;   // m/s over target: a brake, not just less power
// The speed a driver draws up to a signal at, and the only speed he will use power to
// hold on the approach to one. Below it a train climbing to a signal would stall short;
// above it, powering toward a signal at danger is nobody's idea of driving.
constexpr float kApproachMs = 5.0f; // m/s, about 18 km/h
// How long a handle is left alone after it moves. A driver gives a notch time to answer.
constexpr float kDwellS = 1.5f;

// The braking curve asks for a speed that reaches zero AT the mark, which is a speed no
// controller can hold; below this the driver is simply stopping.
constexpr float kCrawlMs = 0.8f;
// Ask for rather more than the arithmetic says, because the cylinders take two or three
// seconds to fill and the train keeps running while they do.
constexpr float kBrakeMargin = 1.15f;

} // namespace

DriverDemand planDrive(const RoadAhead& road, float speedMs) {
    DriverDemand out;
    // The ceiling where the train stands. Everything else can only lower it.
    out.targetMs = static_cast<float>(road.here) * kKmhToMs;

    // Each limit ahead: the speed from which that limit can still be reached by braking
    // at kAutoDecel over the distance to it. Beyond the point where the drop happens the
    // limit itself is the answer, which falls out of d <= 0.
    for (const RoadAhead::Limit& l : road.limits) {
        const float vl = static_cast<float>(l.kmh) * kKmhToMs;
        const float d = std::max(0.0f, l.d - kLimitEarlyM);
        const float allow = std::sqrt(vl * vl + 2.0f * kAutoDecel * d);
        out.targetMs = std::min(out.targetMs, allow);
        // The same early margin in the strength as in the curve, or the brake is chosen
        // for a restriction thirty metres further on than the one being aimed at.
        if (speedMs > vl)
            out.needDecel = std::max(out.needDecel,
                                     (speedMs * speedMs - vl * vl) / (2.0f * std::max(d, 1.0f)));

    }

    // A stop is a limit of zero, placed short of the mark so the train stands before it
    // and not on it. The nearest one that actually binds is the one reported, so the HUD
    // and the caller's "have we arrived" test agree about which mark is being aimed at.
    float bestStopAllow = 1e9f;
    for (const RoadAhead::Stop& s : road.stops) {
        const float d = std::max(0.0f, s.d - kStopShortM);
        const float allow = std::sqrt(2.0f * kAutoDecel * d);
        if (allow < bestStopAllow) {
            bestStopAllow = allow;
            out.stopIn = s.d;
            out.stopKind = s.kind;
            out.stopRef = s.ref;
        }
        out.needDecel =
            std::max(out.needDecel, speedMs * speedMs / (2.0f * std::max(d, 1.0f)));
    }
    if (bestStopAllow < out.targetMs) {
        out.targetMs = bestStopAllow;
        out.stopping = true;
    }

    // The end of the track is a stop like any other, and a train that drives off the end
    // of its road is the one failure this mode must not have.
    if (road.roadRunsOut) out.targetMs = std::min(out.targetMs, kCrawlMs);

    out.targetMs = std::max(0.0f, out.targetMs);
    return out;
}

void applyDrive(Consist& train, int cab, const DriverDemand& demand, float dt) {
    const float v = train.speed();
    const int power = train.powerNotch(cab);
    const int brake = train.brakeNotch(cab);
    const float since = train.autoSinceChange() + dt;
    train.setAutoSinceChange(since);

    // What the handles should be, decided first; whether they are allowed to move yet,
    // decided after. Keeping the two apart is what lets the dwell apply to every case
    // without being written into each of them.
    int wantPower = power, wantBrake = brake;

    if (demand.stopping && demand.targetMs <= kUnderBandMs) {
        // The mark is here: the brake goes on and stays on, moving or not.
        //
        // The threshold is the slowest speed the controller will drive toward, and it has
        // to be, or there is a band between it and a stand that the train can neither
        // reach nor hold: standing six metres short of a signal with a target of 3 km/h,
        // it would not power (too close to the target to be worth a notch) and would not
        // brake (not yet stopping), so it sat there with the handles off and the brakes
        // released, which on a grade is a train that rolls away. Nobody creeps the last
        // five metres up to a signal anyway.
        wantPower = 0;
        wantBrake = 2;
    } else if (v > demand.targetMs + kOverBandMs ||
               (brake > 0 && v > demand.targetMs - kSettleBandMs)) {
        // WHETHER to brake is decided by the target, which already carries the braking
        // curve for everything ahead; HOW HARD by what the road needs. Deciding both from
        // the need was the mistake: the need is over half a metre per second squared the
        // moment a 70 restriction comes within braking distance at 130, so the train
        // braked while sitting exactly ON the curve it was supposed to be following -
        // then released, drifted up, braked again, and arrived at the restriction doing
        // 52 instead of 70 having hunted the whole way down.
        //
        // The two thresholds are deliberately apart: once braking it keeps braking until
        // the speed is properly back under, rather than letting go the moment it touches
        // the curve and having to take it again two seconds later.
        wantPower = 0;
        wantBrake = Vehicle::kMaxBrakeNotch;
        for (int n = 1; n <= Vehicle::kMaxBrakeNotch; ++n)
            if (Vehicle::notchDecel(n) >= demand.needDecel * kBrakeMargin) {
                wantBrake = n;
                break;
            }
        wantBrake = std::max(1, wantBrake);
    } else if (v < demand.targetMs - kUnderBandMs &&
               (!demand.stopping || v < kApproachMs)) {
        // Well under what is allowed: take another notch.
        //
        // Except when what is holding the train back is a STOP, and then only to keep it
        // moving. The braking curve toward a signal at danger allows a great deal of
        // speed a long way out - 87 km/h at 600 m - and a controller that simply tracks
        // it will accelerate hard at a red signal and then brake hard at it, which is
        // safe by construction and is not what anybody does. A driver holds what he has
        // and lets it fall. What he will not do is let the train stall short of the
        // signal, which is the case this allows for: climbing to one, power comes back on
        // below kApproachMs and draws the train up to the mark.
        wantBrake = 0;
        wantPower = std::min(Vehicle::kMaxPowerNotch, power + 1);
    } else if (v > demand.targetMs - kSettleBandMs) {
        wantBrake = 0;                       // closing on it: ease off, do not brake
        wantPower = std::max(0, power - 1);
    } else {
        wantBrake = 0;                       // inside the band: leave everything alone
        // Coasting toward a stop, the power comes off rather than being left where it
        // was: the train is being allowed to run down, not held at a speed.
        if (demand.stopping) wantPower = std::max(0, power - 1);
    }

    // Nothing ever commands emergency. An auto-driver that dumped the pipe at every stop
    // would spend two minutes recharging afterwards, as the 600 m freight showed.
    wantBrake = std::min(wantBrake, Vehicle::kMaxBrakeNotch);
    if (wantBrake > 0) wantPower = 0; // never both, which is the one thing a driver never does

    // A handle that has just moved is left to have its effect. Harder braking is the
    // exception: a brake that has to wait its turn is not a brake.
    // ...and a brake whose reason has gone comes off without waiting either, or the
    // train spends the dwell still braking for a restriction it has already met and
    // arrives well under it.
    const bool urgent = wantBrake > brake || (wantBrake == 0 && brake > 0);
    if (!urgent && since < kDwellS) return;
    if (wantPower == power && wantBrake == brake) return;
    if (wantBrake != brake) train.setBrakeNotch(cab, wantBrake);
    if (wantPower != power) train.setPowerNotch(cab, wantPower);
    train.setAutoSinceChange(0.0f);
}

bool arrivedAtStop(const DriverDemand& demand, float speedMs) {
    return demand.stopping && speedMs < 0.05f && demand.stopIn <= kStopShortM + 2.0f;
}
