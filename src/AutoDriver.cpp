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

// How much under the target to start pulling again, and how much over it to start
// braking. Without a gap between the two the driver hunts: it powers, passes the target
// by a hair, brakes, falls below it by a hair, powers again, several times a second.
constexpr float kPowerBelowMs = 0.6f;  // m/s under target: wind on
constexpr float kBrakeAboveMs = 0.4f;  // m/s over target: wind off and brake

// The braking curve asks for a speed that reaches zero AT the mark, which is a speed no
// controller can hold; below this the driver is simply stopping.
constexpr float kCrawlMs = 0.8f;
// Below this the road is not asking for anything worth a brake application.
constexpr float kBrakeFromDecel = 0.10f; // m/s^2
// Ask for rather more than the arithmetic says, because the cylinders take two or three
// seconds to fill and the train keeps running while they do.
constexpr float kBrakeMargin = 1.35f;

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
        const float d = std::max(0.0f, l.d);
        const float allow = std::sqrt(vl * vl + 2.0f * kAutoDecel * d);
        out.targetMs = std::min(out.targetMs, allow);
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

void applyDrive(Consist& train, int cab, const DriverDemand& demand) {
    const float v = train.speed();
    const int power = train.powerNotch(cab);
    const int brake = train.brakeNotch(cab);

    // Braking is chosen by what the road needs, not by how far over a target the train
    // is. The notch is the smallest one that gives at least the deceleration asked for,
    // with a margin for the two or three seconds the cylinders take to fill - so a
    // restriction a long way off is answered with a touch of the brake and one close at
    // hand with a real application, and the train arrives AT the limit.
    //
    // Nothing here ever commands emergency. An auto-driver that dumped the pipe at every
    // stop would spend two minutes recharging afterwards, as the 600 m freight showed.
    if (demand.needDecel > kBrakeFromDecel && v > 0.05f) {
        if (power != 0) train.setPowerNotch(cab, 0);
        int want = Vehicle::kMaxBrakeNotch;
        for (int n = 1; n <= Vehicle::kMaxBrakeNotch; ++n)
            if (Vehicle::notchDecel(n) >= demand.needDecel * kBrakeMargin) { want = n; break; }
        if (brake != want) train.setBrakeNotch(cab, want);
        return;
    }

    // Standing AT the mark: hold the brake on rather than release it and roll the last
    // metre, because a train that creeps over the mark has passed it.
    //
    // "At" the mark, not merely "somewhere with a mark ahead" - the first version tested
    // only that a stop existed, so a train standing at a station with a red signal a
    // kilometre up the line held its brake and never moved at all. The plan asking for
    // nothing but a crawl is what says the mark is here.
    // Moving or not: once the plan is asking for nothing but a crawl, the mark is here
    // and the brake goes on. Waiting for the train to be stopped before holding it left
    // it creeping at 0.9 km/h a few metres short for ever - too slow for the braking
    // curve to ask for anything, too slow for rolling resistance to finish the job.
    if (demand.stopping && demand.targetMs <= kCrawlMs) {
        if (power != 0) train.setPowerNotch(cab, 0);
        if (brake != 2) train.setBrakeNotch(cab, 2);
        return;
    }

    if (v > demand.targetMs + kBrakeAboveMs) { // over the limit with nothing forcing it
        if (power != 0) train.setPowerNotch(cab, 0);
        if (brake != 1) train.setBrakeNotch(cab, 1);
        return;
    }

    if (v < demand.targetMs - kPowerBelowMs) {
        if (brake != 0) train.setBrakeNotch(cab, 0);
        // Wind on a notch at a time, as a driver takes a train away rather than slamming
        // the controller open.
        const float under = demand.targetMs - v;
        const int cap = under > 5.0f ? Vehicle::kMaxPowerNotch : 3;
        const int want = std::min(cap, power + 1);
        if (want != power) train.setPowerNotch(cab, want);
        return;
    }

    // Inside the band: coast, and ease the power back so it settles on the limit rather
    // than creeping up through it.
    if (brake != 0) train.setBrakeNotch(cab, 0);
    if (v > demand.targetMs && power > 0) train.setPowerNotch(cab, power - 1);
}

bool arrivedAtStop(const DriverDemand& demand, float speedMs) {
    return demand.stopping && speedMs < 0.05f && demand.stopIn <= kStopShortM + 2.0f;
}
