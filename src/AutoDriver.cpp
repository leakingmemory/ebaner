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
// Above this the grids are worth using and are what a driver reaches for; below it they
// have faded and the locomotive's own brake is the first thing to hand.
constexpr float kDynFromMs = 8.33f; // m/s, 30 km/h
// The speed a driver draws up to a signal at, and the only speed he will use power to
// hold on the approach to one. Below it a train climbing to a signal would stall short;
// above it, powering toward a signal at danger is nobody's idea of driving.
constexpr float kApproachMs = 5.0f; // m/s, about 18 km/h
// ...and how close the stop has to be for that to apply at all. Withholding power from
// the moment a stop becomes the binding constraint is far too early: the curve to a stand
// two kilometres off still allows 160 km/h, so a train approaching a terminus would be
// held to a crawl for the last two kilometres of a 40 km/h station approach. Beyond this
// the stop is just another thing on the road and the train runs normally toward it.
constexpr float kApproachFromM = 800.0f;
// Slow enough to call stopped for the purpose of easing the brake right off.
constexpr float kCrawlMs = 0.3f;
// The least distance the braking arithmetic is allowed to work over, and the speed below
// which a heavy application is never the answer.
constexpr float kMinUsableM = 8.0f;
// The shortest thing worth calling a crossing area. Shorter than this and the two
// switches are the same switch counted twice, or a station no train could cross in.
constexpr float kMinBandM = 50.0f;
constexpr float kSlowMs = 4.0f; // m/s, about 14 km/h
// A beat between the steps of an application, so it goes on in stages.
constexpr float kBrakeStepS = 0.5f;
// How long a handle is left alone after it moves. A driver gives a notch time to answer.
constexpr float kDwellS = 1.5f;

// Ask for rather more than the arithmetic says, because the cylinders take two or three
// seconds to fill and the train keeps running while they do.
constexpr float kBrakeMargin = 1.05f;

} // namespace

namespace {

// The deceleration the plan plans for, `d` metres from whatever it is braking for.
// Shallower as it closes, so the last stretch is eased rather than fought.
float plannedDecel(float d) {
    // Squared rather than linear, so the rate is already most of the way down to
    // kEndDecel by the middle of the transition instead of half way. The point of the
    // easing is to be in light braking EARLY; a straight line leaves it too late.
    const float t = std::clamp(d / kShallowM, 0.0f, 1.0f);
    return kEndDecel + (kAutoDecel - kEndDecel) * t * t;
}

// The fastest a train may be doing `d` metres from something it must be down to `vLimit`
// for. The build-up run comes out of the distance first: the brake does nothing for the
// first few seconds, and the train covers that at the speed it already has.
float allowedAt(float d, float vLimit, float speedMs) {
    const float usable = std::max(0.0f, d - speedMs * kBuildUpS);
    return std::sqrt(vLimit * vLimit + 2.0f * plannedDecel(usable) * usable);
}

// ...and the deceleration that would actually be needed from here, which is what picks
// the notch. Measured over the same usable distance, or the answer is the deceleration
// of a brake that comes on instantly.
float neededDecel(float d, float vLimit, float speedMs) {
    if (speedMs <= vLimit) return 0.0f;
    // The build-up run may not eat the distance. Taking v * 3 s off it and then flooring
    // the remainder at a metre is what made the brake savage at low speed: ten km/h with
    // the mark eight metres off leaves d - v*t negative, the floor hands back one metre,
    // and the arithmetic asks for over 3 m/s^2 to lose five km/h. That is full service,
    // for a train five km/h fast - which stops it dead, short of the mark, and it has to
    // be driven up again. Several times over, which is what was reported.
    //
    // So never less than a third of the distance that is actually there, and never less
    // than kMinUsableM. Both are floors on the same thing: the answer stops being a
    // deceleration and becomes a divide-by-nearly-nothing.
    const float usable =
        std::max({kMinUsableM, 0.33f * d, d - speedMs * kBuildUpS});
    return (speedMs * speedMs - vLimit * vLimit) / (2.0f * usable);
}

// The smallest service notch that gives at least this deceleration, with a small margin
// for the cylinders still filling. Full service is what is left when nothing else covers
// it, which is where it belongs: an auto-driver that reaches for everything it has at
// each restriction has nothing in hand for the one it misjudged.
int notchFor(float need) {
    for (int n = 1; n <= Vehicle::kFullServiceNotch; ++n)
        if (Vehicle::notchDecel(n) >= need * kBrakeMargin) return n;
    return Vehicle::kFullServiceNotch;
}

} // namespace

// ...and the hardest notch worth using at this speed.
//
// Being 100 per cent over the limit sounds alarming and is not, when the limit is five
// km/h: it is five km/h of overspeed, and the energy in 5-10 km/h is a quarter of what
// is in 10-15. A brake chosen from the deceleration the arithmetic asks for does not
// know that, and at low speed the arithmetic asks for a great deal. So it is capped by
// how fast the train is actually going - not by how far over it is - and the cap is what
// keeps a crawling train from being slammed to a stand for being a few km/h fast.
int notchCapAt(float speedMs) {
    if (speedMs < kSlowMs) return 2;      // under ~14 km/h: nothing harder than B2
    if (speedMs < 2.0f * kSlowMs) return 3;
    return Vehicle::kFullServiceNotch;
}

float stationStopPoint(const StationStop& s) {
    if (!s.haveBand) return s.fallback;
    const float bLo = std::min(s.bandFrom, s.bandTo);
    const float bHi = std::max(s.bandFrom, s.bandTo);

    // Where the train may stand is not the band: it is the band less the train. The
    // nose has to stop clear of the switch it is running at, and the TAIL has to stop
    // clear of the one behind it - which is the whole length of the train further in.
    // Leaving that out is what sent a 600 m freight into Rognan and stood it at the TXP
    // 65 m inside the first switch, with 555 m of train lying across the throat it had
    // just come through.
    const float dir = s.towardHi ? 1.0f : -1.0f;
    const float entry = s.towardHi ? bLo : bHi; // the end it comes in by
    const float leave = s.towardHi ? bHi : bLo; // the end it is heading for
    const float noseLimit = leave - dir * kSwitchClearM;
    const float tailLimit = entry + dir * (kSwitchClearM + s.trainLength);

    // Longer than the station, which at Rognan a 600 m freight simply is. It cannot
    // stand clear at both ends, so the choice is only which end it fouls - and that is
    // no choice at all: it pulls right up to the far switch, puts as much of itself
    // inside the station as will go, and leaves the overhang behind it where it came
    // from. Stopping anywhere short of that wastes road it has no spare of.
    if (dir * (noseLimit - tailLimit) < 0.0f) return noseLimit;

    const float lo = std::min(tailLimit, noseLimit);
    const float hi = std::max(tailLimit, noseLimit);
    if (s.passenger && s.havePlatform) {
        // Only the part of the platform that is inside the band. A platform standing
        // clear of the switches at one end is no use to a train that has to keep clear
        // of them, so if none of it is in the band it is ignored rather than clamped -
        // clamping would put the train exactly on the switch it is avoiding.
        const float plo = std::max(lo, std::min(s.platformFrom, s.platformTo));
        const float phi = std::min(hi, std::max(s.platformFrom, s.platformTo));
        if (phi > plo)
            return s.haveTxp ? std::clamp(s.txpAt, plo, phi) : 0.5f * (plo + phi);
    }
    // The TXP's own spot, which is authored per track and is the best answer there is
    // where the road has one - as near the person giving the order as the road allows.
    if (s.haveTxp) return std::clamp(s.txpAt, lo, hi);
    return 0.5f * (lo + hi);
}

DriverDemand planDrive(const RoadAhead& road, float speedMs) {
    DriverDemand out;
    // The ceiling where the train stands. Everything else can only lower it.
    out.targetMs = static_cast<float>(road.here) * kKmhToMs;

    for (const RoadAhead::Limit& l : road.limits) {
        const float vl = static_cast<float>(l.kmh) * kKmhToMs;
        const float d = std::max(0.0f, l.d - kLimitEarlyM);
        out.targetMs = std::min(out.targetMs, allowedAt(d, vl, speedMs));
        out.needDecel = std::max(out.needDecel, neededDecel(d, vl, speedMs));
    }

    // A stop is the same with a limit of zero, placed kStopShortM before the mark. The
    // nearest one that binds is the one reported, so the HUD and the caller's "have we
    // arrived" test agree about which mark is being aimed at.
    float bestStopAllow = 1e9f;
    for (const RoadAhead::Stop& s : road.stops) {
        const float d = std::max(0.0f, s.d - kStopShortM);
        const float allow = allowedAt(d, 0.0f, speedMs);
        if (allow < bestStopAllow) {
            bestStopAllow = allow;
            out.stopIn = s.d;
            out.stopKind = s.kind;
            out.stopRef = s.ref;
        }
        out.needDecel = std::max(out.needDecel, neededDecel(d, 0.0f, speedMs));
    }
    if (bestStopAllow < out.targetMs) {
        out.targetMs = bestStopAllow;
        out.stopping = true;
    }

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

    // A machine with separate handles is braked in the order a driver uses them, which
    // is not "apply the train brake harder". Above kDynFromMs the grids go on first:
    // they cost nothing, they are quick and they wear nothing out. Below it the grids
    // have faded, so the LOCOMOTIVE brake goes on first. The train brake is what takes
    // whatever is left over - and on a long train that is most of it, because the grids
    // and the loco's own shoes work on one vehicle out of twenty.
    //
    // What is left over is measured, not guessed: the grids report the force they are
    // actually making, and the locomotive brake is worth a known share of the train.
    const bool separate = train.controls(cab) == ControlSeparate;
    const int ind = train.independentNotch(cab);
    int wantDyn = separate && power < 0 ? -power : 0;
    int wantInd = ind;
    float residual = demand.needDecel;
    if (separate) {
        const float m = std::max(1.0f, train.mass());
        residual -= train.dynamicBrakeForce() / m;
        residual -= train.independentCapacity() *
                    (static_cast<float>(ind) / static_cast<float>(Vehicle::kMaxIndNotch));
        residual = std::max(0.0f, residual);
    }

    if (demand.stopping && (demand.stopIn <= kStopHoldM || demand.targetMs <= kUnderBandMs)) {
        // The mark is here: the brake goes on and stays on, moving or not.
        //
        // The threshold is the slowest speed the controller will drive toward, and it has
        // to be, or there is a band between it and a stand that the train can neither
        // reach nor hold: standing six metres short of a signal with a target of 3 km/h,
        // it would not power (too close to the target to be worth a notch) and would not
        // brake (not yet stopping), so it sat there with the handles off and the brakes
        // released, which on a grade is a train that rolls away. Nobody creeps the last
        // five metres up to a signal anyway.
        // Standing at the mark: the TRAIN brake, firmly, and the grids let go - they do
        // nothing at a stand. The locomotive brake stays where the approach left it.
        wantPower = 0;
        wantDyn = 0;
        wantBrake = std::min(std::max(2, notchFor(demand.needDecel)), notchCapAt(v));
        if (separate) wantBrake = std::max(wantBrake, 3);
    } else if (v > demand.targetMs + kOverBandMs ||
               (brake > 0 && v > demand.targetMs - kSettleBandMs) ||
               (demand.stopping && brake > 1 && v > kCrawlMs)) {
        // The third clause eases a heavy application down rather than throwing it away:
        // anything above the lightest notch is worked down a step at a time while the
        // train is still moving toward a stop. The lightest one may come off, and is
        // meant to - a rate below what B1 gives is held by cycling it, and that average
        // is what the train feels. Holding the application all the way to the stand
        // instead over-braked it: a steady B1 is more than the shallow end of the curve
        // asks for, so the train arrived at a stand fifty metres short and had to be
        // driven up again.
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
        if (separate) {
            // Grids first above the threshold, the locomotive's own brake below it, and
            // the train brake for the shortfall either way.
            const bool grids = v >= kDynFromMs && train.hasDynamicBrake();
            wantDyn = grids ? std::min(Vehicle::kMaxBrakeNotch, wantDyn + 1) : 0;
            wantInd = grids ? 0 : std::min(Vehicle::kMaxIndNotch, wantInd + 1);
            wantBrake = residual > 0.0f
                            ? std::clamp(notchFor(residual), 1, notchCapAt(v))
                            : 0;
        } else {
            wantBrake = std::clamp(notchFor(demand.needDecel), 1, notchCapAt(v));
        }
    } else if (v < demand.targetMs - kUnderBandMs &&
               (!demand.stopping || demand.stopIn > kApproachFromM || v < kApproachMs)) {
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
        wantDyn = 0;
        wantInd = 0; // everything off before anything is pulled
        wantPower = std::min(Vehicle::kMaxPowerNotch, std::max(0, power) + 1);
    } else if (v > demand.targetMs - kSettleBandMs) {
        wantBrake = 0;                       // closing on it: ease off, do not brake
        wantPower = std::max(0, power - 1);
    } else {
        wantBrake = 0;                       // inside the band: leave everything alone
        // Coasting toward a stop, the power comes off rather than being left where it
        // was: the train is being allowed to run down, not held at a speed.
        if (demand.stopping && demand.stopIn <= kApproachFromM)
            wantPower = std::max(0, power - 1);
    }

    // Nothing ever commands emergency, and now that is true. It was not: the ceiling was
    // kMaxBrakeNotch, which is the ELECTRIC brake's range on the power controller and
    // the same number as the emergency notch - so every demand harder than full service
    // dumped the pipe, and the train then spent two minutes recharging, as the 600 m
    // freight showed.
    wantBrake = std::min(wantBrake, Vehicle::kFullServiceNotch);
    if (wantBrake > 0) wantPower = 0; // never both, which is the one thing a driver never does

    // A handle that has just moved is left to have its effect. Harder braking is the
    // exception: a brake that has to wait its turn is not a brake.
    // An application goes on a notch at a time rather than in one movement, which is how
    // a brake is worked and what keeps the train from being snatched: the shoes take
    // seconds to fill, and a handle wound straight to 3 has the train braking harder a
    // moment later than anything asked for. The exception is a demand hard enough that
    // the stop is genuinely in question - then it goes on at once and in full.
    const bool severe = demand.needDecel >= Vehicle::notchDecel(3);
    if (wantBrake > brake + 1 && !severe) wantBrake = brake + 1;

    // How long the handles are left alone: a beat between steps of an application, the
    // full dwell for everything else, and nothing at all for a brake going off - or the
    // train spends the dwell braking for a restriction it has already met, and arrives
    // well under it.
    const float wait = wantBrake > brake  ? (severe ? 0.0f : kBrakeStepS)
                       : wantBrake == 0 && brake > 0 ? 0.0f
                                                     : kDwellS;
    if (since < wait) return;

    // The two handles a machine with separate controls has beyond the train brake. The
    // electric brake lives on the power controller's own range below neutral, so it is
    // written through wantPower; the locomotive brake is stepped, as it is by hand.
    if (separate) {
        if (wantDyn > 0) wantPower = -wantDyn;
        else if (wantPower < 0) wantPower = 0;
        if (wantInd != ind) train.moveIndependent(cab, wantInd > ind ? +1 : -1);
    }
    if (wantPower == power && wantBrake == brake) return;
    if (wantBrake != brake) train.setBrakeNotch(cab, wantBrake);
    if (wantPower != power) train.setPowerNotch(cab, wantPower);
    train.setAutoSinceChange(0.0f);
}

bool arrivedAtStop(const DriverDemand& demand, float speedMs) {
    // Anywhere inside the distance the driver stops planning and simply holds - he was
    // never going to get closer than that, so standing there IS having arrived.
    return demand.stopping && speedMs < 0.05f && demand.stopIn <= kStopHoldM + 2.0f;
}
