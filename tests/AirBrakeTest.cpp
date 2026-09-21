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

// The automatic air brake: a pipe that has to be kept full, and bogies that stop the
// train when it is not.
//
// The one property worth all the rest is that the brake is *failsafe* - it is applied by
// the pipe being lost, not by anything being commanded - so that is the test this file
// exists for. The others are there because each of them is a way of getting the
// distributor wrong that still looks plausible while you are driving: a brake that
// answers the pipe's absolute pressure instead of its fall, an auxiliary that never runs
// out, an emergency that is merely full service in a hurry.
//
// No dataset. TrackPath takes points directly, so a kilometre of straight line is the
// whole world here, as in UncoupleTest.

#include "Consist.h"
#include "TrackPath.h"
#include "Vehicle.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    std::printf("  %-58s %s\n", what.c_str(), ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}
void check(bool ok, const std::string& what, double got, double want) {
    std::printf("  %-58s %s (got %g, want %g)\n", what.c_str(), ok ? "ok" : "FAILED", got,
                want);
    if (!ok) ++failures;
}

constexpr float kDt = 1.0f / 60.0f;

VehicleSpec class93(int units) {
    VehicleSpec sp{};
    for (const VehicleSpec& v : kVehicleSpecs)
        if (v.body == BodyClass93 && v.units == 1) sp = v;
    sp.units = units;
    return sp;
}

// A train of `units` sets on level straight track, with one cab in gear so the reverser
// interlock is satisfied and the handle is actually in charge.
struct Bench {
    std::vector<TrackPath> paths;
    std::optional<Consist> train;

    explicit Bench(int units = 1) {
        std::vector<glm::vec3> pts;
        for (int i = 0; i <= 40; ++i)
            pts.push_back({static_cast<float>(i) * 25.0f, 0.0f, 0.0f});
        paths.emplace_back(1u, 0u, pts, std::vector<std::uint16_t>(pts.size(), 100));
        train.emplace(&paths, &paths[0], class93(units), 500.0f);
        train->setReverser(0, 1); // one cab in gear: the interlock is happy
    }
    void step(int n) {
        for (int i = 0; i < n; ++i) train->update(kDt);
    }
    // Drive it to a fully released, fully charged state and report whether it got there.
    bool release(int steps = 900) {
        train->setBrakeNotch(0, 0);
        step(steps);
        return train->bcPressure(0) < 0.05f;
    }
};

float maxBc(const Consist& c) {
    float m = 0.0f;
    for (int i = 0; i < c.unitCount(); ++i) m = std::max(m, c.unit(i).bcPressure());
    return m;
}
float minAux(const Consist& c) {
    float m = 1e9f;
    for (int i = 0; i < c.unitCount(); ++i)
        for (int b = 0; b < c.unit(i).brakeUnitCount(); ++b)
            m = std::min(m, c.unit(i).brakeUnit(b).aux);
    return m;
}

} // namespace

int main() {
    std::puts("\nCharging: the brake comes off by filling the pipe");
    {
        Bench b;
        // It starts as stock left overnight does: reservoir full, pipe empty, brakes hard
        // on. Nothing is commanding that - it is simply what an empty pipe means.
        check(b.train->bpPressure(0) < 0.01f, "it starts with an empty pipe",
              b.train->bpPressure(0), 0.0);
        check(maxBc(*b.train) > 3.0f, "  and therefore with the brakes on",
              maxBc(*b.train), 3.8);
        check(b.train->unit(0).brakeUnitCount() == 3, "a Class 93 set has three bogies",
              b.train->unit(0).brakeUnitCount(), 3);

        check(b.release(), "charging the pipe releases them");
        check(std::abs(b.train->bpPressure(0) - 5.0f) < 0.05f, "  pipe stands at 5 bar",
              b.train->bpPressure(0), 5.0);
        check(minAux(*b.train) > 4.9f, "  and every auxiliary is charged from it",
              minAux(*b.train), 5.0);
    }

    std::puts("\nA reduction, not a pressure: what each notch is worth");
    {
        // The handle asks the pipe to stand lower; the distributor turns how far it fell
        // into cylinder pressure. Getting this backwards - reading the pipe's absolute
        // pressure - gives a brake that looks fine until the pipe is charged to anything
        // other than exactly 5 bar.
        float bcFor[6] = {};
        for (int notch = 0; notch <= Vehicle::kEmergencyNotch; ++notch) {
            Bench b;
            b.release();
            b.train->setBrakeNotch(0, notch);
            b.step(420); // 7 s: long enough for the pipe and the cylinders to settle
            bcFor[notch] = maxBc(*b.train);
        }
        check(bcFor[0] < 0.05f, "REL leaves the cylinders empty", bcFor[0], 0.0);
        check(bcFor[1] > 0.5f && bcFor[1] < 1.5f, "B1 is a minimum reduction, and small",
              bcFor[1], 1.01);
        for (int n = 2; n <= 4; ++n)
            check(bcFor[n] > bcFor[n - 1] + 0.3f, "B" + std::to_string(n) + " is more again",
                  bcFor[n], bcFor[n - 1]);
        check(std::abs(bcFor[4] - 3.8f) < 0.1f, "B4, full service, fills the cylinders",
              bcFor[4], 3.8);
        check(bcFor[5] >= bcFor[4] - 0.01f, "and emergency is at least full service",
              bcFor[5], 3.8);
    }

    std::puts("\nEmergency is not just full service in a hurry");
    {
        Bench svc, emg;
        svc.release();
        emg.release();
        svc.train->setBrakeNotch(0, 4);
        emg.train->setBrakeNotch(0, Vehicle::kEmergencyNotch);
        svc.step(30); // half a second
        emg.step(30);
        check(maxBc(*emg.train) > maxBc(*svc.train) + 0.5f,
              "half a second in, emergency is well ahead", maxBc(*emg.train),
              maxBc(*svc.train));
        check(emg.train->bpPressure(0) < svc.train->bpPressure(0),
              "  because the pipe is being dumped, not reduced", emg.train->bpPressure(0),
              svc.train->bpPressure(0));
    }

    std::puts("\nFailsafe: losing the pipe applies the brake, with nothing commanding it");
    {
        Bench b;
        check(b.release(), "released, running, handle in release");
        check(b.train->brakeNotch(0) == 0, "  the handle really is in release",
              b.train->brakeNotch(0), 0);
        check(!b.train->emergencyLine(), "  and nothing is calling for emergency");

        // The hose parts. No notch moves, no wire carries anything, the emergency line
        // stays down - and the train stops anyway. This is the whole point of the
        // arrangement and the one thing the old direct brake could not do at all.
        b.train->unit(0).burstBrakePipe();
        b.step(300);
        check(b.train->brakeNotch(0) == 0, "the pipe bursts; the handle has not moved",
              b.train->brakeNotch(0), 0);
        check(!b.train->emergencyLine(), "  and nothing has called for emergency");
        check(maxBc(*b.train) > 3.0f, "  yet the brakes are hard on", maxBc(*b.train), 3.8);
        check(b.train->brakeForce() > 0.0f, "  and the train is being retarded",
              b.train->brakeForce(), 1.0);
    }

    std::puts("\nThe auxiliaries are finite: cycling the brake wears it out");
    {
        Bench b;
        b.release();
        // Apply and release repeatedly, never leaving the pipe charged long enough to
        // refill the auxiliaries. Each application should be weaker than the last. A
        // model whose cylinder is fed from an infinite supply passes everything above and
        // fails here, and it is the failure a driver meets on a long descent.
        b.train->setBrakeNotch(0, 4);
        b.step(240);
        const float first = maxBc(*b.train);
        float last = first;
        for (int cycle = 0; cycle < 4; ++cycle) {
            b.train->setBrakeNotch(0, 0);
            b.step(45); // a short release: not long enough to recharge
            b.train->setBrakeNotch(0, 4);
            b.step(240);
            last = maxBc(*b.train);
        }
        check(first > 3.0f, "the first application is full", first, 3.8);
        check(last < first - 0.2f, "  and after four hurried cycles the brake has faded",
              last, first);
        check(minAux(*b.train) < 4.5f, "  because the auxiliaries are down",
              minAux(*b.train), 5.0);

        // Left alone with the pipe charged, it all comes back.
        b.train->setBrakeNotch(0, 0);
        b.step(900);
        b.train->setBrakeNotch(0, 4);
        b.step(300);
        check(maxBc(*b.train) > first - 0.1f, "left to recharge, the brake is good again",
              maxBc(*b.train), first);
    }

    std::puts("\nThe independent brake holds the locomotive and nothing else");
    {
        // The Zusatzbremse: a second valve straight on to the locomotive's own cylinders,
        // from its own main reservoir, touching neither the train pipe nor anything behind
        // the drawbar. It is what a locomotive is held on at a stand and shunted on, and
        // without it there is no way to stop the engine alone.
        const VehicleSpec* night = specNamed("+ 2 sleepers");
        const VehicleSpec* c93s = specNamed("Class 93 (T");
        if (night == nullptr || c93s == nullptr) {
            std::puts("  (no night train in the table - skipped)");
        } else {
            std::vector<TrackPath> paths;
            std::vector<glm::vec3> pts;
            for (int i = 0; i <= 4000; ++i)
                pts.push_back({static_cast<float>(i) * 25.0f, 0.0f, 0.0f});
            paths.emplace_back(1u, 0u, pts, std::vector<std::uint16_t>(pts.size(), 200));
            Consist c(&paths, &paths[0], *night, 50000.0f, 0.0f);
            c.attachNetwork(&paths, nullptr);
            c.toggleEngines();
            c.setReverser(0, 1);
            auto run = [&](float secs) {
                for (int i = 0; i < static_cast<int>(secs * 60.0f); ++i)
                    c.update(1.0f / 60.0f, 0.0f);
            };
            auto worstCarriage = [&] {
                float m = 0.0f;
                for (int u = 1; u < c.unitCount(); ++u)
                    m = std::max(m, c.unit(u).bcPressure());
                return m;
            };
            for (int i = 0; i < 12; ++i) c.moveBrake(0, -1);
            run(120.0f);
            check(c.lead().bcPressure() < 0.05f, "the train is released to start with",
                  c.lead().bcPressure(), 0.0);

            for (int i = 0; i < Vehicle::kMaxIndNotch; ++i) c.moveIndependent(0, +1);
            run(20.0f);
            check(c.lead().bcPressure() > 3.0f, "the independent fills the loco's cylinders",
                  c.lead().bcPressure(), 3.8);
            check(worstCarriage() < 0.05f, "  and not one carriage's", worstCarriage(), 0.0);
            check(std::abs(c.lead().bpPressure() - 5.0f) < 0.05f,
                  "  with the train pipe untouched at 5 bar", c.lead().bpPressure(), 5.0);

            // It runs off the main reservoir, not the auxiliaries, which is why a
            // locomotive can stand on its own brake for as long as it likes.
            const float mr0 = c.lead().mrPressure();
            run(300.0f);
            check(c.lead().bcPressure() > 3.0f, "five minutes later it is still holding",
                  c.lead().bcPressure(), 3.8);
            check(c.lead().mrPressure() > mr0 - 1.5f,
                  "  having spent little of the reservoir doing it",
                  mr0 - c.lead().mrPressure(), 1.0);

            for (int i = 0; i < Vehicle::kMaxIndNotch; ++i) c.moveIndependent(0, -1);
            run(30.0f);
            check(c.lead().bcPressure() < 0.05f, "and it lets go again",
                  c.lead().bcPressure(), 0.0);

            // A railcar has nothing to be independent of, and its handle does not move.
            Consist r(&paths, &paths[0], *c93s, 20000.0f);
            r.attachNetwork(&paths, nullptr);
            check(!r.hasIndependentBrake(), "a railcar has no independent brake");
            for (int i = 0; i < 3; ++i) r.moveIndependent(0, +1);
            check(r.independentNotch(0) == 0, "  and its handle will not move",
                  r.independentNotch(0), 0.0);
        }
    }

    std::puts("\nA coupled train: EP together, a burst pipe from one end");
    {
        Bench b(3);
        check(b.release(), "three sets, all released");
        b.train->setBrakeNotch(0, 3);
        b.step(6); // a tenth of a second - far less than air takes to travel a train
        float lo = 1e9f, hi = -1e9f;
        for (int u = 0; u < 3; ++u) {
            lo = std::min(lo, b.train->unit(u).bpPressure());
            hi = std::max(hi, b.train->unit(u).bpPressure());
        }
        check(hi - lo < 0.02f, "the EP valves reduce every set's pipe together", hi - lo,
              0.0);
    }
    {
        Bench b(3);
        b.release();
        // Now the pneumatic case: one end loses its pipe and the rest find out through
        // the hoses. It must reach the far end, and it must not be there instantly.
        b.train->unit(0).burstBrakePipe();
        b.step(1);
        check(b.train->unit(2).bpPressure() > b.train->unit(0).bpPressure() + 0.5f,
              "a burst at one end has not reached the other in a frame",
              b.train->unit(2).bpPressure(), b.train->unit(0).bpPressure());
        b.step(300);
        check(maxBc(*b.train) > 3.0f, "  five seconds later the whole train is braked",
              maxBc(*b.train), 3.8);
        float worst = 1e9f;
        for (int u = 0; u < 3; ++u) worst = std::min(worst, b.train->unit(u).bcPressure());
        check(worst > 3.0f, "  every set of it, not just the one that lost the pipe",
              worst, 3.8);
    }

    // --- a 600 m freight train, which is where all of this actually went wrong -------
    //
    // Everything above is a short train, and a short train hides two faults because its
    // pipe is small enough for the hoses alone to carry the air about as fast as the
    // accelerators would.
    //
    // 1. Charging a pipe costs the main reservoir air. Twenty-three vehicles of it cost
    //    more than there was between "full" and the low-reservoir threshold, so the train
    //    could not be charged at all without tripping a safety device meant to catch a
    //    failed compressor - an emergency nobody commanded, every single time.
    // 2. The accelerator then would not let go of it. It closed on the pipe coming back
    //    *up*, which its own open vent prevents, so the only way out was for all 600 m of
    //    train to reach zero and have nothing left to vent. That is the emergency that
    //    sticks on and cannot be released.
    //
    // The driver has no way of knowing what the last wagon is doing, so nothing the last
    // wagon does may hold the driver's handle hostage. That is what this section is for.
    std::puts("\nA 600 m freight train, from cold");
    {
        std::vector<glm::vec3> pts;
        for (int i = 0; i <= 800; ++i)
            pts.push_back({static_cast<float>(i) * 25.0f, 0.0f, 0.0f});
        std::vector<TrackPath> paths;
        paths.emplace_back(1u, 0u, pts, std::vector<std::uint16_t>(pts.size(), 200));
        const VehicleSpec* freight = specNamed("CargoNet freight");
        check(freight != nullptr, "there is a 600 m freight train to test with");
        if (freight) {
            Consist c(&paths, &paths[0], *freight, 8000.0f);
            c.attachNetwork(&paths, nullptr);
            c.toggleEngines();
            c.setReverser(0, 1);
            c.setBrakeNotch(0, 0);
            const int n = c.unitCount();
            auto worstBc = [&] {
                float m = 0.0f;
                for (int u = 0; u < n; ++u) m = std::max(m, c.unit(u).bcPressure());
                return m;
            };
            auto vents = [&] {
                int v = 0;
                for (int u = 0; u < n; ++u) v += c.unit(u).accelVenting() ? 1 : 0;
                return v;
            };

            // The charge. Nothing is commanding a brake; it must simply come off.
            bool tripped = false;
            for (int i = 0; i < 180 * 60; ++i) {
                c.update(kDt, 0.0f);
                tripped = tripped || c.emergencyLine();
            }
            check(!tripped, "charging it never trips a safety device by itself");
            check(worstBc() < 0.05f, "  every one of its brakes comes off", worstBc(), 0.0);
            float lowest = 9.9f;
            for (int u = 0; u < n; ++u) lowest = std::min(lowest, c.unit(u).bpPressure());
            check(lowest > 4.9f, "  and the pipe stands at 5 bar the whole way back",
                  lowest, 5.0);

            // The dump. It has to reach the far end as a wave, not seep down the hoses.
            c.setBrakeNotch(0, Vehicle::kEmergencyNotch);
            float tailBites = -1.0f, allOn = -1.0f;
            for (int i = 0; i < 30 * 60; ++i) {
                c.update(kDt, 0.0f);
                if (tailBites < 0.0f && c.unit(n - 1).bcPressure() > 1.0f)
                    tailBites = float(i) / 60.0f;
                float lo = 9.9f;
                for (int u = 0; u < n; ++u) lo = std::min(lo, c.unit(u).bcPressure());
                if (allOn < 0.0f && lo > 3.0f) allOn = float(i) / 60.0f;
            }
            // 600 m of train, so the wave has a long way to go and the cylinder at the
            // end of it still takes its own three to five seconds to fill afterwards.
            check(tailBites > 0.0f && tailBites < 5.0f,
                  "emergency has the last wagon of 600 m biting inside five seconds",
                  tailBites, 3.6);
            check(allOn > 0.0f && allOn < 8.0f, "  and has every cylinder hard on by five",
                  allOn, 5.0);

            // The release, which is the fault itself. The handle goes to release with 23
            // vehicles venting; the vents have to give way to it rather than outlast it.
            c.setBrakeNotch(0, 0);
            int reopened = 0, before = vents();
            float locoBack = -1.0f, tailMostlyOff = -1.0f, tailOff = -1.0f;
            for (int i = 0; i < 300 * 60; ++i) {
                c.update(kDt, 0.0f);
                const int now = vents();
                if (now > before) ++reopened; // a vent that re-opens eats the refill
                before = now;
                if (locoBack < 0.0f && c.unit(0).bpPressure() > 4.5f)
                    locoBack = float(i) / 60.0f;
                if (tailMostlyOff < 0.0f && c.unit(n - 1).bcPressure() < 1.5f)
                    tailMostlyOff = float(i) / 60.0f;
                if (tailOff < 0.0f && worstBc() < 0.05f) tailOff = float(i) / 60.0f;
            }
            check(reopened == 0, "no vent re-opens once the release has begun",
                  double(reopened), 0.0);
            check(locoBack > 0.0f && locoBack < 40.0f, "  the locomotive's pipe is back in",
                  locoBack, 20.0);
            check(tailMostlyOff > 0.0f && tailMostlyOff < 90.0f,
                  "  the last wagon is mostly off within about a minute", tailMostlyOff,
                  60.0);
            check(tailOff > 0.0f && tailOff < 180.0f, "  and every brake is off in the end",
                  tailOff, 120.0);

            // And the case the whole thing turns on: the handle going back to release
            // while the wagons are still dumping. The driver cannot see them and must not
            // have to wait for them - the refill travels back along the train and the
            // vents close ahead of it as it arrives.
            c.setBrakeNotch(0, Vehicle::kEmergencyNotch);
            for (int i = 0; i < 90; ++i) c.update(kDt, 0.0f); // a second and a half
            int ventingWagons = 0;
            for (int u = 1; u < n; ++u) ventingWagons += c.unit(u).accelVenting() ? 1 : 0;
            check(ventingWagons > 0, "wagons are still venting when the handle comes back",
                  double(ventingWagons), 1.0);
            const float locoAtRelease = c.unit(0).bpPressure();
            c.setBrakeNotch(0, 0);
            bool locoFilledWhileWagonsVented = false;
            float backUp = -1.0f;
            for (int i = 0; i < 300 * 60; ++i) {
                int stillVenting = 0;
                for (int u = 1; u < n; ++u)
                    stillVenting += c.unit(u).accelVenting() ? 1 : 0;
                const float locoStart = c.unit(0).bpPressure();
                c.update(kDt, 0.0f);
                if (stillVenting > 0 && c.unit(0).bpPressure() > locoStart + 1e-4f)
                    locoFilledWhileWagonsVented = true;
                if (backUp < 0.0f && worstBc() < 0.05f) backUp = float(i) / 60.0f;
            }
            check(locoFilledWhileWagonsVented,
                  "  the locomotive's own pipe fills while they do", locoAtRelease, 0.0);
            check(backUp > 0.0f && backUp < 180.0f, "  and the train releases all the same",
                  backUp, 120.0);
        }
    }

    // --- the feed valve gives up before the safety device does -----------------------
    //
    // Filling a brake pipe is what spends main reservoir air, so it is the first thing to
    // stop when the supply is falling behind. The feed closes at a pressure that sits
    // between the two that were already there - below the compressor's cut-in, so the
    // compressor is already working by the time it happens, and a clear bar above the
    // low-reservoir emergency trip, so it is what keeps the reservoir off that trip
    // rather than something that happens after it. Temporary in both directions: it opens
    // again by itself once there is air to fill with.
    std::puts("\nThe feed valve closes on a low main reservoir");
    {
        Bench b;
        b.train->toggleEngines(); // there has to be a compressor for it to wait for
        check(b.release(), "released and charged to begin with");
        check(!b.train->unit(0).feedCutOut(), "  with the feed valve open",
              double(b.train->unit(0).feedCutOut()), 0.0);

        // A reduction, so there is something for the feed to be filling afterwards.
        b.train->setBrakeNotch(0, 3);
        b.step(600);
        const float reduced = b.train->bpPressure(0);
        check(reduced < 4.6f, "  and then a service reduction to fill back from", reduced,
              4.0);

        // Now the supply falls away under it - a compressor that cannot keep up - and the
        // handle goes back to release. The pipe must not fill, and the reservoir must not
        // be drawn any further down by trying.
        b.train->unit(0).ventReservoir(5.0f);
        b.train->setBrakeNotch(0, 0);
        b.step(2);
        check(b.train->unit(0).feedCutOut(), "the feed closes when the supply drops");
        const float heldBp = b.train->bpPressure(0);
        const float heldMr = b.train->unit(0).mrPressure();
        bool everTripped = false, roseWhileCut = false;
        float lastBp = heldBp;
        int cutSteps = 0;
        for (int i = 0; i < 60 * 60; ++i) {
            b.train->update(kDt);
            everTripped = everTripped || b.train->unit(0).safetyBrakeActive();
            // The flag as the step itself used it: the feed is decided before the fill,
            // so the step that re-opens it is allowed to fill on that same step.
            const bool wasCut = b.train->unit(0).feedCutOut();
            if (wasCut) {
                ++cutSteps;
                if (b.train->bpPressure(0) > lastBp + 1e-4f) roseWhileCut = true;
            }
            lastBp = b.train->bpPressure(0);
        }
        check(!roseWhileCut, "  and the pipe does not fill while it is closed", heldBp,
              double(reduced));
        check(cutSteps > 60, "  it stays closed long enough for the compressor to work",
              cutSteps / 60.0, 1.0);
        check(!everTripped, "  the reservoir never reaches the emergency trip", heldMr,
              5.0);
        check(!b.train->unit(0).feedCutOut(), "the feed opens again once it is recharged",
              b.train->unit(0).mrPressure(), 6.0);
        check(std::abs(b.train->bpPressure(0) - 5.0f) < 0.05f,
              "  and the pipe fills the rest of the way by itself",
              b.train->bpPressure(0), 5.0);
        check(maxBc(*b.train) < 0.05f, "  so the brake does come off in the end",
              maxBc(*b.train), 0.0);
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
