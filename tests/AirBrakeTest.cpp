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

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
