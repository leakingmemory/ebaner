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
//
// Getting a train to answer its controls.
//
// Two things are checked here and they arrived together. The first is that a locomotive
// of 1981 is not driven the way a railcar of the 1990s is: the Class 93 has one lever
// that walks power and brake along a single axis, and the Di 4 has a power controller
// under the left hand and a driver's brake valve on a quadrant under the right, worked
// independently. The difference is not decoration - with two handles you can hold the
// train on the brake and wind power on against it, which one lever cannot express at all.
//
// The second is the trap that came with giving the Di 4 a second cab you can sit in. The
// train takes its commands from the cab holding the reverser. Sit in the other one and
// every control still moves, the HUD still reads P5, and the locomotive does nothing -
// no error, no clue, and the way out is not guessable. That is the case the last block
// covers, and it is the reason the reverser now takes charge when it is put into gear.
//
// No dataset: a long straight TrackPath is the whole world, as in AirBrakeTest.

#include "Consist.h"
#include "TrackPath.h"
#include "Vehicle.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const std::string& what) {
    std::printf("  %-62s %s\n", what.c_str(), ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}
void check(bool ok, const std::string& what, double got, double want) {
    std::printf("  %-62s %s (got %g, want %g)\n", what.c_str(), ok ? "ok" : "FAILED", got,
                want);
    if (!ok) ++failures;
}

struct World {
    std::vector<TrackPath> paths;
    World() {
        std::vector<glm::vec3> pts;
        for (int i = 0; i <= 800; ++i)
            pts.push_back({static_cast<float>(i) * 25.0f, 0.0f, 0.0f});
        paths.emplace_back(1u, 0u, pts, std::vector<std::uint16_t>(pts.size(), 200));
    }
};

// A train standing in the middle of it, engines running and the pipe still empty - which
// is how stock is found and why releasing is the first thing a driver does.
struct Loco {
    Consist c;
    Loco(World& w, const VehicleSpec& sp) : c(&w.paths, &w.paths[0], sp, 10000.0f) {
        c.attachNetwork(&w.paths, nullptr);
        c.toggleEngines();
        run(4.0f);
    }
    void run(float secs) {
        for (int i = 0; i < static_cast<int>(secs * 60.0f); ++i) c.update(1.0f / 60.0f, 0.0f);
    }
};

} // namespace

int main() {
    World w;
    const VehicleSpec* c93 = specNamed("Class 93 (Talent)");
    const VehicleSpec* di4 = specNamed("Di 4 (Hen");
    if (c93 == nullptr || di4 == nullptr) {
        std::puts("the vehicle table is missing a Class 93 or a Di 4 - cannot test");
        return 1;
    }

    std::puts("\nWhich machine is driven which way");
    {
        check(c93->controls == ControlCombined, "the railcar has one combined lever");
        check(di4->controls == ControlSeparate, "the locomotive has two handles");
    }

    std::puts("\nAn engine that is off reads nothing, and can be shut down again");
    {
        // Both machines, because this went wrong on exactly one of them and the other is
        // the control. A diesel-electric sets its own revs off the notch - there is no
        // geared speed dragging them - and doing that on every step rather than only
        // while powering put a stopped engine at idle speed, reported it Running, and
        // made it impossible to stop: the revs went back up the same step they came down.
        for (const VehicleSpec* sp : {c93, di4}) {
            const std::string who(sp->name);
            World w2;
            Consist c(&w2.paths, &w2.paths[0], *sp, 10000.0f);
            c.attachNetwork(&w2.paths, nullptr);
            auto run = [&](float secs) {
                for (int i = 0; i < static_cast<int>(secs * 60.0f); ++i)
                    c.update(1.0f / 60.0f, 0.0f);
            };
            run(5.0f);
            check(c.lead().engineRpm(0) == 0.0f, who + ": stopped, it turns at nothing",
                  c.lead().engineRpm(0), 0.0);
            check(c.lead().engineState(0) == EngineState::Off, "  and reports itself Off");
            check(!c.lead().enginesRunning(), "  and is not running");

            c.toggleEngines();
            run(6.0f);
            check(std::abs(c.lead().engineRpm(0) - sp->idleRpm) < 1.0f,
                  "  started, it settles at its own idle", c.lead().engineRpm(0),
                  sp->idleRpm);
            check(c.lead().enginesRunning(), "  and is running");

            c.toggleEngines();
            run(10.0f);
            check(c.lead().engineRpm(0) == 0.0f, "  shut down, it comes back to nothing",
                  c.lead().engineRpm(0), 0.0);
            check(c.lead().engineState(0) == EngineState::Off, "  and reports Off again");
        }
    }

    std::puts("\nAnd a stopped engine pulls nothing, whatever the controls say");
    {
        World w2;
        Consist c(&w2.paths, &w2.paths[0], *di4, 10000.0f);
        c.attachNetwork(&w2.paths, nullptr);
        c.setReverser(0, 1);
        for (int i = 0; i < 8; ++i) c.moveBrake(0, -1);
        for (int i = 0; i < 5; ++i) c.movePower(0, +1);
        for (int i = 0; i < 900; ++i) c.update(1.0f / 60.0f, 0.0f);
        check(c.powerNotch(0) == 5, "the controller is at full power", c.powerNotch(0), 5.0);
        check(c.lead().engineRpm(0) == 0.0f, "  the engine still turns at nothing",
              c.lead().engineRpm(0), 0.0);
        check(std::abs(c.lead().tractiveEffort()) < 1.0f, "  and nothing is pulling",
              c.lead().tractiveEffort(), 0.0);
        check(std::abs(c.velocity()) < 0.01f, "  so it has not moved", c.velocity(), 0.0);
    }

    std::puts("\nThe combined lever still walks one axis (the railcar, unchanged)");
    {
        Loco l(w, *c93);
        l.c.setReverser(0, 1);
        // From EMERG, winding toward power bleeds the brake off first and only then
        // raises power. That ordering IS the combined lever.
        int sawBrakeFall = 0;
        int lastBrake = l.c.brakeNotch(0);
        for (int i = 0; i < 12; ++i) {
            l.c.moveHandle(0, -1);
            if (l.c.brakeNotch(0) < lastBrake) ++sawBrakeFall;
            lastBrake = l.c.brakeNotch(0);
            if (l.c.powerNotch(0) > 0) break;
        }
        check(sawBrakeFall > 0 && l.c.brakeNotch(0) == 0 && l.c.powerNotch(0) == 1,
              "brake comes off before any power goes on");
        // And it cannot hold both at once: raising the brake drops the power first.
        for (int i = 0; i < 4; ++i) l.c.moveHandle(0, -1);
        check(l.c.powerNotch(0) > 1 && l.c.brakeNotch(0) == 0, "power winds up with no brake");
        l.c.moveHandle(0, +1);
        check(l.c.brakeNotch(0) == 0, "one notch back is still power, not brake",
              l.c.brakeNotch(0), 0.0);
    }

    std::puts("\nTwo handles move apart (the locomotive)");
    {
        Loco l(w, *di4);
        l.c.setReverser(0, 1);
        for (int i = 0; i < 8; ++i) l.c.moveBrake(0, -1); // release, on its own handle
        check(l.c.brakeNotch(0) == 0, "the brake handle alone releases the brake",
              l.c.brakeNotch(0), 0.0);
        check(l.c.powerNotch(0) == 0, "  and touches no power doing it", l.c.powerNotch(0),
              0.0);
        for (int i = 0; i < 3; ++i) l.c.movePower(0, +1);
        check(l.c.powerNotch(0) == 3, "the power handle alone notches up", l.c.powerNotch(0),
              3.0);
        // The thing one lever cannot express: brake applied and power on together.
        for (int i = 0; i < 2; ++i) l.c.moveBrake(0, +1);
        check(l.c.brakeNotch(0) == 2 && l.c.powerNotch(0) == 3,
              "power can be held on against an applied brake");
    }

    std::puts("\nAnd it actually pulls");
    {
        Loco l(w, *di4);
        l.c.setReverser(0, 1);
        for (int i = 0; i < 8; ++i) { l.c.moveBrake(0, -1); l.run(0.3f); }
        l.run(6.0f);
        check(l.c.lead().bcPressure() < 0.1f, "brake released", l.c.lead().bcPressure(), 0.0);
        const float still = l.c.velocity();
        for (int i = 0; i < 5; ++i) l.c.movePower(0, +1);
        l.run(10.0f);
        check(l.c.lead().engineRpm(0) > 800.0f, "the engine answers the controller (rpm)",
              l.c.lead().engineRpm(0), 900.0);
        check(std::abs(l.c.velocity()) > std::abs(still) + 3.0f, "and the locomotive moves",
              l.c.velocity(), still);
    }

    std::puts("\nPower comes on over seconds, not at the touch of the handle");
    {
        // Two lags in series, and they are different things. The governor walks a 45-litre
        // two-stroke from idle to governed speed, and the load regulator winds excitation
        // on behind it - slower, so the engine arrives first and the load keeps building
        // after. Taking available power from the NOTCH rather than from the revs, as this
        // did, gives the driver everything the instant he touches the handle with the
        // engine still at idle underneath, which is what made it feel too responsive.
        Loco l(w, *di4);
        l.c.setReverser(0, 1);
        for (int i = 0; i < 8; ++i) { l.c.moveBrake(0, -1); l.run(0.3f); }
        l.run(10.0f); // pipe charged, brake off, engine idling
        const float rpmIdle = l.c.lead().engineRpm(0);
        check(std::abs(rpmIdle - di4->idleRpm) < 1.0f, "idling before the handle moves",
              rpmIdle, di4->idleRpm);

        for (int i = 0; i < 5; ++i) l.c.movePower(0, +1); // straight to full notch
        l.run(0.5f);
        const float teEarly = std::abs(l.c.lead().tractiveEffort());
        const float excEarly = l.c.lead().loadRegulator();
        check(teEarly < 0.25f * 360000.0f,
              "half a second later it is pulling a fraction of what it will", teEarly,
              0.25 * 360000.0);
        check(excEarly < 0.25f, "  because excitation has barely started", excEarly, 0.25);

        // The engine gets there first.
        l.run(7.0f);
        check(l.c.lead().engineRpm(0) > di4->governedRpm - 5.0f,
              "at seven seconds the engine is at governed speed",
              l.c.lead().engineRpm(0), di4->governedRpm);
        const float excAtRpm = l.c.lead().loadRegulator();
        check(excAtRpm < 0.99f, "  and the load is STILL winding on behind it", excAtRpm,
              1.0);
        check(excAtRpm > excEarly, "  well up on where it was", excAtRpm, excEarly);

        l.run(6.0f);
        check(l.c.lead().loadRegulator() > 0.99f, "a few seconds more and it is fully on",
              l.c.lead().loadRegulator(), 1.0);
    }

    std::puts("\nAnd comes off faster than it went on");
    {
        Loco l(w, *di4);
        l.c.setReverser(0, 1);
        for (int i = 0; i < 8; ++i) { l.c.moveBrake(0, -1); l.run(0.3f); }
        for (int i = 0; i < 5; ++i) l.c.movePower(0, +1);
        l.run(20.0f);
        check(l.c.lead().loadRegulator() > 0.99f, "fully excited under power",
              l.c.lead().loadRegulator(), 1.0);
        for (int i = 0; i < 5; ++i) l.c.movePower(0, -1); // handle back to off
        l.run(1.5f);
        check(l.c.lead().loadRegulator() < 0.05f, "a second and a half after shutting off",
              l.c.lead().loadRegulator(), 0.0);
        check(std::abs(l.c.lead().tractiveEffort()) < 1.0f, "  and nothing is pulling",
              l.c.lead().tractiveEffort(), 0.0);
    }

    std::puts("\nThe cab that holds the reverser is the cab that drives");
    {
        Loco l(w, *di4);
        // The driver starts the engines from cab 0 and puts that reverser into gear,
        // then walks to the other end and works the controls there.
        l.c.setReverser(0, 1);
        l.run(1.0f);
        for (int i = 0; i < 8; ++i) { l.c.moveBrake(1, -1); l.run(0.2f); }
        for (int i = 0; i < 5; ++i) l.c.movePower(1, +1);
        l.run(10.0f);
        check(l.c.powerNotch(1) == 5, "cab 1's controller reads full power",
              l.c.powerNotch(1), 5.0);
        check(l.c.activeCab() == 0, "  but cab 0 is still the one in charge",
              l.c.activeCab(), 0.0);
        check(std::abs(l.c.velocity()) < 0.5f, "  so the locomotive does not move",
              l.c.velocity(), 0.0);
        check(l.c.lead().engineRpm(0) < 400.0f, "  and the engine stays at idle",
              l.c.lead().engineRpm(0), 315.0);

        // Taking the reverser in this cab takes charge of the train - a locomotive has
        // one reverser handle, and the driver carries it to the end he is working from.
        l.c.setReverser(1, 1);
        check(l.c.reverser(0) == 0, "taking the reverser here centres the other cab's",
              l.c.reverser(0), 0.0);
        check(l.c.activeCab() == 1, "  and cab 1 is now in charge", l.c.activeCab(), 1.0);
        l.run(10.0f);
        check(l.c.lead().engineRpm(0) > 800.0f, "  the engine comes up",
              l.c.lead().engineRpm(0), 900.0);
        check(std::abs(l.c.velocity()) > 3.0f, "  and the locomotive answers at last",
              l.c.velocity(), 3.0);
    }

    std::puts("\nNever two cabs in gear at once");
    {
        Loco l(w, *di4);
        l.c.setReverser(0, 1);
        l.c.setReverser(1, -1);
        int inGear = 0;
        for (int c = 0; c < l.c.cabCount(); ++c)
            if (l.c.reverser(c) != 0) ++inGear;
        check(inGear == 1, "exactly one cab is in gear", inGear, 1.0);
        check(l.c.activeCab() == 1, "  the one last taken", l.c.activeCab(), 1.0);
        check(!l.c.interlockEmergency(), "  so the interlock is satisfied");
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
