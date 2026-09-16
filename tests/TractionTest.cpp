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

// What each machine pulls with, and how many wheels it stands on.
//
// Two vehicles now, and they are different machines rather than the same one in different
// paint. A diesel-hydraulic hauls through a torque converter and five gears, so its
// tractive effort steps as it shifts; a diesel-electric turns a generator and pulls flat up
// to a corner speed and then falls away as 1/v, holding its rated power. Telling those two
// curves apart is what this file is for - a diesel-electric that quietly kept the gearbox
// would look plausible on the HUD and be wrong in exactly the way that matters.
//
// The Class 93 is here as a regression guard and not because its transmission is new: this
// change moved its power, its wheels and its adhesion out of file-scope constants and into
// its own row of the table, and the way that goes wrong is silently, by a factor of
// something. Its curve is measured, not asserted against numbers written down here.
//
// No dataset. A kilometre of straight TrackPath is the whole world, as in UncoupleTest.

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

struct World {
    std::vector<TrackPath> paths;
    World() {
        std::vector<glm::vec3> pts;
        for (int i = 0; i <= 80; ++i)
            pts.push_back({static_cast<float>(i) * 25.0f, 0.0f, 0.0f});
        paths.emplace_back(1u, 0u, pts, std::vector<std::uint16_t>(pts.size(), 200));
    }
};

// Tractive effort (N) at a held speed and notch.
//
// A Vehicle is stepped with the train's speed handed to it rather than integrating its
// own, which is exactly what is wanted here: the curve can be sampled where it is asked
// for instead of wherever the thing happened to accelerate to. Long enough for the engine
// to crank to idle and the revs to settle at the notch.
float effortAt(World& w, const VehicleSpec& spec, float speed, int notch) {
    Vehicle u(&w.paths[0], spec, 400.0f, speed);
    u.attachNetwork(&w.paths, nullptr);
    u.toggleEngines();
    LinkCommand cmd;
    cmd.brakeNotch = 0;
    cmd.emergency = false;
    cmd.demand = static_cast<float>(notch) / static_cast<float>(Vehicle::kMaxPowerNotch);
    cmd.powering = notch > 0;
    UnitStep st;
    for (int i = 0; i < 1200; ++i) st = u.stepSubsystems(1.0f / 60.0f, cmd, speed);
    return std::abs(st.tractiveEffort);
}

} // namespace

int main() {
    World w;
    const VehicleSpec* c93 = specNamed("Class 93 (Talent)");
    const VehicleSpec* di4 = specNamed("Di 4 (Hen");
    if (c93 == nullptr) {
        std::puts("no Class 93 in the vehicle table - cannot test");
        return 1;
    }

    std::puts("\nSix wheels under a bogie, or four");
    {
        Consist a(&w.paths, &w.paths[0], *c93, 400.0f);
        const std::vector<float> off93 = a.unit(0).axleOffsets();
        check(static_cast<int>(off93.size()) == 2 * c93->bogieCount,
              "the Class 93 stands on two axles a bogie, as it did",
              static_cast<double>(off93.size()), 2.0 * c93->bogieCount);
        if (di4 != nullptr) {
            Consist b(&w.paths, &w.paths[0], *di4, 400.0f);
            const std::vector<float> off = b.unit(0).axleOffsets();
            check(off.size() == 6, "a Co'Co' stands on six", static_cast<double>(off.size()),
                  6.0);
            check(b.unit(0).bogieFrames().size() == 2, "  in two bogies",
                  static_cast<double>(b.unit(0).bogieFrames().size()), 2.0);
            check(b.unit(0).axleFrames().size() == 6, "  and six wheelsets are drawn",
                  static_cast<double>(b.unit(0).axleFrames().size()), 6.0);
            // Evenly spread across the wheelbase, which is outer axle to outer axle: the
            // middle axle of a three-axle bogie sits on its centre.
            std::vector<float> s = off;
            std::sort(s.begin(), s.end());
            const float gapA = s[1] - s[0], gapB = s[2] - s[1];
            check(std::abs(gapA - gapB) < 1e-3f, "  evenly spread within the bogie (m)",
                  gapA, gapB);
            check(std::abs((s[2] - s[0]) - di4->wheelbase) < 1e-3f,
                  "  spanning the wheelbase", s[2] - s[0], di4->wheelbase);
            // The brake and the occupancy both measure a train by its axles, so the
            // consist's own list has to agree with the sets'.
            check(b.axleOffsets().size() == 6, "  and the train reports all six",
                  static_cast<double>(b.axleOffsets().size()), 6.0);
        }
    }

    std::puts("\nThe Class 93 still pulls as it did (a gearbox, with steps)");
    {
        // Measured, not asserted against written-down numbers: what matters is that
        // promoting power, wheels and adhesion into the table did not move the curve.
        const float mg = c93->mass * 9.81f;
        for (const float v : {2.0f, 10.0f, 25.0f}) {
            const float te = effortAt(w, *c93, v, 5);
            check(te > 1000.0f, "at " + std::to_string(static_cast<int>(v)) +
                                    " m/s it pulls something", te, 1000.0);
            check(te <= 0.34f * mg + 1.0f, "  and never beyond adhesion", te, 0.34 * mg);
        }
        const float slow = effortAt(w, *c93, 2.0f, 5);
        const float fast = effortAt(w, *c93, 30.0f, 5);
        check(fast < slow, "and less of it at speed than at a crawl", fast, slow);
        check(effortAt(w, *c93, 10.0f, 0) == 0.0f, "notch 0 pulls nothing",
              effortAt(w, *c93, 10.0f, 0), 0.0);
    }

    if (di4 == nullptr) {
        std::puts("\nno Di 4 in the vehicle table yet - the rest is skipped");
        std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
        return failures == 0 ? 0 : 1;
    }

    std::puts("\nThe Di 4 pulls as a diesel-electric does (flat, then 1/v)");
    {
        const float mg = di4->mass * 9.81f;
        const float adh = 0.33f * di4->drivenFrac * mg;
        const float flat = std::min(di4->startTE, adh);
        // Where the flat meets the hyperbola, from the figures rather than written down.
        const float pRail = di4->powerW * 0.85f;
        const float corner = pRail / flat;
        std::printf("    rated %.0f kW, %.0f kN starting, corner at %.1f m/s (%.0f km/h)\n",
                    di4->powerW / 1000.0f, flat / 1000.0f, corner, corner * 3.6f);

        const float below = effortAt(w, *di4, std::max(1.0f, corner * 0.4f), 5);
        const float atLow = effortAt(w, *di4, std::max(1.5f, corner * 0.7f), 5);
        check(std::abs(below - atLow) < 0.03f * flat,
              "below the corner it is flat - the current limit", below, atLow);
        check(below <= flat * 1.02f, "  and never above the flat limit", below, flat);
        check(below >= flat * 0.95f, "  but right up at it", below, flat);

        // Above the corner it is the power that limits, so effort times speed is the
        // rated power at every speed. This is the property that says diesel-electric.
        float worst = 0.0f;
        for (const float v : {corner * 1.6f, corner * 2.5f, corner * 4.0f}) {
            const float te = effortAt(w, *di4, v, 5);
            worst = std::max(worst, std::abs(te * v - pRail) / pRail);
        }
        check(worst < 0.05f, "above it, effort x speed holds the rated power (worst err)",
              worst, 0.0);

        const float v1 = corner * 2.0f, v2 = corner * 4.0f;
        const float t1 = effortAt(w, *di4, v1, 5), t2 = effortAt(w, *di4, v2, 5);
        check(std::abs(t1 / t2 - 2.0f) < 0.15f, "  so doubling the speed halves the pull",
              t1 / t2, 2.0);

        // No gearbox: nothing steps. Sampled closely across where a Class 93 would shift.
        float biggestStep = 0.0f;
        float prev = effortAt(w, *di4, 4.0f, 5);
        for (float v = 5.0f; v <= 30.0f; v += 1.0f) {
            const float te = effortAt(w, *di4, v, 5);
            biggestStep = std::max(biggestStep, std::abs(te - prev) / std::max(prev, 1.0f));
            prev = te;
        }
        check(biggestStep < 0.30f, "the curve is smooth - there are no gears to change",
              biggestStep, 0.0);
    }

    std::puts("\nEffort is what amperes buy, and at a crawl the diesel is loafing");
    {
        // The flat part of the curve is a CURRENT limit, not a power one, and the
        // consequence is the thing worth checking: power at the rail is effort times
        // speed, so holding full effort at walking pace uses a fraction of what the
        // engine has. A locomotive at full revs with its ammeter against the stop and its
        // diesel barely working is not a bug - it is what a diesel-electric does.
        struct Point { float kmh, amps, load; };
        std::vector<Point> pts;
        for (const float kmh : {2.0f, 5.0f, 10.0f, 30.0f, 80.0f}) {
            const float v = kmh / 3.6f;
            Vehicle u(&w.paths[0], *di4, 4000.0f, v);
            u.attachNetwork(&w.paths, nullptr);
            u.toggleEngines();
            LinkCommand cmd;
            cmd.brakeNotch = 0;
            cmd.emergency = false;
            cmd.demand = 1.0f;
            cmd.powering = true;
            for (int i = 0; i < 3000; ++i) u.stepSubsystems(1.0f / 60.0f, cmd, v);
            pts.push_back({kmh, u.tractionAmpsFrac(), u.enginePowerFrac()});
            std::printf("    %5.0f km/h: amps %3.0f%%, engine load %3.0f%%\n", kmh,
                        u.tractionAmpsFrac() * 100.0f, u.enginePowerFrac() * 100.0f);
        }
        check(pts[0].amps > 0.99f, "at 2 km/h the ammeter is against its limit",
              pts[0].amps, 1.0);
        check(pts[0].load < 0.20f, "  and the engine is giving under a fifth of its power",
              pts[0].load, 0.10);
        check(pts[1].amps > 0.99f, "at 5 km/h the ammeter is still against it", pts[1].amps,
              1.0);
        check(pts[1].load > pts[0].load, "  and the engine is working harder than at 2",
              pts[1].load, pts[0].load);
        check(pts[2].load > pts[1].load && pts[2].load < 0.75f,
              "at 10 km/h harder again, and still not fully loaded", pts[2].load, 0.48);
        // Past the corner it turns over: the engine is flat out and the current falls
        // away, because the same power is being spread over more speed.
        check(pts[3].load > 0.95f, "at 30 km/h the engine is fully loaded", pts[3].load,
              1.0);
        check(pts[3].amps < 0.8f, "  and the current has come off its limit", pts[3].amps,
              0.69);
        check(pts[4].amps < pts[3].amps, "at 80 km/h less current again", pts[4].amps,
              pts[3].amps);
        check(pts[4].load > 0.95f, "  with the engine still flat out", pts[4].load, 1.0);
    }

    std::puts("\nAnd it answers the notch");
    {
        const float full = effortAt(w, *di4, 20.0f, 5);
        const float half = effortAt(w, *di4, 20.0f, 3);
        check(effortAt(w, *di4, 20.0f, 0) == 0.0f, "notch 0 pulls nothing",
              effortAt(w, *di4, 20.0f, 0), 0.0);
        check(half < full * 0.8f, "notch 3 pulls less than notch 5", half, full);
        check(half > full * 0.4f, "  but not nothing", half, full);
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
