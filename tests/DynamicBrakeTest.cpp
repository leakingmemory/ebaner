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
// The Di 4's motstandsbremse - its rheostatic brake.
//
// The traction motors driven as generators, the energy burned in roof grids, and the
// locomotive holding a train back without a shoe touching a wheel. Trials of Di 4 651
// record the "elektrisk motstandsbrems" being tested, so it is not in doubt that the
// machine has one; what its two limiting figures are is not published anywhere I could
// reach, and the numbers in the table are estimates for a 2450 kW machine.
//
// The shape is what is tested here rather than those numbers. It shares its hardware with
// the traction curve and is bounded the same way - current below a corner, grid power above
// it - and the end that matters is the bottom: a machine turning slowly generates nothing.
// A dynamic brake that did NOT fade out would hold a train at a stand, because the consist
// clamps retardation to the speed the train has rather than letting it push the train
// backwards, and a force that never faded would be indistinguishable from a parking brake.
// That is the failure this design exists to avoid, and the first thing checked below.
//
// No dataset: a long straight TrackPath, as in AirBrakeTest and HaulTest.

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
        for (int i = 0; i <= 4000; ++i)
            pts.push_back({static_cast<float>(i) * 25.0f, 0.0f, 0.0f});
        paths.emplace_back(1u, 0u, pts, std::vector<std::uint16_t>(pts.size(), 200));
    }
};

// The brake a lone locomotive makes while held at a speed, with the controller full on.
float brakeAt(World& w, const VehicleSpec& sp, float v) {
    Vehicle u(&w.paths[0], sp, 4000.0f, v);
    u.attachNetwork(&w.paths, nullptr);
    u.toggleEngines();
    LinkCommand cmd;
    cmd.brakeNotch = 0;
    cmd.emergency = false;
    cmd.demand = 0.0f;
    cmd.powering = false;
    cmd.dynamic = 1.0f;
    for (int i = 0; i < 1200; ++i) u.stepSubsystems(1.0f / 60.0f, cmd, v);
    return u.dynamicBrakeForce();
}

} // namespace

int main() {
    World w;
    const VehicleSpec* di4 = specNamed("Di 4 (Hen");
    const VehicleSpec* c93 = specNamed("Class 93 (T");
    const VehicleSpec* night = specNamed("+ 2 sleepers");
    if (di4 == nullptr || c93 == nullptr || night == nullptr) {
        std::puts("the vehicle table is missing the Di 4, the Class 93 or the night train");
        return 1;
    }

    std::puts("\nWhich machines have one at all");
    {
        check(di4->dynBrakeN > 0.0f, "the locomotive has an electric brake", di4->dynBrakeN,
              180000.0);
        check(c93->dynBrakeN == 0.0f, "the railcar has none", c93->dynBrakeN, 0.0);
        // And cannot be wound into a range it does not have, which is the guard that stops
        // a machine without grids pretending to brake on them.
        Consist r(&w.paths, &w.paths[0], *c93, 10000.0f);
        r.attachNetwork(&w.paths, nullptr);
        for (int i = 0; i < 4; ++i) r.movePower(0, -1);
        check(r.powerNotch(0) == 0, "  and its controller stops at neutral",
              r.powerNotch(0), 0.0);

        Consist l(&w.paths, &w.paths[0], *di4, 10000.0f);
        l.attachNetwork(&w.paths, nullptr);
        for (int i = 0; i < 9; ++i) l.movePower(0, -1);
        check(l.powerNotch(0) == -Vehicle::kMaxBrakeNotch,
              "the locomotive's goes on to E5", l.powerNotch(0),
              -double(Vehicle::kMaxBrakeNotch));
    }

    std::puts("\nThe curve, and the end of it that matters");
    {
        const float atRest = brakeAt(w, *di4, 0.0f);
        const float crawl = brakeAt(w, *di4, 0.3f);
        check(atRest == 0.0f, "at a stand it makes NOTHING - it is not a parking brake",
              atRest, 0.0);
        check(crawl == 0.0f, "  and nothing at a crawl either", crawl, 0.0);

        const float b10 = brakeAt(w, *di4, 10.0f / 3.6f);
        const float b20 = brakeAt(w, *di4, 20.0f / 3.6f);
        check(b10 > 0.9f * di4->dynBrakeN, "at 10 km/h it is at its current limit", b10,
              di4->dynBrakeN);
        check(std::abs(b10 - b20) < 0.02f * di4->dynBrakeN,
              "  and flat from there to the corner", b10, b20);

        // Above the corner the grids are the limit, so effort times speed is constant.
        float worst = 0.0f;
        for (const float kmh : {40.0f, 70.0f, 120.0f}) {
            const float v = kmh / 3.6f;
            const float p = brakeAt(w, *di4, v) * v;
            worst = std::max(worst, std::abs(p - di4->dynBrakeW) / di4->dynBrakeW);
            std::printf("    %5.0f km/h: %5.0f kN, %5.0f kW into the grids\n", kmh,
                        brakeAt(w, *di4, v) / 1000.0f, p / 1000.0f);
        }
        check(worst < 0.05f, "above it the grids hold their rating (worst err)", worst, 0.0);
        check(brakeAt(w, *di4, 33.3f) < 0.5f * di4->dynBrakeN,
              "  so there is much less of it at 120 km/h than at 20",
              brakeAt(w, *di4, 33.3f), 0.5 * di4->dynBrakeN);
    }

    std::puts("\nIt holds a train back, and on grids rather than shoes");
    {
        Consist c(&w.paths, &w.paths[0], *night, 50000.0f, 25.0f);
        c.attachNetwork(&w.paths, nullptr);
        c.toggleEngines();
        c.setReverser(0, 1);
        for (int i = 0; i < 12; ++i) c.moveBrake(0, -1);
        for (int i = 0; i < 90 * 60; ++i) c.update(1.0f / 60.0f, 0.0f);
        check(c.lead().bcPressure() < 0.1f, "the train is running with its brakes off",
              c.lead().bcPressure(), 0.0);
        const float v0 = std::abs(c.velocity());

        for (int i = 0; i < 5; ++i) c.movePower(0, -1); // controller back to E5
        for (int i = 0; i < 20 * 60; ++i) c.update(1.0f / 60.0f, 0.0f);
        const float v1 = std::abs(c.velocity());
        std::printf("    %.1f km/h down to %.1f in 20 s, electric brake %.0f kN\n",
                    v0 * 3.6f, v1 * 3.6f, c.lead().dynamicBrakeForce() / 1000.0f);
        check(v1 < v0 - 1.0f, "E5 pulls a 435 t train down", v1, v0);
        check(c.lead().dynamicBrakeForce() > 1000.0f, "  on the electric brake",
              c.lead().dynamicBrakeForce(), 180000.0);
        // The blending: the locomotive's own shoes stay off while the grids are working.
        check(c.lead().bcPressure() < 0.1f, "  with the locomotive's cylinders still empty",
              c.lead().bcPressure(), 0.0);
        float worstBc = 0.0f;
        for (int u = 1; u < c.unitCount(); ++u)
            worstBc = std::max(worstBc, c.unit(u).bcPressure());
        check(worstBc < 0.1f,
              "  and the carriages' too - nothing has been asked of the air", worstBc, 0.0);

        // Emergency takes it away and wants every shoe there is.
        c.setBrakeNotch(0, Vehicle::kEmergencyNotch);
        for (int i = 0; i < 6 * 60; ++i) c.update(1.0f / 60.0f, 0.0f);
        check(c.lead().dynamicBrakeForce() == 0.0f,
              "emergency drops the electric brake out", c.lead().dynamicBrakeForce(), 0.0);
        check(c.lead().bcPressure() > 2.0f, "  and fills the cylinders instead",
              c.lead().bcPressure(), 3.8);
    }

    std::puts("\nThe interlock, held at a speed where the grids are actually working");
    {
        // Tested on a unit at a held speed rather than on a train that is slowing, and the
        // first version of this got that wrong: it applied the train brake after twenty
        // seconds of electric braking, by which time the train was down to 1.8 km/h, the
        // brake had faded out and the interlock was correctly not doing anything. The rule
        // needs the grids working to be a rule at all.
        auto bcAfter = [&](float dyn, int notch, float ind) {
            Vehicle u(&w.paths[0], *di4, 4000.0f, 20.0f);
            u.attachNetwork(&w.paths, nullptr);
            u.toggleEngines();
            LinkCommand cmd;
            cmd.emergency = false;
            cmd.demand = 0.0f;
            cmd.powering = false;
            cmd.brakeNotch = 0;
            cmd.dynamic = 0.0f;
            for (int i = 0; i < 300 * 60; ++i) u.stepSubsystems(1.0f / 60.0f, cmd, 20.0f);
            cmd.brakeNotch = notch;
            cmd.dynamic = dyn;
            cmd.independent = ind;
            for (int i = 0; i < 60 * 60; ++i) u.stepSubsystems(1.0f / 60.0f, cmd, 20.0f);
            return u.bcPressure();
        };
        check(bcAfter(0.0f, 3, 0.0f) > 1.5f,
              "with no grids working, a train application fills the loco's cylinders",
              bcAfter(0.0f, 3, 0.0f), 2.9);
        check(bcAfter(1.0f, 3, 0.0f) < 0.1f,
              "under the grids the interlock holds them empty", bcAfter(1.0f, 3, 0.0f), 0.0);
        // Held empty at the CYLINDER, not discounted from the force afterwards, so the
        // gauge tells the truth about what is in them.
        check(bcAfter(1.0f, 3, 1.0f) > 3.0f,
              "  but the driver's own valve still fills them if he asks for it",
              bcAfter(1.0f, 3, 1.0f), 3.8);
    }

    std::puts("\nAnd it comes off before the train does stop");
    {
        Consist c(&w.paths, &w.paths[0], *di4, 50000.0f, 6.0f);
        c.attachNetwork(&w.paths, nullptr);
        c.toggleEngines();
        c.setReverser(0, 1);
        for (int i = 0; i < 12; ++i) c.moveBrake(0, -1);
        for (int i = 0; i < 90 * 60; ++i) c.update(1.0f / 60.0f, 0.0f);
        for (int i = 0; i < 5; ++i) c.movePower(0, -1);
        // Run it down to where the brake should have let go, and then look at what is
        // still slowing it. Asking whether the locomotive is STILL ROLLING would be the
        // wrong question and this test asked it first: rolling resistance brings anything
        // coasting to a stand sooner or later, so it stops either way. What has to be true
        // is that the ELECTRIC BRAKE is not what did it.
        int steps = 0;
        while (std::abs(c.velocity()) > 0.30f && steps < 300 * 60) {
            c.update(1.0f / 60.0f, 0.0f);
            ++steps;
        }
        const float vFade = std::abs(c.velocity());
        check(c.lead().dynamicBrakeForce() == 0.0f,
              "by walking pace the brake has let go entirely",
              c.lead().dynamicBrakeForce(), 0.0);

        // What is left is Davis and nothing else: on 120 t that is about 0.02 m/s2, where
        // the brake at full would be giving 1.5.
        const float vBefore = std::abs(c.velocity());
        for (int i = 0; i < 60; ++i) c.update(1.0f / 60.0f, 0.0f);
        const float decel = vBefore - std::abs(c.velocity());
        std::printf("    faded out at %.2f m/s; %.3f m/s2 left, against %.2f under the "
                    "brake\n", vFade, decel, di4->dynBrakeN / di4->mass);
        check(decel < 0.10f, "  and what is left is resistance, not braking (m/s2)", decel,
              0.02);
        check(decel < 0.15f * (di4->dynBrakeN / di4->mass),
              "  a small fraction of what the brake would give", decel,
              di4->dynBrakeN / di4->mass);
    }

    // The other locomotive's brake, whose figures are estimates too but a different pair,
    // so the shape has to come out of the numbers rather than out of one hard-coded curve.
    const VehicleSpec* cd = specNamed("CD 312");
    if (cd == nullptr) {
        std::puts("\nno CD 312 in the vehicle table - that part is skipped");
    } else {
        std::puts("\nThe CD 312's brake, from its own two figures");
        check(cd->dynBrakeN > 0.0f, "it has an electric brake", cd->dynBrakeN, 220000.0);
        check(cd->dynBrakeN > di4->dynBrakeN,
              "  a stronger one than the Di 4's, as the bigger machine should have",
              cd->dynBrakeN, di4->dynBrakeN);
        const float corner = cd->dynBrakeW / cd->dynBrakeN;
        std::printf("    %.0f kN flat, %.1f MW grids, corner at %.1f m/s (%.0f km/h)\n",
                    cd->dynBrakeN / 1000.0f, cd->dynBrakeW / 1e6f, corner, corner * 3.6f);
        check(corner > 5.0f && corner < 20.0f,
              "  and the corner lands somewhere a freight train actually runs", corner,
              8.2);
        // Below the corner, flat at the current limit. Asked at 20 m/s to begin with,
        // which is a speed this machine is well PAST the corner at - the answer came back
        // at four tenths of the maximum and looked like a fault, when it was the brake
        // being grid-limited exactly as it should be.
        const float slow = brakeAt(w, *cd, 0.6f * corner);
        check(slow > 0.9f * cd->dynBrakeN, "below the corner it is at its current limit",
              slow / 1000.0f, cd->dynBrakeN / 1000.0f);
        float worst = 0.0f;
        for (const float kmh : {50.0f, 90.0f, 120.0f}) {
            const float v = kmh / 3.6f;
            const float p = brakeAt(w, *cd, v) * v;
            worst = std::max(worst, std::abs(p - cd->dynBrakeW) / cd->dynBrakeW);
            std::printf("    %5.0f km/h: %5.0f kN, %5.0f kW into the grids\n", kmh,
                        brakeAt(w, *cd, v) / 1000.0f, p / 1000.0f);
        }
        check(worst < 0.05f, "above it the grids hold their rating (worst err)", worst, 0.0);
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
