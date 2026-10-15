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

// A multiple unit laid out car by car: the Type 76.
//
// Three things came in with it and each is checked here. Running gear that is not two or
// three evenly spaced bogies - eight bogies under six cars of six lengths - which must put
// every axle and every car body where the layout says, and must leave the vehicles that do
// not use it exactly as they were. A traction battery that adds to the diesel while it has
// charge and is filled again by braking. And the electric brake on a combined lever, which
// blends: the notch asks for a deceleration, the motors give what they can of it, and the
// shoes are asked only for the rest.
//
// No dataset: a long straight TrackPath is the whole world, as in DrivingTest.

#include "Consist.h"
#include "TrackPath.h"
#include "Vehicle.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const std::string& what, double got, double want) {
    std::printf("  %-64s %s (got %g, want %g)\n", what.c_str(), ok ? "ok" : "FAILED", got,
                want);
    if (!ok) ++failures;
}

struct World {
    std::vector<TrackPath> paths;
    World() {
        std::vector<glm::vec3> pts;
        for (int i = 0; i <= 2000; ++i)
            pts.push_back({static_cast<float>(i) * 25.0f, 0.0f, 0.0f});
        paths.emplace_back(1u, 0u, pts, std::vector<std::uint16_t>(pts.size(), 200));
    }
};

constexpr float kDt = 1.0f / 60.0f;
constexpr float kAt = 20000.0f; // where every vehicle is put: x = s on this road

void run(Consist& c, float secs) {
    for (int i = 0; i < static_cast<int>(secs * 60.0f); ++i) c.update(kDt, 0.0f);
}

// A train ready to drive: engines running, pipe charged and the brakes off, driven from
// cab 1, which faces +s - the way a positive initial speed is already moving.
void ready(Consist& c) {
    c.toggleEngines();
    c.setReverser(1, 1);
    c.setBrakeNotch(1, 0);
    run(c, 40.0f);
}

} // namespace

int main() {
    const VehicleSpec* t76 = specNamed("Type 76");
    const VehicleSpec* c93 = specNamed("Class 93 (Talent)");
    const VehicleSpec* pocket = specNamed("pocket wagon");
    if (t76 == nullptr || c93 == nullptr || pocket == nullptr) {
        std::puts("the vehicle table is missing a Type 76, Class 93 or pocket wagon");
        return 1;
    }

    std::puts("\nThe Type 76 laid out car by car");
    {
        World w;
        Vehicle v(&w.paths[0], *t76, kAt);
        const std::vector<float> axles = v.axleOffsets();
        check(axles.size() == 16, "sixteen axles", static_cast<double>(axles.size()), 16);
        const std::vector<VehicleFrame> bogies = v.bogieFrames();
        check(bogies.size() == 8, "on eight bogies", static_cast<double>(bogies.size()), 8);
        bool placed = bogies.size() == 8;
        for (std::size_t i = 0; placed && i < 8; ++i)
            placed = std::abs(bogies[i].pos.x - (kAt + kT76Bogies[i].centre)) < 0.01f;
        check(placed, "  each where the layout puts it", placed ? 1 : 0, 1);
        // XDe's two axles, as drawn, and not a bogie's 2.70 m apart.
        check(std::abs((axles[7] - axles[6]) - 3.26f) < 0.01f,
              "  XDe's axles 3.26 m apart", axles[7] - axles[6], 3.26);
        check(std::abs((axles[1] - axles[0]) - 2.70f) < 0.01f,
              "  a bogie's 2.70 m apart", axles[1] - axles[0], 2.70);

        const std::vector<VehicleFrame> cars = v.bodySectionFrames();
        const std::vector<float> half = v.bodySectionHalfLens();
        check(cars.size() == 6 && half.size() == 6, "six car bodies",
              static_cast<double>(cars.size()), 6);
        double total = 0.0;
        for (const float h : half) total += 2.0 * h;
        check(std::abs(total - 112.70) < 0.01, "  which add up to the set's 112.70 m", total,
              112.70);
        bool middles = cars.size() == 6;
        for (std::size_t i = 0; middles && i < 6; ++i) {
            const float mid = 0.5f * (kT76Sections[i].from + kT76Sections[i].to);
            middles = std::abs(cars[i].pos.x - (kAt + mid)) < 0.01f;
        }
        check(middles, "  each placed at the middle of its own body", middles ? 1 : 0, 1);
        check(std::abs(half[2] - 3.6855f) < 0.01f, "  the power car short, 7.37 m",
              2.0 * half[2], 7.371);
    }

    std::puts("\nEvenly laid out vehicles are exactly as they were");
    {
        // The arithmetic these always had, written out: bogies at -b, 0, +b and two
        // sections tiled evenly along the body.
        World w;
        for (const VehicleSpec* sp : {c93, pocket}) {
            const std::string who(sp->name);
            Vehicle v(&w.paths[0], *sp, kAt);
            const std::vector<VehicleFrame> secs = v.bodySectionFrames();
            const std::vector<float> half = v.bodySectionHalfLens();
            const float q = 0.25f * sp->length;
            const bool ok = secs.size() == 2 && half.size() == 2 &&
                            std::abs(secs[0].pos.x - (kAt - q)) < 0.01f &&
                            std::abs(secs[1].pos.x - (kAt + q)) < 0.01f &&
                            std::abs(half[0] - q) < 1e-4f && std::abs(half[1] - q) < 1e-4f;
            check(ok, who + ": two even sections", static_cast<double>(secs.size()), 2);
            const std::vector<float> axles = v.axleOffsets();
            const float b = 0.5f * sp->bogieSpacing, wb = 0.5f * sp->wheelbase;
            const bool axOk = axles.size() == 6 && std::abs(axles[0] - (-b - wb)) < 1e-4f &&
                              std::abs(axles[5] - (b + wb)) < 1e-4f;
            check(axOk, who + ": six axles where they were", static_cast<double>(axles.size()),
                  6);
        }
    }

    std::puts("\nThe battery adds to the diesel while it has charge");
    {
        // At 100 km/h the drive is on its power limit, so the effort says what power it
        // had: P = TE * v / eta. 1800 kW with the battery helping, 1550 kW without.
        auto railPowerAt = [&](float frac, float& drawn) {
            World w;
            Consist c(&w.paths, &w.paths[0], *t76, kAt, 100.0f / 3.6f);
            c.attachNetwork(&w.paths, nullptr);
            ready(c);
            c.lead().setBatteryFrac(frac);
            const float before = c.lead().batteryFrac();
            c.setPowerNotch(1, Vehicle::kMaxPowerNotch);
            run(c, 15.0f);
            drawn = before - c.lead().batteryFrac();
            return std::abs(c.lead().tractiveEffort()) * c.speed() / 0.85f;
        };
        float drawnFull = 0.0f, drawnLow = 0.0f;
        const float pFull = railPowerAt(1.0f, drawnFull);
        const float pLow = railPowerAt(0.15f, drawnLow);
        check(std::abs(pFull - 1800000.0f) < 30000.0f, "charged: 1800 kW at full power (W)",
              pFull, 1800000.0);
        check(drawnFull > 0.0f, "  and the battery gave it", drawnFull, 0.0);
        check(std::abs(pLow - 1550000.0f) < 30000.0f, "at its floor: the diesel's 1550 kW (W)",
              pLow, 1550000.0);
        check(drawnLow <= 0.0f, "  and nothing more came out of it", drawnLow, 0.0);
    }

    std::puts("\nOn a combined lever the electric brake blends, and charges the battery");
    {
        World w;
        Consist c(&w.paths, &w.paths[0], *t76, kAt, 100.0f / 3.6f);
        c.attachNetwork(&w.paths, nullptr);
        ready(c);
        c.lead().setBatteryFrac(0.5f);
        const float v0 = c.speed();
        c.setBrakeNotch(1, 2);
        run(c, 3.0f); // the application coming on
        const float v1 = c.speed();
        run(c, 4.0f);
        const float v2 = c.speed();
        const float decel = (v1 - v2) / 4.0f;
        check(c.lead().dynamicBrakeForce() > 0.0f, "B2 works the motors as a brake (N)",
              c.lead().dynamicBrakeForce(), 1.0);
        // B2 is worth 0.66 m/s^2 on the shoes alone; blended, the motors give part of it
        // and the shoes the rest, so the train still stops at the notch's rate - plus what
        // the air and the rolling take off it.
        check(decel > 0.60f && decel < 0.80f, "  and the train still slows at B2's rate (m/s^2)",
              decel, 0.66);
        check(c.lead().batteryFrac() > 0.5f, "  and what it makes charges the battery",
              c.lead().batteryFrac(), 0.5);
        check(v0 > v2, "  (it did slow)", v0 - v2, 1.0);
    }

    std::puts("\nWhat it pulls");
    {
        World w;
        Consist c(&w.paths, &w.paths[0], *t76, kAt);
        c.attachNetwork(&w.paths, nullptr);
        ready(c);
        c.setPowerNotch(1, Vehicle::kMaxPowerNotch);
        float peak = 0.0f;
        for (int i = 0; i < 10 * 60; ++i) {
            c.update(kDt, 0.0f);
            peak = std::max(peak, std::abs(c.lead().tractiveEffort()));
        }
        check(std::abs(peak - 160000.0f) < 1600.0f, "160 kN starting (N)", peak, 160000.0);

        // And it will hold 160 km/h on the level, which is its top speed on diesel: at
        // full power with the battery helping, what it pulls there is at least what the
        // rolling and the air take. Worked from the power rather than driven up to it: a
        // set put on the road is braked and coasts while its pipe charges, so it cannot be
        // started at speed, and the drive is on its power limit from well below 100 km/h,
        // so the power measured there is the power at 160.
        World w2;
        Consist d(&w2.paths, &w2.paths[0], *t76, kAt, 100.0f / 3.6f);
        d.attachNetwork(&w2.paths, nullptr);
        ready(d);
        d.setPowerNotch(1, Vehicle::kMaxPowerNotch);
        run(d, 15.0f);
        const float railW = std::abs(d.lead().tractiveEffort()) * d.speed();
        const float v160 = 160.0f / 3.6f;
        const float pull = railW / v160;
        const float drag = d.lead().rollingResistance(v160);
        check(pull >= drag, "at full power it out-pulls the resistance at 160 km/h (N)", pull,
              drag);
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
