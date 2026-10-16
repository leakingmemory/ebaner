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

// A battery train's energy system: the Type 76.
//
// Everything on it is electric and hangs off one bus - traction, the hotel load, the
// compressor - fed by four generator sets and a battery. The sets are there for the
// battery: a burst of power or a brake application is the battery's to carry, and the sets
// follow the load as it is over the last minute, plus what it takes to hold the charge
// where it should be. That is well below full, because a full battery has no room for the
// brake, and near full it takes little: its charge current tapers steeply above 80%.
//
// No dataset: a long straight TrackPath is the whole world, as in RunningGearTest.

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
        for (int i = 0; i <= 4000; ++i)
            pts.push_back({static_cast<float>(i) * 25.0f, 0.0f, 0.0f});
        paths.emplace_back(1u, 0u, pts, std::vector<std::uint16_t>(pts.size(), 200));
    }
};

constexpr float kDt = 1.0f / 60.0f;
constexpr float kAt = 20000.0f;

void run(Consist& c, float secs) {
    for (int i = 0; i < static_cast<int>(secs * 60.0f); ++i) c.update(kDt, 0.0f);
}

// Switched on, pipe charged, brakes off, driven from cab 1.
void ready(Consist& c) {
    c.toggleEngines();
    c.setReverser(1, 1);
    c.setBrakeNotch(1, 0);
    run(c, 40.0f);
}

float railKW(const Consist& c) {
    return std::abs(c.lead().tractiveEffort()) * c.speed() / 0.85f / 1000.0f;
}

// One set held at a speed with its electric brake full on and the battery at `frac`:
// what it brakes with, and where the energy goes, after a few seconds of it.
struct Braked {
    float force, battKW, gridKW;
};
Braked brakeHeld(World& w, const VehicleSpec& sp, float v, float frac) {
    Vehicle u(&w.paths[0], sp, kAt, v);
    u.attachNetwork(&w.paths, nullptr);
    u.toggleEngines();
    LinkCommand cmd;
    cmd.brakeNotch = 0;
    cmd.emergency = false;
    cmd.demand = 0.0f;
    cmd.powering = false;
    cmd.dynamic = 0.0f;
    for (int i = 0; i < 60 * 60; ++i) u.stepSubsystems(kDt, cmd, v);
    u.setBatteryFrac(frac);
    cmd.dynamic = 1.0f;
    for (int i = 0; i < 5 * 60; ++i) u.stepSubsystems(kDt, cmd, v);
    return {u.dynamicBrakeForce(), u.batteryPower() / 1000.0f, u.gridPower() / 1000.0f};
}

} // namespace

int main() {
    const VehicleSpec* t76 = specNamed("Type 76");
    if (t76 == nullptr) {
        std::puts("the vehicle table has no Type 76");
        return 1;
    }

    std::puts("\nThe battery's limits follow its charge");
    {
        World w;
        Vehicle u(&w.paths[0], *t76, kAt);
        const float rated = t76->batteryBoostW;
        struct Pt {
            float soc, want;
        };
        for (const Pt p : {Pt{0.50f, 1.00f}, Pt{0.85f, 0.675f}, Pt{0.90f, 0.35f},
                           Pt{0.95f, 0.12f}, Pt{1.00f, 0.0f}}) {
            const float got = u.batteryChargeLimit(p.soc) / rated;
            check(std::abs(got - p.want) < 0.01f,
                  "charge at " + std::to_string(int(p.soc * 100)) + "% (of rated)", got,
                  p.want);
        }
        // It gives up to its peak, twice its rating, and charges at no more than the rating.
        for (const Pt p : {Pt{0.20f, 0.0f}, Pt{0.25f, 1.0f}, Pt{0.30f, 2.0f}}) {
            const float got = u.batteryDischargeLimit(p.soc) / rated;
            check(std::abs(got - p.want) < 0.01f,
                  "discharge at " + std::to_string(int(p.soc * 100)) + "% (of rated)", got,
                  p.want);
        }
        check(std::abs(u.batteryFrac() - 0.60f) < 1e-3f, "a set is put on the road at 60%",
              u.batteryFrac(), 0.60);
    }

    std::puts("\nFull power: the battery at once, the sets behind it");
    {
        World w;
        Consist c(&w.paths, &w.paths[0], *t76, kAt, 100.0f / 3.6f);
        c.attachNetwork(&w.paths, nullptr);
        ready(c);
        c.setPowerNotch(1, Vehicle::kMaxPowerNotch);
        run(c, 2.0f);
        const Vehicle& u = c.lead();
        check(u.generatorsRunning() < 4, "two seconds in, the sets are still starting",
              u.generatorsRunning(), 1);
        check(u.batteryPower() > 300000.0f,
              "  and the battery is carrying it, past its rating (kW)", u.batteryPower() / 1000.0f, 500.0);
        check(u.batteryPower() <= 500500.0f, "  but no further than its peak (kW)",
              u.batteryPower() / 1000.0f, 500.0);
        run(c, 13.0f);
        check(u.batteryPower() <= 250500.0f,
              "  and once they are up, it is back inside its rating (kW)", u.batteryPower() / 1000.0f, 250.0);
        check(u.generatorsRunning() == 4, "fifteen seconds in, all four sets are on line",
              u.generatorsRunning(), 4);
        check(std::abs(railKW(c) - 1800.0f) < 30.0f, "  and the rail has its 1800 kW",
              railKW(c), 1800.0);

        // Power off. The sets are not dropped at once - the next notch may be a moment
        // away - but they do not run on for ever.
        c.setPowerNotch(1, 0);
        run(c, 30.0f);
        check(u.generatorsRunning() == 4, "power off: thirty seconds later all four still run",
              u.generatorsRunning(), 4);
        run(c, 300.0f);
        check(u.generatorsRunning() <= 1, "  and five minutes later one at most",
              u.generatorsRunning(), 1);
    }

    std::puts("\nA steady load moves to the sets");
    {
        World w;
        Consist c(&w.paths, &w.paths[0], *t76, kAt, 100.0f / 3.6f);
        c.attachNetwork(&w.paths, nullptr);
        ready(c);
        c.setPowerNotch(1, 2); // 720 kW asked
        run(c, 240.0f);
        const Vehicle& u = c.lead();
        std::printf("    %.0f km/h, rail %.0f kW, sets %d making %.0f kW, battery %+.0f kW\n",
                    c.speed() * 3.6f, railKW(c), u.generatorsRunning(),
                    u.generatorPower() / 1000.0f, u.batteryPower() / 1000.0f);
        check(std::abs(u.batteryPower()) < 60000.0f,
              "after four minutes at P2 the battery is nearly idle (kW)",
              u.batteryPower() / 1000.0f, 0.0);
        check(u.generatorPower() > 700000.0f, "  and the sets carry it (kW)",
              u.generatorPower() / 1000.0f, 870.0);
    }

    std::puts("\nThe charge goes back to where it is held");
    {
        World w;
        Consist lo(&w.paths, &w.paths[0], *t76, kAt);
        lo.attachNetwork(&w.paths, nullptr);
        ready(lo);
        lo.lead().setBatteryFrac(0.30f);
        run(lo, 600.0f);
        check(lo.lead().batteryFrac() > 0.42f, "standing at 30%: ten minutes later it is up",
              lo.lead().batteryFrac(), 0.48);
        check(lo.lead().batteryFrac() < 0.61f, "  and no further than the target",
              lo.lead().batteryFrac(), 0.60);

        Consist hi(&w.paths, &w.paths[0], *t76, kAt + 5000.0f);
        hi.attachNetwork(&w.paths, nullptr);
        ready(hi);
        hi.lead().setBatteryFrac(0.90f);
        run(hi, 120.0f);
        check(hi.lead().generatorsRunning() == 0, "standing at 90%: the sets are stopped",
              hi.lead().generatorsRunning(), 0);
        check(hi.tractionReady(), "  and the train is still ready to drive",
              hi.tractionReady() ? 1 : 0, 1);
        run(hi, 480.0f);
        check(hi.lead().batteryFrac() < 0.86f, "  and the hotel load draws it down",
              hi.lead().batteryFrac(), 0.71);
        hi.setPowerNotch(1, 2);
        run(hi, 3.0f);
        check(std::abs(hi.lead().tractiveEffort()) > 50000.0f,
              "  it pulls away on the battery with no set running (N)",
              std::abs(hi.lead().tractiveEffort()), 64000.0);
    }

    std::puts("\nBraking charges it, as far as its charge lets it");
    {
        World w;
        const float v = 120.0f / 3.6f;
        const Braked half = brakeHeld(w, *t76, v, 0.50f);
        const Braked nearFull = brakeHeld(w, *t76, v, 0.92f);
        const Braked full = brakeHeld(w, *t76, v, 1.00f);
        std::printf("    at 120 km/h: %.0f kN; at 50%% %+.0f kW battery, %.0f kW grids; "
                    "at 92%% %+.0f / %.0f; full %+.0f / %.0f\n",
                    half.force / 1000.0f, half.battKW, half.gridKW, nearFull.battKW,
                    nearFull.gridKW, full.battKW, full.gridKW);
        check(half.battKW < -240.0f && half.battKW >= -250.5f,
              "at 50% it takes the full 250 kW (kW, - is in)", half.battKW, -250.0);
        check(nearFull.battKW > -0.35f * 250.0f, "  at 92% under 35% of that",
              nearFull.battKW, -64.0);
        check(nearFull.gridKW > half.gridKW + 150.0f, "  and the grids take the difference",
              nearFull.gridKW - half.gridKW, 186.0);
        check(std::abs(full.force - half.force) < 0.01f * half.force,
              "full or not, the brake is as strong (N)", full.force, half.force);
        const Braked slow = brakeHeld(w, *t76, 20.0f / 3.6f, 0.50f);
        check(slow.force == 0.0f, "at 20 km/h there is no electric brake at all", slow.force,
              0.0);
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
