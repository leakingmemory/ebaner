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
// A locomotive with carriages behind it.
//
// Two things arrived together here. The first is that a train may hold vehicles that are
// not all the same: a Consist used to take one VehicleSpec and make N copies, and its
// spacing came from `lead().length()` alone, so a 20.8 m locomotive hauling 25.3 m
// carriages would have laid every one of them 4.5 m inside the next. The overlap check
// below is the direct test of that, and it is the fault that would have gone unnoticed
// longest - the train looks right from the cab and is wrong everywhere else.
//
// The second is the brake. A railcar of the 1990s has EP, which echoes the driver's valve
// electrically at every vehicle so the application does not have to travel; the Class 93
// reaches full brake at the far end of a three-set train in the same 0.4 s it takes on one,
// and that is correct for it. A locomotive of 1981 and carriages of 1977 have no such
// thing. The valve vents at one point, the application walks the train, and every pipe it
// walks through is a volume that has to be emptied - so a long train brakes slowly, which
// is the property being tested rather than any exact number of seconds.
//
// No dataset: a long straight TrackPath, as in AirBrakeTest and DrivingTest.

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
    std::printf("  %-60s %s\n", what.c_str(), ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}
void check(bool ok, const std::string& what, double got, double want) {
    std::printf("  %-60s %s (got %g, want %g)\n", what.c_str(), ok ? "ok" : "FAILED", got,
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

// Run a train up to line speed with the brake released, then slam it to emergency and
// report when it starts to lose speed. Returns -1 if it never released, which would make
// every other reading meaningless.
float emergencyOnset(World& w, const VehicleSpec& sp, float& farEndBites) {
    Consist c(&w.paths, &w.paths[0], sp, 50000.0f, 20.0f);
    c.attachNetwork(&w.paths, nullptr);
    c.toggleEngines();
    c.setReverser(0, 1);
    for (int i = 0; i < 12; ++i) c.moveBrake(0, -1);  // separate handles
    for (int i = 0; i < 12; ++i) c.moveHandle(0, -1); // or a combined one
    for (int i = 0; i < 90 * 60; ++i) c.update(1.0f / 60.0f, 0.0f);
    farEndBites = -1.0f;
    if (c.lead().bcPressure() > 0.2f) return -1.0f;

    c.setPowerNotch(0, 0);
    const float v0 = std::abs(c.velocity());
    c.setBrakeNotch(0, Vehicle::kEmergencyNotch);
    float onset = -1.0f;
    for (int i = 0; i < 60 * 15; ++i) {
        c.update(1.0f / 60.0f, 0.0f);
        const float t = static_cast<float>(i + 1) / 60.0f;
        if (onset < 0.0f && std::abs(c.velocity()) < v0 * 0.98f) onset = t;
        if (farEndBites < 0.0f && c.unit(c.unitCount() - 1).bcPressure() > 1.9f)
            farEndBites = t;
    }
    return onset;
}

} // namespace

int main() {
    World w;
    const VehicleSpec* c93 = specNamed("Class 93 (T");
    const VehicleSpec* di4 = specNamed("Di 4 (Hen");
    const VehicleSpec* car = specNamed("BC5-3");
    const VehicleSpec* cafe = specNamed("FR5-1");
    const VehicleSpec* plain = specNamed("B5-3 (");
    const VehicleSpec* train = specNamed("Di 4 + 5");
    if (!c93 || !di4 || !car || !cafe || !plain || !train) {
        std::puts("the vehicle table is missing a Class 93 / Di 4 / BC5-3 / FR5-1 / train");
        return 1;
    }

    std::puts("\nThe carriage");
    {
        check(car->cabs == 0, "a carriage has no driving position", car->cabs, 0.0);
        check(car->engines == 0, "  and no engine", car->engines, 0.0);
        check(!car->epBrake, "  and no EP: it brakes on the train pipe and nothing else");
        check(std::abs(car->length - 25.30f) < 0.01f, "25.30 m over the body", car->length,
              25.30);
        check(std::abs(car->mass - 43000.0f) < 1.0f, "43 t", car->mass, 43000.0);
        check(cafe->cabs == 0 && cafe->engines == 0 && !cafe->epBrake,
              "and the cafe car is a carriage in every same respect");
        check(std::abs(cafe->mass - 44000.0f) < 1.0f, "  at 44 t", cafe->mass, 44000.0);
        check(cafe->body != car->body,
              "  but a different body: its windows and doors are nowhere near the same");
        // The B5-3 is the shell the BC5-3 was rebuilt out of, so it is the same carriage
        // from outside and a different one inside - seventeen rows where the family
        // carriage has nine, because the playroom takes the last four metres of its saloon.
        check(std::abs(plain->mass - 43200.0f) < 1.0f, "the plain B5-3 at 43.2 t",
              plain->mass, 43200.0);
        check(plain->body != car->body && plain->body != cafe->body,
              "  and a body of its own, for what is inside it");
        check(std::abs(plain->length - car->length) < 0.01f,
              "  though it is the same length as the carriage rebuilt from it",
              plain->length, car->length);
        // The B5-5 shares the B5-3's body on purpose: the two drawings differ only in a
        // door highlight and in eight seats being green on the plan, which is a seat-map
        // convention for a designated area and not something anyone can see.
        const VehicleSpec* pets = specNamed("B5-5 (");
        check(pets != nullptr, "the B5-5 is in the table");
        if (pets != nullptr)
            check(pets->body == plain->body,
                  "  and shares the B5-3's body, there being nothing visible to tell them "
                  "apart");
        // The A5-1 is the one variant whose shell really is different: 1st class has a
        // window across the centre where every 2nd class one has 2.2 m of blank side.
        const VehicleSpec* first = specNamed("A5-1 (");
        check(first != nullptr, "the A5-1 is in the table");
        if (first != nullptr) {
            check(std::abs(first->mass - 42000.0f) < 1.0f, "  1st class at 42 t",
                  first->mass, 42000.0);
            check(first->body != plain->body,
                  "  and a body of its own - it is glazed differently, not just furnished "
                  "differently");
        }
    }

    std::puts("\nA locomotive and five carriages is one train of six unlike vehicles");
    {
        Consist c(&w.paths, &w.paths[0], *train, 20000.0f);
        c.attachNetwork(&w.paths, nullptr);
        check(c.unitCount() == 6, "six units", c.unitCount(), 6.0);
        // Marshalled as the real rake is: the cafe is the second carriage, not just five
        // of the same thing. A formation is a list, and this is the test of that.
        check(std::string(c.unit(0).name()).find("Di 4") != std::string::npos,
              "the locomotive leads");
        check(std::string(c.unit(2).name()).find("FR5-1") != std::string::npos,
              "  the cafe car is the second carriage");
        check(std::string(c.unit(3).name()).find("B5-3 (") != std::string::npos,
              "  the plain seating coach the third");
        check(std::string(c.unit(4).name()).find("B5-5 (") != std::string::npos,
              "  the one with the pet places the fourth");
        check(std::string(c.unit(5).name()).find("A5-1 (") != std::string::npos,
              "  and 1st class on the tail");
        check(std::string(c.unit(1).name()).find("BC5-3") != std::string::npos,
              "  with the family carriage next to the locomotive");
        check(std::abs(c.mass() - 335400.0f) < 1.0f, "335.4 t", c.mass(), 335400.0);
        const float want = 20.80f + 5.0f * 25.30f + 5.0f * Consist::kCouplerGap;
        check(std::abs(c.length() - want) < 0.05f, "and its length is the sum of them",
              c.length(), want);

        // The fault a single pitch causes, tested directly: with the spacing taken from
        // the lead alone, every carriage would sit 4.5 m inside the one behind it.
        float worst = 1e9f;
        for (int i = 1; i < c.unitCount(); ++i) {
            const float clear = std::abs(c.unit(i).s() - c.unit(i - 1).s()) -
                                0.5f * (c.unit(i).length() + c.unit(i - 1).length());
            worst = std::min(worst, clear);
        }
        check(worst > 0.0f, "no vehicle overlaps the one behind it (m of clearance)", worst,
              Consist::kCouplerGap);

        check(c.cabCount() == 2, "two cabs, not twelve", c.cabCount(), 2.0);
        for (int k = 0; k < c.cabCount(); ++k) {
            check(c.cabUnit(k) == 0, "  cab " + std::to_string(k) + " is on the locomotive",
                  c.cabUnit(k), 0.0);
            check(c.cabDrivable(k), "  and can drive");
        }
        for (int u = 1; u < c.unitCount(); ++u)
            check(c.unit(u).cabCount() == 0,
                  "  carriage " + std::to_string(u) + " offers none",
                  c.unit(u).cabCount(), 0.0);
    }

    std::puts("\nAnd the night train, with two sleepers on the back");
    {
        const VehicleSpec* night = specNamed("+ 2 sleepers");
        const VehicleSpec* slp = specNamed("WLAB-2");
        check(night != nullptr && slp != nullptr, "the night train and the sleeper exist");
        if (night != nullptr && slp != nullptr) {
            // A sleeper is not a Type 5 - it is longer, wider and heavier - so this train
            // has three different vehicle lengths in it and is the real test of a pitch
            // that is worked out per pair rather than taken from the front of the train.
            check(std::abs(slp->length - 27.00f) < 0.01f, "27.0 m, longer than a Type 5",
                  slp->length, 27.0);
            check(std::abs(slp->mass - 50000.0f) < 1.0f, "  and 50 t", slp->mass, 50000.0);
            check(slp->cabs == 0 && !slp->epBrake, "  a carriage like the rest");

            Consist c(&w.paths, &w.paths[0], *night, 20000.0f);
            c.attachNetwork(&w.paths, nullptr);
            check(c.unitCount() == 8, "eight units", c.unitCount(), 8.0);
            for (const int u : {6, 7})
                check(std::string(c.unit(u).name()).find("WLAB-2") != std::string::npos,
                      "  sleeper " + std::to_string(u - 5) + " on the back");
            check(std::abs(c.mass() - 435400.0f) < 1.0f, "435.4 t", c.mass(), 435400.0);
            const float want = 20.80f + 5.0f * 25.30f + 2.0f * 27.00f +
                               7.0f * Consist::kCouplerGap;
            check(std::abs(c.length() - want) < 0.05f, "205.5 m over the couplers",
                  c.length(), want);
            float worst = 1e9f;
            for (int i = 1; i < c.unitCount(); ++i)
                worst = std::min(worst, std::abs(c.unit(i).s() - c.unit(i - 1).s()) -
                                            0.5f * (c.unit(i).length() +
                                                    c.unit(i - 1).length()));
            check(worst > 0.0f,
                  "  and nothing overlaps, with three lengths in one train", worst,
                  Consist::kCouplerGap);
            check(c.cabCount() == 2, "  still two cabs", c.cabCount(), 2.0);
        }
    }

    std::puts("\nThe brake takes as long as the train is long");
    {
        float farC93 = 0.0f, farLight = 0.0f, farTrain = 0.0f;
        const float tC93 = emergencyOnset(w, *c93, farC93);
        const float tLight = emergencyOnset(w, *di4, farLight);
        const float tTrain = emergencyOnset(w, *train, farTrain);
        std::printf("    Class 93 (EP)        onset %.2f s, far end %.2f s\n", tC93, farC93);
        std::printf("    Di 4 light engine    onset %.2f s, far end %.2f s\n", tLight,
                    farLight);
        std::printf("    Di 4 + 5 carriages   onset %.2f s, far end %.2f s\n", tTrain,
                    farTrain);

        check(tC93 > 0.0f && tLight > 0.0f && tTrain > 0.0f, "all three released and braked");
        // The railcar keeps EP, so its application does not travel and its timing is what
        // it always was. This is the regression that would otherwise go unnoticed.
        check(tC93 < 1.0f, "the railcar's EP brake is still immediate", tC93, 0.78);
        check(farC93 < 0.6f, "  and reaches its far end at once", farC93, 0.43);

        // What the delay is actually made of, which is not what it first looked like.
        //
        // The application is not held up by the volume of pipe, and it is not held up by
        // the hoses restricting flow either. Each distributor that sees the pipe fall past
        // the emergency threshold opens its own vent, so the drop propagates as a wave
        // rather than being drawn away down the train through one valve - which is how a
        // UIC brake makes its required 250 m/s over any length. What takes the time is the
        // brake cylinders filling, three to five seconds to 95% in P, and that is local to
        // each vehicle and the same however long the train is.
        //
        // So the train brakes SLOWLY BUT NEARLY TOGETHER: the onset is about where the
        // locomotive's own is, with the far end a fraction of a second behind it.
        const float stagger = farTrain - farLight;
        const float speed = 150.3f / std::max(0.05f, stagger);
        std::printf("    front to rear %.2f s over 150.3 m = %.0f m/s\n", stagger, speed);
        check(speed > 120.0f, "the application travels as a wave, not a seep (m/s)", speed,
              250.0);
        check(std::abs(tTrain - tLight) < 0.6f,
              "so the train bites at about when the locomotive alone does", tTrain, tLight);
        check(farTrain > farLight, "  with the last carriage behind the first", farTrain,
              farLight);
        // And the number that was asked for, to within the roundness it was given with.
        check(tTrain > 1.2f && tTrain < 2.6f,
              "emergency brings the train up in about two seconds", tTrain, 2.0);
    }

    std::puts("\nAnd it is slow to let go again");
    {
        Consist c(&w.paths, &w.paths[0], *train, 50000.0f, 0.0f);
        c.attachNetwork(&w.paths, nullptr);
        c.toggleEngines();
        c.setReverser(0, 1);
        for (int i = 0; i < 40 * 60; ++i) c.update(1.0f / 60.0f, 0.0f);
        for (int i = 0; i < 12; ++i) c.moveBrake(0, -1);
        float tRel = -1.0f;
        for (int i = 0; i < 240 * 60; ++i) {
            c.update(1.0f / 60.0f, 0.0f);
            if (tRel < 0.0f && c.unit(c.unitCount() - 1).bcPressure() < 0.1f)
                tRel = static_cast<float>(i + 1) / 60.0f;
        }
        // Release IS the volume-limited one, and this is where the length of the train
        // genuinely tells: there is no accelerator to help, because nothing local can make
        // air. Every pipe on the train has to be filled from the one main reservoir at the
        // front, through the couplings, and six of them take about six times as long.
        Consist one(&w.paths, &w.paths[0], *di4, 50000.0f, 0.0f);
        one.attachNetwork(&w.paths, nullptr);
        one.toggleEngines();
        one.setReverser(0, 1);
        for (int i = 0; i < 40 * 60; ++i) one.update(1.0f / 60.0f, 0.0f);
        for (int i = 0; i < 12; ++i) one.moveBrake(0, -1);
        float tOne = -1.0f;
        for (int i = 0; i < 240 * 60 && tOne < 0.0f; ++i) {
            one.update(1.0f / 60.0f, 0.0f);
            if (one.lead().bcPressure() < 0.1f) tOne = static_cast<float>(i + 1) / 60.0f;
        }
        std::printf("    light engine releases at %.1f s, last carriage at %.1f s\n", tOne,
                    tRel);
        check(tRel > 0.0f, "the last carriage does release eventually", tRel, 1.0);
        check(tRel > 3.0f * tOne,
              "  and the train takes far longer than the locomotive alone", tRel,
              3.0 * tOne);
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
