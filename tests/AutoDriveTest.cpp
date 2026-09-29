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

// The train that drives itself.
//
// What it has to get right is not "does it move" but the two ways a driver is judged:
// that it never passes a mark it was told to stop at, and that it is at the new limit by
// the time the limit starts rather than a few hundred metres into it. Both of those are
// about braking EARLY enough, and both are easy to pass by braking absurdly early - so
// each one is checked against a floor as well as a ceiling.
//
// The road is synthetic. planDrive takes a list of distances and speeds and knows nothing
// about the dataset, the signals or the stations, which is the whole reason it is a
// function of its own: the driving can be tested to the metre without a tile on disk.
// What the real scan makes of a real road is the sim's business and is checked there.

#include "AutoDriver.h"
#include "Consist.h"
#include "TrackPath.h"
#include "Vehicle.h"

#include <cmath>
#include <cstdio>
#include <optional>
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
constexpr float kMsToKmh = 3.6f;

// A Class 93 on a long level straight, engines running, brakes released and one cab in
// gear - the state the sim puts a train in when the auto-driver is armed.
struct Bench {
    std::vector<TrackPath> paths;
    std::optional<Consist> train;

    explicit Bench(float startMs = 0.0f, float grade = 0.0f) {
        std::vector<glm::vec3> pts;
        for (int i = 0; i <= 2000; ++i)
            pts.push_back({static_cast<float>(i) * 25.0f, 0.0f,
                           static_cast<float>(i) * 25.0f * grade});
        paths.emplace_back(1u, 0u, pts, std::vector<std::uint16_t>(pts.size(), 200));
        const VehicleSpec* c93 = specNamed("Class 93 (T");
        // Driven from CAB 1, and this matters more than it looks. The two cabs of a set
        // face opposite ways: cab 0 drives the train toward -v and cab 1 toward +v, which
        // is the direction a consist given a positive initial speed is already moving.
        // Driving a moving train from cab 0 sets the power against its own momentum - the
        // bench did exactly that at first, and the trace showed a train shedding 90 km/h
        // at full power with no brake on, which reads as a fault in the auto-driver and
        // was a fault in the bench. Cab 1 keeps speed, power and the road ahead all
        // pointing the same way.
        train.emplace(&paths, &paths[0], *c93, 20000.0f, startMs);
        train->attachNetwork(&paths, nullptr);
        train->toggleEngines();
        train->setReverser(1, 1);
        train->setBrakeNotch(1, 0);
    }
    // Charge the pipe and let the brakes come off, holding whatever speed it started at.
    void ready() {
        for (int i = 0; i < 40 * 60; ++i) train->update(kDt, 0.0f);
    }
    float kmh() const { return train->speed() * kMsToKmh; }
};

// Run the driver for `secs`, with the road re-read at 5 Hz as the sim does it. `road` is
// rebuilt each planning tick from the distance run so far, which is what lets a test put
// a mark at a fixed place on the ground rather than a fixed distance ahead.
struct RunResult {
    float travelled = 0.0f;
    float topKmh = 0.0f;
    DriverDemand last;
};
template <typename RoadFn>
RunResult run(Bench& b, float secs, RoadFn road) {
    RunResult r;
    DriverDemand d;
    const int steps = static_cast<int>(secs * 60.0f);
    for (int i = 0; i < steps; ++i) {
        if (i % 12 == 0) { // 5 Hz
            d = planDrive(road(r.travelled), b.train->speed());
            applyDrive(*b.train, 1, d, 0.2f);
        }
        const float before = b.train->speed();
        b.train->update(kDt, 0.0f);
        r.travelled += 0.5f * (before + b.train->speed()) * kDt;
        r.topKmh = std::max(r.topKmh, b.kmh());
    }
    r.last = d;
    return r;
}

RoadAhead openRoad(int kmh) {
    RoadAhead road;
    road.here = kmh;
    return road;
}

} // namespace

int main() {
    std::puts("\nThe plan, before any train is attached to it");
    {
        // The braking curve, read straight off. At kAutoDecel a stop 400 m away may be
        // approached at sqrt(2 * 0.5 * 395) = 19.9 m/s, and no faster.
        RoadAhead road = openRoad(130);
        road.stops.push_back({400.0f, RoadAhead::StopKind::Signal, 7});
        const DriverDemand d = planDrive(road, 0.0f);
        check(d.stopping, "a stop ahead is a stop, not a limit");
        check(std::abs(d.targetMs - 19.87f) < 0.1f, "  and sets the speed it may be met at",
              d.targetMs, 19.87);
        check(d.stopRef == 7, "  naming which mark is being aimed at", d.stopRef, 7.0);

        // Far enough away and it does not bind at all: the line speed wins.
        RoadAhead far = openRoad(130);
        far.stops.push_back({4000.0f, RoadAhead::StopKind::Signal, 1});
        check(!planDrive(far, 0.0f).stopping, "a stop 4 km off does not hold it back yet");
        check(std::abs(planDrive(far, 0.0f).targetMs - 130.0f / 3.6f) < 0.01f,
              "  and 130 is still 130", planDrive(far, 0.0f).targetMs * 3.6, 130.0);

        // The lowest ceiling wins, wherever it comes from.
        RoadAhead two = openRoad(130);
        two.limits.push_back({0.0f, 70});
        two.limits.push_back({0.0f, 40});
        check(std::abs(planDrive(two, 0.0f).targetMs - 40.0f / 3.6f) < 0.01f,
              "the lowest ceiling underfoot is the one obeyed",
              planDrive(two, 0.0f).targetMs * 3.6, 40.0);
    }

    std::puts("\nIt gets up to the limit and stays there");
    {
        Bench b;
        b.ready();
        const RunResult r = run(b, 240.0f, [](float) { return openRoad(70); });
        std::printf("      %.1f km/h after 4 minutes, %.0f m run, topped at %.1f\n", b.kmh(),
                    r.travelled, r.topKmh);
        check(b.kmh() > 66.0f, "it reaches the limit (km/h)", b.kmh(), 70.0);
        check(r.topKmh < 74.0f, "  without running through it", r.topKmh, 70.0);
    }

    std::puts("\nIt stops at the mark, and short of it");
    {
        Bench b(25.0f);
        b.ready();
        const float mark = 400.0f; // metres from where it starts
        const RunResult r = run(b, 180.0f, [&](float run) {
            RoadAhead road = openRoad(130);
            road.stops.push_back({mark - run, RoadAhead::StopKind::Signal, 1});
            return road;
        });
        const float shortBy = mark - r.travelled;
        std::printf("      stood %.1f m short of a mark 400 m out\n", shortBy);
        check(b.train->speed() < 0.05f, "it comes to a stand", b.train->speed(), 0.0);
        check(shortBy > 0.0f, "  before the mark, not on it (m short)", shortBy, kStopShortM);
        check(shortBy < 60.0f, "  and not half a kilometre before it", shortBy, kStopShortM);
        check(arrivedAtStop(r.last, b.train->speed()) || shortBy < 20.0f,
              "  near enough to call it arrived");
    }

    std::puts("\nIt is at the new limit by the time the limit starts");
    {
        // 130 km/h into a 70 stretch beginning 900 m ahead. The speed as the train
        // crosses that point is the whole question: brake late and it is still doing
        // 100 there, which on the ground is a train through a turnout at 100.
        Bench b(36.0f); // 130 km/h
        b.ready();
        const float drop = 900.0f;
        float atDrop = -1.0f;
        DriverDemand d;
        float travelled = 0.0f;
        for (int i = 0; i < 120 * 60; ++i) {
            if (i % 12 == 0) {
                RoadAhead road = openRoad(travelled < drop ? 130 : 70);
                if (travelled < drop) road.limits.push_back({drop - travelled, 70});
                d = planDrive(road, b.train->speed());
                applyDrive(*b.train, 1, d, 0.2f);
            }
            const float before = b.train->speed();
            b.train->update(kDt, 0.0f);
            travelled += 0.5f * (before + b.train->speed()) * kDt;
            if (atDrop < 0.0f && travelled >= drop) atDrop = b.kmh();
        }
        std::printf("      %.1f km/h crossing into the 70, %.1f km/h a minute later\n",
                    atDrop, b.kmh());
        // Within the band it holds anywhere - it sits between 68 and 72 cruising an open
        // 70 as well - and not a train that has thrown away half its speed getting there.
        check(atDrop > 0.0f && atDrop < 73.0f, "at the limit where the limit begins (km/h)",
              atDrop, 70.0);
        check(atDrop > 62.0f, "  and not crawling into it having braked far too early",
              atDrop, 70.0);
    }

    std::puts("\nA signal that clears lets it go again, with nobody touching anything");
    {
        Bench b(20.0f);
        b.ready();
        bool red = true;
        const float mark = 500.0f;
        float travelled = 0.0f;
        DriverDemand d;
        float stoppedAt = -1.0f;
        int standing = 0;
        for (int i = 0; i < 400 * 60; ++i) {
            if (i % 12 == 0) {
                RoadAhead road = openRoad(70);
                if (red) road.stops.push_back({mark - travelled, RoadAhead::StopKind::Signal, 2});
                d = planDrive(road, b.train->speed());
                applyDrive(*b.train, 1, d, 0.2f);
            }
            const float before = b.train->speed();
            b.train->update(kDt, 0.0f);
            travelled += 0.5f * (before + b.train->speed()) * kDt;
            // It stands at the signal; twenty seconds later the road is offered, and
            // nothing else happens - no arming, no handle touched.
            if (b.train->speed() < 0.02f && i > 60) {
                if (stoppedAt < 0.0f) stoppedAt = travelled;
                if (red && ++standing > 20 * 60) red = false;
            }
        }
        std::printf("      stood at %.0f m, ran on to %.0f m once it cleared\n", stoppedAt,
                    travelled);
        check(stoppedAt > 0.0f && stoppedAt < mark, "it stopped at the red (m)", stoppedAt,
              mark - kStopShortM);
        check(!red, "  the signal was cleared while it stood there");
        check(travelled > stoppedAt + 200.0f,
              "  and it went on by itself, with no second arming", travelled - stoppedAt,
              200.0);
    }

    // --- what driving it on the real line found, and the synthetic road did not ------
    std::puts("\nThings only a real line found");
    {
        // Distances are measured from the NOSE. The walk starts at the leading set's
        // CENTRE, so left unshifted every mark reads half a body length further off than
        // it is - 20.75 m on a Class 93 - and the train stands that far PAST every signal
        // it stops at, which is a signal passed at danger.
        Bench b;
        b.ready();
        std::vector<Vehicle::RoadStretch> road;
        check(b.train->roadAhead(500.0f, road), "the road ahead is there to be read");
        check(!road.empty(), "  and comes back in stretches", double(road.size()), 1.0);
        if (!road.empty()) {
            const float half = 0.5f * b.train->lead().length();
            check(std::abs(road.front().dist0 + half) < 0.01f,
                  "  measured from the nose, not the middle of the first set",
                  road.front().dist0, -half);
        }
    }
    {
        // A train standing with a stop a long way off must MOVE. The first version held
        // the brake whenever a stop existed anywhere ahead, so a train standing at a
        // station with a red signal a kilometre up the line never went anywhere at all.
        Bench b;
        b.ready();
        const RunResult r = run(b, 60.0f, [](float run) {
            RoadAhead road = openRoad(70);
            road.stops.push_back({1000.0f - run, RoadAhead::StopKind::Signal, 3});
            return road;
        });
        std::printf("      ran %.0f m toward a signal 1000 m off, now %.1f km/h\n",
                    r.travelled, b.kmh());
        check(r.travelled > 50.0f, "it sets off toward a stop that is far away (m)",
              r.travelled, 50.0);
    }
    {
        // ...and standing AT one it must hold the brake on. Between the speed the
        // controller will drive toward and a stand there was a band it could neither
        // reach nor hold, and it sat six metres short of a signal with the handles off
        // and the brakes released - which on a grade is a train that rolls away.
        Bench b;
        b.ready();
        DriverDemand d;
        for (int i = 0; i < 40 * 60; ++i) {
            if (i % 12 == 0) {
                RoadAhead road = openRoad(70);
                road.stops.push_back({5.8f, RoadAhead::StopKind::Signal, 4}); // just short
                d = planDrive(road, b.train->speed());
                applyDrive(*b.train, 1, d, 0.2f);
            }
            b.train->update(kDt, 0.0f);
        }
        std::printf("      standing 5.8 m short: target %.1f km/h, cylinders %.2f bar\n",
                    d.targetMs * kMsToKmh, b.train->lead().bcPressure());
        check(b.train->speed() < 0.05f, "standing at a mark it stays standing",
              b.train->speed(), 0.0);
        check(b.train->lead().bcPressure() > 1.0f, "  with the brake on, not coasting",
              b.train->lead().bcPressure(), 1.94);
    }

    std::puts("\nA signal at the top of a climb");
    {
        // Two things at once, and they pull opposite ways. Climbing to a signal the train
        // loses speed to gravity and would stall short of it, so power has to come back
        // on - braking reversed to power, which nothing else here ever does. But the
        // braking curve toward a stop allows a great deal of speed a long way out - 87
        // km/h at 600 m - so a controller that simply tracks it accelerates hard AT a
        // signal at danger and brakes hard at it, which is safe and is not driving.
        //
        // The rule that serves both: approaching a stop, power only to keep the train
        // moving, never to gain speed.
        auto approach = [&](float grade, float& shortBy, bool& powered, float& topKmh) {
            Bench b(20.0f, grade);
            b.ready();
            const float mark = 600.0f;
            float travelled = 0.0f;
            powered = false;
            topKmh = 0.0f;
            DriverDemand d;
            for (int i = 0; i < 300 * 60; ++i) {
                if (i % 12 == 0) {
                    RoadAhead road = openRoad(130);
                    road.stops.push_back({mark - travelled, RoadAhead::StopKind::Signal, 5});
                    d = planDrive(road, b.train->speed());
                    applyDrive(*b.train, 1, d, 0.2f);
                    if (b.train->powerNotch(1) > 0) powered = true;
                }
                const float before = b.train->speed();
                b.train->update(kDt, 0.0f);
                travelled += 0.5f * (before + b.train->speed()) * kDt;
                topKmh = std::max(topKmh, b.kmh());
            }
            shortBy = mark - travelled;
        };

        float shortBy = 0.0f, topKmh = 0.0f;
        bool powered = false;
        approach(0.02f, shortBy, powered, topKmh);
        std::printf("      up 2%%: stood %.2f m short, topped %.1f km/h, power used %s\n",
                    shortBy, topKmh, powered ? "yes" : "no");
        check(powered, "climbing to it, power comes back on rather than stalling short");
        check(shortBy > 0.0f && shortBy < 30.0f, "  and it still draws up to the mark (m)",
              shortBy, 5.9);
        check(topKmh < 30.0f, "  without charging at a signal at danger (km/h)", topKmh,
              22.0);

        approach(0.0f, shortBy, powered, topKmh);
        std::printf("      level: stood %.2f m short, power used %s\n", shortBy,
                    powered ? "yes" : "no");
        check(!powered, "on the level it coasts and brakes, using no power at all");
        check(shortBy > 0.0f && shortBy < 30.0f, "  and stops short of the mark (m)",
              shortBy, 3.7);

        approach(-0.02f, shortBy, powered, topKmh);
        std::printf("      down 2%%: stood %.2f m short, power used %s\n", shortBy,
                    powered ? "yes" : "no");
        check(!powered, "running down to it, none either");
        check(shortBy > 0.0f && shortBy < 30.0f, "  and gravity does not carry it past (m)",
              shortBy, 2.9);
    }

    std::puts("\nThe end of the road is a stop like any other");
    {
        RoadAhead road = openRoad(130);
        road.roadRunsOut = true;
        check(planDrive(road, 30.0f).targetMs < 1.0f,
              "a road that runs out inside the lookahead brings it down to a crawl",
              planDrive(road, 30.0f).targetMs, 0.8);
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
