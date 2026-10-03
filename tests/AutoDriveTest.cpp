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
#include "SignalPaths.h"
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
        // The braking curve, read straight off. A stop 400 m away is aimed at kStopShortM
        // before the mark, so 380 m of it are usable; the train is at rest here, so
        // nothing comes off for the build-up run. 380 m is just inside kShallowM, where
        // the planned rate has eased a little off kAutoDecel - 0.25 + 0.3 * 0.95^2 =
        // 0.52 - so sqrt(2 * 0.52 * 380) = 19.9 m/s.
        RoadAhead road = openRoad(130);
        road.stops.push_back({400.0f, RoadAhead::StopKind::Signal, 7});
        const DriverDemand d = planDrive(road, 0.0f);
        check(d.stopping, "a stop ahead is a stop, not a limit");
        check(std::abs(d.targetMs - 19.89f) < 0.1f, "  and sets the speed it may be met at",
              d.targetMs, 19.89);
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
              shortBy, 24.4);
        check(topKmh < 30.0f, "  without charging at a signal at danger (km/h)", topKmh,
              22.0);

        approach(0.0f, shortBy, powered, topKmh);
        std::printf("      level: stood %.2f m short, power used %s\n", shortBy,
                    powered ? "yes" : "no");
        check(!powered, "on the level it coasts and brakes, using no power at all");
        check(shortBy > 0.0f && shortBy < 35.0f, "  and stops short of the mark (m)",
              shortBy, 24.1);

        approach(-0.02f, shortBy, powered, topKmh);
        std::printf("      down 2%%: stood %.2f m short, power used %s\n", shortBy,
                    powered ? "yes" : "no");
        check(!powered, "running down to it, none either");
        check(shortBy > 0.0f && shortBy < 35.0f, "  and gravity does not carry it past (m)",
              shortBy, 23.2);
    }

    std::puts("\nWhich signals actually mean stop");
    {
        // The auto-driver stopped at a distant, which is a warning of a signal a
        // kilometre further on and carries no authority of its own. The rule lives in
        // SignalPaths beside signalGivesAuthority now, so it is one statement in the file
        // that owns what a signal means rather than a condition in the middle of the road
        // scan, where no test could reach it and it was got wrong twice.
        auto sig = [](SignalKind k, SignalAspect a) {
            SignalPlacement sp;
            sp.kind = k;
            sp.aspect = a;
            return sp;
        };
        check(signalStopsTrain(sig(SignalKind::Exit, SignalAspect::Stop)),
              "an exit signal at danger stops the train");
        check(signalStopsTrain(sig(SignalKind::Entry, SignalAspect::Stop)),
              "  so does an entry");
        check(signalStopsTrain(sig(SignalKind::Block, SignalAspect::Stop)),
              "  and a block signal out on the line");
        check(signalStopsTrain(sig(SignalKind::StationEntry, SignalAspect::Stop)),
              "  and a simple station signal at red");
        check(signalStopsTrain(sig(SignalKind::Dwarf, SignalAspect::Stop)),
              "  and a DWARF at danger, which is a signal at danger like any other");
        check(signalStopsTrain(sig(SignalKind::Dwarf, SignalAspect::TrainOnTrack)),
              "  as is one showing a train standing in the road ahead");
        check(!signalStopsTrain(sig(SignalKind::Distant, SignalAspect::Stop)),
              "a distant never does - it repeats a signal a mile further on");
        check(!signalStopsTrain(sig(SignalKind::StationEntry, SignalAspect::Dark)),
              "nor a dark one, which is a station switched off, not a road refused");
        check(!signalStopsTrain(sig(SignalKind::Exit, SignalAspect::Clear)),
              "and a signal offering a road does not stop anything");
        check(!signalStopsTrain(sig(SignalKind::Exit, SignalAspect::ClearReduced)),
              "  including one offering it at a reduced speed");
        check(!signalStopsTrain(sig(SignalKind::Dwarf, SignalAspect::Clear)),
              "  a dwarf included");

        // The one that goes the other way round: a dwarf can authorise a movement PAST a
        // main signal at danger, which is what shunting past a red exit is. It does not
        // work in reverse - a main signal cannot wave a movement past a dwarf at danger.
        SignalPlacement shunt = sig(SignalKind::Exit, SignalAspect::Stop);
        shunt.withDwarf = true;
        shunt.dwarfAspect = SignalAspect::Clear;
        check(!signalStopsTrain(shunt),
              "a dwarf off under a red exit lets a shunt past the exit");
        check(!signalGivesMainAuthority(shunt),
              "  but it is a shunt, not a train: no main authority is given");
        check(signalGivesMainAuthority(sig(SignalKind::Exit, SignalAspect::Clear)),
              "  which a green exit does give");
        check(!signalGivesMainAuthority(sig(SignalKind::Dwarf, SignalAspect::Clear)),
              "  and a dwarf never does, however clear it is");
    }

    std::puts("\nWhere a train stands at a station worked by hand signals");
    {
        // Crossing two trains at Oteraga, the red flag was out and the first was routed
        // into the loop. It ran straight through and nearly met the other head on: the
        // flag post is authored as a point on ONE track - the main - and the driver only
        // looked for marks on the road it was taking, so a train in the loop never met
        // it. The flag belongs to the station; where the train stands is worked out from
        // the road it is on. This is that working out, with no dataset in it.
        StationStop ss;
        check(std::abs(stationStopPoint(ss) - 0.0f) < 0.01f,
              "with nothing known it falls back", stationStopPoint(ss), 0.0);
        ss.fallback = 310.0f;
        check(std::abs(stationStopPoint(ss) - 310.0f) < 0.01f,
              "  to the station itself, put on the road", stationStopPoint(ss), 310.0);

        // The band: between the switches at each end. On a loop that is where its road
        // leaves the main and rejoins it; on the main, the outermost pair. One rule, and
        // a station as complicated as Fauske falls out of it rather than needing its own.
        ss.haveBand = true;
        ss.bandFrom = 200.0f;
        ss.bandTo = 700.0f;
        check(std::abs(stationStopPoint(ss) - 450.0f) < 0.01f,
              "between the switches, it stands in the middle of them",
              stationStopPoint(ss), 450.0);

        // The TXP's own spot is better than the middle, being where the order is given,
        // and is authored per track - Oteraga has one on the main and one on the loop.
        ss.haveTxp = true;
        ss.txpAt = 380.0f;
        check(std::abs(stationStopPoint(ss) - 380.0f) < 0.01f,
              "  unless the TXP stands somewhere inside it", stationStopPoint(ss), 380.0);
        ss.txpAt = 900.0f; // beyond the far switch
        check(std::abs(stationStopPoint(ss) - 700.0f) < 0.01f,
              "  and never outside it, however far out the TXP is",
              stationStopPoint(ss), 700.0);
        ss.txpAt = 380.0f;

        // A passenger train stops at a platform, positioned as near the TXP as the
        // platform allows.
        ss.passenger = true;
        ss.havePlatform = true;
        ss.platformFrom = 250.0f;
        ss.platformTo = 330.0f;
        check(std::abs(stationStopPoint(ss) - 330.0f) < 0.01f,
              "a passenger train stops at the platform, nearest the TXP",
              stationStopPoint(ss), 330.0);
        ss.passenger = false;
        check(std::abs(stationStopPoint(ss) - 380.0f) < 0.01f,
              "  which is no business of a freight train", stationStopPoint(ss), 380.0);
        ss.passenger = true;

        // ...and the crossing area wins where the two disagree. A platform outside the
        // band is ignored rather than clamped to its edge: the edge is the switch, and
        // standing on the switch is the one thing this is all for.
        ss.platformFrom = 800.0f;
        ss.platformTo = 900.0f;
        check(std::abs(stationStopPoint(ss) - 380.0f) < 0.01f,
              "a platform outside the switches is not used at all",
              stationStopPoint(ss), 380.0);
        // Half in, half out: the half that is inside is what it stops at.
        ss.platformFrom = 600.0f;
        ss.platformTo = 900.0f;
        ss.txpAt = 380.0f;
        check(std::abs(stationStopPoint(ss) - 600.0f) < 0.01f,
              "  and one half inside is used for the half that is",
              stationStopPoint(ss), 600.0);
    }

    std::puts("\nA crawl is not an emergency");
    {
        // Being a hundred per cent over the limit sounds alarming and is not, when the
        // limit is five km/h. The energy in 5-10 km/h is a quarter of what is in 10-15,
        // and a brake chosen from the deceleration the arithmetic asks for does not know
        // that: at ten km/h with the mark eight metres off, the old sum took the
        // build-up run out of the distance, floored the remainder at a metre, and asked
        // for over 3 m/s2 - full service, to lose five km/h. The train stopped dead
        // short of the mark, had to be driven up again, and did it over and over.
        Bench b;
        b.ready();
        const float mark = 150.0f;
        int hardest = 0, restarts = 0;
        bool wasStopped = true;
        float travelled = 0.0f;
        DriverDemand d;
        for (int i = 0; i < 200 * 60; ++i) {
            if (i % 12 == 0) {
                RoadAhead road = openRoad(40);
                road.stops.push_back({mark - travelled, RoadAhead::StopKind::Station, 1});
                d = planDrive(road, b.train->speed());
                applyDrive(*b.train, 1, d, 0.2f);
                hardest = std::max(hardest, b.train->brakeNotch(1));
            }
            const float before = b.train->speed();
            b.train->update(kDt, 0.0f);
            travelled += 0.5f * (before + b.train->speed()) * kDt;
            const bool stopped = b.train->speed() < 0.02f;
            if (wasStopped && !stopped) ++restarts;
            wasStopped = stopped;
        }
        std::printf("      stood %.1f m short, hardest B%d, %d start(s) from a stand\n",
                    mark - travelled, hardest, restarts);
        check(hardest <= 2, "nothing harder than B2 is used up to a stop at a crawl",
              hardest, 2.0);
        check(restarts <= 1, "  and it is not stopped dead and driven up again, over and "
              "over (starts)", restarts, 1.0);
        check(mark - travelled > 0.0f && mark - travelled < 35.0f,
              "  it still stands short of the mark (m)", mark - travelled, 24.0);

        // The cap is on the SPEED, not on how far over the limit the train is.
        check(notchCapAt(2.0f) == 2, "under 14 km/h, B2 is the hardest there is",
              notchCapAt(2.0f), 2.0);
        check(notchCapAt(6.0f) == 3, "  B3 up to 29", notchCapAt(6.0f), 3.0);
        check(notchCapAt(20.0f) == Vehicle::kMaxBrakeNotch,
              "  and full service only at a speed worth it", notchCapAt(20.0f),
              double(Vehicle::kMaxBrakeNotch));
    }

    std::puts("\nThe end of the rails");
    {
        // Driven at a real dead end, with the road read the way the sim reads it, because
        // the thing being tested is the distance the walk reports when it runs out.
        //
        // This used to be answered by clamping the whole target to a crawl whenever the
        // lookahead failed to cover its two kilometres. Approaching Bodo, which is the
        // end of the line, that is a train doing 3 km/h for the last two kilometres of a
        // 40 km/h station approach - the driver found the buffers and answered by
        // slowing down everywhere instead of planning a stop at them.
        std::vector<glm::vec3> pts;
        for (int i = 0; i <= 60; ++i) pts.push_back({static_cast<float>(i) * 25.0f, 0, 0});
        std::vector<TrackPath> paths;
        paths.emplace_back(1u, 0u, pts, std::vector<std::uint16_t>(pts.size(), 200));
        const float railsEnd = paths[0].length();
        Consist c(&paths, &paths[0], *specNamed("Class 93 (T"), 700.0f, 0.0f);
        c.attachNetwork(&paths, nullptr);
        c.toggleEngines();
        c.setReverser(1, 1);
        c.setBrakeNotch(1, 0);
        for (int i = 0; i < 40 * 60; ++i) c.update(kDt, 0.0f);

        std::vector<Vehicle::RoadStretch> stretches;
        float topKmh = 0.0f;
        DriverDemand d;
        for (int i = 0; i < 300 * 60; ++i) {
            if (i % 12 == 0) {
                RoadAhead road;
                road.here = 130;
                road.roadRunsOut = !c.roadAhead(kLookAheadM, stretches);
                if (road.roadRunsOut && !stretches.empty()) {
                    const Vehicle::RoadStretch& last = stretches.back();
                    const float reach = last.dist0 + std::abs(last.sTo - last.sFrom);
                    road.stops.push_back({std::max(0.0f, reach - kEndOfTrackM),
                                          RoadAhead::StopKind::EndOfTrack, -1});
                }
                d = planDrive(road, c.speed());
                applyDrive(c, 1, d, 0.2f);
            }
            c.update(kDt, 0.0f);
            topKmh = std::max(topKmh, c.speed() * kMsToKmh);
        }
        // Where the nose finished, against where the rails stop.
        const float nose = c.lead().s() + 0.5f * c.lead().length();
        std::printf("      rails end at %.0f m; the nose stopped %.1f m short, "
                    "having run up to %.1f km/h\n", railsEnd, railsEnd - nose, topKmh);
        check(c.speed() < 0.05f, "it comes to a stand", c.speed(), 0.0);
        check(nose < railsEnd, "  before the rails end, not past them (m short)",
              railsEnd - nose, kEndOfTrackM);
        check(railsEnd - nose < 80.0f, "  and close enough to have used the road",
              railsEnd - nose, kEndOfTrackM + kStopShortM);
        check(topKmh > 20.0f,
              "  and it ran there rather than crawling the whole way (km/h)", topKmh, 29.0);
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
