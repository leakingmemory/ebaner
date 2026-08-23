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

// Uncoupling: parting a train at a coupler, and the hold both portions come away with.
//
// No dataset. TrackPath takes points directly, so a kilometre of straight line and a
// kilometre of 2 % falling grade are the whole world here, and everything measured is
// the shipped Consist rather than a stand-in for it.

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
    std::printf("  %-62s %s\n", what.c_str(), ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}

void check(bool ok, const std::string& what, double got, double want) {
    std::printf("  %-62s %s (got %g, want %g)\n", what.c_str(), ok ? "ok" : "FAILED",
                got, want);
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

// A train of `units` sets standing where it is put. `dropPerStep` is the fall in metres
// over each 25 m of line: 0 for the level, 0.5 for the 2 % the runaway test needs.
struct Bench {
    std::vector<TrackPath> paths;
    std::optional<Consist> train;

    Bench(int units, float dropPerStep = 0.0f) {
        std::vector<glm::vec3> pts;
        for (int i = 0; i <= 40; ++i)
            pts.push_back({static_cast<float>(i) * 25.0f, 0.0f,
                           -dropPerStep * static_cast<float>(i)});
        paths.emplace_back(1u, 0u, pts, std::vector<std::uint16_t>(pts.size(), 100));
        train.emplace(&paths, &paths[0], class93(units), 500.0f);
    }
};

// Everything about one set that an uncoupling must not disturb.
struct Snapshot {
    int pathIdx, orient;
    float s, mr, bc;
    bool engines;
};
Snapshot snap(const Vehicle& u) {
    return {u.pathIdx(), u.orientation(), u.s(), u.mrPressure(), u.bcPressure(),
            u.enginesOn()};
}
bool same(const Snapshot& a, const Snapshot& b) {
    return a.pathIdx == b.pathIdx && a.orient == b.orient && a.s == b.s &&
           a.mr == b.mr && a.bc == b.bc && a.engines == b.engines;
}

// Highest brake-cylinder pressure anywhere on the train.
float maxBc(const Consist& c) {
    float m = 0.0f;
    for (int i = 0; i < c.unitCount(); ++i) m = std::max(m, c.unit(i).bcPressure());
    return m;
}

void step(Consist& c, int n) {
    for (int i = 0; i < n; ++i) c.update(kDt);
}

// --- 1. what may be parted, and into what --------------------------------------

void testPartition() {
    std::printf("Partition\n");
    {
        Bench b(3);
        std::optional<Consist> rear = b.train->uncoupleAfter(0);
        check(rear.has_value(), "3 sets, parted behind set 1: a train comes back");
        check(b.train->unitCount() == 1, "  the front keeps 1 set",
              b.train->unitCount(), 1);
        check(rear && rear->unitCount() == 2, "  the rear takes 2",
              rear ? rear->unitCount() : -1, 2);
    }
    {
        Bench b(3);
        std::optional<Consist> rear = b.train->uncoupleAfter(1);
        check(b.train->unitCount() == 2 && rear && rear->unitCount() == 2 - 1,
              "3 sets, parted behind set 2: 2 stay and 1 goes");
    }
    {
        Bench b(3);
        check(!b.train->uncoupleAfter(2).has_value(),
              "there is no coupler behind the last set");
        check(!b.train->uncoupleAfter(-1).has_value(), "nor behind set -1");
        check(b.train->unitCount() == 3, "  and a refused split changes nothing",
              b.train->unitCount(), 3);
    }
    {
        Bench b(1);
        check(b.train->couplerCount() == 0, "a train of one has no couplers",
              b.train->couplerCount(), 0);
        check(!b.train->uncoupleAfter(0).has_value(), "  and cannot be parted");
        check(b.train->unitCount() == 1, "  and is still a train of one",
              b.train->unitCount(), 1);
    }
}

// --- 2. nothing moves ----------------------------------------------------------

void testNothingTeleports() {
    std::printf("Nothing teleports\n");
    Bench b(3);
    b.train->toggleEngines();
    step(*b.train, 120); // let the air and the engines settle into some state

    std::vector<Snapshot> before;
    for (int i = 0; i < b.train->unitCount(); ++i) before.push_back(snap(b.train->unit(i)));
    const float gapBefore =
        b.train->unit(2).s() - b.train->unit(1).s(); // arc length between set centres

    std::optional<Consist> rear = b.train->uncoupleAfter(1);
    if (!rear) { check(false, "the split went through"); return; }

    bool ok = true;
    for (int i = 0; i < b.train->unitCount(); ++i)
        ok = ok && same(before[i], snap(b.train->unit(i)));
    for (int i = 0; i < rear->unitCount(); ++i)
        ok = ok && same(before[2 + i], snap(rear->unit(i)));
    check(ok, "every set is where it was, on the road it was, with the air it had");

    const float pitch = b.train->unit(1).length() + Consist::kCouplerGap;
    const float gapAfter = rear->unit(0).s() - b.train->unit(1).s();
    check(std::abs(gapAfter - gapBefore) < 1e-4f,
          "the parted coupler has not been opened by the split", gapAfter, gapBefore);
    check(std::abs(gapAfter - pitch) < 0.01f, "  and still stands at the coupler pitch",
          gapAfter, pitch);
    check(rear->speed() == b.train->speed(), "both portions keep the speed they had",
          rear->speed(), b.train->speed());
}

// --- 3. only at a stand --------------------------------------------------------

void testAtAStand() {
    std::printf("Only at a stand\n");
    Bench b(2);
    const char* why = nullptr;
    check(b.train->mayUncouple(0, why), "standing still, the coupler may be parted");

    // Roll it: reverser out, handle in release, and let the level line's own numbers
    // be irrelevant - push it along by hand until it is moving.
    Bench m(2, 0.5f); // 2 % down
    m.train->setBrakeNotch(0, 0);
    m.train->setReverser(0, 1);
    step(*m.train, 600);
    check(m.train->speed() > Consist::kUncoupleMaxSpeed, "a train rolling on the grade",
          m.train->speed(), 1.0);
    check(!m.train->mayUncouple(0, why), "  may not be parted");
    check(why && std::string(why) == "THE TRAIN IS MOVING", "  and says why");
    check(!m.train->uncoupleAfter(0).has_value(), "  and the split is refused");
    check(m.train->unitCount() == 2, "  leaving the train whole", m.train->unitCount(), 2);
}

// --- 4. both portions come away braked -----------------------------------------

void testBothBraked() {
    std::printf("Both portions come away braked\n");
    Bench b(3);
    b.train->toggleEngines();
    step(*b.train, 300); // charge the air, release the brakes
    // Standing, but with the cab live: reverser forward and the handle in full release,
    // which is the state a handle would rule from if anything let it.
    b.train->setReverser(0, 1);
    b.train->setBrakeNotch(0, 0);
    step(*b.train, 120);
    check(!b.train->emergencyLine(), "before: cab live, handle in release, no emergency");

    std::optional<Consist> rear = b.train->uncoupleAfter(1);
    if (!rear) { check(false, "the split went through"); return; }

    // ...and now he winds the power on, on a train that has just come apart.
    b.train->setPowerNotch(0, 5);
    b.train->update(kDt);
    rear->update(kDt);
    check(b.train->emergencyLine(), "the front is in emergency");
    check(rear->emergencyLine(), "the rear is in emergency");
    check(b.train->effectiveNotch() == Vehicle::kEmergencyNotch,
          "  and the front's brakes are at emergency",
          b.train->effectiveNotch(), Vehicle::kEmergencyNotch);
    check(rear->effectiveNotch() == Vehicle::kEmergencyNotch,
          "  and so are the rear's", rear->effectiveNotch(), Vehicle::kEmergencyNotch);

    step(*b.train, 300);
    step(*rear, 300);
    check(maxBc(*b.train) > 2.0f, "the front's cylinders come up", maxBc(*b.train), 2.0);
    check(maxBc(*rear) > 2.0f, "the rear's cylinders come up", maxBc(*rear), 2.0);
    // The driver never let go of the power handle. The hold has to take the traction
    // with it, or a braked train sits there pulling against its own shoes.
    check(b.train->tractiveEffort() == 0.0f,
          "the front makes no tractive effort, handle still at P5",
          b.train->tractiveEffort(), 0.0);
}

// --- 5. the hold cannot be cheated ---------------------------------------------

// A three-set train parted, with the driver at the rear portion's outer cab, reverser
// forward and handle in release: the state the requirement is about.
// Parted behind set 1 so the rear keeps two sets: it needs a coupler of its own for
// the re-latching check, and two cabs for the far-end cheat to be a cheat at all.
struct Parted {
    Bench b{3};
    std::optional<Consist> rear;
    int cab = 0; // the driver's cab on `rear`, at its outer end
    Parted() {
        b.train->toggleEngines();
        step(*b.train, 300);
        b.train->setReverser(0, 1);
        b.train->setBrakeNotch(0, 0);
        rear = b.train->uncoupleAfter(0);
        if (!rear) return;
        cab = rear->cabCount() - 1;
        rear->setReverser(cab, 1);
        rear->setBrakeNotch(cab, 0); // asking for full release, and not getting it
        rear->update(kDt);
    }
};

void testCannotBeCheated() {
    std::printf("The hold cannot be cheated\n");

    { // he never lets go
        Parted p;
        if (!p.rear) { check(false, "the split went through"); return; }
        bool heldThroughout = true;
        for (int i = 0; i < 3600; ++i) {
            p.rear->update(kDt);
            if (!p.rear->emergencyLine()) heldThroughout = false;
        }
        check(heldThroughout, "a minute of holding F in release: held the whole way");
        check(maxBc(*p.rear) > 2.0f, "  brakes still on", maxBc(*p.rear), 2.0);
        check(p.rear->speed() < 0.01f, "  and it has not moved", p.rear->speed(), 0.0);
    }

    { // and it really is the hold doing it, not the interlock or a safety device
        Parted p;
        if (!p.rear) return;
        // Two cabs in gear makes activeCab() -1 and the interlock true; back to one
        // leaves the interlock false with nothing else holding the brakes but the hold.
        p.rear->setReverser(0, 1);
        p.rear->update(kDt);
        p.rear->setReverser(0, 0);
        p.rear->update(kDt);
        check(p.rear->activeCab() == p.cab, "exactly one cab in gear", p.rear->activeCab(),
              p.cab);
        check(!p.rear->interlockEmergency(), "  so the reverser interlock is not holding it");
        check(p.rear->trippedUnit() < 0, "  and no set's safety device is either",
              p.rear->trippedUnit(), -1);
        check(p.rear->emergencyLine(), "  and it is STILL in emergency - the hold");
    }

    { // the far-end cheat: cycle the other cab's reverser, not your own
        Parted p;
        if (!p.rear) return;
        bool everReleased = false;
        auto watch = [&] {
            p.rear->update(kDt);
            if (p.rear->uncoupleHold() == Consist::UncoupleHold::None)
                everReleased = true;
        };
        p.rear->setReverser(0, 1); watch();  // far end into gear
        p.rear->setReverser(0, 0); watch();  // ...and out again
        p.rear->setReverser(0, 1); watch();
        p.rear->setReverser(0, 0); watch();
        check(!everReleased, "cycling the FAR cab's reverser never releases it");
        check(p.rear->emergencyLine(), "  still in emergency");
    }

    { // the sequence that is supposed to work
        Parted p;
        if (!p.rear) return;
        p.rear->setReverser(p.cab, 0); // every cab now at N
        p.rear->update(kDt);
        check(p.rear->uncoupleHold() == Consist::UncoupleHold::AtNeutral,
              "reverser to N: the hold arms");
        check(p.rear->emergencyLine(), "  and the brakes are still on at N");
        p.rear->setReverser(p.cab, 1);
        p.rear->update(kDt);
        check(p.rear->uncoupleHold() == Consist::UncoupleHold::None,
              "out of N again: the hold clears");
        check(!p.rear->emergencyLine(), "  and the emergency line drops");
        check(p.rear->effectiveNotch() == p.rear->brakeNotch(p.cab),
              "  and the handle rules again", p.rear->effectiveNotch(),
              p.rear->brakeNotch(p.cab));
        const float bcHeld = maxBc(*p.rear);
        step(*p.rear, 300);
        check(maxBc(*p.rear) < bcHeld, "  the cylinders bleed off", maxBc(*p.rear), bcHeld);

        // ...and parting it again latches it again.
        p.rear->update(kDt);
        std::optional<Consist> third = p.rear->uncoupleAfter(0);
        check(third.has_value(), "parting the released portion again");
        check(p.rear->uncoupleHold() != Consist::UncoupleHold::None, "  re-latches it");
    }
}

// --- 6. already at Neutral when it was parted -----------------------------------

void testAlreadyAtNeutral() {
    std::printf("Already at Neutral when parted\n");
    Bench b(2);
    step(*b.train, 300);
    std::optional<Consist> rear = b.train->uncoupleAfter(0); // every reverser at N
    if (!rear) { check(false, "the split went through"); return; }
    check(rear->uncoupleHold() == Consist::UncoupleHold::InGear,
          "it is held the instant it is parted, reversers notwithstanding");
    rear->update(kDt);
    check(rear->uncoupleHold() == Consist::UncoupleHold::AtNeutral,
          "one step, and the hold arms itself: nothing was in gear");
    check(rear->emergencyLine(), "  still braked");
    rear->setReverser(rear->cabCount() - 1, 1);
    rear->update(kDt);
    check(rear->uncoupleHold() == Consist::UncoupleHold::None,
          "one movement of the reverser is enough - not N first, then out");
    check(!rear->emergencyLine(), "  and it is released");
}

// --- 7. the detached portion is really simulated --------------------------------

void testItRollsAway() {
    std::printf("The detached portion rolls away\n");
    Bench b(2, 0.5f); // 2 % falling
    step(*b.train, 300);
    const float gap0 = b.train->unit(1).s() - b.train->unit(0).s();
    std::optional<Consist> rear = b.train->uncoupleAfter(0);
    if (!rear) { check(false, "the split went through"); return; }

    // Release only the rear, by its own reverser. The front is left held.
    const int rc = rear->cabCount() - 1;
    rear->update(kDt);                 // -> AtNeutral (nothing in gear)
    rear->setReverser(rc, 1);
    rear->setBrakeNotch(rc, 0);
    rear->update(kDt);                 // -> None
    check(rear->uncoupleHold() == Consist::UncoupleHold::None, "the rear is released");
    check(b.train->uncoupleHold() != Consist::UncoupleHold::None, "the front is not");

    const float rearS0 = rear->unit(0).s(), frontS0 = b.train->unit(0).s();
    float lastS = rearS0;
    bool monotone = true;
    for (int i = 0; i < 1200; ++i) { // 20 s
        b.train->update(kDt);
        rear->update(kDt);
        if (rear->unit(0).s() < lastS - 1e-4f) monotone = false;
        lastS = rear->unit(0).s();
    }
    const float rearMoved = std::abs(rear->unit(0).s() - rearS0);
    const float frontMoved = std::abs(b.train->unit(0).s() - frontS0);
    check(rear->speed() > 1.0f, "20 s later the rear is running away", rear->speed(), 1.0);
    check(monotone, "  and it went one way about it");
    check(rearMoved > frontMoved + 10.0f, "  far further than the held front",
          rearMoved, frontMoved + 10.0);
    const float gap = std::abs(rear->unit(0).s() - b.train->unit(0).s());
    check(gap > gap0 + 1.5f, "  and the gap at the coupler has opened up", gap - gap0, 1.5);
}

// --- 8. the emergency line no longer spans the parted coupler -------------------

void testLineDoesNotSpanTheGap() {
    std::printf("The emergency line stops at the parted coupler\n");
    Bench b(2);
    step(*b.train, 300);
    std::optional<Consist> rear = b.train->uncoupleAfter(0);
    if (!rear) { check(false, "the split went through"); return; }
    // Clear both holds properly, then let the air settle.
    for (Consist* c : {&*b.train, &*rear}) {
        c->update(kDt);
        c->setReverser(c->cabCount() - 1, 1);
        c->setBrakeNotch(c->cabCount() - 1, 0);
        c->update(kDt);
    }
    step(*b.train, 300);
    step(*rear, 300);
    check(!b.train->emergencyLine() && !rear->emergencyLine(), "both released");

    rear->unit(0).ventReservoir(5.0f); // the rear's own low-reservoir device trips
    rear->update(kDt);
    b.train->update(kDt);
    check(rear->emergencyLine(), "the rear's safety device brakes the rear");
    check(rear->trippedUnit() == 0, "  and names the set", rear->trippedUnit(), 0);
    check(!b.train->emergencyLine(), "and the front, no longer coupled to it, is free");
}

// --- 9. cabs renumber -----------------------------------------------------------

void testCabsRenumber() {
    std::printf("Cabs renumber on both portions\n");
    Bench b(3);
    std::optional<Consist> rear = b.train->uncoupleAfter(0);
    if (!rear) { check(false, "the split went through"); return; }
    for (Consist* c : {&*b.train, &*rear}) {
        const std::string who = c == &*rear ? "rear" : "front";
        check(c->cabCount() == 2 * c->unitCount(), who + ": two cabs to a set",
              c->cabCount(), 2 * c->unitCount());
        check(c->cabDrivable(0), who + ": the first cab drives");
        check(c->cabDrivable(c->cabCount() - 1), who + ": the last cab drives");
        bool innerShut = true, innerRefuses = true;
        for (int i = 1; i < c->cabCount() - 1; ++i) {
            if (c->cabDrivable(i)) innerShut = false;
            c->setReverser(i, 1);
            if (c->reverser(i) != 0) innerRefuses = false;
        }
        check(innerShut, who + ": every cab at a coupler is shut down");
        check(innerRefuses, who + ": and refuses the reverser");
    }
}

} // namespace

int main() {
    std::printf("Uncoupling\n");
    testPartition();
    testNothingTeleports();
    testAtAStand();
    testBothBraked();
    testCannotBeCheated();
    testAlreadyAtNeutral();
    testItRollsAway();
    testLineDoesNotSpanTheGap();
    testCabsRenumber();
    std::printf("%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
