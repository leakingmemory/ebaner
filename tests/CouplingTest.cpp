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

// Two trains meeting: coupling, and what happens when it is taken too fast.
//
// The round trip is the assertion this file exists for. Uncoupling has always been able to
// make two trains out of one; if coupling is right, doing both in turn must give back
// exactly what was there before - every set on the road it was, at the arc length it was,
// facing the way it was. That is the same invariant UncoupleTest's testNothingTeleports
// asserts for the split alone, and it is what catches a merge that quietly re-lays the sets
// out, reverses the wrong one, or loses which way round a nose-to-nose meeting was.
//
// The rest are the ways of getting a closing rate wrong that still look plausible: judging
// on speeds instead of on the rate they close at, coupling two portions that are merely
// standing against each other, or coupling to a wreck.
//
// No dataset. TrackPath takes points directly, so a kilometre of straight line is the whole
// world here, as in UncoupleTest and AirBrakeTest.

#include "Consist.h"
#include "Coupling.h"
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
    std::printf("  %-60s %s\n", what.c_str(), ok ? "ok" : "FAILED");
    if (!ok) ++failures;
}
void check(bool ok, const std::string& what, double got, double want) {
    std::printf("  %-60s %s (got %g, want %g)\n", what.c_str(), ok ? "ok" : "FAILED", got,
                want);
    if (!ok) ++failures;
}

VehicleSpec class93(int units) {
    VehicleSpec sp{};
    for (const VehicleSpec& v : kVehicleSpecs)
        if (v.body == BodyClass93 && v.units == 1) sp = v;
    sp.units = units;
    return sp;
}

// A kilometre of straight, level line that several trains can share.
struct World {
    std::vector<TrackPath> paths;
    World() {
        std::vector<glm::vec3> pts;
        for (int i = 0; i <= 40; ++i)
            pts.push_back({static_cast<float>(i) * 25.0f, 0.0f, 0.0f});
        paths.emplace_back(1u, 0u, pts, std::vector<std::uint16_t>(pts.size(), 100));
    }
    Consist train(int units, float s, float v = 0.0f) {
        Consist c(&paths, &paths[0], class93(units), s, v);
        c.attachNetwork(&paths, nullptr);
        return c;
    }
};

// Put b just off a's tail, with the given closing rate, and report what contact says.
// `sep` is the gap left between the coupler faces.
Contact meeting(World& w, float closing, float sep = 0.0f) {
    Consist a = w.train(1, 500.0f, closing); // running toward +s
    const float half = 0.5f * a.unit(0).length();
    Consist b = w.train(1, 500.0f + 2.0f * half + Consist::kCouplerGap + sep, 0.0f);
    // a's tail is its +s end (a single set laid out facing +s), so this is a's tail
    // against b's front.
    return findContact(a, b);
}

// Everything about one set that a couple-then-part must not disturb.
struct Snap {
    int path, orient;
    float s;
};
Snap snap(const Vehicle& u) { return {u.pathIdx(), u.orientation(), u.s()}; }
bool same(const Snap& x, const Snap& y) {
    return x.path == y.path && x.orient == y.orient && std::abs(x.s - y.s) < 1e-3f;
}

} // namespace

int main() {
    std::puts("\nWhat contact is, and what it is not");
    {
        World w;
        check(meeting(w, 0.5f).kind == ContactKind::Couple, "touching at 0.5 m/s: a coupling");
        check(meeting(w, 2.5f).kind == ContactKind::Rough, "at 2.5 m/s: rough");
        check(meeting(w, 6.0f).kind == ContactKind::Crash, "at 6.0 m/s: not a coupling at all");
        check(meeting(w, -0.5f).kind == ContactKind::None, "drawing apart: nothing happens");
        check(meeting(w, 0.0f).kind == ContactKind::None, "standing against each other: nothing");
        check(meeting(w, 0.5f, 5.0f).kind == ContactKind::None, "five metres apart: not touching");
    }
    {
        // Two trains on separate roads are meaningless metres apart in the air and
        // infinitely far apart along the rails. A contact test that measured through
        // space would couple these.
        World w;
        std::vector<glm::vec3> pts;
        for (int i = 0; i <= 40; ++i)
            pts.push_back({static_cast<float>(i) * 25.0f, 4.5f, 0.0f});
        w.paths.emplace_back(2u, 1u, pts, std::vector<std::uint16_t>(pts.size(), 100));
        Consist a(&w.paths, &w.paths[0], class93(1), 500.0f, 1.0f);
        Consist b(&w.paths, &w.paths[1], class93(1), 500.0f, 0.0f);
        a.attachNetwork(&w.paths, nullptr);
        b.attachNetwork(&w.paths, nullptr);
        check(findContact(a, b).kind == ContactKind::None,
              "alongside on the other road, 4.5 m away: no contact");
    }

    std::puts("\nClosing is a rate, not two speeds");
    {
        // Both running hard the same way, one gaining slowly. Neither is going slowly and
        // it is still a gentle coupling. A test on speed() would call this a crash.
        World w;
        Consist a = w.train(1, 500.0f, 12.0f);
        const float half = 0.5f * a.unit(0).length();
        Consist b = w.train(1, 500.0f + 2.0f * half + Consist::kCouplerGap, 11.5f);
        const Contact c = findContact(a, b);
        check(c.kind == ContactKind::Couple, "catching up at 0.5 m/s while both do ~12");
        check(std::abs(c.closing - 0.5f) < 1e-3f, "  and the closing rate is the difference",
              c.closing, 0.5);
        check(!c.opposed, "  they are running the same way, not nose to nose");
    }

    std::puts("\nCoupling: two trains become one");
    {
        World w;
        Consist a = w.train(2, 400.0f, 0.8f);
        const float pitch = a.unit(0).length() + Consist::kCouplerGap;
        // One pitch beyond a's front set, whose centre is half a pitch ahead of a's.
        // The pitch already carries the coupler gap; adding it again leaves them apart.
        Consist b = w.train(1, 400.0f + 1.5f * pitch, 0.0f);
        const Snap a0 = snap(a.unit(0)), a1 = snap(a.unit(1)), b0 = snap(b.unit(0));
        const float mA = a.mass(), mB = b.mass(), vA = a.velocity();
        const Contact c = findContact(a, b);
        check(c.kind == ContactKind::Couple, "they meet gently");

        a.absorb(std::move(b), c.aTail, c.opposed);
        check(a.unitCount() == 3, "three sets in one train now", a.unitCount(), 3);
        check(a.cabCount() == 6, "  and six cabs", a.cabCount(), 6);
        check(same(snap(a.unit(0)), a0) && same(snap(a.unit(1)), a1),
              "  the sets that were already here have not moved");
        check(same(snap(a.unit(2)), b0), "  and neither has the one that arrived");
        check(std::abs(a.velocity() - mA * vA / (mA + mB)) < 1e-3f,
              "  momentum is conserved, not the speed", a.velocity(), mA * vA / (mA + mB));
        check(a.velocity() < vA, "  so the pair runs slower than the shunter did",
              a.velocity(), vA);
    }

    std::puts("\nThe round trip: part it again and nothing has moved");
    {
        World w;
        // Slowly enough that the merged train is still "at a stand" by mayUncouple's
        // reckoning, or it would refuse to part again and the round trip could not be run.
        Consist a = w.train(2, 400.0f, 0.2f);
        const float pitch = a.unit(0).length() + Consist::kCouplerGap;
        // One pitch beyond a's front set, whose centre is half a pitch ahead of a's.
        // The pitch already carries the coupler gap; adding it again leaves them apart.
        Consist b = w.train(1, 400.0f + 1.5f * pitch, 0.0f);
        const Snap before[3] = {snap(a.unit(0)), snap(a.unit(1)), snap(b.unit(0))};

        const Contact c = findContact(a, b);
        a.absorb(std::move(b), c.aTail, c.opposed);
        std::optional<Consist> back = a.uncoupleAfter(1); // at the joint we just made
        check(back.has_value(), "the new joint can be parted again");
        if (back) {
            check(a.unitCount() == 2 && back->unitCount() == 1,
                  "  and gives back a train of two and a train of one");
            check(same(snap(a.unit(0)), before[0]) && same(snap(a.unit(1)), before[1]),
                  "  the first train's sets are exactly where they started");
            check(same(snap(back->unit(0)), before[2]),
                  "  and so is the one that was coupled on");
        }
    }

    std::puts("\nNose to nose: the incoming sets are turned round");
    {
        World w;
        // b laid out facing -s, so the two meet nose to nose rather than one catching the
        // other. Its sets run the opposite way and have to be reversed into the merge, or
        // the train ends up with its middle two sets back to front.
        Consist a = w.train(1, 400.0f, 0.6f);
        const float half = 0.5f * a.unit(0).length();
        // A train that genuinely faces the other way, which means its *layout* reverses
        // too: each set is laid out ahead of the last in the train's own facing, so on a
        // train facing -s, set 1 stands at a lower arc length than set 0. Flipping only
        // the orientation and leaving the positions alone would make a train whose order
        // contradicts its facing - something layOut can never produce.
        auto faceBack = [&](Consist& c, float centre) {
            const float pitch = c.unit(0).length() + Consist::kCouplerGap;
            const float n = static_cast<float>(c.unitCount());
            for (int i = 0; i < c.unitCount(); ++i) {
                Vehicle::TrackAnchor at;
                at.pathIdx = 0;
                at.s = centre - (static_cast<float>(i) - 0.5f * (n - 1.0f)) * pitch;
                at.orient = -1;
                c.unit(i).placeAt(w.paths, at);
            }
        };
        const float bFar = 400.0f + 2.0f * half + Consist::kCouplerGap + 20.0f +
                           0.5f * (a.unit(0).length() + Consist::kCouplerGap);
        Consist b = w.train(2, bFar, 0.0f);
        faceBack(b, bFar);
        const Contact c0 = findContact(a, b);
        check(c0.kind == ContactKind::None, "twenty metres apart, nothing yet");

        // Its front set's centre must sit one set-half plus the coupler gap beyond a's.
        const float bNear = 400.0f + 2.0f * half + Consist::kCouplerGap +
                            0.5f * (a.unit(0).length() + Consist::kCouplerGap);
        Consist b2 = w.train(2, bNear, 0.0f);
        faceBack(b2, bNear);
        const Contact c = findContact(a, b2);
        check(c.kind == ContactKind::Couple, "brought together gently, they couple");
        check(c.opposed, "  and it is seen as nose to nose");
    }

    std::puts("\nToo fast is not a coupling");
    {
        World w;
        Consist a = w.train(2, 400.0f, 6.0f);
        const float pitch = a.unit(0).length() + Consist::kCouplerGap;
        // One pitch beyond a's front set, whose centre is half a pitch ahead of a's.
        // The pitch already carries the coupler gap; adding it again leaves them apart.
        Consist b = w.train(1, 400.0f + 1.5f * pitch, 0.0f);
        const Contact c = findContact(a, b);
        check(c.kind == ContactKind::Crash, "at 6 m/s it is a collision", c.closing, 6.0);
        a.derail();
        b.derail();
        check(a.state() != VehicleState::OnRail, "the shunting train is off the road");
        check(b.state() != VehicleState::OnRail, "and so is the one it hit");
        check(a.unitCount() == 2 && b.unitCount() == 1, "and they are still two trains");
        check(findContact(a, b).kind == ContactKind::None,
              "nothing couples to a wreck afterwards");
    }

    std::printf("\n%s\n", failures == 0 ? "all ok" : "FAILURES");
    return failures == 0 ? 0 : 1;
}
