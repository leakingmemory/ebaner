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

#include "Coupling.h"

#include "Consist.h"
#include "Vehicle.h"

#include <cmath>

namespace {

// How far past its own coupler face a train will look for another. Only a few metres: this
// is asking "is something against my buffers", not "is something down the line".
constexpr float kProbeM = 6.0f;

// The gap from `a`'s end to `b`'s end, measured along the rails, or nothing where the two
// ends are not on the same road within the probe distance.
//
// The easy case, and the one that happens at a platform, is both ends on one path: then the
// answer is a subtraction. Where a turnout lies between them the arc lengths belong to
// different paths and cannot be compared at all, so the end of `a` is walked outward and
// asked where it lands - which follows the points, and is the same walk that puts the
// trailing sets of a train behind the leading one.
bool railGap(const Consist& a, bool aTail, const Consist& b, bool bTail, float& gap) {
    const Consist::End ea = a.end(aTail);
    const Consist::End eb = b.end(bTail);
    if (ea.pathIdx < 0 || eb.pathIdx < 0) return false;

    if (ea.pathIdx == eb.pathIdx) {
        // Signed by the direction leaving `a`, so a positive gap means `b` is out in front
        // of that face and a negative one means they have run into each other.
        gap = static_cast<float>(ea.outward) * (eb.s - ea.s);
        return gap < kProbeM;
    }

    // Over a turnout. Step outward from a's face and watch for the walk to arrive on b's
    // path; the distance stepped when it does is the gap. Half-metre steps: fine enough
    // that a short connector between two paths is not stepped straight over, coarse enough
    // that this is a dozen walks and not a thousand.
    const Vehicle& u = aTail ? a.unit(a.unitCount() - 1) : a.unit(0);
    const float half = 0.5f * u.length();
    for (float d = 0.0f; d <= kProbeM; d += 0.5f) {
        Vehicle::TrackAnchor at;
        if (!u.anchorAtOffset(static_cast<float>(ea.outward) * u.orientation() * (half + d),
                              at))
            break;
        if (at.pathIdx != eb.pathIdx) continue;
        // On b's road. The walk has covered d, and what is left is from where it came out
        // to b's coupler face along that road.
        gap = d + std::abs(eb.s - at.s);
        return gap < kProbeM;
    }
    return false;
}

} // namespace

Contact findContact(const Consist& a, const Consist& b) {
    Contact c;
    if (a.unitCount() == 0 || b.unitCount() == 0) return c;
    // Nothing couples to a wreck. A derailed train is not on the road any more and its
    // arc length is frozen where it left it, so the gap below would be measured against a
    // place it is no longer standing.
    if (a.state() != VehicleState::OnRail || b.state() != VehicleState::OnRail) return c;

    // Four ends to try - either end of one against either end of the other - and the
    // nearest pair that is actually touching is the one that met.
    bool found = false;
    float best = 0.0f;
    for (const bool at : {false, true})
        for (const bool bt : {false, true}) {
            float gap = 0.0f;
            if (!railGap(a, at, b, bt, gap)) continue;
            if (gap > kContactGap || gap < -kMaxOverlap) continue; // apart, or not this end
            // Nearest to touching, judged on how far from zero it is: of the four ends,
            // the pair that actually met is the pair with almost nothing between them.
            if (found && std::abs(gap) >= std::abs(best)) continue;
            found = true;
            best = gap;
            c.aTail = at;
            c.bTail = bt;
        }
    if (!found) return c;

    const Consist::End ea = a.end(c.aTail);
    const Consist::End eb = b.end(c.bTail);
    c.gap = best;

    const Vehicle& ua = c.aTail ? a.unit(a.unitCount() - 1) : a.unit(0);
    const Vehicle& ub = c.bTail ? b.unit(b.unitCount() - 1) : b.unit(0);
    // Nose to nose when the two trains' sets face opposite ways, so that becoming one
    // train means turning the incoming ones round. Not to be read off `outward`: two
    // trains touching end to end always leave in opposite directions, however they face,
    // so that comparison is true of every contact there is and says nothing.
    c.opposed = ua.orientation() != ub.orientation();

    // The closing rate, in the shared road's own direction. Each train carries its speed
    // in its own facing, so both are read into +s first. This is a rate and not a
    // difference of magnitudes: one train catching another from behind at 1.0 m/s while
    // that one runs at 0.9 is closing at 0.1, and couples, though neither is going slowly.
    const float va = a.velocity() * static_cast<float>(ua.orientation());
    const float vb = b.velocity() * static_cast<float>(ub.orientation());
    // a closes on b at the rate a's outward direction is gaining on b.
    c.closing = static_cast<float>(ea.outward) * (va - vb);

    if (c.closing <= 0.0f) {
        // Touching, but drawing apart or holding station. Two portions standing against
        // each other after a parting are here every step, and must not be coupled by it.
        c.kind = ContactKind::None;
        return c;
    }
    c.kind = c.closing <= kCoupleMaxSpeed  ? ContactKind::Couple
             : c.closing <= kRoughMaxSpeed ? ContactKind::Rough
                                           : ContactKind::Crash;
    return c;
}
