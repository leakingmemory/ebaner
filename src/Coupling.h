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

#pragma once

class Consist;

// Two trains meeting.
//
// Until now nothing in the simulator noticed that another train was there at all: every
// consist was stepped alone and two of them driven together passed through each other in
// silence. This is the one place that looks, and what it finds is decided entirely by how
// fast the gap was closing when it ran out.
//
// A Class 93 has Scharfenberg couplers, which engage by being driven together - so there is
// nothing to arm and nothing to aim at, and the collision is not a separate mechanism but
// the same contact judged by a bigger number.

// What a coupling can be taken at (m/s). The Scharfenberg engages up to about walking pace
// and a little over; past that the shock goes into the drawgear and the hoses instead, and
// past that again it is not a coupling at all.
inline constexpr float kCoupleMaxSpeed = 1.4f; //  5 km/h - engages cleanly
inline constexpr float kRoughMaxSpeed = 4.0f;  // 14 km/h - engages, but roughly
// How close the two coupler faces must come to be touching. Small: the faces are drawn
// closed and Consist::kCouplerGap already keeps two coupled noses off each other.
inline constexpr float kContactGap = 0.15f;
// How far one train may already be inside another and still be treated as having just
// met it. A step at the collision threshold moves a train under 7 cm, so anything past
// this is not a contact that happened now. It also keeps the *other* end of the same
// train out of it: measured from one end, the far end of a 42 m set is eighty-odd metres
// negative, which without a floor would read as a very deep interpenetration.
inline constexpr float kMaxOverlap = 2.0f;

enum class ContactKind {
    None,   // not touching, or drawing apart
    Couple, // gently enough to engage
    Rough,  // engages, but hard enough to do damage
    Crash,  // not a coupling
};

struct Contact {
    ContactKind kind = ContactKind::None;
    float gap = 0.0f;     // metres along the rails; negative once they interpenetrate
    float closing = 0.0f; // m/s, + = coming together. What decides `kind`.
    bool aTail = false;   // which end of each train is the one in contact
    bool bTail = false;
    // True where they met nose to nose. Then the second train's sets run the other way
    // round from the first's and have to be reversed into it.
    bool opposed = false;
};

// Whether these two trains are touching, and how hard. `None` for anything that is not:
// too far apart, drawing apart rather than together, or already off the rails.
//
// The gap is measured along the rails between the two coupler faces, not through the air:
// two trains standing on the two roads of a passing loop are meaningless metres apart in
// space and infinitely far apart in the only sense that matters. Where the two ends are on
// one path that is a subtraction; where a turnout lies between them it is a walk, which is
// why this needs the switch network and not just the geometry.
Contact findContact(const Consist& a, const Consist& b);
