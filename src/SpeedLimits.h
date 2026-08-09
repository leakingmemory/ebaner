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

#include <glm/glm.hpp>

#include <vector>

#include "TrackPath.h"

// Line speeds come in gappy: the import knows a limit at most surveyed points and nothing at
// the rest. Two rules fill the gaps, and between them they make the limit *directional* even
// though the data is not - the same undefined stretch inherits from whichever side it is
// approached:
//
//   - a path that begins undefined in the direction travelled starts at kDefaultKmh;
//   - a defined limit carries forward through undefined track until something redefines it.
//
// Resolution runs along one TrackPath, which is already a chained road rather than a single
// imported segment. It deliberately does not cross junctions: a limit does not survive into
// the next path, which resets to the default if it too begins undefined.
inline constexpr int kDefaultKmh = 40;

// A resolved limit taking effect at `s` and holding until the next one. `s` is measured along
// the path's own parameterisation whichever way it is being read, so for dir = -1 the values
// descend.
struct SpeedStretch {
    float s = 0.0f;
    int kmh = 0;
};

// Resolve one path in one direction: dir = +1 along the path, -1 against it. Stretches come
// back in travel order, the first one starting where the path does in that direction.
std::vector<SpeedStretch> resolveSpeeds(const TrackPath& p, int dir);

// A rise is signed once, where it happens. A drop takes two signs: a small marker at the
// point itself, and a larger warning carrying the new limit a braking distance before it.
enum class SpeedSignKind {
    Increase,         // large triangle, apex up, numeral: the limit rises here
    ReductionMarker,  // small triangle, apex down, no numeral: the limit drops here
    ReductionWarning, // large triangle, apex down, numeral: a drop is coming
};

// A lineside speed sign. `tangent` is the direction it faces (the direction of travel it
// applies to) and `right` that direction's cross-track right, which is the side it stands on.
struct SpeedSign {
    glm::vec3 pos{0.0f};
    glm::vec3 tangent{1.0f, 0.0f, 0.0f};
    glm::vec3 right{0.0f, 1.0f, 0.0f};
    int kmh = 0; // the limit the sign announces; unused by a marker, which carries no numeral
    SpeedSignKind kind = SpeedSignKind::Increase;
    // An Increase carrying the opposing direction's reduction marker on its back plate. The
    // marker is then not a sign in its own right - it is the other face of this one.
    bool backMarker = false;
};

// --- How far before the drop its warning stands ---
// Enough room to lose the speed, rather than a fixed distance: a small drop needs less than a
// large one. Real signage then uses a handful of standard distances rather than whatever the
// arithmetic says, so the need is snapped up to one of them. All three knobs live together
// because they are the guess, and retuning it should be one edit.
inline constexpr float kSightingM = 150.0f;   // seeing the sign and acting on it
inline constexpr float kServiceDecel = 0.35f; // m/s^2; service braking, conservative
inline constexpr float kWarnBands[] = {300.0f, 500.0f, 800.0f, 1000.0f, 1200.0f};
float warningDistance(int fromKmh, int toKmh);

// Every speed sign over every path, in both directions: increases, reduction markers and
// their warnings. Where a marker and an opposing increase fall at the same point they are
// folded onto one post, the marker becoming that sign's back face.
//
// The first stretch of a path is never a change: nothing preceded it to rise from or drop to,
// and a path boundary is an artefact of how the import was chained rather than a place a
// driver would expect a sign.
// Only signs on the paths within `radius` of `centre` (scene-relative); radius <= 0 =
// all. The limits themselves are a property of the track and are resolved wherever they
// are asked for - this bounds the posts, not the rules.
std::vector<SpeedSign> speedSigns(const std::vector<TrackPath>& paths,
                                  const glm::vec3& centre, float radius);
