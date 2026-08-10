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

#include "CrossingMesh.h"

#include "LampGeometry.h"
#include "TrackPath.h"

#include <algorithm>
#include <cmath>

namespace {

const glm::vec3 kBody{0.10f, 0.10f, 0.11f}; // near-black cast housing
const glm::vec3 kRedOn{1.0f, 0.14f, 0.10f};
const glm::vec3 kRedOff{0.24f, 0.10f, 0.10f};
const glm::vec3 kWhiteOn{1.0f, 0.97f, 0.88f};
const glm::vec3 kWhiteOff{0.26f, 0.25f, 0.23f};

constexpr float kMastH = 3.2f;      // shorter than a main signal: this is not one
constexpr float kRoadOffsetM = 6.0f; // how far the road heads stand off the track
constexpr float kRoadRightM = 3.0f;  // and to the right of the road traffic they face
constexpr float kLensR = 0.15f;
constexpr float kLensSp = 0.26f;    // half the vertical lamp spacing

// One head: a mast, a housing, and a red over a white. `fwd` is the direction the head
// looks, so a driver coming the other way sees it face-on.
void head(std::vector<TrackVertex>& v, std::vector<std::uint32_t>& idx,
          const glm::vec3& base, const glm::vec3& fwd, bool red, bool white, float period,
          float phase) {
    const glm::vec3 UP(0.0f, 0.0f, 1.0f);
    const glm::vec3 F = glm::normalize(glm::vec3(fwd.x, fwd.y, 0.0f));
    const glm::vec3 R(F.y, -F.x, 0.0f);

    lampgeom::box(v, idx, base + UP * (kMastH * 0.5f), R, F, UP, 0.06f, 0.06f,
                  kMastH * 0.5f, kBody);
    const float hw = 0.28f, hd = 0.14f, hh = 0.50f;
    const glm::vec3 C = base + UP * (kMastH + hh);
    lampgeom::box(v, idx, C, R, F, UP, hw, hd, hh, kBody);
    // Backing plate, so the head reads against a bright sky as a signal does.
    lampgeom::box(v, idx, C + F * (hd + 0.01f), R, F, UP, hw * 1.3f, 0.02f, hh * 1.1f,
                  kBody);

    const glm::vec3 face = C + F * (hd + 0.04f);
    if (red)
        lampgeom::lamp(v, idx, face + UP * kLensSp, R, UP, kLensR, kRedOn, period, phase);
    else
        lampgeom::disc(v, idx, face + UP * kLensSp, F, R, UP, kLensR, kRedOff);
    if (white)
        lampgeom::lamp(v, idx, face - UP * kLensSp, R, UP, kLensR, kWhiteOn, period, phase);
    else
        lampgeom::disc(v, idx, face - UP * kLensSp, F, R, UP, kLensR, kWhiteOff);
}

} // namespace

float crossingPhase(int id) {
    // The same shape of deterministic hash the tunnel wobble uses: stable across runs,
    // and spread enough that neighbouring crossings do not land together.
    std::uint32_t h = static_cast<std::uint32_t>(id) * 0x9E3779B9u;
    h ^= h >> 15;
    h *= 0x2545F491u;
    h ^= h >> 13;
    // Over the slow period, which is the longer of the two - a phase inside it is inside
    // the fast one as well, since the fast period divides it.
    return static_cast<float>(h & 0xFFFFu) / 65535.0f * kCrossingSlowS;
}

void CrossingMesh::build(const std::vector<LevelCrossing>& xs,
                         const std::vector<CrossingSite>& sites,
                         const std::vector<CrossingState>& states,
                         const std::vector<TrackPath>& paths, const glm::dvec3& origin) {
    vertices_.clear();
    indices_.clear();
    (void)origin; // paths are already scene-relative

    for (std::size_t i = 0; i < xs.size(); ++i) {
        if (i >= sites.size() || sites[i].path < 0) continue;
        if (sites[i].path >= static_cast<int>(paths.size())) continue;
        const TrackPath& p = paths[sites[i].path];
        const CrossingPhase ph =
            i < states.size() ? states[i].phase : CrossingPhase::Idle;
        const CrossingLights l = crossingLights(ph);
        const float period = l.fast ? kCrossingFastS : kCrossingSlowS;
        const float phase = crossingPhase(xs[i].id);

        const TrackPose at = p.poseAt(sites[i].s);
        // Across the track, level: `right` is banked by the cant and would tilt the masts.
        const glm::vec3 tan = glm::normalize(glm::vec3(at.tangent.x, at.tangent.y, 0.0f));
        const glm::vec3 across(tan.y, -tan.x, 0.0f);

        // The two heads the train reads. The one before the crossing on the -s side
        // serves trains running in +s, so it has to look back down the line at them -
        // a head that looked the way the train is going would only ever show its back.
        for (const float side : {-1.0f, 1.0f}) {
            const float s = sites[i].s + side * static_cast<float>(kSignalOffsetM);
            const TrackPose q = p.poseAt(std::clamp(s, 0.0f, p.length()));
            const glm::vec3 qt =
                glm::normalize(glm::vec3(q.tangent.x, q.tangent.y, 0.0f));
            const glm::vec3 qa(qt.y, -qt.x, 0.0f); // right of +s
            // Standing to the right of the direction of travel it governs, as a signal
            // does: that is -qa for a train running in -s.
            const glm::vec3 base = q.pos + qa * (side > 0.0f ? -3.2f : 3.2f);
            head(vertices_, indices_, base, qt * side, l.trainRed, l.trainWhite, period,
                 phase);
        }

        // The two the road reads. The head on the +across side looks back along +across
        // at the traffic coming that way, and stands to the right of it: for traffic
        // running in -across, right is (-across.y, across.x), which is the track's own
        // +s direction. So the offset follows `side` just as the standoff does.
        for (const float side : {-1.0f, 1.0f}) {
            const glm::vec3 base = at.pos + across * (side * kRoadOffsetM) +
                                   tan * (side * kRoadRightM);
            head(vertices_, indices_, base, across * side, l.roadRed, l.roadWhite, period,
                 phase);
        }
    }
}
