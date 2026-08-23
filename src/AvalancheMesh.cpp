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

#include "AvalancheMesh.h"

#include "LampGeometry.h"
#include "SignalMesh.h"    // kAvalancheBlinkS
#include "TrackCircuits.h" // TrackPoly, fracToWorld, trackTangent

namespace {

// The same near-black cast housing every signal on this railway is made of, and the same
// red. The white is the cold one a level crossing uses rather than the warm incandescent
// of a signal lamp: this is a modern installation and reads as one.
const glm::vec3 kBody{0.10f, 0.10f, 0.11f};
const glm::vec3 kRedOn{1.00f, 0.14f, 0.10f};
const glm::vec3 kRedOff{0.24f, 0.10f, 0.10f};
const glm::vec3 kWhiteOn{1.00f, 0.97f, 0.88f};
const glm::vec3 kWhiteOff{0.26f, 0.25f, 0.23f};

constexpr float kStandoffM = 3.5f;  // m from the track centre to the mast
constexpr float kMastH = 4.5f;      // m to the underside of the head
constexpr float kHeadHalfW = 0.30f; // housing half-width
constexpr float kHeadHalfD = 0.15f; // ...half-depth
constexpr float kHeadHalfH = 0.72f; // ...half-height, three lenses' worth
constexpr float kLensR = 0.13f;     // lens radius
constexpr float kLensSp = 0.40f;    // vertical spacing between lens centres

} // namespace

void AvalancheMesh::build(const std::vector<AvalancheSignal>& signals,
                          const std::vector<AvalancheAspect>& shown,
                          const std::vector<TrackPoly>& polys, const glm::dvec3& origin) {
    vertices_.clear();
    indices_.clear();

    const glm::vec3 UP(0.0f, 0.0f, 1.0f);
    for (std::size_t i = 0; i < signals.size(); ++i) {
        const AvalancheSignal& s = signals[i];
        const glm::dvec3 w = fracToWorld(polys, s.trackId, s.frac);
        if (w.x == 0.0 && w.y == 0.0) continue; // stale/missing track

        const glm::dvec2 t2 = trackTangent(polys, s.trackId, s.frac, 1);
        const glm::vec3 tan = glm::normalize(
            glm::vec3(static_cast<float>(t2.x), static_cast<float>(t2.y), 0.0f));
        // Two frames, kept apart on purpose. `out` is which side of the track the post
        // stands on and is measured off the raw tangent, so turning the head round does
        // not walk the post across the line. `F` is the direction of travel the head is
        // read from, and the lenses go on its back face, toward the driver coming.
        const glm::vec3 right(tan.y, -tan.x, 0.0f); // right of increasing frac
        const glm::vec3 out = right * static_cast<float>(s.side);
        const glm::vec3 F = tan * static_cast<float>(s.dir);
        const glm::vec3 R(F.y, -F.x, 0.0f);

        const glm::vec3 B(static_cast<float>(w.x - origin.x) + out.x * kStandoffM,
                          static_cast<float>(w.y - origin.y) + out.y * kStandoffM,
                          static_cast<float>(w.z - origin.z));

        lampgeom::box(vertices_, indices_, B + UP * (kMastH * 0.5f), R, F, UP, 0.07f,
                      0.07f, kMastH * 0.5f, kBody);
        const glm::vec3 C = B + UP * (kMastH + kHeadHalfH);
        lampgeom::box(vertices_, indices_, C, R, F, UP, kHeadHalfW, kHeadHalfD,
                      kHeadHalfH, kBody);
        // Backing plate, so the head reads against the sky from a distance.
        lampgeom::box(vertices_, indices_, C - F * (kHeadHalfD + 0.01f), R, F, UP,
                      kHeadHalfW * 1.25f, 0.02f, kHeadHalfH * 1.1f, kBody);

        const glm::vec3 n = -F;
        const glm::vec3 face = C - F * (kHeadHalfD + 0.04f);
        const AvalancheAspect a =
            i < shown.size() ? shown[i] : AvalancheAspect::Clear;
        const bool warn = a == AvalancheAspect::Warning;

        // Only a lit lens is emissive; a dark one is ordinary shaded geometry, so it
        // fades out with the head instead of blooming into a dark blob at range.
        //
        // The two reds take the same phase, so they flash together as one emphatic red
        // rather than as a wig-wag. Passing the phase explicitly rather than defaulting
        // it is the point: alternating them later - the level-crossing look - is giving
        // one of the two `kAvalancheBlinkS * 0.5f` here and changing nothing else.
        const float period = kAvalancheBlinkS;
        if (warn)
            lampgeom::lamp(vertices_, indices_, face + UP * kLensSp, R, UP, kLensR,
                           kRedOn, period, 0.0f);
        else
            lampgeom::disc(vertices_, indices_, face + UP * kLensSp, n, R, UP, kLensR,
                           kRedOff);

        if (!warn)
            lampgeom::lamp(vertices_, indices_, face, R, UP, kLensR, kWhiteOn, period,
                           0.0f);
        else
            lampgeom::disc(vertices_, indices_, face, n, R, UP, kLensR, kWhiteOff);

        if (warn)
            lampgeom::lamp(vertices_, indices_, face - UP * kLensSp, R, UP, kLensR,
                           kRedOn, period, 0.0f);
        else
            lampgeom::disc(vertices_, indices_, face - UP * kLensSp, n, R, UP, kLensR,
                           kRedOff);
    }
}
