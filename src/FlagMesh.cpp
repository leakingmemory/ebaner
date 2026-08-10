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

#include "FlagMesh.h"

#include "LampGeometry.h"
#include "TrackCircuits.h" // TrackPoly, fracToWorld, trackTangent

#include <cmath>

namespace {

// Matte, not the lens colours: those belong to lit lamps and cloth painted with them
// would read as glowing at dusk.
const glm::vec3 kIron{0.62f, 0.64f, 0.66f};  // galvanised post and fixture
const glm::vec3 kStick{0.55f, 0.44f, 0.30f}; // the wooden stick the flag is on
const glm::vec3 kClothRed{0.72f, 0.09f, 0.08f};
const glm::vec3 kClothGreen{0.06f, 0.42f, 0.18f};

constexpr float kPostH = 2.2f;      // to the fixture
constexpr float kStandoffM = 3.0f;  // from the centreline
constexpr float kStickLen = 0.8f;   // reaching back toward the track
constexpr float kClothW = 0.60f;    // across the hang
constexpr float kClothDrop = 0.50f; // how far it hangs

} // namespace

void FlagMesh::build(const std::vector<FlagPost>& posts,
                     const std::vector<FlagColour>& shown,
                     const std::vector<TrackPoly>& polys, const glm::dvec3& origin) {
    vertices_.clear();
    indices_.clear();

    const glm::vec3 UP(0.0f, 0.0f, 1.0f);
    for (std::size_t i = 0; i < posts.size(); ++i) {
        const FlagPost& p = posts[i];
        const glm::dvec3 w = fracToWorld(polys, p.trackId, p.frac);
        if (w.x == 0.0 && w.y == 0.0) continue; // stale/missing track

        const glm::dvec2 t2 = trackTangent(polys, p.trackId, p.frac, 1);
        const glm::vec3 tan =
            glm::normalize(glm::vec3(static_cast<float>(t2.x), static_cast<float>(t2.y), 0.0f));
        const glm::vec3 right(tan.y, -tan.x, 0.0f); // right of increasing frac
        const glm::vec3 out = right * static_cast<float>(p.side); // toward the post
        const glm::vec3 back = -out;                              // toward the track

        const glm::vec3 foot(static_cast<float>(w.x - origin.x) + out.x * kStandoffM,
                             static_cast<float>(w.y - origin.y) + out.y * kStandoffM,
                             static_cast<float>(w.z - origin.z));

        // The ironwork stands whether or not a flag is in it: an empty fixture is itself
        // an indication - the station is unmanned, or has nothing to say.
        lampgeom::box(vertices_, indices_, foot + UP * (kPostH * 0.5f), right, tan, UP,
                      0.05f, 0.05f, kPostH * 0.5f, kIron);
        // The fixture: a short socket at the top, projecting toward the track, which is
        // what the stick slots into.
        const glm::vec3 socket = foot + UP * kPostH + back * 0.10f;
        lampgeom::box(vertices_, indices_, socket, back, tan, UP, 0.10f, 0.07f, 0.07f,
                      kIron);

        const FlagColour col = i < shown.size() ? shown[i] : FlagColour::None;
        if (col == FlagColour::None) continue;

        // The stick, horizontal through the fixture and reaching back over the track, and
        // the cloth hanging from it.
        const glm::vec3 stickMid = foot + UP * kPostH + back * (kStickLen * 0.5f);
        lampgeom::box(vertices_, indices_, stickMid, back, tan, UP, kStickLen * 0.5f,
                      0.025f, 0.025f, kStick);

        const glm::vec3 clothMid = foot + UP * (kPostH - kClothDrop * 0.5f - 0.03f) +
                                   back * (kStickLen - kClothW * 0.5f);
        lampgeom::box(vertices_, indices_, clothMid, back, tan, UP, kClothW * 0.5f, 0.012f,
                      kClothDrop * 0.5f,
                      col == FlagColour::Red ? kClothRed : kClothGreen);
    }
}
