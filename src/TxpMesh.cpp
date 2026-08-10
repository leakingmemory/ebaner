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

#include "TxpMesh.h"

#include "LampGeometry.h"
#include "TrackCircuits.h" // TrackPoly, fracToWorld, trackTangent

#include <cmath>

namespace {

const glm::vec3 kCoat{0.13f, 0.16f, 0.26f};  // dark uniform coat
const glm::vec3 kTrousers{0.11f, 0.12f, 0.15f};
const glm::vec3 kSkin{0.76f, 0.60f, 0.48f};
const glm::vec3 kCap{0.62f, 0.09f, 0.09f};   // the red cap, which is what reads at range
const glm::vec3 kSignFace{0.95f, 0.95f, 0.93f};
const glm::vec3 kSignGreen{0.06f, 0.50f, 0.20f};
const glm::vec3 kHandle{0.35f, 0.28f, 0.20f};

constexpr float kStandoffM = 3.2f; // from the centreline: clear of the loading gauge

} // namespace

void TxpMesh::build(const std::vector<TxpPosition>& ps, const std::vector<char>& showing,
                    const std::vector<TrackPoly>& polys, const glm::dvec3& origin) {
    vertices_.clear();
    indices_.clear();

    const glm::vec3 UP(0.0f, 0.0f, 1.0f);
    for (std::size_t i = 0; i < ps.size(); ++i) {
        if (i < showing.size() && !showing[i]) continue; // not signalling: nobody there
        const TxpPosition& p = ps[i];
        const glm::dvec3 w = fracToWorld(polys, p.trackId, p.frac);
        if (w.x == 0.0 && w.y == 0.0) continue; // stale/missing track

        const glm::dvec2 t2 = trackTangent(polys, p.trackId, p.frac, 1);
        const glm::vec3 tan(static_cast<float>(t2.x), static_cast<float>(t2.y), 0.0f);
        const glm::vec3 along = glm::normalize(tan);
        const glm::vec3 right(along.y, -along.x, 0.0f); // right of increasing frac

        // Where they stand: `side` is absolute, so turning the signal round below does
        // not walk them across the track.
        const glm::vec3 out = right * static_cast<float>(p.side);
        const glm::vec3 foot(static_cast<float>(w.x - origin.x) + out.x * kStandoffM,
                             static_cast<float>(w.y - origin.y) + out.y * kStandoffM,
                             static_cast<float>(w.z - origin.z));

        // Which way they look: the train departs along `dir` and its driver is looking
        // that way, so the TXP has to face back against it to be seen at all.
        const glm::vec3 face = along * static_cast<float>(-p.dir);
        const glm::vec3 sideways(face.y, -face.x, 0.0f); // the figure's own right

        auto box = [&](const glm::vec3& c, float hr, float hf, float hu,
                       const glm::vec3& col) {
            lampgeom::box(vertices_, indices_, c, sideways, face, UP, hr, hf, hu, col);
        };

        // Legs, coat, head, cap - about 1.75 m to the top of the cap.
        box(foot + UP * 0.40f, 0.16f, 0.11f, 0.40f, kTrousers);
        box(foot + UP * 1.10f, 0.22f, 0.14f, 0.30f, kCoat);
        box(foot + UP * 1.52f, 0.10f, 0.10f, 0.12f, kSkin);
        box(foot + UP * 1.68f, 0.12f, 0.12f, 0.04f, kCap);

        // The arm, raised on the figure's right, and the sign held above it.
        const glm::vec3 shoulder = foot + UP * 1.32f + sideways * 0.26f;
        box(shoulder + UP * 0.22f, 0.07f, 0.07f, 0.26f, kCoat);
        const glm::vec3 grip = shoulder + UP * 0.50f;
        box(grip + UP * 0.10f, 0.025f, 0.025f, 0.10f, kHandle);

        // A white paddle facing the driver, with a green circle on that face. The back
        // stays plain: it is the side the driver sees that carries the meaning.
        const glm::vec3 signC = grip + UP * 0.34f;
        box(signC, 0.22f, 0.012f, 0.22f, kSignFace);
        lampgeom::disc(vertices_, indices_, signC + face * 0.016f, face, sideways, UP,
                       0.13f, kSignGreen);
    }
}
