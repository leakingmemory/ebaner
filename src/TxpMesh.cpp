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
#include "PlatformMesh.h"  // platformTopAt
#include "TerrainData.h"
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

// Where the TXP stands: on the track at `frac`, then out to their side by the standoff.
// Shared with txpStandLift so the platform is looked up at the spot the figure actually
// occupies - a few metres to the side, which is often the whole difference between the
// ballast and the platform. Returns false for a stale or missing track.
bool standSpot(const std::vector<TrackPoly>& polys, const TxpPosition& p,
               glm::dvec3& track, glm::dvec2& spot, glm::vec3& along) {
    track = fracToWorld(polys, p.trackId, p.frac);
    if (track.x == 0.0 && track.y == 0.0) return false;
    const glm::dvec2 t2 = trackTangent(polys, p.trackId, p.frac, 1);
    along = glm::normalize(glm::vec3(static_cast<float>(t2.x),
                                     static_cast<float>(t2.y), 0.0f));
    const glm::vec3 right(along.y, -along.x, 0.0f); // right of increasing frac
    const glm::vec3 out = right * static_cast<float>(p.side);
    spot = glm::dvec2(track.x + out.x * kStandoffM, track.y + out.y * kStandoffM);
    return true;
}

// The sign is round: a green ring with a white centre, the shape of a European
// prohibitory road sign with the red swapped for green.
constexpr float kSignR = 0.22f;          // outer edge of the ring
constexpr float kSignInnerR = 0.155f;    // where the white centre begins
constexpr float kSignHalfThick = 0.012f;
constexpr int kSignSegs = 20;

// Built here rather than from lampgeom::disc because the circle is what carries the
// meaning: it is worth more segments than a lamp lens needs, and the ring wants both
// radii stepped through the same angles so the band closes exactly. Two faces and a rim,
// so the sign does not vanish when it is seen edge-on.
void roundSign(std::vector<TrackVertex>& v, std::vector<std::uint32_t>& idx,
               const glm::vec3& c, const glm::vec3& face, const glm::vec3& w,
               const glm::vec3& up) {
    const glm::vec3 front = c + face * kSignHalfThick;
    const glm::vec3 back = c - face * kSignHalfThick;
    auto dir = [&](int i) {
        const float a = 6.2831853f * static_cast<float>(i) / kSignSegs;
        return w * std::cos(a) + up * std::sin(a);
    };
    for (int i = 0; i < kSignSegs; ++i) {
        const glm::vec3 d0 = dir(i), d1 = dir(i + 1);
        // The green ring, as a band between the two radii, and the white centre inside
        // it. They meet at kSignInnerR rather than overlapping, so neither z-fights.
        lampgeom::quad(v, idx, front + d0 * kSignInnerR, front + d0 * kSignR,
                       front + d1 * kSignR, front + d1 * kSignInnerR, face, kSignGreen);
        lampgeom::tri(v, idx, front, front + d0 * kSignInnerR, front + d1 * kSignInnerR,
                      face, kSignFace);
        // The back stays plain - it is the side the driver sees that carries the meaning.
        lampgeom::tri(v, idx, back, back + d0 * kSignR, back + d1 * kSignR, -face,
                      kSignFace);
        lampgeom::quad(v, idx, back + d0 * kSignR, front + d0 * kSignR,
                       front + d1 * kSignR, back + d1 * kSignR,
                       glm::normalize(d0 + d1), kSignGreen);
    }
}

} // namespace

std::vector<float> txpStandLift(const std::vector<TxpPosition>& ps,
                                const std::vector<TrackPoly>& polys,
                                const TerrainData& data,
                                const std::vector<TrackPath>& paths) {
    std::vector<float> lift(ps.size(), 0.0f);
    const glm::dvec3 origin = data.sceneOrigin();
    for (std::size_t i = 0; i < ps.size(); ++i) {
        glm::dvec3 track;
        glm::dvec2 spot;
        glm::vec3 along;
        if (!standSpot(polys, ps[i], track, spot, along)) continue;
        float top = 0.0f;
        if (!platformTopAt(data, paths, spot.x, spot.y, top)) continue;
        // Signed, not clamped upward. A platform between tracks takes its datum from the
        // lower one, so a spot beside a track on higher ground can sit below that track's
        // rail head - and leaving the TXP at rail level there floats them above the slab,
        // which is as wrong as burying them in it. Following the top down cannot bury
        // them either: the slab is itself never built under the terrain.
        lift[i] = top - static_cast<float>(track.z - origin.z);
    }
    return lift;
}

void TxpMesh::build(const std::vector<TxpPosition>& ps, const std::vector<char>& showing,
                    const std::vector<TrackPoly>& polys, const glm::dvec3& origin,
                    const std::vector<float>& lift) {
    vertices_.clear();
    indices_.clear();

    const glm::vec3 UP(0.0f, 0.0f, 1.0f);
    for (std::size_t i = 0; i < ps.size(); ++i) {
        if (i < showing.size() && !showing[i]) continue; // not signalling: nobody there
        const TxpPosition& p = ps[i];
        // `side` is absolute, so turning the signal round below does not walk them
        // across the track.
        glm::dvec3 w;
        glm::dvec2 spot;
        glm::vec3 along;
        if (!standSpot(polys, p, w, spot, along)) continue; // stale/missing track

        // On a platform they stand on the slab, not at rail level, or they are buried in
        // it to the waist.
        const float up = i < lift.size() ? lift[i] : 0.0f;
        const glm::vec3 foot(static_cast<float>(spot.x - origin.x),
                             static_cast<float>(spot.y - origin.y),
                             static_cast<float>(w.z - origin.z) + up);

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

        // The sign held up to the driver: a green ring round a white centre.
        const glm::vec3 signC = grip + UP * 0.34f;
        roundSign(vertices_, indices_, signC, face, sideways, UP);
    }
}
