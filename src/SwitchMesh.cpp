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

#include "SwitchMesh.h"

#include "TerrainData.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_set>

namespace {
// Cast-iron dark grey for the mechanism, timber brown for the base, and a white
// target with a dark frame/painted line.
const glm::vec3 kIron{0.22f, 0.23f, 0.25f};
const glm::vec3 kTimber{0.35f, 0.26f, 0.17f};
const glm::vec3 kTarget{0.90f, 0.90f, 0.88f};
const glm::vec3 kDark{0.12f, 0.12f, 0.13f};

// 2-D distance from p to segment a-b, plus the clamped parameter t along a->b.
struct SegHit { float dist; float t; };
SegHit distToSeg(const glm::dvec2& p, const glm::dvec2& a, const glm::dvec2& b) {
    const glm::dvec2 ab = b - a;
    const double l2 = glm::dot(ab, ab);
    const double t = l2 > 1e-9 ? glm::clamp(glm::dot(p - a, ab) / l2, 0.0, 1.0) : 0.0;
    return {static_cast<float>(glm::length(p - (a + ab * t))), static_cast<float>(t)};
}

// A detected turnout: world point on the through track, plus the (horizontal, unit)
// through direction and the diverging track's leaving direction.
struct Turnout {
    glm::dvec3 x;   // world point on the crossed track (z = rail head there)
    glm::dvec2 thru; // along the crossed (through) track
    glm::dvec2 div;  // the diverging track's interior direction from its end
};
} // namespace

void SwitchMesh::build(const TerrainData& data) {
    const glm::dvec3 origin = data.sceneOrigin();

    // Unique track polylines (a segment repeats across every tile it touches).
    std::unordered_set<std::uint32_t> seen;
    std::vector<const TrackSegment*> segs;
    for (const Tile& t : data.tiles())
        for (const TrackSegment& s : t.tracks)
            if (s.pts.size() >= 2 && seen.insert(s.trackId).second) segs.push_back(&s);

    // A turnout is where one track's endpoint lies on a *different* track's line.
    std::vector<Turnout> turnouts;
    for (const TrackSegment* sp : segs) {
        const std::vector<glm::dvec3>& p = sp->pts;
        const std::pair<glm::dvec3, glm::dvec3> endsWithNb[2] = {
            {p.front(), p[1]}, {p.back(), p[p.size() - 2]}};
        for (const auto& [end, nb] : endsWithNb) {
            const glm::dvec2 e2(end.x, end.y);
            float best = 2.0f;    // metres; the touch tolerance
            glm::dvec3 hitX{0.0};
            glm::dvec2 hitThru{0.0};
            const TrackSegment* hitSeg = nullptr;
            for (const TrackSegment* op : segs) {
                if (op == sp) continue;
                const std::vector<glm::dvec3>& q = op->pts;
                for (std::size_t k = 0; k + 1 < q.size(); ++k) {
                    const glm::dvec2 a(q[k].x, q[k].y), b(q[k + 1].x, q[k + 1].y);
                    const SegHit h = distToSeg(e2, a, b);
                    if (h.dist < best) {
                        best = h.dist;
                        const double z = q[k].z + (q[k + 1].z - q[k].z) * h.t;
                        hitX = glm::dvec3(e2.x, e2.y, z);
                        hitThru = b - a;
                        hitSeg = op;
                    }
                }
            }
            if (!hitSeg) continue;
            const double tl = glm::length(hitThru);
            const glm::dvec2 dv(nb.x - end.x, nb.y - end.y);
            const double dl = glm::length(dv);
            if (tl < 1e-6 || dl < 1e-6) continue;
            const glm::dvec2 T = hitThru / tl, D = dv / dl;

            // Reject a straight end-to-end stitch (two track ends joined collinearly,
            // e.g. a tunnel break rejoined into a continuous route) — that is not a
            // turnout. A real turnout has the through track passing *through* the
            // point (the touch lands well inside it) or, at a yard track's end, the
            // branch leaves at a clear angle. So keep the junction only if one holds.
            constexpr double kEndMargin = 10.0;   // m: through track continues past the point
            constexpr double kMinDivergeDeg = 8.0; // deg: a genuine branch at an end-to-end meet
            const double endDist =
                std::min(std::hypot(e2.x - hitSeg->pts.front().x, e2.y - hitSeg->pts.front().y),
                         std::hypot(e2.x - hitSeg->pts.back().x, e2.y - hitSeg->pts.back().y));
            const double angDeg =
                glm::degrees(std::acos(std::clamp(std::abs(T.x * D.x + T.y * D.y), 0.0, 1.0)));
            if (endDist < kEndMargin && angDeg < kMinDivergeDeg) continue;

            turnouts.push_back({hitX, T, D});
        }
    }

    // Collapse turnouts sharing a point (both tracks' ends meet there) to one stand.
    std::vector<Turnout> uniq;
    for (const Turnout& t : turnouts) {
        bool dup = false;
        for (const Turnout& u : uniq)
            if (std::hypot(t.x.x - u.x.x, t.x.y - u.x.y) < 3.0) { dup = true; break; }
        if (!dup) uniq.push_back(t);
    }

    // --- Geometry helpers (scene-relative floats) ---
    auto tri = [&](const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                   const glm::vec3& n, const glm::vec3& c) {
        const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
        vertices_.push_back({p0, n, c, {0.0f, 0.0f}, -1.0f});
        vertices_.push_back({p1, n, c, {0.0f, 0.0f}, -1.0f});
        vertices_.push_back({p2, n, c, {0.0f, 0.0f}, -1.0f});
        indices_.push_back(base + 0);
        indices_.push_back(base + 1);
        indices_.push_back(base + 2);
    };
    auto quad = [&](const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                    const glm::vec3& p3, const glm::vec3& n, const glm::vec3& c) {
        tri(p0, p1, p2, n, c);
        tri(p0, p2, p3, n, c);
    };
    // Box with orthonormal local axes r/f/u and half-extents hr/hf/hu about centre c.
    auto box = [&](const glm::vec3& c, const glm::vec3& r, const glm::vec3& f,
                   const glm::vec3& u, float hr, float hf, float hu,
                   const glm::vec3& col) {
        const glm::vec3 R = r * hr, F = f * hf, U = u * hu;
        auto P = [&](float sr, float sf, float su) { return c + R * sr + F * sf + U * su; };
        quad(P(1, -1, -1), P(1, 1, -1), P(1, 1, 1), P(1, -1, 1), r, col);   // +r
        quad(P(-1, 1, -1), P(-1, -1, -1), P(-1, -1, 1), P(-1, 1, 1), -r, col); // -r
        quad(P(1, 1, -1), P(-1, 1, -1), P(-1, 1, 1), P(1, 1, 1), f, col);   // +f
        quad(P(-1, -1, -1), P(1, -1, -1), P(1, -1, 1), P(-1, -1, 1), -f, col); // -f
        quad(P(-1, -1, 1), P(1, -1, 1), P(1, 1, 1), P(-1, 1, 1), u, col);   // top
        quad(P(-1, 1, -1), P(1, 1, -1), P(1, -1, -1), P(-1, -1, -1), -u, col); // bottom
    };
    // N-gon prism (a stand-in cylinder) from p0 to p1 with the given radius.
    auto prism = [&](const glm::vec3& p0, const glm::vec3& p1, float rad, int n,
                     const glm::vec3& col) {
        glm::vec3 axis = p1 - p0;
        const float L = glm::length(axis);
        if (L < 1e-6f) return;
        axis /= L;
        const glm::vec3 ref = std::abs(axis.z) < 0.9f ? glm::vec3(0, 0, 1)
                                                      : glm::vec3(1, 0, 0);
        const glm::vec3 e1 = glm::normalize(glm::cross(axis, ref));
        const glm::vec3 e2 = glm::cross(axis, e1);
        const glm::vec3 c0 = (p0 + p1) * 0.5f;
        for (int i = 0; i < n; ++i) {
            const float a0 = 6.2831853f * i / n, a1 = 6.2831853f * (i + 1) / n;
            const glm::vec3 d0 = e1 * std::cos(a0) + e2 * std::sin(a0);
            const glm::vec3 d1 = e1 * std::cos(a1) + e2 * std::sin(a1);
            quad(p0 + d0 * rad, p1 + d0 * rad, p1 + d1 * rad, p0 + d1 * rad,
                 glm::normalize(d0 + d1), col);
            tri(p1 + d0 * rad, c0 + axis * (L * 0.5f), p1 + d1 * rad, axis, col);
            tri(p0 + d1 * rad, c0 - axis * (L * 0.5f), p0 + d0 * rad, -axis, col);
        }
    };

    // --- One switch stand per turnout ---
    const glm::vec3 UP(0, 0, 1);
    for (const Turnout& t : uniq) {
        const glm::vec3 T(static_cast<float>(t.thru.x), static_cast<float>(t.thru.y), 0.0f);
        // Side unit S: horizontal perpendicular to the track, toward the diverging
        // route, so the stand sits on the side the points swing to.
        glm::vec3 S(-T.y, T.x, 0.0f);
        if (t.div.x * S.x + t.div.y * S.y < 0.0f) S = -S;
        // Local ground origin, ~0.25 m below the rail head, offset clear of the gauge.
        const glm::vec3 C(static_cast<float>(t.x.x - origin.x) + S.x * 2.6f,
                          static_cast<float>(t.x.y - origin.y) + S.y * 2.6f,
                          static_cast<float>(t.x.z - origin.z) - 0.25f);
        // worldPos(local right, fwd, up)
        auto L = [&](float lr, float lf, float lu) { return C + S * lr + T * lf + UP * lu; };

        // Timber base slab.
        box(L(0, 0, 0.09f), S, T, UP, 0.55f, 0.5f, 0.09f, kTimber);
        // Throw rod running toward the rails (-S) to the point blades.
        box(L(-1.05f, 0, 0.22f), S, T, UP, 0.95f, 0.03f, 0.03f, kIron);
        // Pivot pedestal.
        box(L(0, 0, 0.5f), S, T, UP, 0.13f, 0.13f, 0.34f, kIron);

        // Weighted lever: from the pedestal top, out at right angles to the track
        // (+S) and near horizontal (as it lies in either thrown position), so the
        // counterweight sits clear of the rails and in view.
        const float ang = 0.17f; // ~10 deg below horizontal
        const glm::vec3 leverDir = glm::normalize(S * std::cos(ang) - UP * std::sin(ang));
        const glm::vec3 pivotTop = L(0, 0, 0.84f);
        const glm::vec3 le1 = glm::normalize(glm::cross(leverDir, UP));
        const glm::vec3 le2 = glm::cross(leverDir, le1);
        const float leverLen = 1.0f;
        box(pivotTop + leverDir * (leverLen * 0.5f), leverDir, le1, le2,
            leverLen * 0.5f, 0.05f, 0.05f, kIron);
        // Cylindrical counterweight at the low end, drum axis across the lever.
        const glm::vec3 wc = pivotTop + leverDir * leverLen;
        prism(wc - le1 * 0.18f, wc + le1 * 0.18f, 0.18f, 8, kIron);

        // Signal post and rotating indicator target, offset a little along the track.
        const glm::vec3 postC = L(0.0f, 0.32f, 1.1f);
        box(postC, S, T, UP, 0.05f, 0.05f, 0.95f, kIron);
        // Target plate: thin along the track-normal S, its broad faces parallel to
        // the track, so a train sighting along the rails sees only the narrow edge ->
        // the vertical-line ("go straight") indication. White plate, dark frame +
        // painted vertical stripe (each layer proud of the last so it shows).
        const glm::vec3 tgtC = L(0.0f, 0.32f, 2.15f);
        box(tgtC, S, T, UP, 0.05f, 0.20f, 0.32f, kDark);        // frame
        box(tgtC, S, T, UP, 0.06f, 0.17f, 0.28f, kTarget);      // white face, proud
        box(tgtC, S, T, UP, 0.07f, 0.03f, 0.24f, kDark);        // painted vertical line
    }

    std::printf("[SwitchMesh] %zu stands, %zu vertices, %zu triangles\n",
                uniq.size(), vertices_.size(), indices_.size() / 3);
}
