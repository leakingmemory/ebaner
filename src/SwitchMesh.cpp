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

#include "SwitchNetwork.h"

#include <cmath>
#include <cstdio>

namespace {
// Cast-iron dark grey for the mechanism, timber brown for the base, and a white
// target with a dark frame/painted symbol.
const glm::vec3 kIron{0.22f, 0.23f, 0.25f};
const glm::vec3 kTimber{0.35f, 0.26f, 0.17f};
const glm::vec3 kTarget{0.90f, 0.90f, 0.88f};
const glm::vec3 kDark{0.12f, 0.12f, 0.13f};
} // namespace

void SwitchMesh::build(const SwitchNetwork& net, const glm::vec3& centre,
                       float radius) {
    vertices_.clear();
    indices_.clear();
    const glm::dvec3 origin = net.sceneOrigin();

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
        const float Ln = glm::length(axis);
        if (Ln < 1e-6f) return;
        axis /= Ln;
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
            tri(p1 + d0 * rad, c0 + axis * (Ln * 0.5f), p1 + d1 * rad, axis, col);
            tri(p0 + d1 * rad, c0 - axis * (Ln * 0.5f), p0 + d0 * rad, -axis, col);
        }
    };
    // A filled disc (the "filled circle" target face), facing +n, in the (w,up) plane.
    auto disc = [&](const glm::vec3& c, const glm::vec3& n, const glm::vec3& w,
                    const glm::vec3& up, float rad, const glm::vec3& col) {
        constexpr int N = 12;
        for (int i = 0; i < N; ++i) {
            const float a0 = 6.2831853f * i / N, a1 = 6.2831853f * (i + 1) / N;
            const glm::vec3 p0 = c + (w * std::cos(a0) + up * std::sin(a0)) * rad;
            const glm::vec3 p1 = c + (w * std::cos(a1) + up * std::sin(a1)) * rad;
            tri(c, p0, p1, n, col);
        }
    };
    // A filled arrowhead (the diverging target face), facing +n, pointing along +w.
    auto arrow = [&](const glm::vec3& c, const glm::vec3& n, const glm::vec3& w,
                     const glm::vec3& up, float sz, const glm::vec3& col) {
        tri(c + w * sz, c - w * sz * 0.5f + up * sz, c - w * sz * 0.5f - up * sz, n, col);
    };

    const glm::vec3 UP(0, 0, 1);
    const auto& turnouts = net.turnouts();
    int stands = 0;
    for (std::size_t idx = 0; idx < turnouts.size(); ++idx) {
        const Turnout& t = turnouts[idx];
        if (t.mainPath < 0) continue; // inert turnout (a crossing) — no working switch
        if (radius > 0.0f &&
            std::hypot(static_cast<float>(t.world.x - origin.x) - centre.x,
                       static_cast<float>(t.world.y - origin.y) - centre.y) > radius)
            continue;
        ++stands;
        const SwitchState st = net.state(static_cast<int>(idx));
        const SwitchType ty = net.type(static_cast<int>(idx));

        const glm::vec3 T(static_cast<float>(t.thru.x), static_cast<float>(t.thru.y), 0.0f);
        // Side unit S: horizontal perpendicular to the track, toward the diverging
        // route, so the stand sits on the side the points swing to.
        glm::vec3 S(-T.y, T.x, 0.0f);
        if (t.div.x * S.x + t.div.y * S.y < 0.0f) S = -S;
        // Frog direction along the main (toward where the branch heads); toe = -frog.
        const glm::vec3 frog =
            T * ((t.div.x * T.x + t.div.y * T.y) >= 0.0f ? 1.0f : -1.0f);
        const glm::vec3 C(static_cast<float>(t.world.x - origin.x) + S.x * 2.6f,
                          static_cast<float>(t.world.y - origin.y) + S.y * 2.6f,
                          static_cast<float>(t.world.z - origin.z) - 0.25f);
        auto L = [&](float lr, float lf, float lu) { return C + S * lr + T * lf + UP * lu; };

        // Static parts shared by both types: timber base, throw rod, indicator post.
        box(L(0, 0, 0.09f), S, T, UP, 0.55f, 0.5f, 0.09f, kTimber);     // base
        box(L(-1.05f, 0, 0.22f), S, T, UP, 0.95f, 0.03f, 0.03f, kIron); // throw rod to points
        const glm::vec3 postC = L(0.0f, 0.32f, 1.1f);
        box(postC, S, T, UP, 0.05f, 0.05f, 0.95f, kIron);              // indicator post

        if (ty == SwitchType::Manual) {
            // Hand stand: a pivot pedestal and a weighted lever, out at right angles to
            // the track (+S), near horizontal, swung fore/aft along the track to show
            // the state (centred and drooping when broken).
            box(L(0, 0, 0.5f), S, T, UP, 0.13f, 0.13f, 0.34f, kIron); // pivot pedestal
            const float yaw = st == SwitchState::Straight    ? 0.32f
                              : st == SwitchState::Diverging  ? -0.32f
                                                              : 0.0f;      // radians about UP
            const float ang = st == SwitchState::Broken ? 0.30f : 0.17f;   // droop when broken
            const glm::vec3 leverHoriz = glm::normalize(S * std::cos(yaw) + T * std::sin(yaw));
            const glm::vec3 leverDir =
                glm::normalize(leverHoriz * std::cos(ang) - UP * std::sin(ang));
            const glm::vec3 pivotTop = L(0, 0, 0.84f);
            const glm::vec3 le1 = glm::normalize(glm::cross(leverDir, UP));
            const glm::vec3 le2 = glm::cross(leverDir, le1);
            const float leverLen = 1.0f;
            box(pivotTop + leverDir * (leverLen * 0.5f), leverDir, le1, le2,
                leverLen * 0.5f, 0.05f, 0.05f, kIron);                 // lever bar
            const glm::vec3 wc = pivotTop + leverDir * leverLen;
            prism(wc - le1 * 0.18f, wc + le1 * 0.18f, 0.18f, 8, kIron); // counterweight
        } else {
            // Motor (point machine): a low housing beside the points, no hand-lever or
            // counterweight - it is thrown by the mechanism, not by hand.
            box(L(0.0f, 0.0f, 0.28f), S, T, UP, 0.30f, 0.55f, 0.28f, kIron); // housing
            box(L(0.0f, 0.0f, 0.57f), S, T, UP, 0.31f, 0.40f, 0.03f, kDark); // lid
        }

        // --- Mechanical target plate ---
        // Thin along its normal `n`, broad faces ±n. Straight: n = S (edge-on to the
        // track -> both approaches see the vertical-line edge). Diverging: n = toe dir
        // (broad faces across the track -> the toe/facing side sees the arrow, the
        // frog/branch side the circle). Broken: n at ~45deg (ambiguous).
        const glm::vec3 tgtC = L(0.0f, 0.32f, 2.15f);
        glm::vec3 n = S;
        if (st == SwitchState::Diverging) n = -frog;                    // toe faces the plate
        else if (st == SwitchState::Broken) n = glm::normalize(S - frog);
        const glm::vec3 w = glm::normalize(glm::cross(UP, n));          // plate width axis
        box(tgtC, n, w, UP, 0.05f, 0.20f, 0.32f, kDark);               // frame
        box(tgtC, n, w, UP, 0.06f, 0.17f, 0.28f, kTarget);            // white face
        if (st == SwitchState::Straight) {
            // Vertical line painted on both broad faces.
            box(tgtC, n, w, UP, 0.07f, 0.03f, 0.24f, kDark);
        } else if (st == SwitchState::Diverging) {
            // Arrow toward the toe (+n), filled circle toward the branches (-n); the
            // arrow points to the side the diverging route leaves (toward S).
            const float dir = glm::dot(w, S) >= 0.0f ? 1.0f : -1.0f;
            arrow(tgtC + n * 0.075f, n, w * dir, UP, 0.14f, kDark);
            disc(tgtC - n * 0.075f, -n, w, UP, 0.13f, kDark);
        }
    }

    std::printf("[SwitchMesh] %d stands, %zu vertices, %zu triangles\n",
                stands, vertices_.size(), indices_.size() / 3);
}
