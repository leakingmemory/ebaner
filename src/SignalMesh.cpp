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

#include "SignalMesh.h"

#include <cmath>

namespace {
const glm::vec3 kBody{0.10f, 0.10f, 0.11f};   // near-black cast housing
const glm::vec3 kLampOn{1.0f, 0.90f, 0.62f};  // warm incandescent white
const glm::vec3 kLampOff{0.20f, 0.20f, 0.20f}; // dark, unlit lamp position
} // namespace

void SignalMesh::build(const std::vector<SignalPlacement>& signals, glm::dvec3 origin) {
    vertices_.clear();
    indices_.clear();

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
                   const glm::vec3& u, float hr, float hf, float hu, const glm::vec3& col) {
        const glm::vec3 R = r * hr, F = f * hf, U = u * hu;
        auto P = [&](float sr, float sf, float su) { return c + R * sr + F * sf + U * su; };
        quad(P(1, -1, -1), P(1, 1, -1), P(1, 1, 1), P(1, -1, 1), r, col);
        quad(P(-1, 1, -1), P(-1, -1, -1), P(-1, -1, 1), P(-1, 1, 1), -r, col);
        quad(P(1, 1, -1), P(-1, 1, -1), P(-1, 1, 1), P(1, 1, 1), f, col);
        quad(P(-1, -1, -1), P(1, -1, -1), P(1, -1, 1), P(-1, -1, 1), -f, col);
        quad(P(-1, -1, 1), P(1, -1, 1), P(1, 1, 1), P(-1, 1, 1), u, col);
        quad(P(-1, 1, -1), P(1, 1, -1), P(1, -1, -1), P(-1, -1, -1), -u, col);
    };
    // A filled disc facing +n, in the (w,up) plane - a lamp lens.
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

    const glm::vec3 UP(0.0f, 0.0f, 1.0f);
    for (const SignalPlacement& s : signals) {
        const glm::dvec2 f2 = glm::length(s.forward) > 1e-9 ? glm::normalize(s.forward)
                                                            : glm::dvec2(1.0, 0.0);
        const glm::vec3 F(static_cast<float>(f2.x), static_cast<float>(f2.y), 0.0f);
        const glm::vec3 R(F.y, -F.x, 0.0f); // right of the travel direction
        // Base: on the ground, offset to the right of the track; scene-relative.
        const glm::vec3 B(static_cast<float>(s.world.x - origin.x) + R.x * 3.5f,
                          static_cast<float>(s.world.y - origin.y) + R.y * 3.5f,
                          static_cast<float>(s.world.z - origin.z));

        // Short pole (~1 m) with the black housing on top - low to the ground, but the
        // lamp face clears sleepers/snow.
        const float poleH = 1.0f;
        box(B + UP * (poleH * 0.5f), R, F, UP, 0.035f, 0.035f, poleH * 0.5f, kBody);
        const float hw = 0.30f, hd = 0.16f, hh = 0.28f; // half extents (across/along/up)
        const glm::vec3 C = B + UP * (poleH + hh);
        box(C, R, F, UP, hw, hd, hh, kBody);

        // Lamp face toward the approaching driver (normal -F, just proud of the housing).
        const glm::vec3 n = -F;
        const glm::vec3 face = C - F * (hd + 0.02f);
        // Face-plane axes: `a` = the driver's rightward axis (their right is +R; offsets
        // are in the driver's own frame, so no mirror flip), `up` = vertical. The two Stop
        // lamps form a horizontal pair low on the face (reference left + Stop right); the
        // 45 deg and vertical arc positions are drawn as dark, unlit spots.
        const glm::vec3 a = R;
        const float d = 0.16f, r = 0.055f, lo = -0.10f;
        const glm::vec3 ref = face + a * (-d * 0.5f) + UP * lo;     // reference (lower-left)
        // The arc lamp that is lit picks out the aspect; the other two stay dark.
        const bool stop = s.aspect == SignalAspect::Stop;
        const bool train = s.aspect == SignalAspect::TrainOnTrack;
        const bool clear = s.aspect == SignalAspect::Clear;
        disc(ref, n, a, UP, r, kLampOn);                            // reference: always on
        disc(ref + a * d, n, a, UP, r, stop ? kLampOn : kLampOff);  // horizontal = stop
        disc(ref + (a + UP) * (d * 0.70711f), n, a, UP, r,          // 45 deg = train ahead
             train ? kLampOn : kLampOff);
        disc(ref + UP * d, n, a, UP, r, clear ? kLampOn : kLampOff); // vertical = clear
    }
}
