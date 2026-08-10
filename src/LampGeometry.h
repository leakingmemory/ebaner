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

#include "SignalMesh.h" // blink constants
#include "TrackMesh.h"  // TrackVertex

#include <cmath>
#include <cstdint>
#include <vector>

// The shapes a lit signal is made of: housings, lenses and the tagging that makes a lens
// emissive and gives it its blink.
//
// Shared rather than copied because `lamp` carries a convention the shaders depend on -
// the sentinel in texLayer, the lamp centre smuggled through the normal, and the period
// and phase riding in the otherwise unused uv - and two independently maintained copies
// of that would drift the first time any of it changed.
namespace lampgeom {

// A triangle with an explicit normal; nothing here is smooth-shaded.
inline void tri(std::vector<TrackVertex>& v, std::vector<std::uint32_t>& idx,
                const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                const glm::vec3& n, const glm::vec3& col) {
    const std::uint32_t base = static_cast<std::uint32_t>(v.size());
    v.push_back({a, n, col, {0.0f, 0.0f}, -1.0f});
    v.push_back({b, n, col, {0.0f, 0.0f}, -1.0f});
    v.push_back({c, n, col, {0.0f, 0.0f}, -1.0f});
    idx.push_back(base + 0);
    idx.push_back(base + 1);
    idx.push_back(base + 2);
}

inline void quad(std::vector<TrackVertex>& v, std::vector<std::uint32_t>& idx,
                 const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                 const glm::vec3& p3, const glm::vec3& n, const glm::vec3& col) {
    tri(v, idx, p0, p1, p2, n, col);
    tri(v, idx, p0, p2, p3, n, col);
}

// Box with orthonormal local axes r/f/u and half-extents hr/hf/hu about centre c.
inline void box(std::vector<TrackVertex>& v, std::vector<std::uint32_t>& idx,
                const glm::vec3& c, const glm::vec3& r, const glm::vec3& f,
                const glm::vec3& u, float hr, float hf, float hu, const glm::vec3& col) {
    const glm::vec3 R = r * hr, F = f * hf, U = u * hu;
    auto P = [&](float sr, float sf, float su) { return c + R * sr + F * sf + U * su; };
    quad(v, idx, P(1, -1, -1), P(1, 1, -1), P(1, 1, 1), P(1, -1, 1), r, col);
    quad(v, idx, P(-1, 1, -1), P(-1, -1, -1), P(-1, -1, 1), P(-1, 1, 1), -r, col);
    quad(v, idx, P(1, 1, -1), P(-1, 1, -1), P(-1, 1, 1), P(1, 1, 1), f, col);
    quad(v, idx, P(-1, -1, -1), P(1, -1, -1), P(1, -1, 1), P(-1, -1, 1), -f, col);
    quad(v, idx, P(-1, -1, 1), P(1, -1, 1), P(1, 1, 1), P(-1, 1, 1), u, col);
    quad(v, idx, P(-1, 1, -1), P(1, 1, -1), P(1, -1, -1), P(-1, -1, -1), -u, col);
}

// A filled disc facing +n, in the (w,up) plane - an unlit lens.
inline void disc(std::vector<TrackVertex>& v, std::vector<std::uint32_t>& idx,
                 const glm::vec3& c, const glm::vec3& n, const glm::vec3& w,
                 const glm::vec3& up, float rad, const glm::vec3& col) {
    constexpr int N = 12;
    for (int i = 0; i < N; ++i) {
        const float a0 = 6.2831853f * i / N, a1 = 6.2831853f * (i + 1) / N;
        tri(v, idx, c, c + (w * std::cos(a0) + up * std::sin(a0)) * rad,
            c + (w * std::cos(a1) + up * std::sin(a1)) * rad, n, col);
    }
}

// A lit lens. The same disc, tagged for the shader three ways:
//  - texLayer -3 steady / -4 blinking, which is what makes it emissive at all;
//  - the lamp centre in the normal slot, unused once the sun term is skipped, so the
//    vertex shader can hold the lens to a minimum apparent size and keep it readable far
//    down the line;
//  - `period` and `phase` in seconds in the uv slot, which the fragment shader is free to
//    use because it only ever samples uv for the ballast top. That is what lets each lamp
//    blink on its own clock - two crossings out of step, or a fast lamp beside a slow one
//    - without a wider vertex or a second lane in the push constant.
// period 0 means steady.
inline void lamp(std::vector<TrackVertex>& v, std::vector<std::uint32_t>& idx,
                 const glm::vec3& c, const glm::vec3& w, const glm::vec3& up, float rad,
                 const glm::vec3& col, float period = 0.0f, float phase = 0.0f) {
    const float layer = period > 0.0f ? -4.0f : -3.0f;
    const glm::vec2 uv(period, phase);
    constexpr int N = 12;
    for (int i = 0; i < N; ++i) {
        const float a0 = 6.2831853f * i / N, a1 = 6.2831853f * (i + 1) / N;
        const glm::vec3 p0 = c + (w * std::cos(a0) + up * std::sin(a0)) * rad;
        const glm::vec3 p1 = c + (w * std::cos(a1) + up * std::sin(a1)) * rad;
        const std::uint32_t base = static_cast<std::uint32_t>(v.size());
        v.push_back({c, c, col, uv, layer});
        v.push_back({p0, c, col, uv, layer});
        v.push_back({p1, c, col, uv, layer});
        idx.push_back(base + 0);
        idx.push_back(base + 1);
        idx.push_back(base + 2);
    }
}

} // namespace lampgeom
