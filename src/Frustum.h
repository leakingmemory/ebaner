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

// The six planes of a view-projection, for asking whether a bounding sphere could be on
// screen. Lived in VulkanRenderer.cpp, where the terrain and building draws use it; the
// vehicle mesh wants the same question answered before it spends the time BUILDING
// geometry, not only before drawing it, so it is shared rather than copied.
struct Frustum {
    glm::vec4 plane[6];

    explicit Frustum(const glm::mat4& m) {
        auto row = [&](int i) { return glm::vec4(m[0][i], m[1][i], m[2][i], m[3][i]); };
        const glm::vec4 r3 = row(3);
        for (int i = 0; i < 3; ++i) {
            plane[2 * i] = r3 + row(i);
            plane[2 * i + 1] = r3 - row(i);
        }
        for (glm::vec4& q : plane) {
            const float n = glm::length(glm::vec3(q));
            if (n > 1e-9f) q /= n; // so the distance below is in metres
        }
    }

    // Conservative: a sphere is only rejected when it is wholly outside a plane, so
    // anything that might be on screen is kept. Cheap to be wrong the safe way.
    bool sees(const glm::vec3& c, float r) const {
        for (const glm::vec4& q : plane)
            if (glm::dot(glm::vec3(q), c) + q.w < -r) return false;
        return true;
    }
};
