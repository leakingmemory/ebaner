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

// Free-fly camera. World is right-handed with +z up (matches terrain coords).
class Camera {
public:
    // Places the camera at `pos` looking along horizontal `dir`.
    void init(const glm::vec3& pos, const glm::vec3& dir);

    // Directly sets position and orientation (radians). For scripted views.
    void setPose(const glm::vec3& pos, float yawRad, float pitchRad);

    // Advances position from held movement keys (dt in seconds).
    void move(float forward, float right, float up, float dt, bool fast);

    // Applies mouse-look deltas (pixels).
    void look(float dx, float dy);

    glm::mat4 viewMatrix() const;
    glm::mat4 projMatrix(float aspect) const;

    glm::vec3 position() const { return pos_; }
    glm::vec3 forward() const;

private:
    glm::vec3 pos_{0.0f};
    float yaw_ = 0.0f;    // radians, around +z; 0 = +x
    float pitch_ = 0.0f;  // radians, clamped to avoid gimbal flip

    float speed_ = 40.0f;       // metres/second (base)
    float mouseSens_ = 0.0022f; // radians per pixel
    float fovY_ = glm::radians(60.0f);
    float nearZ_ = 0.5f;
    float farZ_ = 60000.0f;
};

// Where the ray through `cursorPx` meets the horizontal plane at `planeZ`, in scene
// coordinates. The editor never casts rays - it picks by projecting world geometry to the
// screen and comparing pixel distance - but drawing a new track needs the other direction,
// and a ray meets a plane in exactly one point, which is what makes a click answerable at
// all once the height is fixed.
//
// `cursorPx` and `fb` are framebuffer pixels with y down, the convention the picks above
// use (proj[1][1] is flipped for Vulkan, so the projection already lands that way).
//
// False when the ray runs too near parallel to the plane to answer - a camera looking
// level, where the point would race off to the horizon - or when the plane is behind it.
bool screenRayToPlane(const glm::mat4& proj, const glm::vec3& camPos, const glm::vec3& fwd,
                      const glm::vec2& cursorPx, const glm::vec2& fb, float planeZ,
                      glm::vec3& hit);
