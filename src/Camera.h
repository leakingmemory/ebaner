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
