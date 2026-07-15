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

#include "Camera.h"

#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>

namespace {
const glm::vec3 kWorldUp(0.0f, 0.0f, 1.0f);
constexpr float kPitchLimit = 1.5533f; // ~89 degrees
} // namespace

void Camera::init(const glm::vec3& pos, const glm::vec3& dir) {
    pos_ = pos;
    yaw_ = std::atan2(dir.y, dir.x);
    pitch_ = glm::radians(-6.0f); // look slightly down the line
}

void Camera::setPose(const glm::vec3& pos, float yawRad, float pitchRad) {
    pos_ = pos;
    yaw_ = yawRad;
    pitch_ = std::clamp(pitchRad, -kPitchLimit, kPitchLimit);
}

glm::vec3 Camera::forward() const {
    const float cp = std::cos(pitch_);
    return glm::vec3(cp * std::cos(yaw_), cp * std::sin(yaw_), std::sin(pitch_));
}

void Camera::move(float forward, float right, float up, float dt, bool fast) {
    const glm::vec3 fwd = Camera::forward();
    const glm::vec3 rgt = glm::normalize(glm::cross(fwd, kWorldUp));

    const float v = speed_ * (fast ? 8.0f : 1.0f) * dt;
    pos_ += fwd * (forward * v);
    pos_ += rgt * (right * v);
    pos_ += kWorldUp * (up * v);
}

void Camera::look(float dx, float dy) {
    yaw_ -= dx * mouseSens_;
    pitch_ -= dy * mouseSens_;
    pitch_ = std::clamp(pitch_, -kPitchLimit, kPitchLimit);
}

glm::mat4 Camera::viewMatrix() const {
    return glm::lookAt(pos_, pos_ + forward(), kWorldUp);
}

glm::mat4 Camera::projMatrix(float aspect) const {
    glm::mat4 proj = glm::perspective(fovY_, aspect, nearZ_, farZ_);
    proj[1][1] *= -1.0f; // flip Y for Vulkan's clip space
    return proj;
}
