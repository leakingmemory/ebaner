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

bool screenRayToPlane(const glm::mat4& proj, const glm::vec3& camPos, const glm::vec3& fwd,
                      const glm::vec2& cursorPx, const glm::vec2& fb, float planeZ,
                      glm::vec3& hit) {
    if (fb.x < 1.0f || fb.y < 1.0f) return false;
    const float p00 = proj[0][0], p11 = proj[1][1];
    if (std::abs(p00) < 1e-6f || std::abs(p11) < 1e-6f) return false;

    // Rebuild the ray from the camera's own basis rather than by unprojecting a depth:
    // this build defines no GLM_FORCE_DEPTH_ZERO_TO_ONE and flips Y by hand, so the depth
    // convention is not something to lean on, while the basis is the same either way.
    // lookAt's rows are (right, up, -forward), so a view-space direction (x, y, -1) is
    // right*x + up*y + forward in the world.
    const glm::vec3 f = glm::normalize(fwd);
    const glm::vec3 right = glm::normalize(glm::cross(f, kWorldUp));
    const glm::vec3 up = glm::cross(right, f);
    const float ndcX = cursorPx.x / fb.x * 2.0f - 1.0f;
    const float ndcY = cursorPx.y / fb.y * 2.0f - 1.0f;
    const glm::vec3 dir = f + right * (ndcX / p00) + up * (ndcY / p11);

    const float len = glm::length(dir);
    if (len < 1e-9f) return false;
    // The sine of the ray's angle to the plane. Below ~3 deg the answer is both far away
    // and wildly sensitive to a pixel, so it is no answer at all.
    constexpr float kMinSlope = 0.0523f; // sin(3 deg)
    if (std::abs(dir.z) < kMinSlope * len) return false;

    const float t = (planeZ - camPos.z) / dir.z;
    if (t <= 0.0f) return false; // the plane is behind the camera
    hit = camPos + dir * t;
    return true;
}
