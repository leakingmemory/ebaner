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

#include "VehicleMesh.h"

#include "Vehicle.h" // Vehicle, VehicleFrame

#include <cmath>

namespace {
constexpr float kGauge = 1.435f;
constexpr float kWheelWidth = 0.135f;
constexpr float kAxleRadius = 0.06f;
constexpr float kAxleHalf = kGauge * 0.5f + kWheelWidth; // axle ends outside wheels
const glm::vec3 kAxleCol(0.36f, 0.36f, 0.38f);  // steel
const glm::vec3 kWheelCol(0.32f, 0.30f, 0.30f); // steel with a rusty tinge
const glm::vec3 kFrameCol(0.30f, 0.31f, 0.34f); // bogie frame steel
const glm::vec3 kUnderframeCol(0.24f, 0.25f, 0.27f); // carriage solebar/floor
constexpr int kSeg = 20;                        // segments per circle
constexpr float kPi = 3.14159265358979f;
// Bogie frame box, above the wheels.
constexpr float kFrameHalfWidth = 1.05f;  // across the track (m)
constexpr float kFrameHalfHeight = 0.18f; // vertical (m)
// Carriage underframe (floor plate) resting on the two bogies.
constexpr float kUnderframeHalfHeight = 0.15f; // thickness/2 (m)
} // namespace

void VehicleMesh::build(const Vehicle& vehicle) {
    vertices_.clear();
    indices_.clear();

    const glm::vec2 uv(0.0f);
    auto push = [&](const glm::vec3& p, const glm::vec3& n, const glm::vec3& c) {
        vertices_.push_back({p, n, c, uv, -1.0f});
    };

    // One wheelset (axle + two wheels) at the given on-rail frame.
    auto emitWheelset = [&](const VehicleFrame& fr) {
        const glm::vec3 X = fr.right, Y = fr.tangent, Z = fr.up;
        const glm::vec3 origin = fr.pos + Z * wheelset::kAxleCentreAboveBed;
        auto worldPt = [&](float lx, float ly, float lz) {
            return origin + X * lx + Y * ly + Z * lz;
        };
        auto worldNrm = [&](float nx, float ny, float nz) {
            return glm::normalize(X * nx + Y * ny + Z * nz);
        };
        // Cylinder with axis along local X, centred at x = cx, radius r,
        // half-length halfLen; side + two end caps.
        auto emitCyl = [&](float cx, float r, float halfLen, const glm::vec3& color) {
            const float x0 = cx - halfLen, x1 = cx + halfLen;
            for (int k = 0; k < kSeg; ++k) {
                const float a0 = 2.0f * kPi * k / kSeg;
                const float a1 = 2.0f * kPi * (k + 1) / kSeg;
                const float c0 = std::cos(a0), s0 = std::sin(a0);
                const float c1 = std::cos(a1), s1 = std::sin(a1);
                const glm::vec3 n0 = worldNrm(0.0f, c0, s0);
                const glm::vec3 n1 = worldNrm(0.0f, c1, s1);
                const std::uint32_t base =
                    static_cast<std::uint32_t>(vertices_.size());
                push(worldPt(x0, r * c0, r * s0), n0, color);
                push(worldPt(x1, r * c0, r * s0), n0, color);
                push(worldPt(x1, r * c1, r * s1), n1, color);
                push(worldPt(x0, r * c1, r * s1), n1, color);
                indices_.push_back(base + 0);
                indices_.push_back(base + 1);
                indices_.push_back(base + 2);
                indices_.push_back(base + 0);
                indices_.push_back(base + 2);
                indices_.push_back(base + 3);
            }
            for (int e = 0; e < 2; ++e) {
                const float xe = (e == 0) ? x0 : x1;
                const glm::vec3 an = worldNrm((e == 0) ? -1.0f : 1.0f, 0.0f, 0.0f);
                const std::uint32_t centre =
                    static_cast<std::uint32_t>(vertices_.size());
                push(worldPt(xe, 0.0f, 0.0f), an, color);
                for (int k = 0; k < kSeg; ++k) {
                    const float a = 2.0f * kPi * k / kSeg;
                    push(worldPt(xe, r * std::cos(a), r * std::sin(a)), an, color);
                }
                for (int k = 0; k < kSeg; ++k) {
                    indices_.push_back(centre);
                    indices_.push_back(centre + 1 + k);
                    indices_.push_back(centre + 1 + (k + 1) % kSeg);
                }
            }
        };
        emitCyl(0.0f, kAxleRadius, kAxleHalf, kAxleCol);
        emitCyl(-kGauge * 0.5f, wheelset::kWheelRadius, kWheelWidth * 0.5f, kWheelCol);
        emitCyl(kGauge * 0.5f, wheelset::kWheelRadius, kWheelWidth * 0.5f, kWheelCol);
    };

    // Solid box centred at c with half-extents (hx,hy,hz) along frame axes X,Y,Z.
    auto emitBox = [&](const glm::vec3& X, const glm::vec3& Y, const glm::vec3& Z,
                       const glm::vec3& c, float hx, float hy, float hz,
                       const glm::vec3& col) {
        auto P = [&](float sx, float sy, float sz) {
            return c + X * (sx * hx) + Y * (sy * hy) + Z * (sz * hz);
        };
        auto face = [&](const glm::vec3& p0, const glm::vec3& p1,
                        const glm::vec3& p2, const glm::vec3& p3, const glm::vec3& n) {
            const std::uint32_t b = static_cast<std::uint32_t>(vertices_.size());
            push(p0, n, col);
            push(p1, n, col);
            push(p2, n, col);
            push(p3, n, col);
            indices_.push_back(b + 0);
            indices_.push_back(b + 1);
            indices_.push_back(b + 2);
            indices_.push_back(b + 0);
            indices_.push_back(b + 2);
            indices_.push_back(b + 3);
        };
        face(P(1, -1, -1), P(1, 1, -1), P(1, 1, 1), P(1, -1, 1), X);
        face(P(-1, -1, -1), P(-1, 1, -1), P(-1, 1, 1), P(-1, -1, 1), -X);
        face(P(-1, 1, -1), P(1, 1, -1), P(1, 1, 1), P(-1, 1, 1), Y);
        face(P(-1, -1, -1), P(1, -1, -1), P(1, -1, 1), P(-1, -1, 1), -Y);
        face(P(-1, -1, 1), P(1, -1, 1), P(1, 1, 1), P(-1, 1, 1), Z);
        face(P(-1, -1, -1), P(1, -1, -1), P(1, 1, -1), P(-1, 1, -1), -Z);
    };

    for (const VehicleFrame& fr : vehicle.axleFrames()) emitWheelset(fr);

    // Height of a bogie frame box centre / its top above the pose bed.
    const float frameCentreZ =
        wheelset::kAxleCentreAboveBed + wheelset::kWheelRadius;
    const float frameTopZ = frameCentreZ + kFrameHalfHeight;

    // A bogie frame box (low steel box above the wheels, spanning the wheelbase).
    auto emitBogieFrame = [&](const VehicleFrame& b) {
        emitBox(b.right, b.tangent, b.up, b.pos + b.up * frameCentreZ,
                kFrameHalfWidth, 0.5f * vehicle.wheelbase(), kFrameHalfHeight,
                kFrameCol);
    };

    // A bogie frame per bogie (none for a bare wheelset).
    for (const VehicleFrame& bf : vehicle.bogieFrames()) emitBogieFrame(bf);

    // An underframe/floor plate per body section, sitting just above the bogie
    // frame tops. A carriage has one full-length plate; a 3-bogie module has two
    // half plates that hinge over the shared middle bogie (each section's frame
    // is oriented by its own bogie pair).
    const std::vector<VehicleFrame> sections = vehicle.bodySectionFrames();
    if (!sections.empty()) {
        const float halfLen =
            0.5f * vehicle.length() / static_cast<float>(sections.size());
        for (const VehicleFrame& s : sections) {
            const glm::vec3 centre =
                s.pos + s.up * (frameTopZ + kUnderframeHalfHeight);
            emitBox(s.right, s.tangent, s.up, centre, 0.5f * vehicle.width(),
                    halfLen, kUnderframeHalfHeight, kUnderframeCol);
        }
    }
}
