#include "WheelsetMesh.h"

#include "TrackPath.h" // TrackPose

#include <cmath>
#include <cstdio>

namespace {
constexpr float kGauge = 1.435f;
constexpr float kWheelWidth = 0.135f;
constexpr float kAxleRadius = 0.06f;
constexpr float kAxleHalf = kGauge * 0.5f + kWheelWidth; // axle ends outside wheels
const glm::vec3 kAxleCol(0.36f, 0.36f, 0.38f);  // steel
const glm::vec3 kWheelCol(0.32f, 0.30f, 0.30f); // steel with a rusty tinge
constexpr int kSeg = 20;                        // segments per circle
constexpr float kPi = 3.14159265358979f;
} // namespace

void WheelsetMesh::build(const TrackPose& pose) {
    // Pose frame: X across (right), Y forward (tangent), Z up — all banked.
    const glm::vec3 X = pose.right, Y = pose.tangent, Z = pose.up;
    const glm::vec3 origin =
        pose.pos + Z * wheelset::kAxleCentreAboveBed; // axle centre

    auto worldPt = [&](float lx, float ly, float lz) {
        return origin + X * lx + Y * ly + Z * lz;
    };
    auto worldNrm = [&](float nx, float ny, float nz) {
        return glm::normalize(X * nx + Y * ny + Z * nz);
    };
    const glm::vec2 uv(0.0f);
    auto push = [&](const glm::vec3& p, const glm::vec3& n, const glm::vec3& c) {
        vertices_.push_back({p, n, c, uv, -1.0f});
    };

    // Cylinder with axis along local X, centred at local x = cx, radius r,
    // half-length halfLen; side + two end caps.
    auto emitCylinder = [&](float cx, float r, float halfLen,
                            const glm::vec3& color) {
        const float x0 = cx - halfLen, x1 = cx + halfLen;
        for (int k = 0; k < kSeg; ++k) {
            const float a0 = 2.0f * kPi * k / kSeg;
            const float a1 = 2.0f * kPi * (k + 1) / kSeg;
            const float c0 = std::cos(a0), s0 = std::sin(a0);
            const float c1 = std::cos(a1), s1 = std::sin(a1);
            const glm::vec3 n0 = worldNrm(0.0f, c0, s0);
            const glm::vec3 n1 = worldNrm(0.0f, c1, s1);
            const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
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
        // End caps (fans). Normal along ±X; culling is off so winding is moot.
        for (int e = 0; e < 2; ++e) {
            const float xe = (e == 0) ? x0 : x1;
            const glm::vec3 an = worldNrm((e == 0) ? -1.0f : 1.0f, 0.0f, 0.0f);
            const std::uint32_t centre = static_cast<std::uint32_t>(vertices_.size());
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

    // Axle spanning the gauge, and a wheel at each rail.
    emitCylinder(0.0f, kAxleRadius, kAxleHalf, kAxleCol);
    emitCylinder(-kGauge * 0.5f, wheelset::kWheelRadius, kWheelWidth * 0.5f, kWheelCol);
    emitCylinder(kGauge * 0.5f, wheelset::kWheelRadius, kWheelWidth * 0.5f, kWheelCol);

    std::printf("[WheelsetMesh] 1 wheelset, %zu vertices, %zu triangles\n",
                vertices_.size(), indices_.size() / 3);
}
