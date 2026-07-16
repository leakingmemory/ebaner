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

#include <algorithm>
#include <cmath>
#include <vector>

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

// NSB Class 93 (Bombardier Talent) exterior, classic NSB livery.
namespace c93 {
const glm::vec3 kBody(0.72f, 0.73f, 0.75f);  // silver-grey car body
const glm::vec3 kBand(0.11f, 0.12f, 0.14f);  // dark window band / glazing
const glm::vec3 kRed(0.74f, 0.10f, 0.12f);   // NSB red: doors + cab front
const glm::vec3 kRoof(0.55f, 0.56f, 0.58f);  // grey roof
const glm::vec3 kUnder(0.18f, 0.19f, 0.21f); // dark underframe/floor
constexpr float kHalfWidth = 1.36f;  // body half width (m)
constexpr float kHeight = 2.75f;     // floor-to-roof (m)
constexpr float kFloorAbove = 0.05f; // body floor above the bogie frame top (m)
constexpr float kWinLow = 1.00f;     // window band bottom above floor (m)
constexpr float kWinHigh = 2.00f;    // window band top above floor (m)
constexpr float kNoseLen = 3.20f;    // raked cab overhang (m)
constexpr float kDoorWidth = 1.30f;  // passenger door width (m)
} // namespace
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

    // A flat quad (p0..p3); its normal is oriented to point away from `inside`
    // so winding order doesn't matter (the body is roughly convex).
    auto quadN = [&](const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                     const glm::vec3& p3, const glm::vec3& col,
                     const glm::vec3& inside) {
        glm::vec3 n = glm::cross(p1 - p0, p3 - p0);
        const float l = glm::length(n);
        n = (l > 1e-9f) ? n / l : glm::vec3(0.0f, 0.0f, 1.0f);
        const glm::vec3 cen = (p0 + p1 + p2 + p3) * 0.25f;
        if (glm::dot(n, cen - inside) < 0.0f) n = -n;
        const std::uint32_t b = static_cast<std::uint32_t>(vertices_.size());
        push(p0, n, col); push(p1, n, col); push(p2, n, col); push(p3, n, col);
        indices_.push_back(b + 0); indices_.push_back(b + 1); indices_.push_back(b + 2);
        indices_.push_back(b + 0); indices_.push_back(b + 2); indices_.push_back(b + 3);
    };

    // One NSB Class 93 car-body section. `f` is the section frame (X=right,
    // Y=tangent, Z=up); the body spans y in [-halfLen, halfLen] with the raked
    // cab at the outer end (cabNegY picks which) and the articulation gangway at
    // the other. Silver body, dark window band, red doors, red raked cab front.
    auto emitClass93 = [&](const VehicleFrame& f, float halfLen, bool cabNegY) {
        const glm::vec3 X = f.right, Y = f.tangent, Z = f.up;
        const float hw = c93::kHalfWidth;
        const float z0 = frameTopZ + c93::kFloorAbove;                 // floor
        const float z1 = z0 + c93::kHeight;                            // roof
        const float zwl = z0 + c93::kWinLow, zwh = z0 + c93::kWinHigh;  // window band
        auto P = [&](float x, float y, float z) { return f.pos + X * x + Y * y + Z * z; };

        const float tip = cabNegY ? -halfLen : halfLen;                // cab tip
        const float base = cabNegY ? -halfLen + c93::kNoseLen          // nose base
                                   : halfLen - c93::kNoseLen;
        const float gang = cabNegY ? halfLen : -halfLen;               // gangway end
        const float bodyLo = std::min(base, gang), bodyHi = std::max(base, gang);
        const float allLo = std::min(tip, gang), allHi = std::max(tip, gang);
        const glm::vec3 in = P(0.0f, 0.5f * (allLo + allHi), 0.5f * (z0 + z1));

        // Roof, floor (full length incl. nose), and the flat gangway end.
        quadN(P(-hw, bodyLo, z1), P(hw, bodyLo, z1), P(hw, bodyHi, z1), P(-hw, bodyHi, z1), c93::kRoof, in);
        quadN(P(-hw, allLo, z0), P(hw, allLo, z0), P(hw, allHi, z0), P(-hw, allHi, z0), c93::kUnder, in);
        quadN(P(-hw, gang, z0), P(hw, gang, z0), P(hw, gang, z1), P(-hw, gang, z1), c93::kUnder, in);

        // Sides split into window / door panels (two doors per side).
        const float Lb = bodyHi - bodyLo, hdw = 0.5f * c93::kDoorWidth;
        const float d1 = bodyLo + 0.26f * Lb, d2 = bodyLo + 0.74f * Lb;
        struct Seg { float a, b; bool door; };
        const Seg segs[] = {{bodyLo, d1 - hdw, false}, {d1 - hdw, d1 + hdw, true},
                            {d1 + hdw, d2 - hdw, false}, {d2 - hdw, d2 + hdw, true},
                            {d2 + hdw, bodyHi, false}};
        for (const float xs : {hw, -hw}) {
            auto sq = [&](float ya, float yb, float za, float zb, const glm::vec3& c) {
                quadN(P(xs, ya, za), P(xs, yb, za), P(xs, yb, zb), P(xs, ya, zb), c, in);
            };
            for (const Seg& s : segs) {
                if (s.door) { sq(s.a, s.b, z0, z1, c93::kRed); }
                else {
                    sq(s.a, s.b, z0, zwl, c93::kBody);  // lower body
                    sq(s.a, s.b, zwl, zwh, c93::kBand); // window band
                    sq(s.a, s.b, zwh, z1, c93::kBody);  // cantrail
                }
            }
        }

        // Rounded cab nose, lofted from the full body cross-section at the nose
        // base to a blunt, lowered, narrower snout at the tip. Each ring is an
        // open profile (bottom-right, around a rounded top, to bottom-left); the
        // underside is closed by the floor quad above.
        const float ts = (tip < base) ? -1.0f : 1.0f; // outward (forward) sign
        const glm::vec3 fdir = Y * ts;
        auto noseRing = [&](float u) {
            const float y = base + (tip - base) * u;
            const float hwu = hw * std::sqrt(std::max(0.18f, 1.0f - 0.72f * u * u));
            const float zt = z1 - (z1 - zwh) * (u * u); // roof drops toward the tip
            float r = 0.05f + 0.5f * u;                 // top corners round off
            r = std::min(r, std::min(hwu * 0.9f, (zt - z0) * 0.45f));
            std::vector<glm::vec3> p;
            p.push_back(P(hwu, y, z0));
            const int arcN = 3;
            for (int i = 0; i <= arcN; ++i) { // right shoulder, 0..90 deg
                const float a = kPi * 0.5f * i / arcN;
                p.push_back(P((hwu - r) + r * std::cos(a), y, (zt - r) + r * std::sin(a)));
            }
            for (int i = 0; i <= arcN; ++i) { // left shoulder, 90..180 deg
                const float a = kPi * 0.5f + kPi * 0.5f * i / arcN;
                p.push_back(P((-hwu + r) + r * std::cos(a), y, (zt - r) + r * std::sin(a)));
            }
            p.push_back(P(-hwu, y, z0));
            return p;
        };
        // Colour a nose facet: grey roof on top, dark windscreen on the forward-
        // facing upper surface, red elsewhere.
        auto noseCol = [&](const glm::vec3& n, const glm::vec3& cen) {
            if (glm::dot(n, Z) > 0.55f) return c93::kRoof;
            const float cz = glm::dot(cen - f.pos, Z);
            if (glm::dot(n, fdir) > 0.35f && cz > zwh - 0.15f) return c93::kBand;
            return c93::kRed;
        };
        const int N = 6;
        std::vector<glm::vec3> prev = noseRing(0.0f);
        for (int s = 1; s <= N; ++s) {
            const std::vector<glm::vec3> cur = noseRing(static_cast<float>(s) / N);
            for (std::size_t k = 0; k + 1 < prev.size(); ++k) {
                const glm::vec3 a = prev[k], b = prev[k + 1], c = cur[k + 1], d = cur[k];
                glm::vec3 nn = glm::cross(b - a, d - a);
                const float l = glm::length(nn);
                nn = (l > 1e-9f) ? nn / l : Z;
                const glm::vec3 cen = 0.25f * (a + b + c + d);
                if (glm::dot(nn, cen - in) < 0.0f) nn = -nn;
                quadN(a, b, c, d, noseCol(nn, cen), in);
            }
            prev = cur;
        }
        // Blunt tip cap (fan the last ring closed, incl. the bottom chord).
        { glm::vec3 c(0.0f);
          for (const glm::vec3& q : prev) c += q;
          c /= static_cast<float>(prev.size());
          for (std::size_t k = 0; k < prev.size(); ++k) {
              const glm::vec3& a = prev[k];
              const glm::vec3& b = prev[(k + 1) % prev.size()];
              const glm::vec3 cen = (a + b + c) / 3.0f;
              const float cz = glm::dot(cen - f.pos, Z);
              quadN(a, b, c, c, (cz > zwh - 0.15f) ? c93::kBand : c93::kRed, in);
          } }
    };

    // Body per section. A Class 93 draws a liveried car body (cab at each outer
    // end); everything else draws a bare underframe/floor plate. A carriage has
    // one full-length plate; a 3-bogie module two half plates hinging over the
    // shared middle bogie (each section oriented by its own bogie pair).
    const std::vector<VehicleFrame> sections = vehicle.bodySectionFrames();
    if (!sections.empty()) {
        const float halfLen =
            0.5f * vehicle.length() / static_cast<float>(sections.size());
        for (std::size_t i = 0; i < sections.size(); ++i) {
            if (vehicle.bodyStyle() == BodyClass93) {
                emitClass93(sections[i], halfLen, i == 0); // front cab at -Y
            } else {
                const glm::vec3 centre =
                    sections[i].pos + sections[i].up * (frameTopZ + kUnderframeHalfHeight);
                emitBox(sections[i].right, sections[i].tangent, sections[i].up,
                        centre, 0.5f * vehicle.width(), halfLen,
                        kUnderframeHalfHeight, kUnderframeCol);
            }
        }
    }
}
