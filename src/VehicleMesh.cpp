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
#include <array>
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
const glm::vec3 kLight(0.93f, 0.92f, 0.84f); // headlight lens
constexpr float kHalfWidth = 1.36f;  // body half width (m)
constexpr float kHeight = 2.75f;     // floor-to-roof (m)
constexpr float kFloorAbove = 0.05f; // body floor above the bogie frame top (m)
constexpr float kWinLow = 1.00f;     // window band bottom above floor (m)
constexpr float kWinHigh = 2.05f;    // window band top above floor (m)
constexpr float kCantAbove = 2.20f;  // cantrail (shoulder base) above floor (m)
constexpr float kRoofHalf = 0.74f;   // domed-roof half width, fraction of body
constexpr float kTumble = 0.90f;     // floor-line half width (tumblehome), fraction
constexpr float kNoseLen = 3.60f;    // raked cab overhang (m)
constexpr float kDoorWidth = 1.30f;  // passenger door width (m)
constexpr float kGangGap = 0.45f;    // half the inter-car gap for the bellows (m)
const glm::vec3 kEquip(0.27f, 0.28f, 0.30f); // underfloor equipment box
const glm::vec3 kTank(0.32f, 0.32f, 0.34f);  // underfloor tank / lighter box
const glm::vec3 kRoofKit(0.40f, 0.41f, 0.43f); // roof exhaust / cooling boxes
const glm::vec3 kSkirt(0.09f, 0.09f, 0.10f);   // black coupler skirt / valance
const glm::vec3 kCoupler(0.22f, 0.23f, 0.25f); // steel automatic coupler head
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
        const float rhw = hw * c93::kRoofHalf;         // domed-roof half width
        const float z0 = frameTopZ + c93::kFloorAbove; // floor
        const float z1 = z0 + c93::kHeight;            // roof top
        const float zwl = z0 + c93::kWinLow, zwh = z0 + c93::kWinHigh; // window band
        const float zc = z0 + c93::kCantAbove;         // cantrail (shoulder base)
        auto P = [&](float x, float y, float z) { return f.pos + X * x + Y * y + Z * z; };

        const float tip = cabNegY ? -halfLen : halfLen;                // cab tip
        const float base = cabNegY ? -halfLen + c93::kNoseLen          // nose base
                                   : halfLen - c93::kNoseLen;
        // Gangway end, inset from the section boundary to leave a bellows gap.
        const float gang = (cabNegY ? 1.0f : -1.0f) * (halfLen - c93::kGangGap);
        const float ts = (tip < base) ? -1.0f : 1.0f;                  // forward sign
        const glm::vec3 fdir = Y * ts;
        const float bodyLo = std::min(base, gang), bodyHi = std::max(base, gang);
        const float allLo = std::min(tip, gang), allHi = std::max(tip, gang);
        const glm::vec3 in = P(0.0f, 0.5f * (allLo + allHi), 0.5f * (z0 + z1));

        // Shared cross-section: a rounded body profile (open at the bottom; the
        // floor pan closes it). Straight sides carry window-band boundary points,
        // then rounded shoulder arcs sweep in to a domed roof narrower than the
        // waist. `wS` scales the width and `drop` lowers the roof (used to taper
        // and rake the nose into a windscreen). Facets are coloured by geometry,
        // so the ring resolution can change freely.
        const int arcN = 4;
        auto ring = [&](float y, float wS, float drop) {
            const float w = hw * wS, rw = rhw * wS;
            const float zr = z1 - drop;
            const float zC = std::min(zc, zr - 0.04f);
            const float zH = std::min(zwh, zC - 0.02f), zL = std::min(zwl, zH - 0.02f);
            const float wb = w * c93::kTumble; // narrower at the floor line
            std::vector<glm::vec3> p;
            p.push_back(P(wb, y, z0));         // tumblehome: side leans in to floor
            p.push_back(P(w, y, zL));
            p.push_back(P(w, y, zH));
            for (int i = 0; i <= arcN; ++i) { // right shoulder (w,zC) -> (rw,zr)
                const float a = kPi * 0.5f * i / arcN;
                p.push_back(P(rw + (w - rw) * std::cos(a), y, zC + (zr - zC) * std::sin(a)));
            }
            for (int i = arcN; i >= 0; --i) { // left shoulder (-rw,zr) -> (-w,zC)
                const float a = kPi * 0.5f * i / arcN;
                p.push_back(P(-(rw + (w - rw) * std::cos(a)), y, zC + (zr - zC) * std::sin(a)));
            }
            p.push_back(P(-w, y, zH));
            p.push_back(P(-w, y, zL));
            p.push_back(P(-wb, y, z0));
            return p;
        };
        // Loft between two rings, colouring each facet by geometry: grey domed
        // roof (up-facing), the window band at cantrail height in `bandCol`, and
        // the rest of the side in `lowerCol`. Nose facets are coloured by
        // orientation (grey roof, dark forward windscreen, red sides).
        auto loft = [&](const std::vector<glm::vec3>& A, const std::vector<glm::vec3>& B,
                        bool nose, const glm::vec3& lowerCol, const glm::vec3& bandCol) {
            for (std::size_t k = 0; k + 1 < A.size(); ++k) {
                const glm::vec3 a = A[k], b = A[k + 1], c = B[k + 1], d = B[k];
                glm::vec3 nn = glm::cross(b - a, d - a);
                const float l = glm::length(nn);
                nn = (l > 1e-9f) ? nn / l : Z;
                const glm::vec3 cen = 0.25f * (a + b + c + d);
                if (glm::dot(nn, cen - in) < 0.0f) nn = -nn;
                const float locZ = glm::dot(cen - f.pos, Z);
                const float up = glm::dot(nn, Z);
                glm::vec3 col;
                if (nose) {
                    if (up > 0.45f || locZ > zc - 0.05f)
                        col = (glm::dot(nn, fdir) > 0.28f) ? c93::kBand : c93::kRoof;
                    else
                        col = c93::kRed;
                } else if (up > 0.45f) {
                    col = c93::kRoof; // domed roof / shoulders
                } else if (locZ > zwl - 0.03f && locZ < zwh + 0.03f) {
                    col = bandCol;    // window band (glazing or a pillar)
                } else {
                    col = lowerCol;   // lower body / cantrail (silver or door)
                }
                quadN(a, b, c, d, col, in);
            }
        };

        // Floor pan and the flat gangway end (matched to the tumblehome width).
        const float wf = hw * c93::kTumble;
        quadN(P(-wf, allLo, z0), P(wf, allLo, z0), P(wf, allHi, z0), P(-wf, allHi, z0), c93::kUnder, in);
        quadN(P(-wf, gang, z0), P(wf, gang, z0), P(wf, gang, z1), P(-wf, gang, z1), c93::kUnder, in);

        // Main body: a panel sequence along the car — end margins, glazed
        // windows separated by body-colour pillars, and two glazed doors. Each
        // panel lofts the constant profile with its lower/window-band colours.
        const float Lb = bodyHi - bodyLo;
        struct Panel { float a, b; glm::vec3 lower, band; };
        std::vector<Panel> panels;
        const float dhw = 0.5f * c93::kDoorWidth;
        const float dc1 = bodyLo + 0.27f * Lb, dc2 = bodyLo + 0.73f * Lb;
        auto tile = [&](float a, float b) { // fill [a,b] with windows + pillars
            const float m = 0.18f, winW = 1.35f, pilW = 0.32f;
            float x = a;
            panels.push_back({x, x + m, c93::kBody, c93::kBody}); // start margin
            x += m;
            while (b - m - x >= winW - 1e-3f) {
                panels.push_back({x, x + winW, c93::kBody, c93::kBand}); // window
                x += winW;
                if (b - m - x >= winW + pilW) {
                    panels.push_back({x, x + pilW, c93::kBody, c93::kBody}); // pillar
                    x += pilW;
                }
            }
            panels.push_back({x, b, c93::kBody, c93::kBody}); // end margin
        };
        tile(bodyLo, dc1 - dhw);
        panels.push_back({dc1 - dhw, dc1 + dhw, c93::kRed, c93::kBand}); // glazed door
        tile(dc1 + dhw, dc2 - dhw);
        panels.push_back({dc2 - dhw, dc2 + dhw, c93::kRed, c93::kBand});
        tile(dc2 + dhw, bodyHi);
        for (const Panel& p : panels)
            loft(ring(p.a, 1.0f, 0.0f), ring(p.b, 1.0f, 0.0f), false, p.lower, p.band);

        // Nose: taper width to a rounded prow while the roof rakes down into a
        // deep windscreen. Ease-in width and eased roof drop keep it smooth.
        const int N = 12;
        auto noseStation = [&](int i) {
            const float u = static_cast<float>(i) / N;
            const float y = base + (tip - base) * u;
            const float wS = std::sqrt(std::max(0.12f, 1.0f - 0.90f * u * u));
            const float drop = (z1 - (zwl + 0.05f)) * (u * u); // roof → window level
            return ring(y, wS, drop);
        };
        std::vector<glm::vec3> prev = noseStation(0);
        for (int i = 1; i <= N; ++i) {
            const std::vector<glm::vec3> cur = noseStation(i);
            loft(prev, cur, true, c93::kRed, c93::kRed); // colours unused for nose
            prev = cur;
        }
        // Prow cap (fan the last narrow ring closed).
        { glm::vec3 c(0.0f);
          for (const glm::vec3& q : prev) c += q;
          c /= static_cast<float>(prev.size());
          const float czc = glm::dot(c - f.pos, Z);
          for (std::size_t k = 0; k < prev.size(); ++k) {
              const glm::vec3& a = prev[k];
              const glm::vec3& b = prev[(k + 1) % prev.size()];
              quadN(a, b, c, c, (czc > z0 + 0.55f * (z1 - z0)) ? c93::kBand : c93::kRed, in);
          } }

        // Headlights: a lens block at each lower front corner.
        const float yh = base + (tip - base) * 0.92f;
        for (const float xh : {hw * 0.46f, -hw * 0.46f})
            emitBox(X, Y, Z, P(xh, yh, z0 + 0.42f), 0.14f, 0.16f, 0.12f, c93::kLight);

        // Underfloor systems: a shallow equipment raft under the whole body plus
        // a couple of deeper boxes (engine / tank) slung between the bogies.
        emitBox(X, Y, Z, P(0.0f, 0.5f * (bodyLo + bodyHi), z0 - 0.24f),
                hw * 0.86f, 0.5f * Lb * 0.96f, 0.22f, c93::kEquip);
        emitBox(X, Y, Z, P(0.0f, bodyLo + 0.34f * Lb, z0 - 0.56f),
                hw * 0.82f, 0.20f * Lb, 0.40f, c93::kEquip);
        emitBox(X, Y, Z, P(0.0f, bodyLo + 0.66f * Lb, z0 - 0.50f),
                hw * 0.74f, 0.15f * Lb, 0.34f, c93::kTank);

        // Roof equipment: exhaust / cooling boxes along the car roof, sitting on
        // the domed roof crown.
        emitBox(X, Y, Z, P(0.0f, bodyLo + 0.32f * Lb, z1 + 0.13f),
                rhw * 0.74f, 0.11f * Lb, 0.13f, c93::kRoofKit);
        emitBox(X, Y, Z, P(0.0f, bodyLo + 0.58f * Lb, z1 + 0.16f),
                rhw * 0.60f, 0.08f * Lb, 0.16f, c93::kRoofKit); // taller (exhaust)
        emitBox(X, Y, Z, P(0.0f, bodyLo + 0.80f * Lb, z1 + 0.11f),
                rhw * 0.72f, 0.09f * Lb, 0.11f, c93::kRoofKit);

        // Coupler skirt: a black valance under the cab front around the coupler.
        emitBox(X, Y, Z, P(0.0f, base + (tip - base) * 0.86f, z0 - 0.28f),
                hw * 0.52f, (tip - base) * 0.15f, 0.30f, c93::kSkirt);

        // Automatic (Scharfenberg) coupler poking out of the skirt: a draft-gear
        // block, the coupler shaft, and a wider knuckle head at the very front.
        const float cz = z0 - 0.30f;
        emitBox(X, Y, Z, P(0.0f, tip + ts * 0.02f, cz), hw * 0.26f, 0.12f, 0.16f, c93::kSkirt);
        emitBox(X, Y, Z, P(0.0f, tip + ts * 0.22f, cz), 0.07f, 0.12f, 0.09f, c93::kCoupler);
        emitBox(X, Y, Z, P(0.0f, tip + ts * 0.38f, cz), hw * 0.20f, 0.06f, 0.15f, c93::kCoupler);

        // Two obstacle deflectors ahead of the leading bogie: a larger upper
        // snowplow below the coupler and a smaller lower lifeguard just above the
        // rail. Each is a dark V-blade wedge with its point raked forward.
        {
            auto FY = [&](float d) { return tip + ts * d; };
            auto plow = [&](float zTop, float zBot, float wT, float wB,
                            float dBack, float dFront) {
                const glm::vec3 TBL = P(-wT, FY(dBack), zTop);
                const glm::vec3 TBR = P(wT, FY(dBack), zTop);
                const glm::vec3 TBC = P(0.0f, FY(dBack), zTop);
                const glm::vec3 FBL = P(-wB, FY(dFront), zBot);
                const glm::vec3 FBR = P(wB, FY(dFront), zBot);
                const glm::vec3 FBC = P(0.0f, FY(dFront + 0.16f), zBot - 0.08f); // point
                const glm::vec3 back = P(0.0f, FY(dBack - 0.3f), zTop + 0.3f);
                const glm::vec3 down = P(0.0f, FY(dFront), zTop - 2.0f);
                quadN(TBL, FBL, FBC, TBC, c93::kSkirt, back); // blade faces
                quadN(TBC, FBC, FBR, TBR, c93::kSkirt, back);
                quadN(TBL, TBC, FBC, FBL, c93::kSkirt, down); // undersides
                quadN(TBC, TBR, FBR, FBC, c93::kSkirt, down);
            };
            plow(z0 - 0.34f, z0 - 0.64f, hw * 0.62f, hw * 0.42f, -0.05f, 0.28f); // upper
            plow(z0 - 0.72f, z0 - 0.98f, hw * 0.34f, hw * 0.22f, 0.12f, 0.34f);  // lower
        }
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

        // Gangway bellows: a dark concertina tube bridging each pair of adjacent
        // Class 93 car bodies over the shared middle bogie. Its ends follow the
        // two sections' inner faces, so it flexes with the articulation.
        if (vehicle.bodyStyle() == BodyClass93 && sections.size() >= 2) {
            const float z0b = frameTopZ + c93::kFloorAbove;
            const float zCb = z0b + c93::kCantAbove;
            const float bw = c93::kHalfWidth * 0.70f;
            auto gwEnd = [&](const VehicleFrame& f, float sign) {
                const glm::vec3 e = f.pos + f.tangent * (sign * (halfLen - c93::kGangGap));
                return std::array<glm::vec3, 4>{
                    e + f.right * bw + f.up * z0b, e + f.right * bw + f.up * zCb,
                    e - f.right * bw + f.up * zCb, e - f.right * bw + f.up * z0b};
            };
            for (std::size_t i = 0; i + 1 < sections.size(); ++i) {
                const std::array<glm::vec3, 4> A = gwEnd(sections[i], +1.0f);
                const std::array<glm::vec3, 4> B = gwEnd(sections[i + 1], -1.0f);
                glm::vec3 mid(0.0f);
                for (int k = 0; k < 4; ++k) mid += A[k] + B[k];
                mid *= 0.125f;
                const int M = 6; // concertina segments (alternate in/out)
                std::array<glm::vec3, 4> prev = A;
                for (int j = 1; j <= M; ++j) {
                    const float t = static_cast<float>(j) / M;
                    std::array<glm::vec3, 4> cur;
                    glm::vec3 cen(0.0f);
                    for (int k = 0; k < 4; ++k) cen += glm::mix(A[k], B[k], t);
                    cen *= 0.25f;
                    const float s = (j % 2 == 0 || j == M) ? 1.0f : 0.82f; // ribs
                    for (int k = 0; k < 4; ++k)
                        cur[k] = cen + (glm::mix(A[k], B[k], t) - cen) * s;
                    for (int k = 0; k < 4; ++k)
                        quadN(prev[k], prev[(k + 1) % 4], cur[(k + 1) % 4], cur[k],
                              c93::kSkirt, mid);
                    prev = cur;
                }
            }
        }
    }
}
