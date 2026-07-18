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
const glm::vec3 kFloor(0.42f, 0.42f, 0.45f);   // interior saloon floor
const glm::vec3 kStep(0.32f, 0.32f, 0.34f);    // interior step / riser
const glm::vec3 kWall(0.74f, 0.70f, 0.62f);    // interior cab partition wall
const glm::vec3 kLining(0.82f, 0.80f, 0.75f);  // interior wall lining (neutral)
const glm::vec3 kCeiling(0.87f, 0.87f, 0.88f); // interior ceiling lining (light)
const glm::vec3 kGlass(0.17f, 0.19f, 0.23f);   // window glazing seen from inside
const glm::vec3 kWood(0.52f, 0.37f, 0.23f);    // wood-tone module panels
const glm::vec3 kSeat(0.30f, 0.32f, 0.38f);    // passenger seat
const glm::vec3 kDash(0.14f, 0.15f, 0.17f);    // driver's desk / console
const glm::vec3 kScreen(0.13f, 0.22f, 0.30f);  // MFD / LCD panel
const glm::vec3 kGauge(0.82f, 0.81f, 0.77f);   // analog dial face
const glm::vec3 kButton(0.34f, 0.35f, 0.38f);  // desk control buttons
const glm::vec3 kEngBar(0.30f, 0.78f, 0.36f);  // engine progress-bar fill (green)
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
    auto emitClass93 = [&](const VehicleFrame& f, float halfLen, bool cabNegY,
                           bool hasWC) {
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
        // Outer skin sill: the side sheeting hangs low almost everywhere (nose
        // included), with a tight rectangular cut-up over each bogie to clear the
        // wheels. The two bogies in this car (section frame) are the leading/outer
        // one and the shared middle bogie at the gangway end.
        const float bc = 0.5f * vehicle.bogieSpacing();
        const float yBogieOuter = ts * (bc - halfLen);
        const float yBogieMid = -ts * halfLen;
        // The nose and the skin over both bogies share one fairly low end level
        // (nose flush with the over-bogie skin); only the mid-car, between the
        // bogies, dips lower as a valance. A rectangular step joins them.
        const float zEnd = frameCentreZ - 0.10f;   // nose + over-bogie skin level
        const float zLow = frameCentreZ - 0.36f;   // mid-car valance (lower)
        const float dipInset = 0.5f * vehicle.wheelbase() + 0.42f;
        const float dipA = std::min(yBogieOuter, yBogieMid) + dipInset;
        const float dipB = std::max(yBogieOuter, yBogieMid) - dipInset;
        auto sillAt = [&](float y) { return (y > dipA && y < dipB) ? zLow : zEnd; };
        const int arcN = 4;
        auto ring = [&](float y, float wS, float drop, float zb, float zbL, float zbH) {
            const float w = hw * wS, rw = rhw * wS;
            const float zr = z1 - drop;
            const float zC = std::min(zc, zr - 0.04f);
            const float zH = std::min(zbH, zC - 0.02f), zL = std::min(zbL, zH - 0.02f);
            const float wb = w * c93::kTumble; // narrower at the sill line
            std::vector<glm::vec3> p;
            p.push_back(P(wb, y, zb));          // tumblehome: side leans in to sill
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
            p.push_back(P(-wb, y, zb));
            return p;
        };
        // Loft between two rings, colouring each facet by geometry: grey domed
        // roof (up-facing), the window band at cantrail height in `bandCol`, and
        // the rest of the side in `lowerCol`. Nose facets are coloured by
        // orientation (grey roof, dark forward windscreen, red sides).
        auto loft = [&](const std::vector<glm::vec3>& A, const std::vector<glm::vec3>& B,
                        bool nose, const glm::vec3& lowerCol,
                        const glm::vec3& bandColR, const glm::vec3& bandColL) {
            for (std::size_t k = 0; k + 1 < A.size(); ++k) {
                const glm::vec3 a = A[k], b = A[k + 1], c = B[k + 1], d = B[k];
                glm::vec3 nn = glm::cross(b - a, d - a);
                const float l = glm::length(nn);
                nn = (l > 1e-9f) ? nn / l : Z;
                const glm::vec3 cen = 0.25f * (a + b + c + d);
                if (glm::dot(nn, cen - in) < 0.0f) nn = -nn;
                const float locZ = glm::dot(cen - f.pos, Z);
                glm::vec3 col;
                if (nose) {
                    // Classify nose facets by orientation: the forward-raked front
                    // is the windscreen (glazed); the near-horizontal crown is
                    // opaque roof; the sideways-facing panels are the solid cab
                    // sides, glazed only over the window band (a small side
                    // window); red fascia below the glazing line.
                    const float fwd = glm::dot(nn, fdir);
                    const float side = std::abs(glm::dot(nn, X));
                    if (locZ < z0 + 0.65f)
                        col = c93::kRed;                       // fascia
                    else if (fwd > 0.28f)
                        col = c93::kBand;                      // windscreen (glass)
                    else if (side > 0.5f)
                        col = (k == 1 || k == 13) ? c93::kBand // small side window
                                                  : c93::kBody; // solid cab side
                    else
                        col = c93::kRoof;                      // crown (opaque roof)
                } else if (k >= 3 && k <= 11) {
                    col = c93::kRoof; // shoulders + domed roof (by ring index)
                } else if (k == 1) {
                    col = bandColR;   // +x window band (glazing or blank)
                } else if (k == 13) {
                    col = bandColL;   // -x window band
                } else {
                    col = lowerCol;   // lower body / cantrail (silver or door)
                }
                quadN(a, b, c, d, col, in);
            }
        };

        // Floor pan and the gangway end. The end wall has a central aisle
        // doorway (a low-floor walk-through to the next car via the bellows).
        const float wf = hw * c93::kTumble;
        quadN(P(-wf, allLo, z0), P(wf, allLo, z0), P(wf, allHi, z0), P(-wf, allHi, z0), c93::kUnder, in);
        {
            const float aisle = 0.52f;          // half aisle-doorway width
            const float zFlr = z0 + 0.03f;      // gangway floor
            const float zDoor = zFlr + 2.0f;    // doorway top
            quadN(P(-wf, gang, z0), P(-aisle, gang, z0), P(-aisle, gang, z1), P(-wf, gang, z1), c93::kUnder, in);
            quadN(P(aisle, gang, z0), P(wf, gang, z0), P(wf, gang, z1), P(aisle, gang, z1), c93::kUnder, in);
            quadN(P(-aisle, gang, zDoor), P(aisle, gang, zDoor), P(aisle, gang, z1), P(-aisle, gang, z1), c93::kUnder, in);
            quadN(P(-aisle, gang, z0), P(aisle, gang, z0), P(aisle, gang, zFlr), P(-aisle, gang, zFlr), c93::kUnder, in);
        }

        // Main body: one passenger door per car, set inboard of the cab. The
        // window band steps in level at the door — higher over the raised cab
        // vestibule, lower over the low-floor saloon. Windows are glazed panels
        // separated by body-colour pillars.
        const float Lb = bodyHi - bodyLo;
        const float dhw = 0.5f * c93::kDoorWidth;
        // The floor/window level steps at the stairs; the exterior door sits on
        // the low floor, past the stairs (so it doesn't open onto them).
        const float yStep = base + (gang - base) * 0.36f;
        const float yDoor = base + (gang - base) * 0.46f;
        auto cabSide = [&](float y) { return std::abs(y - base) < std::abs(yStep - base); };
        auto bandLo = [&](float y) { return z0 + (cabSide(y) ? 1.18f : 0.82f); };
        auto bandHi = [&](float y) { return z0 + (cabSide(y) ? 2.05f : 1.68f); };
        // Boxed interior areas have no windows, so the glazing is blanked over
        // them: the tech cabinets behind the cab, the stairs, and (one car only)
        // the WC/utility module on the gangway side.
        const float dirSaloon = (gang > base) ? 1.0f : -1.0f;
        // WC/utility module: one side only (+x), directly beside the door,
        // extending toward the gangway (a long box).
        const float wc0 = yDoor + dirSaloon * (dhw + 0.05f);
        const float wc1 = wc0 + dirSaloon * std::abs(gang - base) * 0.40f;
        // Tech cabinets and the stairs block both sides; the WC blocks its side.
        const std::pair<float, float> solidsBoth[] = {
            {base, base + dirSaloon * 2.0f}, {yStep - 0.4f, yStep + 0.4f}};
        auto inRange = [&](const std::pair<float, float>& s, float y) {
            return y > std::min(s.first, s.second) && y < std::max(s.first, s.second);
        };
        auto inBoth = [&](float y) {
            for (const auto& s : solidsBoth)
                if (inRange(s, y)) return true;
            return false;
        };
        auto inWC = [&](float y) { return hasWC && inRange({wc0, wc1}, y); };
        // Blank the glazing over boxed areas (per side): +x facet vs -x facet.
        auto bandR = [&](float y) { return (inBoth(y) || inWC(y)) ? c93::kBody : c93::kBand; };
        auto bandL = [&](float y) { return inBoth(y) ? c93::kBody : c93::kBand; };
        struct Panel { float a, b; glm::vec3 lower, bandR, bandL; };
        std::vector<Panel> panels;
        auto tile = [&](float a, float b) { // fill [a,b] with windows + pillars
            const float m = 0.18f, winW = 1.35f, pilW = 0.32f;
            float x = a;
            panels.push_back({x, x + m, c93::kBody, c93::kBody, c93::kBody}); // margin
            x += m;
            while (b - m - x >= winW - 1e-3f) {
                // A window each side unless blanked by a boxed interior area.
                const float wcen = x + 0.5f * winW;
                panels.push_back({x, x + winW, c93::kBody, bandR(wcen), bandL(wcen)});
                x += winW;
                if (b - m - x >= winW + pilW) {
                    panels.push_back({x, x + pilW, c93::kBody, c93::kBody, c93::kBody}); // pillar
                    x += pilW;
                }
            }
            panels.push_back({x, b, c93::kBody, c93::kBody, c93::kBody}); // end margin
        };
        tile(bodyLo, yDoor - dhw);
        panels.push_back({yDoor - dhw, yDoor + dhw, c93::kRed, c93::kRed, c93::kRed}); // door
        tile(yDoor + dhw, bodyHi);
        // Loft each panel, subdividing at the bogie-notch edges (doubled with a
        // tiny gap) so the sill steps up near-vertically over each bogie.
        const float eps = 0.02f;
        const float edges[] = {dipA, dipB};
        auto bodyRing = [&](float y) {
            return ring(y, 1.0f, 0.0f, sillAt(y), bandLo(y), bandHi(y));
        };
        for (const Panel& p : panels) {
            std::vector<float> ys = {p.a, p.b};
            for (const float e : edges)
                if (e > p.a + eps && e < p.b - eps) {
                    ys.push_back(e - eps);
                    ys.push_back(e + eps);
                }
            std::sort(ys.begin(), ys.end());
            for (std::size_t i = 0; i + 1 < ys.size(); ++i)
                loft(bodyRing(ys[i]), bodyRing(ys[i + 1]), false, p.lower, p.bandR, p.bandL);
        }

        // Nose: taper width to a rounded prow while the roof rakes down into a
        // deep windscreen. Ease-in width and eased roof drop keep it smooth.
        const int N = 12;
        auto noseStation = [&](int i) {
            const float u = static_cast<float>(i) / N;
            const float y = base + (tip - base) * u;
            const float wS = std::sqrt(std::max(0.12f, 1.0f - 0.90f * u * u));
            const float drop = (z1 - (zwl + 0.05f)) * (u * u); // roof → window level
            // Nose skin follows the same sill; band levels are unused (nose is red).
            return ring(y, wS, drop, sillAt(y), zwl, zwh);
        };
        std::vector<glm::vec3> prev = noseStation(0);
        for (int i = 1; i <= N; ++i) {
            const std::vector<glm::vec3> cur = noseStation(i);
            loft(prev, cur, true, c93::kRed, c93::kRed, c93::kRed); // colours unused for nose
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

        // Headlights / marker lights set into the lower corners of the
        // windscreen glazing: compact lamp blocks just above the red fascia.
        const float yh = base + (tip - base) * 0.90f;
        for (const float xh : {hw * 0.42f, -hw * 0.42f})
            emitBox(X, Y, Z, P(xh, yh, z0 + 0.82f), 0.14f, 0.09f, 0.07f, c93::kLight);

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

        // Two obstacle deflectors hanging below the floor: a large upper snowplow
        // under the cab front, and a lower lifeguard set back just ahead of the
        // leading bogie's wheelset, dropping to near rail level. Each is a dark
        // V-blade wedge (top edge back, sweeping down-forward to a low V point).
        {
            auto deflector = [&](float yBack, float fwd, float zTop, float zBot,
                                 float wT, float wB) {
                const float yF = yBack + ts * fwd;
                const glm::vec3 TL = P(-wT, yBack, zTop), TR = P(wT, yBack, zTop);
                const glm::vec3 TC = P(0.0f, yBack, zTop);
                const glm::vec3 BL = P(-wB, yF, zBot), BR = P(wB, yF, zBot);
                const glm::vec3 BC = P(0.0f, yF + ts * 0.14f, zBot - 0.06f); // V point
                const glm::vec3 nref = P(0.0f, yBack - ts * 0.4f, zTop + 0.4f);
                const glm::vec3 dref = P(0.0f, yF, zBot - 2.0f);
                quadN(TL, BL, BC, TC, c93::kSkirt, nref); // blade faces
                quadN(TC, BC, BR, TR, c93::kSkirt, nref);
                quadN(TL, TC, BC, BL, c93::kSkirt, dref); // undersides
                quadN(TC, TR, BR, BC, c93::kSkirt, dref);
            };
            // Upper snowplow under the cab front (wide), lower lifeguard back at
            // the leading bogie (narrower, near rail).
            deflector(tip + ts * -0.9f, 0.45f, z0 - 0.34f, z0 - 0.86f, hw * 0.98f, hw * 0.66f);
            deflector(yBogieOuter + ts * 1.5f, 0.35f, z0 - 0.62f, z0 - 1.06f, hw * 0.52f, hw * 0.34f);
        }

        // ---- Interior (seen by flying the camera inside the shell). The low
        // saloon floor sits between two raised end vestibules (over the bogies,
        // in front of the doors), joined by two-step risers; a partition wall
        // with a central aisle doorway closes off the driver's cab. ----
        {
            const float ihw = hw * 0.90f;             // interior half width
            const float zFl = z0 + 0.03f;             // saloon floor
            const float zSt = zFl + 0.19f;            // mid step
            const float zFh = zFl + 0.38f;            // raised vestibule floor
            const float so = (base < gang) ? -1.0f : 1.0f; // toward the cab
            const float aisle = 0.52f;                // half aisle width
            auto plate = [&](float ya, float yb, float z, const glm::vec3& col) {
                emitBox(X, Y, Z, P(0.0f, 0.5f * (ya + yb), z - 0.025f), ihw,
                        0.5f * std::abs(yb - ya), 0.025f, col);
            };

            // Interior lining: an inset shell (walls + domed ceiling) facing the
            // aisle, so the inside carries its own colours — neutral walls, light
            // ceiling, dark glazing at the windows — independent of the exterior.
            auto liningRing = [&](float y) {
                return ring(y, 0.955f, 0.06f, zFl, bandLo(y), bandHi(y));
            };
            auto liningLoft = [&](const std::vector<glm::vec3>& A, const std::vector<glm::vec3>& B) {
                for (std::size_t k = 0; k + 1 < A.size(); ++k) {
                    const glm::vec3 a = A[k], b = A[k + 1], c = B[k + 1], d = B[k];
                    glm::vec3 n = glm::cross(b - a, d - a);
                    const float l = glm::length(n);
                    n = (l > 1e-9f) ? n / l : Z;
                    const glm::vec3 cen = 0.25f * (a + b + c + d);
                    const glm::vec3 target = P(0.0f, glm::dot(cen - f.pos, Y), 0.5f * (zFl + z1));
                    if (glm::dot(n, target - cen) < 0.0f) n = -n; // face the aisle
                    const glm::vec3 col = (k == 1 || k == 13) ? c93::kGlass
                                          : (k >= 3 && k <= 11) ? c93::kCeiling
                                                                : c93::kLining;
                    const glm::vec2 uv(0.0f);
                    const std::uint32_t vb = static_cast<std::uint32_t>(vertices_.size());
                    push(a, n, col); push(b, n, col); push(c, n, col); push(d, n, col);
                    indices_.push_back(vb + 0); indices_.push_back(vb + 1); indices_.push_back(vb + 2);
                    indices_.push_back(vb + 0); indices_.push_back(vb + 2); indices_.push_back(vb + 3);
                }
            };
            {
                const int steps = 24;
                std::vector<glm::vec3> prev = liningRing(bodyLo);
                for (int i = 1; i <= steps; ++i) {
                    std::vector<glm::vec3> cur =
                        liningRing(bodyLo + (bodyHi - bodyLo) * i / steps);
                    liningLoft(prev, cur);
                    prev = cur;
                }
            }

            // The cab end is raised (over the leading bogie); the low saloon floor
            // runs straight through the gangway (low-floor articulation). The step
            // is at the door; two risers with a mid tread step up toward the cab.
            const float yTop = yStep + so * 0.30f; // top of the steps (raised side)
            plate(base, yTop, zFh, c93::kFloor);   // raised cab vestibule
            plate(yStep, gang, zFl, c93::kFloor);  // saloon + low-floor gangway
            emitBox(X, Y, Z, P(0.0f, yStep, 0.5f * (zFl + zSt)), ihw, 0.02f, 0.5f * (zSt - zFl), c93::kStep);
            plate(yStep, yTop, zSt, c93::kStep);   // mid tread
            emitBox(X, Y, Z, P(0.0f, yTop, 0.5f * (zSt + zFh)), ihw, 0.02f, 0.5f * (zFh - zSt), c93::kStep);
            // Interior ceiling height at lateral x (follows the domed roof).
            auto ceilingAt = [&](float x) {
                const float ax = std::abs(x), zCrown = z1 - 0.12f;
                if (ax <= rhw) return zCrown;
                return zCrown + (zc - zCrown) * std::min(1.0f, (ax - rhw) / (hw - rhw));
            };
            // A floor-to-roof partition from a 4-point footprint; the top follows
            // the domed ceiling, walls oriented outward from the footprint centre.
            auto partition = [&](const std::array<glm::vec2, 4>& fp, float zFloor,
                                 const glm::vec3& col) {
                glm::vec2 c(0.0f);
                for (const glm::vec2& q : fp) c += q;
                const glm::vec3 ref = P(0.25f * c.x, 0.25f * c.y, zFloor + 1.0f);
                for (int i = 0; i < 4; ++i) {
                    const glm::vec2 a = fp[i], b = fp[(i + 1) % 4];
                    quadN(P(a.x, a.y, zFloor), P(b.x, b.y, zFloor),
                          P(b.x, b.y, ceilingAt(b.x)), P(a.x, a.y, ceilingAt(a.x)), col, ref);
                }
                quadN(P(fp[0].x, fp[0].y, ceilingAt(fp[0].x)), P(fp[1].x, fp[1].y, ceilingAt(fp[1].x)),
                      P(fp[2].x, fp[2].y, ceilingAt(fp[2].x)), P(fp[3].x, fp[3].y, ceilingAt(fp[3].x)), col, ref);
            };

            // Box in the stairs: full-height walls each side, right out to the
            // body sides (only the aisle steps through), saloon-facing end angled.
            for (const float sx : {1.0f, -1.0f})
                partition({glm::vec2(sx * aisle, yStep - so * 0.22f), glm::vec2(sx * ihw, yStep),
                           glm::vec2(sx * ihw, yTop), glm::vec2(sx * aisle, yTop)}, zFl, c93::kWall);

            // Cab partition: full-height side panels (domed top) either side of a
            // central aisle doorway, with a header over the door.
            const float wallY = base + so * 0.15f;
            for (const float sx : {1.0f, -1.0f})
                partition({glm::vec2(sx * aisle, wallY - 0.05f), glm::vec2(sx * ihw, wallY - 0.05f),
                           glm::vec2(sx * ihw, wallY + 0.05f), glm::vec2(sx * aisle, wallY + 0.05f)},
                          zFh, c93::kWall);
            const float doorTop = zFh + 1.95f, zCrown = z1 - 0.12f;
            emitBox(X, Y, Z, P(0.0f, wallY, 0.5f * (doorTop + zCrown)), aisle, 0.05f, 0.5f * (zCrown - doorTop), c93::kWall);

            // Full-height technical cabinets each side, behind the cab partition,
            // with the wall facing the door section angled a little.
            const float cabEnd = wallY - so * 0.1f, techLen = 1.7f, techDepth = 0.55f;
            const float doorEnd = cabEnd - so * techLen;
            for (const float sx : {1.0f, -1.0f})
                partition({glm::vec2(sx * ihw, cabEnd), glm::vec2(sx * ihw, doorEnd),
                           glm::vec2(sx * (ihw - techDepth), doorEnd + so * 0.35f),
                           glm::vec2(sx * (ihw - techDepth), cabEnd)}, zFh, c93::kWood);

            // WC / utility module (one car only): a full-height box on the +x
            // side of the low floor, directly beside the door; ends angled.
            if (hasWC)
                partition({glm::vec2(aisle, wc0 - so * 0.25f), glm::vec2(ihw, wc0),
                           glm::vec2(ihw, wc1), glm::vec2(aisle, wc1 + so * 0.25f)},
                          zFl, c93::kWood);

            // Seats: 2+2 rows through each saloon, oriented from the NSB seat
            // plans (the backrest is the darker end on the plan). Car A (no WC):
            // both aisle sides match, and only the end-most column of each section
            // is reversed to face inward (a facing bay against the cab / the
            // gangway). WC car: the two aisle sides face opposite ways — the +x
            // side toward the door, the -x side toward the car end except its
            // end-most column, which faces the door too.
            auto seat = [&](float x, float y, float zf, float face) {
                emitBox(X, Y, Z, P(x, y, zf + 0.42f), 0.22f, 0.24f, 0.05f, c93::kSeat);
                emitBox(X, Y, Z, P(x, y - face * 0.22f, zf + 0.72f), 0.22f, 0.05f, 0.28f,
                        c93::kSeat * 0.8f); // backrest a shade darker (shows facing)
            };
            {
                auto lerp = [&](float t) { return base + (gang - base) * t; };
                const float pitchT = 0.92f / std::abs(gang - base);
                std::vector<float> cabRows, gwRows; // seat rows, cab vs gangway section
                for (float t = 0.06f; t < 0.94f; t += pitchT) {
                    if (t < 0.15f) continue;              // tech behind cab
                    if (t > 0.32f && t < 0.40f) continue; // stairs
                    if (t > 0.42f && t < 0.51f) continue; // door
                    (t < 0.46f ? cabRows : gwRows).push_back(t);
                }
                // Facing for column `i` (0 = the section's end-most row) on aisle
                // side `sx`. toEnd faces the car end (cab / gangway); toDoor faces
                // the vestibule.
                auto faceFor = [&](int i, float sx, float toEnd, float toDoor) {
                    if (!hasWC)                          // car A: both sides match
                        return (i == 0) ? toDoor : toEnd;
                    return (sx > 0.0f) ? toDoor          // WC car +x: all to door
                                       : ((i == 0) ? toDoor : toEnd); // -x side
                };
                auto placeRow = [&](float t, int i, float toEnd, float toDoor) {
                    const float zf = (t < 0.34f) ? zFh : zFl;
                    const bool wcRegion = hasWC && t > 0.50f && t < 0.90f;
                    for (const float sx : {1.0f, -1.0f}) {
                        if (wcRegion && sx > 0.0f) continue; // WC on the +x side
                        if (wcRegion) {
                            // Opposite the WC: flip-up cushion on the window wall.
                            emitBox(X, Y, Z, P(-(ihw - 0.18f), lerp(t), zf + 0.42f),
                                    0.16f, 0.22f, 0.04f, c93::kSeat);
                            continue;
                        }
                        const float face = faceFor(i, sx, toEnd, toDoor);
                        seat(sx * 0.50f, lerp(t), zf, face); // aisle seat
                        seat(sx * 0.98f, lerp(t), zf, face); // window seat
                    }
                };
                for (std::size_t i = 0; i < cabRows.size(); ++i)
                    placeRow(cabRows[i], static_cast<int>(i), so, -so);
                for (std::size_t i = 0; i < gwRows.size(); ++i)
                    placeRow(gwRows[gwRows.size() - 1 - i], static_cast<int>(i), -so, so);
            }

            // Driver's cab (both ends, symmetric): a raised floor into the nose, a
            // central seat facing the windscreen and a raked desk in front of it.
            // No controls/gauges/screens yet. `so` points into the cab; the driver
            // faces `so`, toward the raked nose.
            {
                plate(base, base + so * 2.2f, zFh, c93::kFloor); // cab floor
                const float sy = base + so * 0.7f;               // seat centre
                seat(0.0f, sy, zFh, so);                         // driver's seat
                emitBox(X, Y, Z, P(0.0f, sy, zFh + 0.20f), 0.12f, 0.12f, 0.20f, c93::kDash); // pedestal
                for (const float sx : {1.0f, -1.0f})             // armrests
                    emitBox(X, Y, Z, P(sx * 0.30f, sy, zFh + 0.56f), 0.03f, 0.20f, 0.08f,
                            c93::kSeat * 0.7f);
                // Talent-style desk: a low flat table with the power/brake lever on
                // the right, and a taller three-facet instrument dome that wraps
                // toward the driver.
                const float dw = ihw * 0.78f;            // desk half-width
                const float xc = 0.30f;                  // centre-facet half-width
                const float zTab = zFh + 0.60f;          // table-top height (lowered)
                const float dyN = base + so * 1.02f;     // table edge near the driver
                const float dyD = base + so * 1.72f;     // table / dome junction
                const float dyF = base + so * 2.06f;     // dome back (toward glass)
                const float zDome = zFh + 1.12f;         // dome top (raised)
                const float rakeY = 0.16f;               // facet top forward rake
                const float wrap = 0.26f;                // side facets' wrap-back
                const glm::vec3 eye = P(0.0f, sy, zFh + 1.15f);            // driver's eye
                const glm::vec3 domeC = P(0.0f, dyD + so * 0.18f, 0.5f * (zTab + zDome));

                // Table top (solid dark console under it).
                emitBox(X, Y, Z, P(0.0f, 0.5f * (dyN + dyD), 0.5f * (zFh + zTab)),
                        dw, 0.5f * std::abs(dyD - dyN), 0.5f * (zTab - zFh), c93::kDash);

                // Live brake state drives the dials and the lever position.
                const float spd = vehicle.speed();
                const float mrP = vehicle.mrPressure();
                const float bcP = vehicle.bcPressure();
                const int cab = cabNegY ? 0 : 1;
                const int handle = vehicle.handlePosition(cab); // +brake / 0 / -power
                const float eng0 = vehicle.engineRpm(0) / Vehicle::kMaxRpm; // rev counter
                const float eng1 = vehicle.engineRpm(1) / Vehicle::kMaxRpm;

                // Combined power/brake lever on the driver's right of the table (the
                // two cabs face opposite ways, so -so keeps it right in both); the
                // stick swings toward the driver as the brake rises and away from the
                // driver (forward) as power rises.
                {
                    const float lx = -so * 0.54f, ly = dyN + so * 0.30f;
                    emitBox(X, Y, Z, P(lx, ly, zTab + 0.03f), 0.10f, 0.12f, 0.03f, c93::kButton); // base
                    const float tilt =
                        handle >= 0
                            ? 0.5f * static_cast<float>(handle) /
                                  static_cast<float>(Vehicle::kEmergencyNotch) // brake: toward driver
                            : -0.4f * static_cast<float>(-handle) /
                                  static_cast<float>(Vehicle::kMaxPowerNotch); // power: away (forward)
                    const glm::vec3 pv = P(lx, ly, zTab + 0.06f);
                    const glm::vec3 dir = Z * std::cos(tilt) - Y * (so * std::sin(tilt));
                    const glm::vec3 Yb = glm::normalize(glm::cross(dir, X));
                    const float L = 0.16f;
                    emitBox(X, Yb, dir, pv + dir * (0.5f * L), 0.025f, 0.025f, 0.5f * L, c93::kDash);   // stick
                    emitBox(X, Yb, dir, pv + dir * (L + 0.03f), 0.05f, 0.09f, 0.035f, c93::kButton);    // handle
                }

                // Reverser (R/N/F) handle on the driver's left (lx = so*0.54,
                // mirroring the brake). Smaller than the power lever; Neutral is
                // upright, Forward pushes away from the driver, Reverse pulls back
                // toward the driver (dir tilts fore/aft along ±Y).
                {
                    const int rev = vehicle.reverser(cab);
                    const float lx = so * 0.54f, ly = dyN + so * 0.24f;
                    emitBox(X, Y, Z, P(lx, ly, zTab + 0.03f), 0.07f, 0.09f, 0.03f, c93::kButton); // base
                    const float tilt = static_cast<float>(rev) * 0.35f; // +F away, -R toward driver
                    const glm::vec3 pv = P(lx, ly, zTab + 0.06f);
                    const glm::vec3 dir = Z * std::cos(tilt) + Y * (so * std::sin(tilt));
                    const glm::vec3 Yb = glm::normalize(glm::cross(dir, X));
                    const float L = 0.11f;
                    emitBox(X, Yb, dir, pv + dir * (0.5f * L), 0.02f, 0.02f, 0.5f * L, c93::kDash);   // stick
                    emitBox(X, Yb, dir, pv + dir * (L + 0.02f), 0.035f, 0.05f, 0.028f, c93::kButton); // knob
                }

                // Instrument dome: three distinct facets. The centre faces the
                // driver; the sides wrap back toward the driver so the panels are
                // clearly angled relative to one another.
                auto frontY = [&](float x) {
                    const float t = std::clamp((std::abs(x) - xc) / (dw - xc), 0.0f, 1.0f);
                    return dyD - so * wrap * t;
                };
                auto ptBot = [&](float x) { return P(x, frontY(x), zTab); };
                auto ptTop = [&](float x) { return P(x, frontY(x) + so * rakeY, zDome); };
                auto rectFace = [&](const glm::vec3& C, const glm::vec3& u, const glm::vec3& v,
                                    const glm::vec3& n, float cx, float cy, float hu, float hv,
                                    float pr, const glm::vec3& col) {
                    const glm::vec3 o = C + u * cx + v * cy + n * pr;
                    quadN(o - u * hu - v * hv, o + u * hu - v * hv, o + u * hu + v * hv,
                          o - u * hu + v * hv, col, o - n);
                };
                auto discFace = [&](const glm::vec3& C, const glm::vec3& u, const glm::vec3& v,
                                    const glm::vec3& n, float cx, float cy, float r, float pr,
                                    const glm::vec3& col) {
                    const glm::vec3 o = C + u * cx + v * cy + n * pr;
                    const int M = 16;
                    glm::vec3 prev = o + u * r;
                    for (int i = 1; i <= M; ++i) {
                        const float a = 2.0f * kPi * i / M;
                        const glm::vec3 cur = o + u * (r * std::cos(a)) + v * (r * std::sin(a));
                        quadN(o, prev, cur, cur, col, o - n);
                        prev = cur;
                    }
                };
                // A gauge needle: value fraction in [0,1] sweeps a 240-degree arc.
                auto needle = [&](const glm::vec3& C, const glm::vec3& u, const glm::vec3& v,
                                  const glm::vec3& n, float cx, float cy, float r, float frac,
                                  const glm::vec3& col) {
                    const glm::vec3 o = C + u * cx + v * cy + n * 0.016f;
                    const float a = kPi * (210.0f - 240.0f * std::clamp(frac, 0.0f, 1.0f)) / 180.0f;
                    const glm::vec3 dir = u * std::cos(a) + v * std::sin(a);
                    const glm::vec3 perp = -u * std::sin(a) + v * std::cos(a);
                    const glm::vec3 tip = o + dir * r;
                    quadN(o - perp * 0.006f, o + perp * 0.006f, tip, tip, col, o - n);
                };
                glm::vec3 C, u, v, n;
                auto facet = [&](float xa, float xb) {
                    const glm::vec3 BL = ptBot(xa), BR = ptBot(xb), TR = ptTop(xb), TL = ptTop(xa);
                    C = 0.25f * (BL + BR + TR + TL);
                    u = glm::normalize(BR - BL);
                    v = glm::normalize(TL - BL);
                    n = glm::normalize(glm::cross(u, v));
                    if (glm::dot(n, eye - C) < 0.0f) n = -n;
                    quadN(BL, BR, TR, TL, c93::kDash, C - n); // facet backing
                };

                // Close the dome shell behind the facets (top, back, sides, floor).
                const float xs[] = {-dw, -xc, xc, dw};
                for (int s = 0; s < 3; ++s)
                    quadN(ptTop(xs[s]), ptTop(xs[s + 1]), P(xs[s + 1], dyF, zDome),
                          P(xs[s], dyF, zDome), c93::kDash, domeC);
                quadN(P(-dw, dyF, zTab), P(dw, dyF, zTab), P(dw, dyF, zDome),
                      P(-dw, dyF, zDome), c93::kDash, domeC); // back
                quadN(P(-dw, dyD, zTab), P(dw, dyD, zTab), P(dw, dyF, zTab),
                      P(-dw, dyF, zTab), c93::kDash, domeC); // floor
                quadN(ptBot(-dw), ptTop(-dw), P(-dw, dyF, zDome), P(-dw, dyF, zTab), c93::kDash, domeC);
                quadN(ptBot(dw), ptTop(dw), P(dw, dyF, zDome), P(dw, dyF, zTab), c93::kDash, domeC);

                // Left facet: an MFD LCD screen framed by rectangular buttons; on the
                // screen, a vertical engine tachometer bar per engine (rpm as a
                // fraction of full power, filled from the bottom).
                facet(-dw, -xc);
                rectFace(C, u, v, n, 0.0f, 0.02f, 0.15f, 0.13f, 0.01f, c93::kScreen);
                {
                    const float y0 = -0.08f, y1 = 0.11f, bw = 0.035f; // bar extent / half-width
                    const std::pair<float, float> bars[] = {{-0.055f, eng0}, {0.055f, eng1}};
                    for (const auto& b : bars) {
                        rectFace(C, u, v, n, b.first, 0.5f * (y0 + y1), bw, 0.5f * (y1 - y0),
                                 0.013f, c93::kDash); // dark track
                        const float fill = glm::clamp(b.second, 0.0f, 1.0f) * (y1 - y0);
                        // Always emit (a zero-height, invisible quad when empty) so the
                        // vehicle mesh keeps a CONSTANT vertex/index count: the
                        // renderer's vehicle buffer is fixed-size and drops any update
                        // whose size changed, which would freeze the moving train.
                        rectFace(C, u, v, n, b.first, y0 + 0.5f * fill, bw * 0.8f,
                                 0.5f * fill, 0.015f, c93::kEngBar); // fill from bottom
                    }
                }
                for (int i = 0; i < 3; ++i) {
                    const float yy = 0.13f - 0.13f * i;
                    rectFace(C, u, v, n, -0.22f, yy, 0.02f, 0.025f, 0.012f, c93::kButton);
                    rectFace(C, u, v, n, 0.22f, yy, 0.02f, 0.025f, 0.012f, c93::kButton);
                }

                // Centre facet: two round analog dials over a row of buttons. Left
                // dial = speedometer; right dial = duplex air gauge (main reservoir
                // red needle + brake cylinder dark needle). Needles track the sim.
                facet(-xc, xc);
                discFace(C, u, v, n, -0.12f, 0.06f, 0.09f, 0.01f, c93::kGauge);
                discFace(C, u, v, n, 0.12f, 0.06f, 0.09f, 0.01f, c93::kGauge);
                needle(C, u, v, n, -0.12f, 0.06f, 0.076f, spd / 40.0f, c93::kDash);  // speed (0..40 m/s)
                needle(C, u, v, n, 0.12f, 0.06f, 0.072f, mrP / 10.0f, c93::kRed);    // reservoir
                needle(C, u, v, n, 0.12f, 0.06f, 0.078f, bcP / 10.0f, c93::kDash);   // brake cylinder
                for (const float gx : {-0.12f, 0.12f})
                    discFace(C, u, v, n, gx, 0.06f, 0.015f, 0.02f, c93::kDash); // hubs (over needle roots)
                for (int i = 0; i < 5; ++i)
                    rectFace(C, u, v, n, -0.18f + 0.09f * i, -0.14f, 0.028f, 0.022f, 0.012f, c93::kButton);

                // Right facet: signalling / ATC panel — a screen and buttons.
                facet(xc, dw);
                rectFace(C, u, v, n, 0.0f, 0.05f, 0.14f, 0.09f, 0.01f, c93::kScreen);
                for (int i = 0; i < 4; ++i)
                    rectFace(C, u, v, n, -0.15f + 0.10f * i, -0.11f, 0.03f, 0.025f, 0.012f, c93::kButton);
            }
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
                emitClass93(sections[i], halfLen, i == 0, i == 1); // WC in the 2nd car
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
                // Low-floor bridge across the gap so the aisle walks through.
                quadN(A[3], A[0], B[0], B[3], c93::kFloor,
                      mid - glm::vec3(0.0f, 0.0f, 2.0f));
            }
        }
    }

    // Split the glazing out into a trailing, transparent run so it can be drawn
    // after the opaque geometry with alpha blending. Glass is the exterior window
    // band / windscreen (kBand) and the interior glazing (kGlass); its vertices are
    // tagged with a texLayer sentinel the track shader reads as "translucent".
    {
        constexpr float kGlassLayer = -2.0f;
        auto isGlass = [](const glm::vec3& c) {
            auto eq = [&](const glm::vec3& g) {
                return std::abs(c.x - g.x) < 0.005f && std::abs(c.y - g.y) < 0.005f &&
                       std::abs(c.z - g.z) < 0.005f;
            };
            return eq(c93::kBand) || eq(c93::kGlass);
        };
        std::vector<std::uint32_t> opaque, glass;
        opaque.reserve(indices_.size());
        for (std::size_t i = 0; i + 2 < indices_.size(); i += 3) {
            const std::uint32_t a = indices_[i], b = indices_[i + 1], c = indices_[i + 2];
            if (isGlass(vertices_[a].color) && isGlass(vertices_[b].color) &&
                isGlass(vertices_[c].color)) {
                vertices_[a].texLayer = vertices_[b].texLayer = vertices_[c].texLayer =
                    kGlassLayer;
                glass.push_back(a); glass.push_back(b); glass.push_back(c);
            } else {
                opaque.push_back(a); opaque.push_back(b); opaque.push_back(c);
            }
        }
        glassFirstIndex_ = static_cast<std::uint32_t>(opaque.size());
        opaque.insert(opaque.end(), glass.begin(), glass.end());
        indices_ = std::move(opaque);
    }
}

namespace drivercam {

int count(const Vehicle& v) {
    return (v.bodyStyle() == BodyClass93 && v.bodySectionFrames().size() >= 2) ? 2 : 0;
}

// Mirrors the cab geometry in emitClass93: the seat sits at base + so*0.7 on the
// raised vestibule floor, facing the nose (so). Uses the same frame-height stack.
bool eyePose(const Vehicle& v, int position, glm::vec3& eye, glm::vec3& forward) {
    const std::vector<VehicleFrame> sections = v.bodySectionFrames();
    if (v.bodyStyle() != BodyClass93 || position < 0 ||
        position >= static_cast<int>(sections.size()))
        return false;
    const VehicleFrame& f = sections[position];
    const float halfLen = 0.5f * v.length() / static_cast<float>(sections.size());
    const bool cabNegY = (position == 0);
    const float so = cabNegY ? -1.0f : 1.0f;                       // toward this cab
    const float base = cabNegY ? -halfLen + c93::kNoseLen : halfLen - c93::kNoseLen;
    const float sy = base + so * 0.7f;                             // seat centre
    const float frameCentreZ = wheelset::kAxleCentreAboveBed + wheelset::kWheelRadius;
    const float zFh = frameCentreZ + kFrameHalfHeight + c93::kFloorAbove + 0.41f; // cab floor
    eye = f.pos + f.tangent * sy + f.up * (zFh + 1.4f);            // seated eye height
    forward = f.tangent * so;                                     // out the windscreen
    return true;
}

} // namespace drivercam
