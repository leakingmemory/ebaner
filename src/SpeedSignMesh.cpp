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

#include "SpeedSignMesh.h"

#include "Font.h"

#include <string>

namespace {
const glm::vec3 kPost{0.20f, 0.20f, 0.21f};  // galvanised post, near-black at distance
const glm::vec3 kAmber{1.00f, 0.62f, 0.05f}; // the sign face
const glm::vec3 kInk{0.05f, 0.04f, 0.03f};   // the numeral, and the blank back
constexpr float kSide = 3.0f;                // m right of the track centre
constexpr float kPostH = 2.0f;               // m to the foot of the triangle
constexpr float kTriW = 1.30f;               // m across the base
constexpr float kTriH = 1.15f;               // m base to apex
constexpr float kMarkerScale = 0.45f;        // the border marker is a smaller plate
// Where a marker rides on the back of an increase sign, its base sits this far up that
// plate. An apex-down triangle is widest at its base, so hung level with the apex it would
// stick out either side of the plate carrying it; down here the silhouette is wide enough to
// contain it, and it reads as the small plate on the back that it is.
constexpr float kBackMarkerAt = 0.52f;
} // namespace

void SpeedSignMesh::build(const std::vector<SpeedSign>& signs) {
    vertices_.clear();
    indices_.clear();

    auto tri = [&](const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                   const glm::vec3& n, const glm::vec3& c) {
        const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
        vertices_.push_back({p0, n, c, {0.0f, 0.0f}, -1.0f});
        vertices_.push_back({p1, n, c, {0.0f, 0.0f}, -1.0f});
        vertices_.push_back({p2, n, c, {0.0f, 0.0f}, -1.0f});
        indices_.push_back(base + 0);
        indices_.push_back(base + 1);
        indices_.push_back(base + 2);
    };
    auto quad = [&](const glm::vec3& p0, const glm::vec3& p1, const glm::vec3& p2,
                    const glm::vec3& p3, const glm::vec3& n, const glm::vec3& c) {
        tri(p0, p1, p2, n, c);
        tri(p0, p2, p3, n, c);
    };
    // Box with orthonormal local axes r/f/u and half-extents about centre c.
    auto box = [&](const glm::vec3& c, const glm::vec3& r, const glm::vec3& f,
                   const glm::vec3& u, float hr, float hf, float hu, const glm::vec3& col) {
        const glm::vec3 R = r * hr, F = f * hf, U = u * hu;
        auto P = [&](float sr, float sf, float su) { return c + R * sr + F * sf + U * su; };
        quad(P(1, -1, -1), P(1, 1, -1), P(1, 1, 1), P(1, -1, 1), r, col);
        quad(P(-1, 1, -1), P(-1, -1, -1), P(-1, -1, 1), P(-1, 1, 1), -r, col);
        quad(P(1, 1, -1), P(-1, 1, -1), P(-1, 1, 1), P(1, 1, 1), f, col);
        quad(P(-1, -1, -1), P(1, -1, -1), P(1, -1, 1), P(-1, -1, 1), -f, col);
        quad(P(-1, -1, 1), P(1, -1, 1), P(1, 1, 1), P(-1, 1, 1), u, col);
        quad(P(-1, 1, -1), P(1, 1, -1), P(1, -1, -1), P(-1, -1, -1), -u, col);
    };

    const glm::vec3 UP(0.0f, 0.0f, 1.0f);
    for (const SpeedSign& s : signs) {
        // The sign faces the driver it applies to, so its face normal is back along the
        // travel direction; it stands to the right of that direction.
        const glm::vec3 F(s.tangent.x, s.tangent.y, 0.0f);
        const glm::vec3 R(s.right.x, s.right.y, 0.0f);
        const glm::vec3 base = s.pos + R * kSide;
        const glm::vec3 n = -F;
        const glm::vec3 out = F * 0.03f; // plate thickness, so the two faces do not z-fight

        // A marker is a small plate and needs less post under it; the other two are full size.
        const bool marker = s.kind == SpeedSignKind::ReductionMarker;
        const float scale = marker ? kMarkerScale : 1.0f;
        const float postH = marker ? kPostH + kTriH * (1.0f - kMarkerScale) : kPostH;
        box(base + UP * (postH * 0.5f), R, F, UP, 0.045f, 0.045f, postH * 0.5f, kPost);

        // A triangle standing on `foot`, apex up or down, drawn toward `nrm`. The three signs
        // differ in which way the apex points and how big the plate is, nothing else.
        auto plate = [&](const glm::vec3& foot, float w, float h, bool apexUp,
                         const glm::vec3& nrm, const glm::vec3& off, const glm::vec3& col) {
            const glm::vec3 up = UP * (apexUp ? h : -h);
            const glm::vec3 apex = foot + up;
            const glm::vec3 l = foot - R * (w * 0.5f);
            const glm::vec3 r = foot + R * (w * 0.5f);
            // Wound so the face points along `nrm`: swapping the base corners flips it.
            if (glm::dot(nrm, n) > 0.0f) tri(apex + off, l + off, r + off, nrm, col);
            else tri(apex + off, r + off, l + off, nrm, col);
        };
        // Where the plate's base sits, and which way its apex points.
        const bool apexUp = s.kind == SpeedSignKind::Increase;
        const float triW = kTriW * scale, triH = kTriH * scale;
        // An apex-down plate hangs from the top of the post, so its base is a plate higher.
        const glm::vec3 foot = base + UP * (apexUp ? postH : postH + triH);

        plate(foot, triW, triH, apexUp, n, -out, kAmber);
        // The back is inset a little: sharing the silhouette exactly, it peeks out past the
        // amber edge by a pixel wherever the two rasterise differently off-axis.
        const glm::vec3 mid = foot + UP * ((apexUp ? triH : -triH) / 3.0f);
        auto in = [&](const glm::vec3& p) { return mid + (p - mid) * 0.94f; };
        plate(in(foot), triW * 0.94f, triH * 0.94f, apexUp, F, out, kInk);
        // A rise one way is a drop the other, so where the two coincide this post carries the
        // opposing direction's reduction marker as its back face rather than a blank plate.
        if (s.backMarker) {
            const glm::vec3 mfoot = base + UP * (postH + triH * kBackMarkerAt);
            plate(mfoot, kTriW * kMarkerScale, kTriH * kMarkerScale, false, F,
                  out + F * 0.01f, kAmber);
        }

        // The numeral: the limit in tens, in the same 8x8 font the HUD uses, one small quad
        // per lit pixel just proud of the face. It sits toward the plate's wide edge - two
        // digits out near the apex would run off the sloping sides. A marker carries none:
        // it says only "the limit changes here", the warning already said what to.
        if (marker) continue;
        const std::string txt = std::to_string(s.kmh / 10);
        constexpr float kGlyph = 0.26f;      // m, one 8x8 cell
        const float px = kGlyph / 8.0f;      // one font pixel
        const float wide = txt.size() * kGlyph;
        const glm::vec3 ink = out + F * 0.01f; // in front of the amber face
        // Measured from the base, which is the bottom of an apex-up plate and the top of an
        // apex-down one - so the numeral hangs below the base in the latter.
        const glm::vec3 origin = foot - R * (wide * 0.5f) +
                                 UP * (apexUp ? triH * 0.16f : -(triH * 0.16f + kGlyph));
        for (std::size_t g = 0; g < txt.size(); ++g) {
            const unsigned char* bits = fontGlyph(txt[g]);
            if (!bits) continue;
            const glm::vec3 gx = origin + R * (static_cast<float>(g) * kGlyph);
            for (int row = 0; row < 8; ++row)
                for (int col = 0; col < 8; ++col) {
                    if (!((bits[row] >> col) & 1)) continue;
                    // Font rows run downward; the sign's rows run up.
                    const glm::vec3 p = gx + R * (static_cast<float>(col) * px) +
                                        UP * (static_cast<float>(7 - row) * px);
                    quad(p - ink, p + R * px - ink, p + R * px + UP * px - ink,
                         p + UP * px - ink, n, kInk);
                }
        }
    }
}
