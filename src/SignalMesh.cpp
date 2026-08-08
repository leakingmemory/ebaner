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

#include "SignalMesh.h"

#include <cmath>

namespace {
const glm::vec3 kBody{0.10f, 0.10f, 0.11f};   // near-black cast housing
const glm::vec3 kLampOn{1.0f, 0.90f, 0.62f};  // warm incandescent white
const glm::vec3 kLampOff{0.20f, 0.20f, 0.20f}; // dark, unlit lamp position
const glm::vec3 kRedOn{1.0f, 0.14f, 0.10f};    // main signal: danger
const glm::vec3 kRedOff{0.24f, 0.10f, 0.10f};
const glm::vec3 kGreenOn{0.15f, 1.0f, 0.35f};  // main signal: proceed
const glm::vec3 kGreenOff{0.10f, 0.22f, 0.13f};
const glm::vec3 kAmberOn{1.0f, 0.62f, 0.05f};  // distant signal: expect stop
const glm::vec3 kAmberOff{0.26f, 0.17f, 0.06f};
} // namespace

void SignalMesh::build(const std::vector<SignalPlacement>& signals, glm::dvec3 origin) {
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
    // Box with orthonormal local axes r/f/u and half-extents hr/hf/hu about centre c.
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
    // A filled disc facing +n, in the (w,up) plane - a lamp lens.
    auto disc = [&](const glm::vec3& c, const glm::vec3& n, const glm::vec3& w,
                    const glm::vec3& up, float rad, const glm::vec3& col) {
        constexpr int N = 12;
        for (int i = 0; i < N; ++i) {
            const float a0 = 6.2831853f * i / N, a1 = 6.2831853f * (i + 1) / N;
            const glm::vec3 p0 = c + (w * std::cos(a0) + up * std::sin(a0)) * rad;
            const glm::vec3 p1 = c + (w * std::cos(a1) + up * std::sin(a1)) * rad;
            tri(c, p0, p1, n, col);
        }
    };

    // A lit main-signal lens. Same disc, but tagged emissive for the shader: it carries
    // its own centre in the normal slot (unused once the sun term is skipped) so the
    // shader can hold it to a minimum apparent size, which is what keeps a signal
    // readable from far down the line.
    auto lamp = [&](const glm::vec3& c, const glm::vec3& w, const glm::vec3& up, float rad,
                    const glm::vec3& col, bool flashing = false) {
        // -3 steady, -4 flashing: the shader blinks the latter and, crucially, leaves it at
        // true scale while dark so it does not bloom into a blob at range.
        const float kLampLayer = flashing ? -4.0f : -3.0f;
        constexpr int N = 12;
        for (int i = 0; i < N; ++i) {
            const float a0 = 6.2831853f * i / N, a1 = 6.2831853f * (i + 1) / N;
            const glm::vec3 p0 = c + (w * std::cos(a0) + up * std::sin(a0)) * rad;
            const glm::vec3 p1 = c + (w * std::cos(a1) + up * std::sin(a1)) * rad;
            const std::uint32_t base = static_cast<std::uint32_t>(vertices_.size());
            vertices_.push_back({c, c, col, {0.0f, 0.0f}, kLampLayer});
            vertices_.push_back({p0, c, col, {0.0f, 0.0f}, kLampLayer});
            vertices_.push_back({p1, c, col, {0.0f, 0.0f}, kLampLayer});
            indices_.push_back(base + 0);
            indices_.push_back(base + 1);
            indices_.push_back(base + 2);
        }
    };

    const glm::vec3 UP(0.0f, 0.0f, 1.0f);
    for (const SignalPlacement& s : signals) {
        const glm::dvec2 f2 = glm::length(s.forward) > 1e-9 ? glm::normalize(s.forward)
                                                            : glm::dvec2(1.0, 0.0);
        const glm::vec3 F(static_cast<float>(f2.x), static_cast<float>(f2.y), 0.0f);
        const glm::vec3 R(F.y, -F.x, 0.0f); // right of the travel direction
        // Base: on the ground, offset to the right of the track; scene-relative.
        const glm::vec3 B(static_cast<float>(s.world.x - origin.x) + R.x * 3.5f,
                          static_cast<float>(s.world.y - origin.y) + R.y * 3.5f,
                          static_cast<float>(s.world.z - origin.z));

        // A distant signal is its own thing: a mast with two lamps, amber over green, both
        // flashing, and no red - it warns, it never commands a stop. Its aspect values are
        // the main's read as "expect that".
        if (s.kind == SignalKind::Distant) {
            const float mastH = 3.5f;
            box(B + UP * (mastH * 0.5f), R, F, UP, 0.06f, 0.06f, mastH * 0.5f, kBody);
            const float hw = 0.26f, hd = 0.14f, hh = 0.48f; // head half extents
            const glm::vec3 C = B + UP * (mastH + hh);
            box(C, R, F, UP, hw, hd, hh, kBody);
            box(C - F * (hd + 0.01f), R, F, UP, hw * 1.25f, 0.02f, hh * 1.1f, kBody);
            const glm::vec3 n = -F;
            const glm::vec3 face = C - F * (hd + 0.04f);
            const float r = 0.13f, sp = 0.22f; // lens radius, half the lamp spacing
            // Expect stop (or nothing reachable) is amber; expect C1 is green; expect a
            // clear that is not C1 is both.
            const bool amber = s.aspect == SignalAspect::Stop ||
                               s.aspect == SignalAspect::ClearReduced;
            const bool green = s.aspect == SignalAspect::Clear ||
                               s.aspect == SignalAspect::ClearReduced;
            if (amber) lamp(face + UP * sp, R, UP, r, kAmberOn, true); // top: amber
            else disc(face + UP * sp, n, R, UP, r, kAmberOff);
            if (green) lamp(face - UP * sp, R, UP, r, kGreenOn, true); // bottom: green
            else disc(face - UP * sp, n, R, UP, r, kGreenOff);
            continue;
        }

        // How high the dwarf box sits: on its own short post normally, or low on the
        // main signal's mast when the two share a pole.
        const bool isExit = s.kind != SignalKind::Dwarf; // exit or entry: same head
        const bool drawDwarf = !isExit || s.withDwarf;
        const float dwarfH = 1.0f;

        if (isExit) {
            // Main signal: a tall mast carrying a head with three lamps in a vertical line
            // - top green, middle red, bottom green. Danger lights the middle red only.
            const float mastH = 4.5f;
            box(B + UP * (mastH * 0.5f), R, F, UP, 0.07f, 0.07f, mastH * 0.5f, kBody);
            const float hw = 0.30f, hd = 0.15f, hh = 0.72f; // head half extents
            const glm::vec3 C = B + UP * (mastH + hh);
            box(C, R, F, UP, hw, hd, hh, kBody);
            // Backing plate, so the head reads against the sky from a distance.
            box(C - F * (hd + 0.01f), R, F, UP, hw * 1.25f, 0.02f, hh * 1.1f, kBody);
            const glm::vec3 n = -F;
            const glm::vec3 face = C - F * (hd + 0.04f);
            const float r = 0.13f, sp = 0.40f; // lens radius, vertical lamp spacing
            // Norwegian main-signal aspects: red alone is stop; the upper green alone is
            // proceed over a deviation (C2); both greens is proceed, no restriction (C1).
            const bool danger = s.aspect == SignalAspect::Stop;
            const bool proceed = s.aspect == SignalAspect::Clear ||
                                 s.aspect == SignalAspect::ClearReduced;
            const bool noRestriction = s.aspect == SignalAspect::Clear;
            // Only a lit lens is emissive; a dark lens is ordinary shaded geometry, so it
            // fades out with the head instead of blooming into a dark blob at range.
            if (proceed) lamp(face + UP * sp, R, UP, r, kGreenOn);   // top: green
            else disc(face + UP * sp, n, R, UP, r, kGreenOff);
            // An entry signal shows its danger as a flashing red; every other lamp on the
            // head, and every lamp on an exit signal, is steady.
            if (danger) lamp(face, R, UP, r, kRedOn, s.kind == SignalKind::Entry);
            else disc(face, n, R, UP, r, kRedOff);
            if (noRestriction) lamp(face - UP * sp, R, UP, r, kGreenOn); // bottom: green
            else disc(face - UP * sp, n, R, UP, r, kGreenOff);
        }

        if (drawDwarf) {
            // Dwarf: its own short post when alone; when sharing the exit's mast it just
            // hangs low on that mast, so only the housing is drawn.
            if (!isExit)
                box(B + UP * (dwarfH * 0.5f), R, F, UP, 0.035f, 0.035f, dwarfH * 0.5f, kBody);
            const float hw = 0.30f, hd = 0.16f, hh = 0.28f; // half extents (across/along/up)
            const glm::vec3 C = B + UP * (dwarfH + hh);
            // Offset a shared dwarf clear of the mast so both heads are visible.
            const glm::vec3 Cd = isExit ? C + R * 0.30f : C;
            box(Cd, R, F, UP, hw, hd, hh, kBody);

            // Lamp face toward the approaching driver (normal -F, just proud of the housing).
            const glm::vec3 n = -F;
            const glm::vec3 face = Cd - F * (hd + 0.02f);
            // Face-plane axes: `a` = the driver's rightward axis (their right is +R; offsets
            // are in the driver's own frame, so no mirror flip), `up` = vertical. The two Stop
            // lamps form a horizontal pair low on the face (reference left + Stop right); the
            // 45 deg and vertical arc positions are drawn as dark, unlit spots.
            const glm::vec3 a = R;
            const float d = 0.16f, r = 0.055f, lo = -0.10f;
            const glm::vec3 ref = face + a * (-d * 0.5f) + UP * lo; // reference (lower-left)
            // The arc lamp that is lit picks out the aspect; the other two stay dark. A
            // shared dwarf shows its own indication, not the exit head's.
            const SignalAspect da = isExit ? s.dwarfAspect : s.aspect;
            const bool stop = da == SignalAspect::Stop;
            const bool train = da == SignalAspect::TrainOnTrack;
            const bool clear = da == SignalAspect::Clear;
            disc(ref, n, a, UP, r, kLampOn);                            // reference: always on
            disc(ref + a * d, n, a, UP, r, stop ? kLampOn : kLampOff);  // horizontal = stop
            disc(ref + (a + UP) * (d * 0.70711f), n, a, UP, r,          // 45 deg = train ahead
                 train ? kLampOn : kLampOff);
            disc(ref + UP * d, n, a, UP, r, clear ? kLampOn : kLampOff); // vertical = clear
        }
    }
}
