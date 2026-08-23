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

#include "LampGeometry.h"

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
// Head half-heights. A main signal's three lamps need the taller head; two lamps sit in the
// shorter one, which is what the simple station signals and the siding masts carry.
constexpr float kThreeLampHalfH = 0.72f;
constexpr float kTwoLampHalfH = 0.52f;
// How far from the track centre a post stands, whichever side it is on.
constexpr float kStandoffM = 3.5f;
} // namespace

void SignalMesh::build(const std::vector<SignalPlacement>& signals, glm::dvec3 origin) {
    vertices_.clear();
    indices_.clear();

    // The drawing vocabulary lives in LampGeometry.h so the signals and the level
    // crossings share one copy of it - above all one copy of how a lit lens is tagged.
    auto tri = [&](const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
                   const glm::vec3& n, const glm::vec3& col) {
        lampgeom::tri(vertices_, indices_, a, b, c, n, col);
    };
    auto box = [&](const glm::vec3& c, const glm::vec3& r, const glm::vec3& f,
                   const glm::vec3& u, float hr, float hf, float hu, const glm::vec3& col) {
        lampgeom::box(vertices_, indices_, c, r, f, u, hr, hf, hu, col);
    };
    auto disc = [&](const glm::vec3& c, const glm::vec3& n, const glm::vec3& w,
                    const glm::vec3& up, float rad, const glm::vec3& col) {
        lampgeom::disc(vertices_, indices_, c, n, w, up, rad, col);
    };
    // Every signal here blinks at the one rate the signalling uses, all in step.
    auto lamp = [&](const glm::vec3& c, const glm::vec3& w, const glm::vec3& up, float rad,
                    const glm::vec3& col, bool flashing = false) {
        lampgeom::lamp(vertices_, indices_, c, w, up, rad, col,
                       flashing ? kSignalBlinkS : 0.0f, 0.0f);
    };

    const glm::vec3 UP(0.0f, 0.0f, 1.0f);
    // The distant head, wherever it hangs: on its own post out on the line, or on a main
    // signal's mast. One copy, so the two cannot drift apart. `C` is the housing centre.
    // Amber over green, both flashing, and no red - it warns, it never commands a stop.
    // Its aspect values are the main's, read as "expect that"; Dark needs no case of its
    // own, since both tests fail and both lenses fall through to their unlit discs, which
    // is exactly what a switched-off signal looks like.
    auto distantHead = [&](const glm::vec3& C, const glm::vec3& R, const glm::vec3& F,
                           SignalAspect aspect) {
        const float hw = 0.26f, hd = 0.14f, hh = 0.48f; // head half extents
        box(C, R, F, UP, hw, hd, hh, kBody);
        box(C - F * (hd + 0.01f), R, F, UP, hw * 1.25f, 0.02f, hh * 1.1f, kBody);
        const glm::vec3 n = -F;
        const glm::vec3 face = C - F * (hd + 0.04f);
        const float r = 0.13f, sp = 0.22f; // lens radius, half the lamp spacing
        // Expect stop (or nothing reachable) is amber; expect C1 is green; expect a clear
        // that is not C1 is both.
        const bool amber =
            aspect == SignalAspect::Stop || aspect == SignalAspect::ClearReduced;
        const bool green =
            aspect == SignalAspect::Clear || aspect == SignalAspect::ClearReduced;
        if (amber) lamp(face + UP * sp, R, UP, r, kAmberOn, true); // top: amber
        else disc(face + UP * sp, n, R, UP, r, kAmberOff);
        if (green) lamp(face - UP * sp, R, UP, r, kGreenOn, true); // bottom: green
        else disc(face - UP * sp, n, R, UP, r, kGreenOff);
    };

    // A head with two lamps, red over green: stop, or go. That is the whole vocabulary of a
    // simple station signal, and it is also the head a siding's own main signal carries -
    // one green, lit for any clearance, because there is no second lamp to tell C1 from C2
    // with. Dark needs no case of its own; both tests fail and both lenses fall through.
    // Only an entry signal's danger flashes, which is what `flashDanger` carries.
    auto twoLampHead = [&](const glm::vec3& C, const glm::vec3& R, const glm::vec3& F,
                           SignalAspect aspect, bool flashDanger) {
        const float hw = 0.30f, hd = 0.15f, hh = kTwoLampHalfH; // head half extents
        box(C, R, F, UP, hw, hd, hh, kBody);
        box(C - F * (hd + 0.01f), R, F, UP, hw * 1.25f, 0.02f, hh * 1.1f, kBody);
        const glm::vec3 n = -F;
        const glm::vec3 face = C - F * (hd + 0.04f);
        const float r = 0.13f, sp = 0.26f; // lens radius, half the lamp spacing
        const bool danger = aspect == SignalAspect::Stop;
        const bool proceed =
            aspect == SignalAspect::Clear || aspect == SignalAspect::ClearReduced;
        if (danger) lamp(face + UP * sp, R, UP, r, kRedOn, flashDanger); // top: red
        else disc(face + UP * sp, n, R, UP, r, kRedOff);
        if (proceed) lamp(face - UP * sp, R, UP, r, kGreenOn); // bottom: green
        else disc(face - UP * sp, n, R, UP, r, kGreenOff);
    };

    for (const SignalPlacement& s : signals) {
        const glm::dvec2 f2 = glm::length(s.forward) > 1e-9 ? glm::normalize(s.forward)
                                                            : glm::dvec2(1.0, 0.0);
        const glm::vec3 F(static_cast<float>(f2.x), static_cast<float>(f2.y), 0.0f);
        const glm::vec3 R(F.y, -F.x, 0.0f); // right of the travel direction
        // Base: on the ground, offset to one side of the track; scene-relative. Only the
        // offset takes the side - R itself stays the right of travel, because it is also
        // the basis vector every head and housing below is built on and mirroring that
        // would turn the signal inside out rather than move it across the line.
        const float off = kStandoffM * static_cast<float>(s.side < 0 ? -1 : 1);
        const glm::vec3 B(static_cast<float>(s.world.x - origin.x) + R.x * off,
                          static_cast<float>(s.world.y - origin.y) + R.y * off,
                          static_cast<float>(s.world.z - origin.z));

        // A distant signal on its own post out on the line: a short mast under the head.
        if (s.kind == SignalKind::Distant) {
            const float mastH = 3.5f;
            box(B + UP * (mastH * 0.5f), R, F, UP, 0.06f, 0.06f, mastH * 0.5f, kBody);
            distantHead(B + UP * (mastH + 0.48f), R, F, s.aspect);
            continue;
        }

        // A simple station signal is a main signal's mast under that two-lamp head: it can
        // only stop you or let you by, and nothing on it flashes - there is no third thing
        // for a flash to distinguish it from. Dark (an unmanned station) lights neither.
        if (s.kind == SignalKind::StationEntry) {
            const float mastH = 4.5f;
            box(B + UP * (mastH * 0.5f), R, F, UP, 0.07f, 0.07f, mastH * 0.5f, kBody);
            twoLampHead(B + UP * (mastH + kTwoLampHalfH), R, F, s.aspect, false);
            continue;
        }

        // How high the dwarf box sits: on its own short post normally, or low on the
        // main signal's mast when the two share a pole.
        const bool isExit = s.kind != SignalKind::Dwarf; // any main: the same head
        const bool drawDwarf = !isExit || s.withDwarf;
        const float dwarfH = 1.0f;

        if (isExit) {
            // Main signal: a tall mast carrying a head with three lamps in a vertical line
            // - top green, middle red, bottom green. Danger lights the middle red only.
            //
            // A siding's own signal is built with the two-lamp head instead. It is the same
            // signal in every other way - same mast, same routes, and it may still share a
            // pole with a dwarf or carry a distant - so only the head differs.
            const float mastH = 4.5f;
            box(B + UP * (mastH * 0.5f), R, F, UP, 0.07f, 0.07f, mastH * 0.5f, kBody);
            const float hd = 0.15f;
            const float hh = s.twoLamp ? kTwoLampHalfH : kThreeLampHalfH;
            const glm::vec3 C = B + UP * (mastH + hh);
            // An entry signal shows its danger as a flashing red; every other lamp on the
            // head, and every lamp on an exit signal, is steady.
            const bool flashDanger = s.kind == SignalKind::Entry;
            if (s.twoLamp) {
                twoLampHead(C, R, F, s.aspect, flashDanger);
            } else {
                const float hw = 0.30f;
                box(C, R, F, UP, hw, hd, hh, kBody);
                // Backing plate, so the head reads against the sky from a distance.
                box(C - F * (hd + 0.01f), R, F, UP, hw * 1.25f, 0.02f, hh * 1.1f, kBody);
                const glm::vec3 n = -F;
                const glm::vec3 face = C - F * (hd + 0.04f);
                const float r = 0.13f, sp = 0.40f; // lens radius, vertical lamp spacing
                // Norwegian main-signal aspects: red alone is stop; the upper green alone
                // is proceed over a deviation (C2); both greens is proceed, no restriction
                // (C1). Dark lights nothing: every lens falls through to its unlit disc.
                const bool danger = s.aspect == SignalAspect::Stop;
                const bool proceed = s.aspect == SignalAspect::Clear ||
                                     s.aspect == SignalAspect::ClearReduced;
                const bool noRestriction = s.aspect == SignalAspect::Clear;
                // Only a lit lens is emissive; a dark lens is ordinary shaded geometry, so
                // it fades out with the head instead of blooming into a dark blob at range.
                if (proceed) lamp(face + UP * sp, R, UP, r, kGreenOn);   // top: green
                else disc(face + UP * sp, n, R, UP, r, kGreenOff);
                if (danger) lamp(face, R, UP, r, kRedOn, flashDanger);
                else disc(face, n, R, UP, r, kRedOff);
                if (noRestriction) lamp(face - UP * sp, R, UP, r, kGreenOn); // bottom green
                else disc(face - UP * sp, n, R, UP, r, kGreenOff);
            }

            // A distant on the same mast: two separate signals on one pole, the distant on
            // an adapter that carries it 0.20 m in front of the main head and 0.40 m below
            // it. Its own aspect, kept apart from the main's - it is repeating a signal
            // further on, not saying anything about this one.
            //
            // Both gaps are measured between what you can actually see, which is each
            // signal's backing plate rather than its housing: the plate is 1.1 times the
            // head's height, so it hangs below the box it is behind.
            //
            // No bracket is drawn. At 0.20 m proud the head's back face lands flush against
            // the front of the mast (both 0.07 m from the axis), so the adapter is entirely
            // behind the housing and any geometry for it would be inside solid metal.
            if (s.withDistant) {
                const float hhD = 0.48f, hdD = 0.14f; // the distant head's own half extents
                const float below = (mastH - 0.1f * hh) - 0.40f - 1.1f * hhD;
                distantHead(B + UP * below - F * (hd + 0.20f - hdD), R, F, s.distantAspect);
            }
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
