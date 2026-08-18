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

#pragma once

#include <glm/glm.hpp>

#include <cstdint>
#include <string>
#include <vector>

#include "SwitchNetwork.h"
#include "TrackCircuits.h" // TrackCircuits, TrackPoly, fracToWorld (for lock resolution)

// Per-switch type (manual vs motor-driven) authoring, kept as a drop-in overlay
// separate from the geometry so a regenerated import doesn't lose it. Manual is the
// default, so only motor-driven switches are stored; a switch absent from the file is
// manual. Each record is anchored by the diverging branch's track id plus the switch's
// world position (the same key SwitchNetwork uses to dedup turnouts), so it follows
// small geometry changes and survives re-imports as long as the branch keeps its id.
//
// A motor switch may also carry a locking set: the track-circuit section ids that must
// be clear for it to be thrown remotely. Absent (no `lock`) -> resolve to the sections
// the switch sits within; present (even empty) -> use it verbatim.
//
// File `<datasetRoot>/overlay/switch-types.txt` (x/y are the anchor + a staleness hint):
//   switch <sidingTrackHex> <x> <y> motor [lock <sectionId> <sectionId> ...]
//   noswitch <x> <y> <radius> [<sidingTrackHex>] [all]
//
// The second says a turnout the detector found is not really there (SwitchSuppression in
// SwitchNetwork.h). It lives in this file because it is switch authoring like the rest,
// and because the writer below rewrites the file whole - kept anywhere else in it, a save
// from the editor would drop it.

struct SwitchTypeOverride {
    std::uint32_t sidingTrack = 0; // diverging branch track id (Turnout::sidingTrack)
    glm::dvec2 world{0.0};         // switch world position (Turnout::world x,y)
    SwitchType type = SwitchType::Motor;
    bool hasLock = false;          // true if a `lock` list was authored (else default)
    std::vector<int> lock;         // authored locking section ids
};

// --- File IO (mirrors loadTrackCircuits/writeTrackCircuits) ---
std::vector<SwitchTypeOverride> loadSwitchTypes(const std::string& datasetRoot);
std::vector<SwitchSuppression> loadSwitchSuppressions(const std::string& datasetRoot);
// Rewrites the file whole, so the suppressions have to come back through it or a save
// would lose them - they are not derivable from the built network, which by then no
// longer contains what they suppressed.
bool writeSwitchTypes(const std::string& datasetRoot,
                      const std::vector<SwitchTypeOverride>& overrides,
                      const std::vector<SwitchSuppression>& suppress = {});

// --- Apply / collect against a built SwitchNetwork ---
// Set each matching turnout's type from the overrides (match on sidingTrack + world
// within a small tolerance). Turnouts with no override keep their default (Manual).
void applySwitchTypes(SwitchNetwork& net,
                      const std::vector<SwitchTypeOverride>& overrides);
// The non-default (motor) switches of a network, ready to write out (with their locks).
std::vector<SwitchTypeOverride> collectSwitchOverrides(const SwitchNetwork& net);

// The section ids whose track passes within a few metres of the turnout - "the circuits
// the switch is located within" (its default locking set).
std::vector<int> defaultLockSections(const Turnout& t, const TrackCircuits& circuits,
                                     const std::vector<TrackPoly>& polys);
// Resolve each motor switch's locking set: the authored list if present, else the
// default. Call after applySwitchTypes(), once circuits + polys are available.
void applySwitchLocks(SwitchNetwork& net,
                      const std::vector<SwitchTypeOverride>& overrides,
                      const TrackCircuits& circuits,
                      const std::vector<TrackPoly>& polys);
