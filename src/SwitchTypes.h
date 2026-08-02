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

// Per-switch type (manual vs motor-driven) authoring, kept as a drop-in overlay
// separate from the geometry so a regenerated import doesn't lose it. Manual is the
// default, so only motor-driven switches are stored; a switch absent from the file is
// manual. Each record is anchored by the diverging branch's track id plus the switch's
// world position (the same key SwitchNetwork uses to dedup turnouts), so it follows
// small geometry changes and survives re-imports as long as the branch keeps its id.
//
// File `<datasetRoot>/overlay/switch-types.txt` (x/y are the anchor + a staleness hint):
//   switch <sidingTrackHex> <x> <y> motor

struct SwitchTypeOverride {
    std::uint32_t sidingTrack = 0; // diverging branch track id (Turnout::sidingTrack)
    glm::dvec2 world{0.0};         // switch world position (Turnout::world x,y)
    SwitchType type = SwitchType::Motor;
};

// --- File IO (mirrors loadTrackCircuits/writeTrackCircuits) ---
std::vector<SwitchTypeOverride> loadSwitchTypes(const std::string& datasetRoot);
bool writeSwitchTypes(const std::string& datasetRoot,
                      const std::vector<SwitchTypeOverride>& overrides);

// --- Apply / collect against a built SwitchNetwork ---
// Set each matching turnout's type from the overrides (match on sidingTrack + world
// within a small tolerance). Turnouts with no override keep their default (Manual).
void applySwitchTypes(SwitchNetwork& net,
                      const std::vector<SwitchTypeOverride>& overrides);
// The non-default (motor) switches of a network, ready to write out.
std::vector<SwitchTypeOverride> collectSwitchOverrides(const SwitchNetwork& net);
