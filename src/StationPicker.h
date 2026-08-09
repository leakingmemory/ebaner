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

#include "Stations.h"

struct GLFWwindow;
class VulkanRenderer;

// Ask which station to open at, on screen, before anything is read.
//
// Both programs need this and neither can do it the easy way: the station decides where
// the terrain window goes, so it has to be settled before a single tile is loaded - which
// means the window and the renderer come up on an empty world first, and this runs in the
// gap. The renderer must already be initialised; nothing but text is drawn.
//
// Returns the chosen station, or null if the window was closed instead (the caller should
// shut down). `initial` is what the list opens on - whatever the command line asked for.
//
// Skipped, returning `initial` unchanged, when EBANER_STATION names one or when
// EBANER_SCREENSHOT is set: scripted runs must not stop for a menu.
const Station* runStationPicker(GLFWwindow* window, VulkanRenderer& renderer,
                                const std::vector<Station>& all, const Station* initial);

// The one-frame "loading" notice, drawn before the caller blocks for the several seconds
// it takes to read and build a world. An unpainted window looks like a hang.
void drawLoadingNotice(GLFWwindow* window, VulkanRenderer& renderer,
                       const Station& station);
