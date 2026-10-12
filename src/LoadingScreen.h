// ebaner - a Norwegian railway simulator.
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

#include <chrono>
#include <string>
#include <vector>

struct GLFWwindow;
class VulkanRenderer;

// One step of reading and building a world, as the loading screen lists it. `weight` is
// roughly how many seconds it takes on a slow machine; only the proportions matter, and
// they are what keep the bar moving at an even pace rather than in lurches.
struct LoadStage {
    const char* label;
    float weight;
};

// The screen shown between picking a station and the first frame of the world: what is
// being read or built, the steps still to come, and a bar for how far it has got.
//
// Loading blocks the thread that draws, so this draws only when told something: a new
// stage always redraws, progress within one at most every 100 ms - a frame costs a vsync,
// and the ground alone reports 67 tiles. Each draw also polls the window's events, which
// is what keeps the desktop from declaring it hung.
//
// Drawn in the renderer's map mode, which skips the 3-D meshes: the terrain is uploaded
// well before there is a camera to look at it with, and would draw as wreckage around the
// panel. finish() turns map mode off again.
class LoadingScreen {
public:
    LoadingScreen(GLFWwindow* window, VulkanRenderer& renderer, const Station& station,
                  std::vector<LoadStage> stages);

    // Move on to the stage with this label, the same pointer or the same text. Stages may
    // be skipped but not revisited; a label that is not in the list is reported once.
    void stage(const char* label);
    // How far through the current stage it has got: `done` of `total`.
    void within(double done, double total);
    // Both at once, which is the shape TerrainData::load reports progress in.
    void report(const char* label, double frac);
    // Everything done: draws the full bar and leaves map mode.
    void finish();

private:
    void draw(bool force);

    GLFWwindow* window_;
    VulkanRenderer& renderer_;
    std::string title_;
    std::vector<LoadStage> stages_;
    float totalWeight_ = 0.0f;
    int current_ = 0;
    double within_ = 0.0;
    std::chrono::steady_clock::time_point lastDraw_{};
};
