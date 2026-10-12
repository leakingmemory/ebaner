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


#include "LoadingScreen.h"

#include "Font.h"
#include "VulkanRenderer.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cstdio>
#include <cstring>

LoadingScreen::LoadingScreen(GLFWwindow* window, VulkanRenderer& renderer,
                             const Station& station, std::vector<LoadStage> stages)
    : window_(window), renderer_(renderer),
      title_("LOADING  " + station.name + "  (" + station.line + ")"),
      stages_(std::move(stages)) {
    for (const LoadStage& s : stages_) totalWeight_ += s.weight;
    draw(true);
}

void LoadingScreen::stage(const char* label) {
    for (int i = current_; i < static_cast<int>(stages_.size()); ++i) {
        if (stages_[i].label != label && std::strcmp(stages_[i].label, label) != 0) continue;
        if (i == current_) return; // already there; progress within it is within()'s
        current_ = i;
        within_ = 0.0;
        draw(true);
        return;
    }
    std::fprintf(stderr, "[Loading] no stage \"%s\" after \"%s\"\n", label,
                 stages_.empty() ? "" : stages_[current_].label);
}

void LoadingScreen::within(double done, double total) {
    within_ = total > 0.0 ? std::clamp(done / total, 0.0, 1.0) : 0.0;
    draw(false);
}

void LoadingScreen::report(const char* label, double frac) {
    stage(label);
    within(frac, 1.0);
}

void LoadingScreen::finish() {
    current_ = static_cast<int>(stages_.size()); // every stage dimmed: all done
    within_ = 0.0;
    draw(true);
    renderer_.setMapMode(false);
}

void LoadingScreen::draw(bool force) {
    const auto now = std::chrono::steady_clock::now();
    if (!force && now - lastDraw_ < std::chrono::milliseconds(100)) return;
    lastDraw_ = now;

    glfwPollEvents();
    int fw = 0, fh = 0;
    glfwGetFramebufferSize(window_, &fw, &fh);
    if (fw == 0 || fh == 0) return; // minimised: nothing to draw into

    float done = 0.0f;
    for (int i = 0; i < std::min(current_, static_cast<int>(stages_.size())); ++i)
        done += stages_[i].weight;
    if (current_ < static_cast<int>(stages_.size()))
        done += stages_[current_].weight * static_cast<float>(within_);
    const float frac = totalWeight_ > 0.0f ? done / totalWeight_ : 1.0f;

    std::vector<std::string> labels;
    labels.reserve(stages_.size());
    for (const LoadStage& s : stages_) labels.emplace_back(s.label);

    std::vector<TextVertex> tv;
    appendLoadingPanel(tv, title_, labels, current_, frac, fw, fh);
    renderer_.setOverlayText(tv);
    renderer_.setMapMode(true);
    PushConstants pc{};
    pc.viewProj = glm::mat4(1.0f); // the panel is screen-space; no world is drawn
    renderer_.drawFrame(pc);
}
