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

#include "StationPicker.h"

#include "Font.h"
#include "VulkanRenderer.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cstdlib>
#include <string>
#include <vector>

namespace {
// Drawn at once; the dataset has hundreds of stations and a screen holds this many.
constexpr int kShown = 15;
constexpr int kPageStep = 10;

void drawPanel(GLFWwindow* window, VulkanRenderer& renderer, const std::string& title,
               const std::vector<std::string>& items, int selected) {
    int fw = 0, fh = 0;
    glfwGetFramebufferSize(window, &fw, &fh);
    if (fw == 0 || fh == 0) return;
    std::vector<TextVertex> tv;
    appendMenu(tv, title, items, selected, fw, fh);
    renderer.setOverlayText(tv);
    PushConstants pc{};
    pc.viewProj = glm::mat4(1.0f); // nothing in the world yet; the panel is screen-space
    renderer.drawFrame(pc);
}
} // namespace

const Station* runStationPicker(GLFWwindow* window, VulkanRenderer& renderer,
                                const std::vector<Station>& all, const Station* initial) {
    if (all.empty()) return initial;
    if (const char* env = std::getenv("EBANER_STATION")) {
        if (const Station* s = findStation(all, env)) return s;
        return initial;
    }
    if (std::getenv("EBANER_SCREENSHOT")) return initial; // scripted: never stop here

    int pick = 0;
    for (std::size_t i = 0; i < all.size(); ++i)
        if (&all[i] == initial) pick = static_cast<int>(i);

    std::vector<std::string> names;
    names.reserve(all.size());
    for (const Station& s : all)
        names.push_back(s.name + (s.isStop() ? "  (stop)" : "") + "  " + s.line);

    bool pUp = false, pDn = false, pPgU = false, pPgD = false, pEnter = false;
    const int n = static_cast<int>(names.size());
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        int fw = 0, fh = 0;
        glfwGetFramebufferSize(window, &fw, &fh);
        if (fw == 0 || fh == 0) continue; // minimised

        auto down = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
        const bool u = down(GLFW_KEY_UP), d = down(GLFW_KEY_DOWN);
        const bool pu = down(GLFW_KEY_PAGE_UP), pd = down(GLFW_KEY_PAGE_DOWN);
        const bool en = down(GLFW_KEY_ENTER);
        if (u && !pUp) pick = (pick + n - 1) % n;
        if (d && !pDn) pick = (pick + 1) % n;
        if (pu && !pPgU) pick = (pick + n - kPageStep) % n;
        if (pd && !pPgD) pick = (pick + kPageStep) % n;
        if (down(GLFW_KEY_ESCAPE)) glfwSetWindowShouldClose(window, GLFW_TRUE);
        if (en && !pEnter) return &all[pick];
        pUp = u; pDn = d; pPgU = pu; pPgD = pd; pEnter = en;

        // A window onto the list, kept centred on the selection.
        const int first = std::clamp(pick - kShown / 2, 0, std::max(0, n - kShown));
        const std::vector<std::string> page(
            names.begin() + first, names.begin() + std::min(n, first + kShown));
        drawPanel(window, renderer, "START AT  (arrows, PgUp/PgDn, Enter)", page,
                  pick - first);
    }
    return nullptr; // closed
}

void drawLoadingNotice(GLFWwindow* window, VulkanRenderer& renderer,
                       const Station& station) {
    drawPanel(window, renderer, "LOADING", {station.name, station.line}, -1);
}
