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

#include "StationPicker.h"

#include "Font.h"
#include "VulkanRenderer.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cstdlib>
#include <string>
#include <vector>

namespace {
// PgUp/PgDn jump the selection; appendMenu windows the list around it, so the view is
// no longer this file's business - it used to keep its own window of 15, which was the
// only place in the program that knew a screen holds that many.
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

    // Two levels: the stations open for play, and behind the last row of those, the ones
    // still being built. A station asked for on the command line may be either, so the
    // picker opens on whichever level it is on.
    const StationMenu menu = stationMenu(all);
    const int initialIdx = initial ? static_cast<int>(initial - all.data()) : -1;
    bool inDev = initialIdx >= 0 && !menu.onTop(initialIdx);
    int pick = initialIdx >= 0 ? menu.row(initialIdx) : 0;

    bool pUp = false, pDn = false, pPgU = false, pPgD = false, pEnter = false, pEsc = true;
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        int fw = 0, fh = 0;
        glfwGetFramebufferSize(window, &fw, &fh);
        if (fw == 0 || fh == 0) continue; // minimised

        const std::vector<std::string>& names = inDev ? menu.dev : menu.top;
        const int n = std::max(1, static_cast<int>(names.size()));
        auto down = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
        const bool u = down(GLFW_KEY_UP), d = down(GLFW_KEY_DOWN);
        const bool pu = down(GLFW_KEY_PAGE_UP), pd = down(GLFW_KEY_PAGE_DOWN);
        const bool en = down(GLFW_KEY_ENTER), esc = down(GLFW_KEY_ESCAPE);
        if (u && !pUp) pick = (pick + n - 1) % n;
        if (d && !pDn) pick = (pick + 1) % n;
        if (pu && !pPgU) pick = (pick + n - kPageStep % n) % n;
        if (pd && !pPgD) pick = (pick + kPageStep) % n;
        if (esc && !pEsc) {
            // Out of the sub-list one level, back on the row that opened it.
            if (inDev) { inDev = false; pick = menu.devRowIndex(); }
            else glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
        if (en && !pEnter && !names.empty()) {
            const int idx = inDev ? menu.devRow[static_cast<std::size_t>(pick)]
                                  : menu.topRow[static_cast<std::size_t>(pick)];
            if (idx >= 0) return &all[static_cast<std::size_t>(idx)];
            inDev = true;
            pick = 0;
        }
        pUp = u; pDn = d; pPgU = pu; pPgD = pd; pEnter = en; pEsc = esc;

        drawPanel(window, renderer,
                  inDev ? "START AT - UNDER DEVELOPMENT  (arrows, PgUp/PgDn, Enter, Esc)"
                        : "START AT  (arrows, PgUp/PgDn, Enter)",
                  inDev ? menu.dev : menu.top, pick);
    }
    return nullptr; // closed
}
