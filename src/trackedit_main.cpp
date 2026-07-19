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

// ebaner-trackedit: a WYSIWYG track-network viewer that reuses ebaner's engine.
// It renders the same scene (terrain, roads, buildings, rails) and overlays the
// raw rail geo-points (round markers) and the links between consecutive points of
// each track (lines). Walk around with the same free-fly controls. Editing of the
// network is not implemented yet; this is the viewing foundation.

#include "BuildingMesh.h"
#include "Camera.h"
#include "Font.h"
#include "PlatformMesh.h"
#include "RoadMesh.h"
#include "TerrainData.h"
#include "TerrainMesh.h"
#include "Textures.h"
#include "TrackGraph.h"
#include "TrackMesh.h"
#include "TrackPath.h"
#include "VulkanRenderer.h"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>
#include <vector>

namespace {

Camera g_camera;
double g_lastX = 0.0, g_lastY = 0.0;
bool g_firstMouse = true;
bool g_mouseCaptured = true;
VulkanRenderer* g_renderer = nullptr;

void cursorCallback(GLFWwindow*, double x, double y) {
    if (!g_mouseCaptured) { g_firstMouse = true; return; }
    if (g_firstMouse) { g_lastX = x; g_lastY = y; g_firstMouse = false; return; }
    const float dx = static_cast<float>(x - g_lastX);
    const float dy = static_cast<float>(y - g_lastY);
    g_lastX = x;
    g_lastY = y;
    g_camera.look(dx, dy);
}

void keyCallback(GLFWwindow* win, int key, int, int action, int) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(win, GLFW_TRUE);
    if (key == GLFW_KEY_TAB && action == GLFW_PRESS) {
        g_mouseCaptured = !g_mouseCaptured;
        glfwSetInputMode(win, GLFW_CURSOR,
                         g_mouseCaptured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
        g_firstMouse = true;
    }
}

void resizeCallback(GLFWwindow*, int, int) {
    if (g_renderer) g_renderer->notifyResize();
}

} // namespace

int main(int argc, char** argv) {
    const std::string datasetRoot = (argc > 1) ? argv[1] : "../norway-rails";

    // --- Load terrain data and build the scene meshes (same as the viewer) ---
    TerrainData data;
    TerrainMesh mesh;
    TrackMesh tracks;
    RoadMesh roads;
    BuildingMesh buildings;
    PlatformMesh platforms;
    TrackGraph graph;
    std::vector<TrackPath> paths;
    try {
        data.load(datasetRoot);
        paths = buildTrackPaths(data);
        mesh.build(data);
        tracks.build(paths);
        roads.build(data);
        buildings.build(data);
        platforms.build(data, paths);
        graph = buildTrackGraph(data); // raw geo-points + links overlay
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Failed to load terrain: %s\n", e.what());
        return EXIT_FAILURE;
    }
    std::printf("[trackedit] %zu tracks, %zu geo-points, %zu links\n",
                graph.trackCount, graph.points.size(), graph.lines.size() / 2);

    // --- Window ---
    if (!glfwInit()) {
        std::fprintf(stderr, "glfwInit failed\n");
        return EXIT_FAILURE;
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow* window =
        glfwCreateWindow(1280, 720, "ebaner-trackedit - Bodo track network",
                         nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "window creation failed (is Vulkan/WSI available?)\n");
        glfwTerminate();
        return EXIT_FAILURE;
    }

    glm::vec3 startPos = data.startPos() + glm::vec3(0.0f, 0.0f, 5.0f);
    g_camera.init(startPos, data.startDir());
    if (const char* cam = std::getenv("EBANER_CAM")) {
        float x, y, z, yaw, pitch;
        if (std::sscanf(cam, "%f,%f,%f,%f,%f", &x, &y, &z, &yaw, &pitch) == 5)
            g_camera.setPose(glm::vec3(x, y, z), glm::radians(yaw),
                             glm::radians(pitch));
    }

    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetCursorPosCallback(window, cursorCallback);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetFramebufferSizeCallback(window, resizeCallback);

    // --- Land-cover textures ---
    std::vector<std::uint8_t> texPixels = landtex::generate();
    LandTextureData texData;
    texData.pixels = texPixels.data();
    texData.size = landtex::SIZE;
    texData.layers = landtex::LAYERS;
    texData.byteSize = texPixels.size();

    // Platforms draw as solid-lit static geometry identical to buildings; merge
    // them into the building buffers (offsetting indices), as the viewer does.
    std::vector<TrackVertex> structVerts = buildings.vertices();
    std::vector<std::uint32_t> structIndices = buildings.indices();
    {
        const std::uint32_t vbase = static_cast<std::uint32_t>(structVerts.size());
        structVerts.insert(structVerts.end(), platforms.vertices().begin(),
                           platforms.vertices().end());
        structIndices.reserve(structIndices.size() + platforms.indices().size());
        for (std::uint32_t idx : platforms.indices())
            structIndices.push_back(idx + vbase);
    }

    // --- Renderer ---
    VulkanRenderer renderer;
    g_renderer = &renderer;
    try {
        renderer.init(window, mesh.vertices(), mesh.indices(), texData,
                      tracks.vertices(), tracks.indices(), tracks.alwaysIndexCount(),
                      tracks.sleeperChunks(), roads.vertices(), roads.indices(),
                      structVerts, structIndices);
        renderer.attachTrackGraph(graph.lines, graph.points);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Vulkan init failed: %s\n", e.what());
        glfwDestroyWindow(window);
        glfwTerminate();
        return EXIT_FAILURE;
    }

    std::printf("\nControls: WASD move, Q/E down/up, mouse look, Shift boost, "
                "Tab release cursor, Esc quit\n"
                "Overlay: amber = main line, cyan = siding, magenta = yard; "
                "dots = raw geo-points, lines = links.\n\n");

    const glm::vec3 sunDir = glm::normalize(glm::vec3(0.4f, -0.5f, 0.75f));
    const char* shotPath = std::getenv("EBANER_SCREENSHOT");
    int frame = 0;
    double lastTime = glfwGetTime();

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        const double now = glfwGetTime();
        const float dt = static_cast<float>(now - lastTime);
        lastTime = now;

        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(window, &fbw, &fbh);
        if (fbw == 0 || fbh == 0) continue; // minimised

        // Free-fly movement.
        float fwd = 0.0f, right = 0.0f, up = 0.0f;
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) fwd += 1.0f;
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) fwd -= 1.0f;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) right += 1.0f;
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) right -= 1.0f;
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) up += 1.0f;
        if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) up -= 1.0f;
        const bool fast = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
        g_camera.move(fwd, right, up, dt, fast);

        // HUD: a dark backing panel plus a couple of info lines.
        {
            std::vector<TextVertex> tv;
            const float sc = std::max(2.0f, static_cast<float>(fbh) / 240.0f);
            const float x = 40.0f, lh = 12.0f * sc;
            {
                auto ndc = [&](float px, float py) {
                    return glm::vec2(px / fbw * 2.0f - 1.0f, py / fbh * 2.0f - 1.0f);
                };
                const float x1 =
                    std::min(static_cast<float>(fbw) - 20.0f, 40.0f + 30.0f * 8.0f * sc);
                const float y1 = 40.0f + 4.0f * lh;
                const glm::vec3 bg(0.04f, 0.05f, 0.09f);
                const glm::vec2 a = ndc(20.0f, 20.0f), b = ndc(x1, 20.0f),
                                c = ndc(x1, y1), d = ndc(20.0f, y1);
                tv.push_back({a, bg}); tv.push_back({b, bg}); tv.push_back({c, bg});
                tv.push_back({a, bg}); tv.push_back({c, bg}); tv.push_back({d, bg});
            }
            char buf[128];
            appendText(tv, "EBANER-TRACKEDIT", x, 40.0f, sc,
                       glm::vec3(1.0f, 0.95f, 0.5f), fbw, fbh);
            std::snprintf(buf, sizeof(buf), "TRACKS %zu   GEO-POINTS %zu",
                          graph.trackCount, graph.points.size());
            appendText(tv, buf, x, 40.0f + lh, sc, glm::vec3(0.8f, 0.9f, 1.0f), fbw, fbh);
            const glm::vec3 p = g_camera.position();
            std::snprintf(buf, sizeof(buf), "POS %.0f %.0f %.0f", p.x, p.y, p.z);
            appendText(tv, buf, x, 40.0f + 2 * lh, sc, glm::vec3(0.7f, 0.85f, 0.7f),
                       fbw, fbh);
            renderer.setOverlayText(tv);
        }

        const float aspect = static_cast<float>(fbw) / static_cast<float>(fbh);
        PushConstants pc{};
        pc.viewProj = g_camera.projMatrix(aspect) * g_camera.viewMatrix();
        pc.sunDir = glm::vec4(sunDir, data.minElevation());
        pc.camPos = glm::vec4(g_camera.position(), data.maxElevation());
        // Ghost the terrain/rails so the raw geo-point network reads clearly on top.
        pc.params = glm::vec4(0.5f, 0.0f, 0.0f, 0.0f);

        if (shotPath) {
            if (frame == 20) renderer.requestCapture(shotPath);
            if (frame == 24) glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
        renderer.drawFrame(pc);
        ++frame;
    }

    renderer.waitIdle();
    renderer.cleanup();
    g_renderer = nullptr;
    glfwDestroyWindow(window);
    glfwTerminate();
    return EXIT_SUCCESS;
}
