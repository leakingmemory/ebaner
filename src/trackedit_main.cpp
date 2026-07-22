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

// ebaner-trackedit: a WYSIWYG track-network editor that reuses ebaner's engine.
// It renders the same scene (terrain, roads, buildings, rails) and overlays the
// raw rail geo-points (round markers) and the links between consecutive points of
// each track (lines). Dead ends (loose ends of broken links) are marked in red.
// Aim the crosshair at two dead ends and link them; the edit is written to a
// drop-in overlay (`<dataset>/overlay/track-edits.txt`) that the loader applies
// over the generated tiles, so the train can then run across the former gap.

#include "BuildingMesh.h"
#include "Camera.h"
#include "Font.h"
#include "PlatformMesh.h"
#include "RoadMesh.h"
#include "SwitchMesh.h"
#include "TerrainData.h"
#include "TerrainMesh.h"
#include "Textures.h"
#include "TrackGraph.h"
#include "TrackMesh.h"
#include "TrackOverlay.h"
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
#include <set>
#include <string>
#include <utility>
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
    SwitchMesh switches;
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
        switches.build(data);
        graph = buildTrackGraph(data); // raw geo-points + links overlay
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Failed to load terrain: %s\n", e.what());
        return EXIT_FAILURE;
    }
    std::printf("[trackedit] %zu tracks, %zu geo-points, %zu links, %zu dead-ends\n",
                graph.trackCount, graph.points.size(), graph.lines.size() / 2,
                graph.deadEnds.size());

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
    // Switch stands are the same solid-lit static geometry; merge them in too.
    {
        const std::uint32_t vbase = static_cast<std::uint32_t>(structVerts.size());
        structVerts.insert(structVerts.end(), switches.vertices().begin(),
                           switches.vertices().end());
        structIndices.reserve(structIndices.size() + switches.indices().size());
        for (std::uint32_t idx : switches.indices())
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
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Vulkan init failed: %s\n", e.what());
        glfwDestroyWindow(window);
        glfwTerminate();
        return EXIT_FAILURE;
    }

    // --- Editor state ---
    int selA = -1, selB = -1;      // picked dead-end indices (into graph.deadEnds)
    std::set<int> selected;        // selected geo-points (indices into graph.points)
    std::vector<TrackEdit> pending; // edits made this session, not yet saved
    auto rebuildOverlay = [&]() {
        std::vector<LineVertex> lns = graph.lines;
        const glm::vec3 yellow(1.0f, 0.95f, 0.2f), white(1.0f, 1.0f, 1.0f);
        std::vector<LineVertex> pts = graph.points;
        pts.insert(pts.end(), graph.deadEnds.begin(), graph.deadEnds.end());
        if (selA >= 0) pts.push_back({graph.deadEnds[selA].pos, yellow});
        if (selB >= 0) pts.push_back({graph.deadEnds[selB].pos, yellow});
        for (int i : selected) pts.push_back({graph.points[i].pos, white}); // on top
        renderer.attachTrackGraph(lns, pts);
    };
    // Apply edits to the loaded data in-session and rebuild the graph, so they
    // preview immediately (same path ebaner uses at load). Nothing is written to
    // disk until the user saves.
    auto applyEditsLive = [&](const std::vector<TrackEdit>& es,
                              bool keepSelection = false) {
        for (const TrackEdit& e : es) {
            // De-dupe elevation overrides by (x,y) so repeated nudges keep one line
            // per point (last write wins; fewer `elev` lines on save).
            if (e.kind == TrackEdit::Elev)
                pending.erase(std::remove_if(pending.begin(), pending.end(),
                    [&](const TrackEdit& p) {
                        return p.kind == TrackEdit::Elev &&
                               std::hypot(p.a.x - e.a.x, p.a.y - e.a.y) < 0.5;
                    }), pending.end());
            pending.push_back(e);
        }
        data.applyTrackEdits(es);
        graph = buildTrackGraph(data);
        // Elev edits keep the point count/order, so the selection stays valid; link
        // edits add a segment (indices shift) so the selection must clear.
        if (!keepSelection) { selected.clear(); selA = selB = -1; }
        rebuildOverlay();
    };
    // Straighten the selected span's elevation onto an endpoint-anchored grade.
    auto doRegrade = [&]() {
        if (selected.size() < 2) {
            std::printf("[trackedit] regrade: select >=2 points first\n");
            return;
        }
        const int lo = *selected.begin(), hi = *selected.rbegin();
        const std::uint32_t tid = graph.pointTrack[lo];
        for (int i : selected)
            if (graph.pointTrack[i] != tid) {
                std::printf("[trackedit] regrade: select points on ONE track\n");
                return;
            }
        std::vector<double> cum(hi - lo + 1, 0.0);
        for (int i = lo + 1; i <= hi; ++i)
            cum[i - lo] = cum[i - 1 - lo] +
                std::hypot(graph.pointWorld[i].x - graph.pointWorld[i - 1].x,
                           graph.pointWorld[i].y - graph.pointWorld[i - 1].y);
        const double dtot = cum[hi - lo];
        const double z0 = graph.pointWorld[lo].z, z1 = graph.pointWorld[hi].z;
        std::vector<TrackEdit> es;
        for (int i = lo; i <= hi; ++i) {
            TrackEdit e;
            e.kind = TrackEdit::Elev;
            const double nz = dtot > 1e-6 ? z0 + (z1 - z0) * (cum[i - lo] / dtot) : z0;
            e.a = glm::dvec3(graph.pointWorld[i].x, graph.pointWorld[i].y, nz);
            es.push_back(e);
        }
        const double grade = dtot > 1e-6 ? (z1 - z0) / dtot * 100.0 : 0.0;
        std::printf("[trackedit] regraded %zu points, grade %+.2f%% over %.0f m "
                    "(preview; Ctrl+S to save)\n", es.size(), grade, dtot);
        applyEditsLive(es, /*keepSelection=*/true);
    };
    // Raise (+) / lower (-) every selected point's elevation by `delta` metres.
    auto doElevStep = [&](double delta) {
        if (selected.empty()) return;
        std::vector<TrackEdit> es;
        for (int i : selected) {
            TrackEdit e;
            e.kind = TrackEdit::Elev;
            e.a = glm::dvec3(graph.pointWorld[i].x, graph.pointWorld[i].y,
                             graph.pointWorld[i].z + delta);
            es.push_back(e);
        }
        applyEditsLive(es, /*keepSelection=*/true);
    };
    // Connect a selected track *endpoint* to the nearest track its end trajectory
    // crosses: move the endpoint onto that track (trims a siding that overshoots).
    auto doConnect = [&]() {
        constexpr double kMaxSnap = 25.0; // m from the endpoint
        auto cross2 = [](const glm::dvec2& a, const glm::dvec2& b) {
            return a.x * b.y - a.y * b.x;
        };
        std::vector<TrackEdit> moves;
        int skipped = 0;
        const int np = static_cast<int>(graph.points.size());
        for (int i : selected) {
            const std::uint32_t tid = graph.pointTrack[i];
            const bool isFirst = i == 0 || graph.pointTrack[i - 1] != tid;
            const bool isLast = i + 1 >= np || graph.pointTrack[i + 1] != tid;
            if (isFirst == isLast) { ++skipped; continue; } // mid-track, not an endpoint
            const int nb = isFirst ? i + 1 : i - 1;
            const glm::dvec3 D = graph.pointWorld[i];
            const glm::dvec2 A(graph.pointWorld[nb].x, graph.pointWorld[nb].y); // P
            const glm::dvec2 dir(D.x - A.x, D.y - A.y); // P -> D
            // Nearest crossing (to D) of the infinite P->D line with another track.
            double best = kMaxSnap;
            glm::dvec3 X(0.0);
            bool has = false;
            for (int j = 0; j + 1 < np; ++j) {
                if (graph.pointTrack[j] != graph.pointTrack[j + 1]) continue; // track break
                if (graph.pointTrack[j] == tid) continue;                     // same track
                const glm::dvec3 M0 = graph.pointWorld[j], M1 = graph.pointWorld[j + 1];
                const glm::dvec2 seg(M1.x - M0.x, M1.y - M0.y);
                const double denom = cross2(dir, seg);
                if (std::abs(denom) < 1e-9) continue; // parallel
                const glm::dvec2 w(M0.x - A.x, M0.y - A.y);
                const double t = cross2(w, seg) / denom; // along the P->D line
                const double u = cross2(w, dir) / denom; // along the crossed edge
                if (u < 0.0 || u > 1.0 || t < 0.0) continue; // off the edge / behind P
                const glm::dvec3 P(M0.x + seg.x * u, M0.y + seg.y * u,
                                   M0.z + (M1.z - M0.z) * u);
                const double d = std::hypot(P.x - D.x, P.y - D.y);
                if (d < best) { best = d; X = P; has = true; }
            }
            if (has) {
                TrackEdit e;
                e.kind = TrackEdit::Move;
                e.a = D;
                e.b = X;
                moves.push_back(e);
            } else {
                ++skipped;
            }
        }
        if (!moves.empty()) {
            std::printf("[trackedit] connected %zu end(s) to intersecting track "
                        "(preview; Ctrl+S to save)\n", moves.size());
            applyEditsLive(moves, /*keepSelection=*/true);
        } else {
            std::printf("[trackedit] connect: select a dead-end whose line crosses a "
                        "track (skipped %d)\n", skipped);
        }
    };
    rebuildOverlay();

    std::printf("\nControls: WASD move, Q/E down/up, mouse look, Shift boost, "
                "Tab release cursor, Esc quit\n"
                "Select (cursor freed with Tab): click a geo-point, Ctrl+click for "
                "several, click empty to clear.\n"
                "Edit: G straighten the selected span's grade; Up/Down raise/lower the "
                "selection; J join a selected dead-end onto the track it crosses; "
                "Enter/Enter+L link two red dead ends. Live preview; Ctrl+S saves.\n"
                "Overlay: amber = main line, cyan = siding, magenta = yard, "
                "red = dead end, white = selected.\n\n");

    const glm::vec3 sunDir = glm::normalize(glm::vec3(0.4f, -0.5f, 0.75f));
    const char* shotPath = std::getenv("EBANER_SCREENSHOT");
    int frame = 0;
    bool prevEnter = false, prevL = false, prevX = false, prevML = false,
         prevG = false, prevS = false, prevUp = false, prevDown = false,
         prevJ = false;
    float elevRepeat = 0.0f; // auto-repeat throttle for Up/Down raise/lower
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

        const float aspect = static_cast<float>(fbw) / static_cast<float>(fbh);
        const glm::mat4 viewProj = g_camera.projMatrix(aspect) * g_camera.viewMatrix();

        // Link tool: pick the dead-end nearest the camera's forward ray (crosshair).
        int hover = -1;
        {
            const glm::vec3 cp = g_camera.position(), fwv = g_camera.forward();
            float bestAng = 0.06f; // ~3.4 deg cone
            for (std::size_t i = 0; i < graph.deadEnds.size(); ++i) {
                const glm::vec3 v = graph.deadEnds[i].pos - cp;
                const float t = glm::dot(v, fwv);
                if (t < 1.0f) continue; // behind / too close
                const float ang = glm::length(v - fwv * t) / t;
                if (ang < bestAng) { bestAng = ang; hover = static_cast<int>(i); }
            }
        }

        // Selection: pick the geo-point under the free cursor (screen-space). Only
        // when the cursor is released (Tab) — captured mode is for mouse-look.
        int pointHover = -1;
        glm::vec2 hoverPx(0.0f);
        if (!g_mouseCaptured) {
            double mx = 0.0, my = 0.0;
            glfwGetCursorPos(window, &mx, &my);
            int winw = fbw, winh = fbh;
            glfwGetWindowSize(window, &winw, &winh);
            const glm::vec2 cur(static_cast<float>(mx) * fbw / std::max(winw, 1),
                                static_cast<float>(my) * fbh / std::max(winh, 1));
            float best = 14.0f; // px pick radius
            for (std::size_t i = 0; i < graph.points.size(); ++i) {
                const glm::vec4 clip = viewProj * glm::vec4(graph.points[i].pos, 1.0f);
                if (clip.w <= 0.0f) continue; // behind the camera
                const glm::vec2 px((clip.x / clip.w * 0.5f + 0.5f) * fbw,
                                   (clip.y / clip.w * 0.5f + 0.5f) * fbh);
                const float d = glm::length(px - cur);
                if (d < best) { best = d; pointHover = static_cast<int>(i); hoverPx = px; }
            }
        }

        // Editing keys (edge-triggered): Enter pick A/B, L link, X clear, G grade,
        // S save. Edits preview immediately (applyEditsLive); S writes to disk.
        const bool ctrl = glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS ||
                          glfwGetKey(window, GLFW_KEY_RIGHT_CONTROL) == GLFW_PRESS;
        const bool kEnter = glfwGetKey(window, GLFW_KEY_ENTER) == GLFW_PRESS;
        const bool kL = glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS;
        const bool kX = glfwGetKey(window, GLFW_KEY_X) == GLFW_PRESS;
        const bool kG = glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS;
        const bool kJ = glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS;
        // Save is Ctrl+S (plain S is the backward-movement key).
        const bool kSave = ctrl && glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;
        if (kEnter && !prevEnter && hover >= 0) {
            if (selA < 0) selA = hover;
            else if (hover != selA) selB = hover;
            rebuildOverlay();
        }
        if (kX && !prevX) { selA = selB = -1; rebuildOverlay(); }
        if (kL && !prevL && selA >= 0 && selB >= 0) {
            TrackEdit e;
            e.kind = TrackEdit::Link;
            e.a = graph.deadEndWorld[selA];
            e.b = graph.deadEndWorld[selB];
            applyEditsLive({e});
            std::printf("[trackedit] linked dead-ends (preview; S to save)\n");
        }
        // G: straighten the selected span's elevation to a continuous grade.
        if (kG && !prevG) doRegrade();
        // J: connect the selected dead-end(s) onto the track their line crosses.
        if (kJ && !prevJ) doConnect();
        // Up/Down: raise/lower the selected point(s). Auto-repeats while held (an
        // immediate first step, then throttled) so big changes don't need many taps.
        constexpr double kElevStep = 0.1; // metres per step (auto-repeats while held)
        const bool kUp = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
        const bool kDn = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;
        if ((kUp || kDn) && !selected.empty()) {
            elevRepeat -= dt;
            if ((kUp && !prevUp) || (kDn && !prevDown) || elevRepeat <= 0.0f) {
                doElevStep(kUp ? kElevStep : -kElevStep);
                elevRepeat = 0.09f;
            }
        } else {
            elevRepeat = 0.0f;
        }
        prevUp = kUp; prevDown = kDn;
        // Ctrl+S: save pending edits to the overlay file.
        if (kSave && !prevS && !pending.empty()) {
            if (appendTrackEdits(datasetRoot, pending)) {
                std::printf("[trackedit] saved %zu edit(s) -> %s/overlay/track-edits.txt\n",
                            pending.size(), datasetRoot.c_str());
                pending.clear();
            } else {
                std::fprintf(stderr, "[trackedit] failed to write overlay file\n");
            }
        }
        prevEnter = kEnter; prevL = kL; prevX = kX; prevG = kG; prevS = kSave;
        prevJ = kJ;

        // Click to select (cursor freed only). Ctrl+click toggles for multi-select;
        // plain click selects one; clicking empty space clears.
        const bool mL = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        if (!g_mouseCaptured && mL && !prevML) {
            if (pointHover >= 0) {
                if (ctrl) {
                    if (!selected.insert(pointHover).second) selected.erase(pointHover);
                } else {
                    selected.clear();
                    selected.insert(pointHover);
                }
            } else if (!ctrl) {
                selected.clear();
            }
            rebuildOverlay();
        }
        prevML = mL;

        // HUD: a dark backing panel plus info lines, and a centre crosshair.
        {
            std::vector<TextVertex> tv;
            const float sc = std::max(2.0f, static_cast<float>(fbh) / 240.0f);
            const float x = 40.0f, lh = 12.0f * sc;
            auto ndc = [&](float px, float py) {
                return glm::vec2(px / fbw * 2.0f - 1.0f, py / fbh * 2.0f - 1.0f);
            };
            auto quad = [&](float cx, float cy, float hw, float hh, const glm::vec3& col) {
                const glm::vec2 a = ndc(cx - hw, cy - hh), b = ndc(cx + hw, cy - hh),
                                c = ndc(cx + hw, cy + hh), d = ndc(cx - hw, cy + hh);
                tv.push_back({a, col}); tv.push_back({b, col}); tv.push_back({c, col});
                tv.push_back({a, col}); tv.push_back({c, col}); tv.push_back({d, col});
            };
            {
                const float x1 =
                    std::min(static_cast<float>(fbw) - 20.0f, 40.0f + 54.0f * 8.0f * sc);
                const float y1 = 40.0f + 8.0f * lh;
                const glm::vec3 bg(0.04f, 0.05f, 0.09f);
                const glm::vec2 a = ndc(20.0f, 20.0f), b = ndc(x1, 20.0f),
                                c = ndc(x1, y1), d = ndc(20.0f, y1);
                tv.push_back({a, bg}); tv.push_back({b, bg}); tv.push_back({c, bg});
                tv.push_back({a, bg}); tv.push_back({c, bg}); tv.push_back({d, bg});
            }
            char buf[160];
            appendText(tv, "EBANER-TRACKEDIT", x, 40.0f, sc,
                       glm::vec3(1.0f, 0.95f, 0.5f), fbw, fbh);
            std::snprintf(buf, sizeof(buf), "TRACKS %zu  GEO-POINTS %zu  DEAD-ENDS %zu",
                          graph.trackCount, graph.points.size(), graph.deadEnds.size());
            appendText(tv, buf, x, 40.0f + lh, sc, glm::vec3(0.8f, 0.9f, 1.0f), fbw, fbh);
            const glm::vec3 p = g_camera.position();
            std::snprintf(buf, sizeof(buf), "POS %.0f %.0f %.0f", p.x, p.y, p.z);
            appendText(tv, buf, x, 40.0f + 2 * lh, sc, glm::vec3(0.7f, 0.85f, 0.7f),
                       fbw, fbh);
            std::snprintf(buf, sizeof(buf), "LINK: Enter A/B (A %s B %s), L link, X clear",
                          selA >= 0 ? "set" : "-", selB >= 0 ? "set" : "-");
            appendText(tv, buf, x, 40.0f + 3 * lh, sc, glm::vec3(0.85f, 0.85f, 0.7f),
                       fbw, fbh);
            // Selection + elevation (single z, or min..max over the selection).
            char selz[48] = "";
            if (!selected.empty()) {
                double zmin = 1e9, zmax = -1e9;
                for (int i : selected) {
                    const double z = graph.pointWorld[i].z;
                    zmin = std::min(zmin, z); zmax = std::max(zmax, z);
                }
                if (selected.size() == 1) std::snprintf(selz, sizeof(selz), " z=%.2f", zmin);
                else std::snprintf(selz, sizeof(selz), " z=%.2f..%.2f", zmin, zmax);
            }
            std::snprintf(buf, sizeof(buf),
                          "SELECTED %zu%s   G grade  Up/Dn elev  J join-track",
                          selected.size(), selz);
            appendText(tv, buf, x, 40.0f + 4 * lh, sc,
                       selected.empty() ? glm::vec3(0.7f, 0.85f, 0.7f)
                                        : glm::vec3(1.0f, 0.9f, 0.3f),
                       fbw, fbh);
            std::snprintf(buf, sizeof(buf), "UNSAVED %zu   %s", pending.size(),
                          pending.empty() ? "" : "Ctrl+S to save");
            appendText(tv, buf, x, 40.0f + 5 * lh, sc,
                       pending.empty() ? glm::vec3(0.6f, 0.9f, 0.6f)
                                       : glm::vec3(1.0f, 0.6f, 0.3f),
                       fbw, fbh);
            appendText(tv,
                       g_mouseCaptured ? "SELECT: press Tab to free the cursor"
                                       : "SELECT: click point, Ctrl+click multi, "
                                         "click empty to clear",
                       x, 40.0f + 6 * lh, sc, glm::vec3(0.85f, 0.85f, 0.7f), fbw, fbh);
            // Crosshair: a '+' at screen centre (tinted when a dead-end is hovered).
            appendText(tv, "+", fbw * 0.5f - 4.0f * sc, fbh * 0.5f - 4.0f * sc, sc,
                       hover >= 0 ? glm::vec3(1.0f, 0.3f, 0.25f) : glm::vec3(1.0f),
                       fbw, fbh);
            // Hover marker: a green square ring around the geo-point under the cursor,
            // with its elevation labelled beside it.
            if (pointHover >= 0) {
                const glm::vec3 hc(0.2f, 1.0f, 0.4f);
                const float R = 9.0f, T = 1.5f;
                quad(hoverPx.x, hoverPx.y - R, R, T, hc);
                quad(hoverPx.x, hoverPx.y + R, R, T, hc);
                quad(hoverPx.x - R, hoverPx.y, T, R, hc);
                quad(hoverPx.x + R, hoverPx.y, T, R, hc);
                std::snprintf(buf, sizeof(buf), "z=%.2f", graph.pointWorld[pointHover].z);
                appendText(tv, buf, hoverPx.x + 13.0f, hoverPx.y - 4.0f, sc * 0.7f,
                           glm::vec3(0.85f, 1.0f, 0.85f), fbw, fbh);
            }
            renderer.setOverlayText(tv);
        }

        PushConstants pc{};
        pc.viewProj = viewProj;
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
