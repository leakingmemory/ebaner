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
#include "SwitchNetwork.h"
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
bool g_menuOpen = false; // Escape menu overlay
int g_menuSel = 0;
VulkanRenderer* g_renderer = nullptr;

void cursorCallback(GLFWwindow*, double x, double y) {
    if (g_menuOpen) return; // menu open: freeze mouselook
    if (!g_mouseCaptured) { g_firstMouse = true; return; }
    if (g_firstMouse) { g_lastX = x; g_lastY = y; g_firstMouse = false; return; }
    const float dx = static_cast<float>(x - g_lastX);
    const float dy = static_cast<float>(y - g_lastY);
    g_lastX = x;
    g_lastY = y;
    g_camera.look(dx, dy);
}

void keyCallback(GLFWwindow* win, int key, int, int action, int) {
    // Escape toggles the menu overlay; while it is open the other hotkeys are inert.
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) g_menuOpen = !g_menuOpen;
    if (g_menuOpen) return;
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
    SwitchNetwork switchNet;
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
        switchNet.build(data, paths);   // turnout detection + routing (all straight)
        switches.build(switchNet);
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

    // Existing overlay lines, so a save can rewrite them (not just append). Any legacy
    // elevation override (no track id) whose point coincides with vertices from more
    // than one track is ambiguous -- it may have snapped to the wrong siding -- so it
    // is auto-staged for removal; saving drops it and the user can re-do it per-track.
    std::vector<TrackEdit> existing = loadTrackOverlay(datasetRoot);
    std::vector<char> removeExisting(existing.size(), 0);
    for (std::size_t k = 0; k < existing.size(); ++k) {
        const TrackEdit& e = existing[k];
        if (e.kind != TrackEdit::Elev || e.track != 0) continue;
        std::set<std::uint32_t> tracks;
        for (std::size_t i = 0; i < graph.points.size(); ++i)
            if (std::hypot(graph.pointWorld[i].x - e.a.x, graph.pointWorld[i].y - e.a.y) < 1.0)
                tracks.insert(graph.pointTrack[i]);
        if (tracks.size() > 1) removeExisting[k] = 1;
    }
    {
        std::size_t amb = 0;
        for (char r : removeExisting) amb += r;
        if (amb) std::printf("[trackedit] %zu ambiguous elev override(s) staged for "
                             "removal (Ctrl+S to apply)\n", amb);
    }

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
            // De-dupe elevation overrides by (x,y) AND track so repeated nudges keep
            // one line per point (last write wins), while a coincident point on a
            // different siding keeps its own edit.
            if (e.kind == TrackEdit::Elev)
                pending.erase(std::remove_if(pending.begin(), pending.end(),
                    [&](const TrackEdit& p) {
                        return p.kind == TrackEdit::Elev && p.track == e.track &&
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
    // Rebuild the *rendered* world (rails/sleepers, carved terrain, switch stands) from
    // the current edited data, so the preview shows how an edit actually renders — not
    // just the wireframe overlay. Heavy (terrain is ~1.8M verts), so it is on demand (P),
    // not per edit. Mirrors the load-time build + the struct merge below.
    auto rebuildRenderPreview = [&]() {
        const double t0 = glfwGetTime();
        data.recarve();                       // re-cut terrain from pristine + edited track
        paths = buildTrackPaths(data);
        tracks.build(paths);
        mesh.build(data);
        switchNet.build(data, paths);
        switches.build(switchNet);
        std::vector<TrackVertex> sv = buildings.vertices();
        std::vector<std::uint32_t> si = buildings.indices();
        auto merge = [&](const std::vector<TrackVertex>& v,
                         const std::vector<std::uint32_t>& idx) {
            const std::uint32_t base = static_cast<std::uint32_t>(sv.size());
            sv.insert(sv.end(), v.begin(), v.end());
            for (std::uint32_t k : idx) si.push_back(k + base);
        };
        merge(platforms.vertices(), platforms.indices());
        merge(switches.vertices(), switches.indices());
        renderer.updateTerrain(mesh.vertices(), mesh.indices());
        renderer.updateTracks(tracks.vertices(), tracks.indices(),
                              tracks.alwaysIndexCount(), tracks.sleeperChunks());
        renderer.updateStructs(sv, si);
        std::printf("[trackedit] render preview refreshed (%.0f ms)\n",
                    (glfwGetTime() - t0) * 1000.0);
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
            e.track = graph.pointTrack[i]; // target this track's vertex specifically
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
            e.track = graph.pointTrack[i]; // target this track's vertex specifically
            es.push_back(e);
        }
        applyEditsLive(es, /*keepSelection=*/true);
    };
    // Cycle the single selection to the next geo-point coincident with it (within ~1 m
    // in x,y), ordered by track, so overlapping siding points at a shared node can each
    // be reached and edited. Raising one changes its z and separates it thereafter.
    auto doCycleCoincident = [&]() {
        if (selected.size() != 1) return;
        const int cur = *selected.begin();
        const glm::dvec3 P = graph.pointWorld[cur];
        std::vector<int> group;
        for (int i = 0; i < static_cast<int>(graph.points.size()); ++i)
            if (std::hypot(graph.pointWorld[i].x - P.x, graph.pointWorld[i].y - P.y) < 1.0)
                group.push_back(i);
        if (group.size() < 2) return;
        std::sort(group.begin(), group.end(), [&](int a, int b) {
            if (graph.pointTrack[a] != graph.pointTrack[b])
                return graph.pointTrack[a] < graph.pointTrack[b];
            return a < b;
        });
        const auto it = std::find(group.begin(), group.end(), cur);
        const int next = group[((it - group.begin()) + 1) % group.size()];
        selected.clear();
        selected.insert(next);
        rebuildOverlay();
        std::printf("[trackedit] cycled to track %#x z=%.2f (%zu coincident here)\n",
                    graph.pointTrack[next], graph.pointWorld[next].z, group.size());
    };
    // Add a connecting rail between two selected geo-points on different tracks. Its
    // ends land on those tracks, so the switch detection makes a switch at each end
    // (a crossover/slip the export omitted). Appears live; switches form on reload.
    auto doAddRail = [&]() {
        if (selected.size() != 2) {
            std::printf("[trackedit] add rail: select exactly 2 points (on 2 tracks)\n");
            return;
        }
        auto it = selected.begin();
        const int i0 = *it++, i1 = *it;
        if (graph.pointTrack[i0] == graph.pointTrack[i1]) {
            std::printf("[trackedit] add rail: the 2 points are on the same track\n");
            return;
        }
        TrackEdit e;
        e.kind = TrackEdit::Rail;
        e.a = graph.pointWorld[i0];
        e.b = graph.pointWorld[i1];
        applyEditsLive({e}); // rail adds a segment -> indices shift -> selection clears
        std::printf("[trackedit] added connecting rail (preview; Ctrl+S to save)\n");
    };
    // Build a scissors (double) crossover between two roughly-parallel tracks: select
    // one point on each track (opposite each other) and press C. Lays two short
    // crossover rails that cross in the middle (the diamond) — a switch on each track
    // at each end, so trains can cross between the two lines either way. Uses the
    // overlay-applied elevations, so pick a spot where both tracks are at grade.
    auto doScissors = [&]() {
        if (selected.size() != 2) {
            std::printf("[trackedit] scissors: select 1 point on each of the two tracks\n");
            return;
        }
        auto it = selected.begin();
        const int i0 = *it++, i1 = *it;
        const std::uint32_t tA = graph.pointTrack[i0], tB = graph.pointTrack[i1];
        if (tA == tB) {
            std::printf("[trackedit] scissors: the 2 points are on the same track\n");
            return;
        }
        std::vector<glm::dvec3> A, B;
        for (std::size_t i = 0; i < graph.points.size(); ++i) {
            if (graph.pointTrack[i] == tA) A.push_back(graph.pointWorld[i]);
            else if (graph.pointTrack[i] == tB) B.push_back(graph.pointWorld[i]);
        }
        if (A.size() < 2 || B.size() < 2) return;
        auto dirAt = [](const std::vector<glm::dvec3>& poly, glm::dvec2 x) {
            double bd = 1e30; glm::dvec2 dir(1, 0);
            for (std::size_t i = 0; i + 1 < poly.size(); ++i) {
                glm::dvec2 a(poly[i].x, poly[i].y), b(poly[i + 1].x, poly[i + 1].y);
                glm::dvec2 ab = b - a; double L2 = glm::dot(ab, ab);
                double t = L2 > 1e-9 ? glm::clamp(glm::dot(x - a, ab) / L2, 0.0, 1.0) : 0.0;
                double d = glm::length(x - (a + ab * t));
                if (d < bd) { bd = d; dir = L2 > 1e-9 ? ab / std::sqrt(L2) : glm::dvec2(1, 0); }
            }
            return dir;
        };
        auto snap = [](const std::vector<glm::dvec3>& poly, glm::dvec2 target) {
            double bd = 1e30; glm::dvec3 best(0.0);
            for (std::size_t i = 0; i + 1 < poly.size(); ++i) {
                glm::dvec2 a(poly[i].x, poly[i].y), b(poly[i + 1].x, poly[i + 1].y);
                glm::dvec2 ab = b - a; double L2 = glm::dot(ab, ab);
                double t = L2 > 1e-9 ? glm::clamp(glm::dot(target - a, ab) / L2, 0.0, 1.0) : 0.0;
                double d = glm::length(target - (a + ab * t));
                if (d < bd) { bd = d; best = glm::mix(poly[i], poly[i + 1], t); }
            }
            return best;
        };
        const glm::dvec3 P0 = graph.pointWorld[i0], P1 = graph.pointWorld[i1];
        const glm::dvec2 p0(P0.x, P0.y), p1(P1.x, P1.y);
        const glm::dvec2 along = dirAt(A, p0);
        constexpr double half = 13.0; // half the crossover length along the tracks
        std::vector<TrackEdit> rails;
        auto add = [&](glm::dvec3 a, glm::dvec3 b) {
            if (std::hypot(a.x - b.x, a.y - b.y) > 2.0) {
                TrackEdit e; e.kind = TrackEdit::Rail; e.a = a; e.b = b; rails.push_back(e);
            }
        };
        add(snap(A, p0 - along * half), snap(B, p1 + along * half)); // one diagonal
        add(snap(B, p1 - along * half), snap(A, p0 + along * half)); // the crossing one
        if (rails.empty()) { std::printf("[trackedit] scissors: degenerate\n"); return; }
        applyEditsLive(rails);
        std::printf("[trackedit] scissors: added %zu crossover rail(s) between %#x and "
                    "%#x (preview; Ctrl+S to save)\n", rails.size(), tA, tB);
    };
    // Auto-build a slip switch (kryssveksel) at a diamond crossing: select the point in
    // the middle of the crossing and press K. Finds the two tracks that cross there and
    // adds the two shallow diagonal connecting rails (a switch on each track at each
    // end), so trains can turn through it. Uses the overlay-applied elevations.
    auto doAutoDiamond = [&]() {
        if (selected.size() != 1) {
            std::printf("[trackedit] slip: select 1 point in the middle of the crossing\n");
            return;
        }
        const glm::dvec3 P = graph.pointWorld[*selected.begin()];
        // Per-track polylines (each track is one contiguous run in the graph).
        std::vector<std::pair<std::uint32_t, std::vector<glm::dvec3>>> polys;
        for (std::size_t i = 0; i < graph.points.size(); ++i) {
            if (polys.empty() || polys.back().first != graph.pointTrack[i])
                polys.push_back({graph.pointTrack[i], {}});
            polys.back().second.push_back(graph.pointWorld[i]);
        }
        // Nearest edge crossing (interior of both) to the selected point.
        auto seg2 = [](glm::dvec2 a, glm::dvec2 b, glm::dvec2 c, glm::dvec2 d,
                       glm::dvec2& ip) {
            const double den = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x);
            if (std::abs(den) < 1e-9) return false;
            const double t = ((c.x - a.x) * (d.y - c.y) - (c.y - a.y) * (d.x - c.x)) / den;
            const double u = ((c.x - a.x) * (b.y - a.y) - (c.y - a.y) * (b.x - a.x)) / den;
            if (t <= 0.0 || t >= 1.0 || u <= 0.0 || u >= 1.0) return false;
            ip = a + (b - a) * t;
            return true;
        };
        double best = 12.0; // m from P
        int ai = -1, bi = -1;
        glm::dvec2 X(0.0);
        for (std::size_t p = 0; p < polys.size(); ++p)
            for (std::size_t q = p + 1; q < polys.size(); ++q) {
                const auto& A = polys[p].second;
                const auto& B = polys[q].second;
                for (std::size_t i = 0; i + 1 < A.size(); ++i)
                    for (std::size_t j = 0; j + 1 < B.size(); ++j) {
                        glm::dvec2 ip;
                        if (seg2({A[i].x, A[i].y}, {A[i + 1].x, A[i + 1].y},
                                 {B[j].x, B[j].y}, {B[j + 1].x, B[j + 1].y}, ip)) {
                            const double d = std::hypot(ip.x - P.x, ip.y - P.y);
                            if (d < best) { best = d; ai = (int)p; bi = (int)q; X = ip; }
                        }
                    }
            }
        if (ai < 0) {
            std::printf("[trackedit] slip: no crossing within 12 m of the selected point\n");
            return;
        }
        const auto& A = polys[ai].second;
        const auto& B = polys[bi].second;
        // Unit direction of a polyline at its nearest edge to x; and nearest point on it
        // (interpolated, carrying z).
        auto dirAt = [](const std::vector<glm::dvec3>& poly, glm::dvec2 x) {
            double bd = 1e30; glm::dvec2 dir(1, 0);
            for (std::size_t i = 0; i + 1 < poly.size(); ++i) {
                glm::dvec2 a(poly[i].x, poly[i].y), b(poly[i + 1].x, poly[i + 1].y);
                glm::dvec2 ab = b - a; double L2 = glm::dot(ab, ab);
                double t = L2 > 1e-9 ? glm::clamp(glm::dot(x - a, ab) / L2, 0.0, 1.0) : 0.0;
                double d = glm::length(x - (a + ab * t));
                if (d < bd) { bd = d; dir = L2 > 1e-9 ? ab / std::sqrt(L2) : glm::dvec2(1, 0); }
            }
            return dir;
        };
        auto snap = [](const std::vector<glm::dvec3>& poly, glm::dvec2 target) {
            double bd = 1e30; glm::dvec3 best(0.0);
            for (std::size_t i = 0; i + 1 < poly.size(); ++i) {
                glm::dvec2 a(poly[i].x, poly[i].y), b(poly[i + 1].x, poly[i + 1].y);
                glm::dvec2 ab = b - a; double L2 = glm::dot(ab, ab);
                double t = L2 > 1e-9 ? glm::clamp(glm::dot(target - a, ab) / L2, 0.0, 1.0) : 0.0;
                double d = glm::length(target - (a + ab * t));
                if (d < bd) { bd = d; best = glm::mix(poly[i], poly[i + 1], t); }
            }
            return best;
        };
        glm::dvec2 dA = dirAt(A, X), dB = dirAt(B, X);
        if (glm::dot(dA, dB) < 0.0) dB = -dB; // acute crossing
        const double theta = std::acos(glm::clamp(glm::dot(dA, dB), -1.0, 1.0));
        const double deg = glm::degrees(theta);
        // Reject only a genuinely parallel pair (no real crossing) or one so steep no
        // branch could ever take it. A shallow diamond is fine: the connecting rails run
        // longer and each meets the crossed tracks shallowly, but they are the diversion
        // lines the user wants, and the switch detector makes a switch at each end.
        if (deg < 2.0) {
            std::printf("[trackedit] slip: tracks are parallel here (%.0f deg) — no crossing\n", deg);
            return;
        }
        if (deg * 0.5 > 35.0) {
            std::printf("[trackedit] slip: crossing angle %.0f deg too steep for a switch\n", deg);
            return;
        }
        // Diversion diagonals across the crossing: each runs from one track (short of the
        // crossing) to the other (past it), so a switch forms at each end — throw both to
        // divert. Offset the ends from the crossing so the tracks are ~a track's width
        // apart there. A steep diamond yields two distinct diagonals (a double slip); a
        // shallow one collapses them to a single line (the second would just overlap), so
        // drop the near-duplicate.
        const double off = glm::clamp(4.5 / std::sin(theta), 7.0, 14.0);
        std::vector<TrackEdit> rails;
        // Reject a diagonal that duplicates one we're adding now OR one already in the
        // overlay (existing on disk + this session's pending), so re-running K on a
        // half-built slip only adds the missing diagonal instead of a duplicate.
        // Only a near-identical rail counts as a duplicate. The tolerance is tight (1 m)
        // because at a shallow diamond the two diagonals of a double slip are nearly the
        // same line (~2 m apart) yet connect different track pairs — they must both be
        // allowed. A genuine re-run reproduces a rail exactly (0 m), so 1 m still catches it.
        auto sameRail = [](const TrackEdit& r, const TrackEdit& e) {
            const bool same = std::hypot(r.a.x - e.a.x, r.a.y - e.a.y) < 1.0 &&
                              std::hypot(r.b.x - e.b.x, r.b.y - e.b.y) < 1.0;
            const bool swap = std::hypot(r.a.x - e.b.x, r.a.y - e.b.y) < 1.0 &&
                              std::hypot(r.b.x - e.a.x, r.b.y - e.a.y) < 1.0;
            return same || swap;
        };
        auto dup = [&](const TrackEdit& e) {
            for (const TrackEdit& r : rails) if (sameRail(r, e)) return true;
            for (const TrackEdit& r : pending)
                if (r.kind == TrackEdit::Rail && sameRail(r, e)) return true;
            for (const TrackEdit& r : existing)
                if (r.kind == TrackEdit::Rail && sameRail(r, e)) return true;
            return false;
        };
        // If a diagonal's end lands on a track's *terminus* (a siding stubbing into the
        // diamond, e.g. one line ending where the next begins), hop onto the track that
        // continues collinearly from there, so the connector reaches a through track and a
        // real switch forms at each end (rather than skipping that diagonal).
        auto resolveEnd = [&](const std::vector<glm::dvec3>& poly0,
                              glm::dvec2 target) -> glm::dvec3 {
            const std::vector<glm::dvec3>* cur = &poly0;
            glm::dvec3 p = snap(*cur, target);
            for (int hop = 0; hop < 2; ++hop) {
                const glm::dvec3 f = cur->front(), b = cur->back();
                const bool atF = std::hypot(f.x - p.x, f.y - p.y) < 1.5;
                const bool atB = std::hypot(b.x - p.x, b.y - p.y) < 1.5;
                if (!atF && !atB) break;
                const glm::dvec3 term = atF ? f : b;
                glm::dvec2 want(target.x - term.x, target.y - term.y);
                const double wl = glm::length(want);
                if (wl < 1e-6) break;
                want /= wl;
                const std::vector<glm::dvec3>* cont = nullptr;
                double bestAlign = 0.3;
                for (const auto& pr : polys) {
                    const auto& pl = pr.second;
                    if (&pl == cur || pl.size() < 2) continue;
                    for (int end = 0; end < 2; ++end) {
                        const glm::dvec3 e = end ? pl.back() : pl.front();
                        if (std::hypot(e.x - term.x, e.y - term.y) > 2.5) continue;
                        const glm::dvec3 nb = end ? pl[pl.size() - 2] : pl[1];
                        glm::dvec2 dir(nb.x - e.x, nb.y - e.y);
                        const double L = glm::length(dir);
                        if (L < 1e-6) continue;
                        const double a = glm::dot(dir / L, want);
                        if (a > bestAlign) { bestAlign = a; cont = &pl; }
                    }
                }
                if (!cont) break; // a genuine dead end
                cur = cont;
                p = snap(*cur, target);
            }
            return p;
        };
        for (double s : {1.0, -1.0}) {
            TrackEdit e; e.kind = TrackEdit::Rail;
            e.a = resolveEnd(A, X - dA * (off * s));
            e.b = resolveEnd(B, X + dB * (off * s));
            if (std::hypot(e.a.x - e.b.x, e.a.y - e.b.y) > 2.0 && !dup(e))
                rails.push_back(e);
        }
        if (rails.empty()) {
            std::printf("[trackedit] slip: nothing to add (already built here?)\n");
            return;
        }
        applyEditsLive(rails);
        std::printf("[trackedit] slip: added %zu rail(s) at %#x x %#x (%.0f deg) "
                    "(preview; Ctrl+S to save)\n", rails.size(), polys[ai].first,
                    polys[bi].first, glm::degrees(theta));
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
                "Tab release cursor, Esc menu\n"
                "Select (cursor freed with Tab): click a geo-point, Ctrl+click for "
                "several, click empty to clear.\n"
                "Edit: G straighten the selected span's grade; Up/Down raise/lower the "
                "selection; N cycle the selection through coincident points (shared "
                "nodes); R add a connecting rail between 2 selected points on different "
                "tracks (builds a switch at each end); C build a scissors (double) "
                "crossover between 2 selected points on two parallel tracks; K auto-build "
                "a slip switch at the crossing under the selected point; J "
                "join a selected dead-end onto the track it crosses; Enter/Enter+L link "
                "two red dead ends. P re-renders the actual track + terrain + switch "
                "stands to preview how the edits look. Live wireframe preview; Ctrl+S "
                "saves.\n"
                "Overlay: amber = main line, cyan = siding, magenta = yard, "
                "red = dead end, white = selected.\n\n");

    const glm::vec3 sunDir = glm::normalize(glm::vec3(0.4f, -0.5f, 0.75f));
    const char* shotPath = std::getenv("EBANER_SCREENSHOT");
    int frame = 0;
    bool prevEnter = false, prevL = false, prevX = false, prevML = false,
         prevG = false, prevS = false, prevUp = false, prevDown = false,
         prevJ = false, prevN = false, prevR = false, prevK = false, prevC = false,
         prevP = false;
    bool prevMenuEnter = false, prevMenuUp = false, prevMenuDown = false;
    const std::vector<std::string> kMenuItems = {"Exit"};
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

        if (g_menuOpen) {
            // --- Escape menu: draw over the frozen scene, ignore editing input ---
            auto down = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
            const bool mU = down(GLFW_KEY_UP), mD = down(GLFW_KEY_DOWN),
                       mE = down(GLFW_KEY_ENTER);
            const int n = static_cast<int>(kMenuItems.size());
            if (mU && !prevMenuUp) g_menuSel = (g_menuSel + n - 1) % n;
            if (mD && !prevMenuDown) g_menuSel = (g_menuSel + 1) % n;
            if (mE && !prevMenuEnter && kMenuItems[g_menuSel] == "Exit")
                glfwSetWindowShouldClose(window, GLFW_TRUE);
            prevMenuUp = mU; prevMenuDown = mD; prevMenuEnter = mE;

            std::vector<TextVertex> tv;
            appendMenu(tv, "MENU", kMenuItems, g_menuSel, fbw, fbh);
            renderer.setOverlayText(tv);

            const float aspect = static_cast<float>(fbw) / static_cast<float>(fbh);
            PushConstants pc{};
            pc.viewProj = g_camera.projMatrix(aspect) * g_camera.viewMatrix();
            pc.sunDir = glm::vec4(sunDir, data.minElevation());
            pc.camPos = glm::vec4(g_camera.position(), data.maxElevation());
            pc.params = glm::vec4(0.5f, 0.0f, 0.0f, 0.0f);
            if (shotPath) {
                if (frame == 20) renderer.requestCapture(shotPath);
                if (frame == 24) glfwSetWindowShouldClose(window, GLFW_TRUE);
            }
            renderer.drawFrame(pc);
            ++frame;
            continue;
        }

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
        const bool kN = glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS;
        const bool kR = glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS;
        const bool kK = glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS;
        const bool kC = glfwGetKey(window, GLFW_KEY_C) == GLFW_PRESS;
        const bool kP = glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS;
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
        // N: cycle the selection through geo-points coincident with it (shared nodes).
        if (kN && !prevN) doCycleCoincident();
        // R: add a connecting rail between 2 selected points (builds a switch each end).
        if (kR && !prevR) doAddRail();
        // K: auto-build a slip (kryssveksel) at the crossing under the selected point.
        if (kK && !prevK) doAutoDiamond();
        // C: build a scissors (double) crossover between the 2 selected tracks' points.
        if (kC && !prevC) doScissors();
        // P: re-render the actual track + terrain (cuttings) + switch stands from the
        // current edits, so the preview shows how it will really look (heavy; on demand).
        if (kP && !prevP) rebuildRenderPreview();
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
        // Ctrl+S: rewrite the overlay = kept existing lines + this session's edits,
        // dropping any staged-for-removal (ambiguous) overrides.
        const std::size_t removals =
            static_cast<std::size_t>(std::count(removeExisting.begin(),
                                                removeExisting.end(), char(1)));
        if (kSave && !prevS && (!pending.empty() || removals > 0)) {
            std::vector<TrackEdit> out;
            for (std::size_t k = 0; k < existing.size(); ++k)
                if (!removeExisting[k]) out.push_back(existing[k]);
            out.insert(out.end(), pending.begin(), pending.end());
            if (writeTrackOverlay(datasetRoot, out)) {
                std::printf("[trackedit] saved: %zu line(s) (%zu added, %zu removed) -> "
                            "%s/overlay/track-edits.txt\n", out.size(), pending.size(),
                            removals, datasetRoot.c_str());
                existing = std::move(out);            // new on-disk baseline
                removeExisting.assign(existing.size(), 0);
                pending.clear();
            } else {
                std::fprintf(stderr, "[trackedit] failed to write overlay file\n");
            }
        }
        prevEnter = kEnter; prevL = kL; prevX = kX; prevG = kG; prevS = kSave;
        prevJ = kJ; prevN = kN; prevR = kR; prevK = kK; prevC = kC; prevP = kP;

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
                    std::min(static_cast<float>(fbw) - 20.0f, 40.0f + 76.0f * 8.0f * sc);
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
            char selz[64] = "";
            if (!selected.empty()) {
                double zmin = 1e9, zmax = -1e9;
                for (int i : selected) {
                    const double z = graph.pointWorld[i].z;
                    zmin = std::min(zmin, z); zmax = std::max(zmax, z);
                }
                if (selected.size() == 1) // show the track id so coincident points differ
                    std::snprintf(selz, sizeof(selz), " trk %#x z=%.2f",
                                  graph.pointTrack[*selected.begin()], zmin);
                else std::snprintf(selz, sizeof(selz), " z=%.2f..%.2f", zmin, zmax);
            }
            std::snprintf(buf, sizeof(buf),
                          "SELECTED %zu%s   keys: G Up/Dn N J R K C   P render",
                          selected.size(), selz);
            appendText(tv, buf, x, 40.0f + 4 * lh, sc,
                       selected.empty() ? glm::vec3(0.7f, 0.85f, 0.7f)
                                        : glm::vec3(1.0f, 0.9f, 0.3f),
                       fbw, fbh);
            const std::size_t rmv = static_cast<std::size_t>(
                std::count(removeExisting.begin(), removeExisting.end(), char(1)));
            char rmvbuf[48] = "";
            if (rmv) std::snprintf(rmvbuf, sizeof(rmvbuf), "  %zu ambiguous to remove", rmv);
            std::snprintf(buf, sizeof(buf), "UNSAVED %zu%s   %s", pending.size(), rmvbuf,
                          (pending.empty() && rmv == 0) ? "" : "Ctrl+S to save");
            appendText(tv, buf, x, 40.0f + 5 * lh, sc,
                       (pending.empty() && rmv == 0) ? glm::vec3(0.6f, 0.9f, 0.6f)
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
