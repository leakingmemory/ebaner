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
#include "SignalMesh.h"
#include "SignalPaths.h"
#include "SpeedLimits.h"
#include "SpeedSignMesh.h"
#include "TunnelMesh.h"
#include "SwitchTypes.h"
#include "TerrainData.h"
#include "TerrainMesh.h"
#include "Textures.h"
#include "TrackCircuits.h"
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
bool g_naming = false;    // typing a section name (modal text entry)
std::string g_nameBuf;    // the name being typed
VulkanRenderer* g_renderer = nullptr;

// Text entry for section names: printable characters go into g_nameBuf while naming.
// Space -> '_' and non-token characters are dropped so the name stays a single token
// (the overlay stores `section <id> <name> ...` whitespace-delimited).
void charCallback(GLFWwindow*, unsigned int cp) {
    if (!g_naming) return;
    // Names are read back by people (and will label the eligible routes when route setting
    // arrives), so spaces are allowed - the overlay files quote them. A double quote is the
    // one printable character that cannot appear, being the delimiter itself.
    if (cp == '"') return;
    if (cp >= 32 && cp < 127 && g_nameBuf.size() < 48) g_nameBuf.push_back(static_cast<char>(cp));
}

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
    if (g_naming) return; // typing a name: only charCallback + loop-polled edit keys act
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
    TunnelMesh tunnels;
    TrackMesh tracks;
    RoadMesh roads;
    BuildingMesh buildings;
    PlatformMesh platforms;
    SpeedSignMesh speedSignMesh;
    SwitchMesh switches;
    SignalMesh signals;
    SwitchNetwork switchNet;
    TrackGraph graph;
    std::vector<TrackPath> paths;
    try {
        data.load(datasetRoot);
        paths = buildTrackPaths(data);
        // The bores first: the terrain needs them to know which of its triangles stand in
        // a tunnel mouth. A geometry edit can move a portal, so both are rebuilt together.
        tunnels.build(data);
        mesh.build(data, &tunnels);
        tracks.build(paths, glm::vec3(0.0f), data.loadedRadius());
        roads.build(data);
        buildings.build(data);
        platforms.build(data, paths);
        switchNet.build(data, paths);   // turnout detection + routing (all straight)
        applySwitchTypes(switchNet, loadSwitchTypes(datasetRoot)); // manual/motor overrides
        switches.build(switchNet, glm::vec3(0.0f), data.loadedRadius());
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
    glfwSetCharCallback(window, charCallback);
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

    // --- Modes: geometry editing (default), track-circuit (sensing-section) authoring,
    // switch (turnout) properties, mini signal paths and exit (main) signals, switched
    // from the Escape menu. ---
    enum class EdMode {
        Geometry, Circuits, Switches, SignalPaths, ExitSignals, EntrySignals, DistantSignals
    };
    EdMode mode = EdMode::Geometry;
    int selTurnout = -1;             // selected turnout (index into switchNet.turnouts())
    bool switchTypesDirty = false;   // unsaved switch-type changes
    // Mini signal paths (own overlay): directional routes between two circuit borders.
    std::vector<SignalPath> signalPaths = loadSignalPaths(datasetRoot);
    bool pathsDirty = false;         // unsaved signal-path changes
    int pathStart = -1;              // in-progress path's start border (index into tc.borders)
    std::vector<Border> pendingVias; // borders the in-progress path must pass through
    int selPath = -1;                // selected signal path (index into signalPaths)
    int nextPathId = 1;              // auto-increment path id
    for (const SignalPath& p : signalPaths) nextPathId = std::max(nextPathId, p.id + 1);
    std::string pathMsg;             // transient feedback (no route / ambiguous)
    double pathMsgUntil = 0.0;       // glfwGetTime() until which pathMsg shows
    // Exit (main) signals: authored exactly like a mini path - the signal stands on the
    // start border and protects the route to the destination - but kept in their own
    // collection and overlay file. The two modes share the handlers below.
    std::vector<SignalPath> exitSignals = loadExitSignals(datasetRoot);
    bool exitDirty = false;          // unsaved exit-signal changes
    int selExit = -1;                // selected exit signal (index into exitSignals)
    int nextExitId = 1;              // auto-increment exit-signal id
    for (const SignalPath& e : exitSignals) nextExitId = std::max(nextExitId, e.id + 1);
    // Exit routes: the authority to move from a border inside the station up to an exit
    // signal. A main signal's authority starts back at the platform road, and one signal
    // commonly serves several roads, so these are many-to-one onto an exit signal.
    std::vector<SignalPath> exitRoutes = loadExitRoutes(datasetRoot);
    bool exitRoutesDirty = false;    // unsaved exit-route changes
    int selExitRoute = -1;           // selected exit route; exclusive with selExit
    int armedExit = -1;              // B pressed: the exit signal a new route is drawn to
    int nextExitRouteId = 1;         // auto-increment exit-route id
    for (const SignalPath& r : exitRoutes) nextExitRouteId = std::max(nextExitRouteId, r.id + 1);
    // Entry signals: the mast stands on the start border and the record is the whole route
    // into the station, so unlike an exit there is no separate approach to author. Several
    // sharing a start border become one signal governing them all.
    std::vector<SignalPath> entrySignals = loadEntrySignals(datasetRoot);
    bool entryDirty = false;         // unsaved entry-signal changes
    int selEntry = -1;               // selected entry route (index into entrySignals)
    int nextEntryId = 1;             // auto-increment entry-signal id
    for (const SignalPath& e : entrySignals) nextEntryId = std::max(nextEntryId, e.id + 1);
    // Distant signals: a plain point along a track with a facing, repeating the first main
    // signal ahead. Not a route, so none of the route-mode bindings apply to them.
    std::vector<DistantSignal> distantSignals = loadDistantSignals(datasetRoot);
    bool distantDirty = false;       // unsaved distant-signal changes
    int selDistant = -1;             // selected distant (index into distantSignals)
    int nextDistantId = 1;           // auto-increment distant id
    for (const DistantSignal& d : distantSignals)
        nextDistantId = std::max(nextDistantId, d.id + 1);
    TrackCircuits tc = loadTrackCircuits(datasetRoot); // borders + sections (own overlay)
    bool circuitsDirty = false;      // unsaved circuit changes
    int selBorder = -1;              // selected border (index into tc.borders)
    bool moveArmed = false;          // M pressed: the next track click moves selBorder
    int selSection = -1;             // selected section (index into tc.sections)
    // What the shared rename modal is editing. Every route carries a real name - they are
    // what the operator will pick from when setting a route - so creating one opens this
    // straight away, pre-filled with the auto default.
    enum class NameTarget { None, Section, Path, Exit, ExitRoute, Entry, Distant };
    NameTarget namingWhat = NameTarget::None;
    int namingIdx = -1;              // index into whichever collection that names
    int nextSectionId = 1;           // auto-increment section id
    SectionResult pendingFlood;      // last flood result, highlighted as a preview
    bool showPending = false;
    // Every section carries a name (used later by signaling); give existing data loaded
    // without one a default "S<id>" so it is always identifiable.
    for (Section& s : tc.sections)
        if (s.name.empty() || s.name == "-") s.name = "S" + std::to_string(s.id);
    // The collection a NameTarget refers to, so the modal and its callers stay in step.
    auto namedRoutes = [&](NameTarget t) -> std::vector<SignalPath>* {
        switch (t) {
        case NameTarget::Path: return &signalPaths;
        case NameTarget::Exit: return &exitSignals;
        case NameTarget::ExitRoute: return &exitRoutes;
        case NameTarget::Entry: return &entrySignals;
        default: return nullptr;
        }
    };
    // Open the rename modal on something, pre-filled with its current name. Used both by F2
    // and straight after a create, so nothing is ever left carrying only its auto default.
    auto beginNaming = [&](NameTarget what, int idx) {
        const std::vector<SignalPath>* rs = namedRoutes(what);
        const std::string* cur = nullptr;
        if (rs && idx >= 0 && idx < static_cast<int>(rs->size())) cur = &(*rs)[idx].name;
        else if (what == NameTarget::Section && idx >= 0 &&
                 idx < static_cast<int>(tc.sections.size()))
            cur = &tc.sections[idx].name;
        else if (what == NameTarget::Distant && idx >= 0 &&
                 idx < static_cast<int>(distantSignals.size()))
            cur = &distantSignals[idx].name;
        if (!cur) return;
        namingWhat = what;
        namingIdx = idx;
        g_nameBuf = *cur == "-" ? "" : *cur;
        g_naming = true;
    };
    // Per-track world polylines (rebuilt from graph), for border projection/rendering
    // and the section flood-fill. Kept in sync with the graph.
    std::vector<TrackPoly> polys;
    // The junction graph the distant-signal walk follows. Geometry edits can change it, so
    // unlike the viewer it is rebuilt alongside the polylines.
    TrackJunctions junctions;
    auto buildPolys = [&]() {
        polys.clear();
        for (std::size_t i = 0; i < graph.pointWorld.size(); ++i) {
            if (polys.empty() || polys.back().id != graph.pointTrack[i])
                polys.push_back({graph.pointTrack[i], {}});
            polys.back().pts.push_back(graph.pointWorld[i]);
        }
        junctions = trackJunctions(polys);
    };
    buildPolys();
    for (const Section& s : tc.sections) nextSectionId = std::max(nextSectionId, s.id + 1);
    // Resolve each motor switch's locking set now that polys + circuits exist (authored
    // set, else the sections the switch sits within). Editable below in Switches mode.
    applySwitchLocks(switchNet, loadSwitchTypes(datasetRoot), tc, polys);
    // The exit and entry masts as one list, the shape mergeSignals wants. Rebuilt on demand
    // because either collection can change while editing.
    auto mainPlacements = [&]() {
        std::vector<SignalPlacement> out =
            signalPlacements(exitSignals, polys, SignalKind::Exit);
        const std::vector<SignalPlacement> entries =
            signalPlacements(entrySignals, polys, SignalKind::Entry);
        out.insert(out.end(), entries.begin(), entries.end());
        return out;
    };
    // A distant stands at a plain point rather than on a route, so it is placed directly and
    // appended after the merge - it must never fold onto a dwarf's pole.
    auto allPlacements = [&]() {
        std::vector<SignalPlacement> out =
            mergeSignals(signalPlacements(signalPaths, polys), mainPlacements());
        for (std::size_t i = 0; i < distantSignals.size(); ++i) {
            const DistantSignal& d = distantSignals[i];
            const glm::dvec3 w = fracToWorld(polys, d.trackId, d.frac);
            if (w.x == 0.0 && w.y == 0.0) continue;
            SignalPlacement sp;
            sp.kind = SignalKind::Distant;
            sp.world = w;
            sp.forward = trackTangent(polys, d.trackId, d.frac, d.dir);
            sp.at = {d.trackId, d.frac};
            sp.paths.push_back(static_cast<int>(i));
            out.push_back(std::move(sp));
        }
        return out;
    };

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
        if (mode == EdMode::Circuits) {
            const glm::dvec3 o = data.sceneOrigin();
            auto sc = [&](glm::dvec3 w, float lift) {
                return glm::vec3(float(w.x - o.x), float(w.y - o.y), float(w.z - o.z) + lift);
            };
            auto dirAt = [&](std::uint32_t t, double f) {
                glm::dvec3 a = fracToWorld(polys, t, std::max(0.0, f - 0.002));
                glm::dvec3 b = fracToWorld(polys, t, std::min(1.0, f + 0.002));
                glm::dvec2 d(b.x - a.x, b.y - a.y);
                const double L = glm::length(d);
                return L > 1e-6 ? glm::dvec2(d / L) : glm::dvec2(1.0, 0.0);
            };
            // Vivid section hues, distinct from cyan sidings and red borders.
            const glm::vec3 secPal[] = {{0.2f, 1.0f, 0.4f}, {1.0f, 0.2f, 1.0f},
                                        {1.0f, 0.6f, 0.1f}, {1.0f, 1.0f, 0.2f},
                                        {0.6f, 0.3f, 1.0f}};
            auto drawRun = [&](const SectionInterval& iv, glm::vec3 col) {
                glm::dvec3 prev = fracToWorld(polys, iv.trackId, iv.from);
                if (prev.x == 0.0 && prev.y == 0.0) return; // track gone (stale)
                for (int k = 1; k <= 24; ++k) {
                    const double f = iv.from + (iv.to - iv.from) * k / 24.0;
                    const glm::dvec3 cur = fracToWorld(polys, iv.trackId, f);
                    // Float the ribbon ~1.2 m above the rail so it reads clearly, distinct
                    // from the ground-level graph lines.
                    lns.push_back({sc(prev, 1.2f), col});
                    lns.push_back({sc(cur, 1.2f), col});
                    prev = cur;
                }
            };
            for (std::size_t si = 0; si < tc.sections.size(); ++si) {
                const glm::vec3 col = static_cast<int>(si) == selSection
                    ? glm::vec3(1.0f, 1.0f, 1.0f)               // selected -> white
                    : secPal[((tc.sections[si].id - 1) % 5 + 5) % 5];
                for (const auto& iv : tc.sections[si].parts) drawRun(iv, col);
            }
            if (showPending) {
                const glm::vec3 col = pendingFlood.enclosed ? glm::vec3(0.3f, 1.0f, 0.5f)
                                                            : glm::vec3(1.0f, 0.5f, 0.1f);
                for (const auto& iv : pendingFlood.parts) drawRun(iv, col);
            }
            for (std::size_t bi = 0; bi < tc.borders.size(); ++bi) {
                const Border& b = tc.borders[bi];
                const glm::dvec3 w = fracToWorld(polys, b.trackId, b.frac);
                if (w.x == 0.0 && w.y == 0.0) continue; // track gone
                const glm::dvec2 d = dirAt(b.trackId, b.frac);
                const glm::dvec3 perp(-d.y, d.x, 0.0);
                const glm::vec3 col = static_cast<int>(bi) == selBorder
                                          ? glm::vec3(1.0f, 1.0f, 1.0f)
                                          : glm::vec3(1.0f, 0.15f, 0.15f);
                // A visible gate: a perpendicular tick across the rails plus a vertical
                // post, and a point marker on top — clear from any angle.
                lns.push_back({sc(w - perp * 2.5, 0.6f), col});
                lns.push_back({sc(w + perp * 2.5, 0.6f), col});
                lns.push_back({sc(w, 0.3f), col});
                lns.push_back({sc(w, 4.0f), col});
                pts.push_back({sc(w, 4.0f), col});
                pts.push_back({sc(w, 0.7f), col});
            }
        } else if (mode == EdMode::Switches) {
            // A marker at each working turnout, coloured by type (manual = amber,
            // motor = cyan); the selected one is white and larger.
            const glm::dvec3 o = data.sceneOrigin();
            const auto& tos = switchNet.turnouts();
            // The selected motor switch's locking circuits, drawn as red-orange ribbons
            // (float above the rail) so the authored set is visible.
            if (selTurnout >= 0 && switchNet.type(selTurnout) == SwitchType::Motor) {
                auto scv = [&](glm::dvec3 w, float lift) {
                    return glm::vec3(float(w.x - o.x), float(w.y - o.y), float(w.z - o.z) + lift);
                };
                const glm::vec3 lockCol(1.0f, 0.35f, 0.1f);
                for (int id : switchNet.lock(selTurnout))
                    for (const Section& s : tc.sections) {
                        if (s.id != id) continue;
                        for (const SectionInterval& iv : s.parts) {
                            glm::dvec3 prev = fracToWorld(polys, iv.trackId, iv.from);
                            if (prev.x == 0.0 && prev.y == 0.0) continue;
                            for (int k = 1; k <= 24; ++k) {
                                const double f = iv.from + (iv.to - iv.from) * k / 24.0;
                                const glm::dvec3 cur = fracToWorld(polys, iv.trackId, f);
                                lns.push_back({scv(prev, 1.4f), lockCol});
                                lns.push_back({scv(cur, 1.4f), lockCol});
                                prev = cur;
                            }
                        }
                    }
            }
            for (std::size_t i = 0; i < tos.size(); ++i) {
                if (tos[i].mainPath < 0) continue; // inert crossing: no working switch
                const bool selh = static_cast<int>(i) == selTurnout;
                const bool motor = switchNet.type(static_cast<int>(i)) == SwitchType::Motor;
                const glm::vec3 col = selh    ? glm::vec3(1.0f, 1.0f, 1.0f)
                                      : motor ? glm::vec3(0.3f, 0.8f, 1.0f)
                                              : glm::vec3(1.0f, 0.7f, 0.2f);
                const glm::vec3 c(static_cast<float>(tos[i].world.x - o.x),
                                  static_cast<float>(tos[i].world.y - o.y),
                                  static_cast<float>(tos[i].world.z - o.z) + 3.0f);
                const float r = selh ? 3.5f : 2.2f; // m, marker half-diagonal
                const glm::vec3 n(0, r, 0), s(0, -r, 0), e(r, 0, 0), w(-r, 0, 0);
                auto seg = [&](glm::vec3 a, glm::vec3 b) {
                    lns.push_back({c + a, col}); lns.push_back({c + b, col});
                };
                seg(n, e); seg(e, s); seg(s, w); seg(w, n); // diamond outline
                pts.push_back({c, col});
            }
        } else if (mode == EdMode::DistantSignals) {
            const glm::dvec3 o = data.sceneOrigin();
            auto scv = [&](glm::dvec3 w, float lift) {
                return glm::vec3(float(w.x - o.x), float(w.y - o.y), float(w.z - o.z) + lift);
            };
            const std::vector<SignalPlacement> pl = allPlacements();
            for (std::size_t i = 0; i < distantSignals.size(); ++i) {
                const DistantSignal& d = distantSignals[i];
                const glm::dvec3 w = fracToWorld(polys, d.trackId, d.frac);
                if (w.x == 0.0 && w.y == 0.0) continue;
                const bool sel = static_cast<int>(i) == selDistant;
                const glm::vec3 col = sel ? glm::vec3(1.0f, 1.0f, 1.0f)
                                          : glm::vec3(1.0f, 0.72f, 0.15f);
                lns.push_back({scv(w, 0.3f), col});
                lns.push_back({scv(w, 5.0f), col});
                pts.push_back({scv(w, 5.0f), col});
                // An arrow along the facing: which way it reads is the whole configuration,
                // so it has to be visible without selecting the thing.
                const glm::dvec2 f = trackTangent(polys, d.trackId, d.frac, d.dir);
                const glm::dvec2 p(-f.y, f.x);
                const glm::dvec3 tip(w.x + f.x * 18.0, w.y + f.y * 18.0, w.z);
                lns.push_back({scv(w, 1.2f)}); lns.back().color = col;
                lns.push_back({scv(tip, 1.2f), col});
                for (double side : {-1.0, 1.0}) {
                    const glm::dvec3 b(tip.x - f.x * 5.0 + p.x * 2.5 * side,
                                       tip.y - f.y * 5.0 + p.y * 2.5 * side, tip.z);
                    lns.push_back({scv(tip, 1.2f), col});
                    lns.push_back({scv(b, 1.2f), col});
                }
                // The selected one shows the road it currently sees, and what it reads at
                // the end of it - the only way to check the switch-following on a layout
                // with any complexity to it.
                if (!sel) continue;
                std::vector<SectionInterval> walked;
                const int hit = firstMainSignalAhead(polys, junctions, switchNet, pl,
                                                     d.trackId, d.frac, d.dir, kDistantReach,
                                                     &walked);
                const glm::vec3 rc = hit >= 0 ? glm::vec3(0.3f, 1.0f, 0.5f)
                                              : glm::vec3(1.0f, 0.45f, 0.2f);
                for (const SectionInterval& iv : walked) {
                    glm::dvec3 prev = fracToWorld(polys, iv.trackId, iv.from);
                    if (prev.x == 0.0 && prev.y == 0.0) continue;
                    for (int k = 1; k <= 24; ++k) {
                        const double t = iv.from + (iv.to - iv.from) * k / 24.0;
                        const glm::dvec3 c = fracToWorld(polys, iv.trackId, t);
                        lns.push_back({scv(prev, 1.0f), rc});
                        lns.push_back({scv(c, 1.0f), rc});
                        prev = c;
                    }
                }
                if (hit >= 0) {
                    const glm::dvec3 sw = pl[hit].world;
                    lns.push_back({scv(sw, 0.3f), rc});
                    lns.push_back({scv(sw, 7.0f), rc});
                    pts.push_back({scv(sw, 7.0f), rc});
                }
            }
        } else if (mode == EdMode::SignalPaths || mode == EdMode::ExitSignals ||
                   mode == EdMode::EntrySignals) {
            // All three draw the same thing: the routes, their vias and the in-progress
            // start; only the collection and the hue differ.
            const bool exitMode = mode == EdMode::ExitSignals;
            const bool entryMode = mode == EdMode::EntrySignals;
            const std::vector<SignalPath>& routes = exitMode    ? exitSignals
                                                    : entryMode ? entrySignals
                                                                : signalPaths;
            const int selRoute = exitMode ? selExit : entryMode ? selEntry : selPath;
            const glm::dvec3 o = data.sceneOrigin();
            auto scv = [&](glm::dvec3 w, float lift) {
                return glm::vec3(float(w.x - o.x), float(w.y - o.y), float(w.z - o.z) + lift);
            };
            // Vivid path hues (distinct from cyan sidings / red borders), white when selected.
            const glm::vec3 pathPal[] = {{0.2f, 1.0f, 0.5f}, {1.0f, 0.5f, 1.0f},
                                         {1.0f, 0.8f, 0.2f}, {0.4f, 0.8f, 1.0f},
                                         {0.7f, 0.5f, 1.0f}};
            // Exit signals get their own warm palette, so a route protected by a main
            // signal never reads as a shunting path.
            const glm::vec3 exitPal[] = {{1.0f, 0.35f, 0.3f}, {1.0f, 0.6f, 0.15f},
                                         {1.0f, 0.45f, 0.6f}, {0.95f, 0.75f, 0.35f},
                                         {1.0f, 0.3f, 0.55f}};
            // Exit routes get a cooler palette again, so the three kinds of ribbon on
            // screen - shunting path, the signal's protected route, the authority up to it -
            // never blur together.
            const glm::vec3 approachPal[] = {{0.35f, 0.85f, 1.0f}, {0.55f, 0.7f, 1.0f},
                                             {0.4f, 1.0f, 0.85f}, {0.7f, 0.8f, 1.0f},
                                             {0.3f, 0.65f, 1.0f}};
            auto drawRoute = [&](const SignalPath& p, const glm::vec3& col, bool mast) {
                // Mast marker at the border the signal itself stands on.
                if (mast) {
                    const glm::dvec3 w = fracToWorld(polys, p.start.trackId, p.start.frac);
                    if (!(w.x == 0.0 && w.y == 0.0)) {
                        lns.push_back({scv(w, 0.3f), col});
                        lns.push_back({scv(w, 6.0f), col});
                        pts.push_back({scv(w, 6.0f), col});
                    }
                }
                // Directional ribbon: sample each interval from -> to (travel direction).
                for (const SectionInterval& iv : p.parts) {
                    glm::dvec3 prev = fracToWorld(polys, iv.trackId, iv.from);
                    if (prev.x == 0.0 && prev.y == 0.0) continue;
                    for (int k = 1; k <= 24; ++k) {
                        const double f = iv.from + (iv.to - iv.from) * k / 24.0;
                        const glm::dvec3 cur = fracToWorld(polys, iv.trackId, f);
                        lns.push_back({scv(prev, 1.6f), col});
                        lns.push_back({scv(cur, 1.6f), col});
                        prev = cur;
                    }
                }
                // Arrowhead at the destination, pointing along the travel direction.
                if (!p.parts.empty()) {
                    const SectionInterval& last = p.parts.back();
                    const glm::dvec3 tip = fracToWorld(polys, last.trackId, last.to);
                    const double back = last.to + (last.from - last.to) * 0.02; // a bit upstream
                    const glm::dvec3 bpt = fracToWorld(polys, last.trackId, back);
                    glm::dvec2 dir(tip.x - bpt.x, tip.y - bpt.y);
                    const double L = glm::length(dir);
                    dir = L > 1e-6 ? dir / L : glm::dvec2(1.0, 0.0);
                    const glm::dvec2 perp(-dir.y, dir.x);
                    const double al = 6.0, aw = 3.0; // m
                    const glm::vec3 tc3 = scv(tip, 1.6f);
                    const glm::vec3 l3(float(tip.x - dir.x * al + perp.x * aw - o.x),
                                       float(tip.y - dir.y * al + perp.y * aw - o.y),
                                       float(tip.z - o.z) + 1.6f);
                    const glm::vec3 r3(float(tip.x - dir.x * al - perp.x * aw - o.x),
                                       float(tip.y - dir.y * al - perp.y * aw - o.y),
                                       float(tip.z - o.z) + 1.6f);
                    lns.push_back({tc3, col}); lns.push_back({l3, col});
                    lns.push_back({tc3, col}); lns.push_back({r3, col});
                }
            };
            // An entry signal's own palette, cool and distinct from both the exit's warm
            // hues and the shunting greens.
            const glm::vec3 entryPal[] = {{0.45f, 1.0f, 1.0f}, {0.3f, 0.9f, 0.8f},
                                          {0.6f, 1.0f, 0.9f}, {0.35f, 0.75f, 0.9f},
                                          {0.5f, 0.95f, 0.75f}};
            for (std::size_t pi = 0; pi < routes.size(); ++pi)
                drawRoute(routes[pi],
                          static_cast<int>(pi) == selRoute
                              ? glm::vec3(1.0f, 1.0f, 1.0f)
                              : (exitMode    ? exitPal
                                 : entryMode ? entryPal
                                             : pathPal)[((routes[pi].id - 1) % 5 + 5) % 5],
                          exitMode || entryMode); // a mast marker at the signal's border
            if (exitMode) {
                for (std::size_t ri = 0; ri < exitRoutes.size(); ++ri) {
                    const SignalPath& r = exitRoutes[ri];
                    // A route whose signal is gone can only come from a hand-edited file
                    // (deleting a signal takes its routes with it); flag it rather than
                    // drawing it as if it still led somewhere.
                    const bool dangling = exitRouteTarget(r, exitSignals, polys) < 0;
                    const glm::vec3 col =
                        static_cast<int>(ri) == selExitRoute ? glm::vec3(1.0f, 1.0f, 1.0f)
                        : dangling                          ? glm::vec3(1.0f, 0.15f, 0.15f)
                                     : approachPal[((r.id - 1) % 5 + 5) % 5];
                    drawRoute(r, col, false);
                }
                // The signal a route is being drawn to, ringed so the target is obvious.
                if (armedExit >= 0 && armedExit < static_cast<int>(exitSignals.size())) {
                    const Border& b = exitSignals[armedExit].start;
                    const glm::dvec3 w = fracToWorld(polys, b.trackId, b.frac);
                    if (!(w.x == 0.0 && w.y == 0.0)) {
                        const glm::vec3 col(1.0f, 1.0f, 0.3f);
                        for (int k = 0; k < 16; ++k) {
                            const double a0 = 6.2831853 * k / 16, a1 = 6.2831853 * (k + 1) / 16;
                            const glm::dvec3 p0(w.x + std::cos(a0) * 6.0,
                                                w.y + std::sin(a0) * 6.0, w.z);
                            const glm::dvec3 p1(w.x + std::cos(a1) * 6.0,
                                                w.y + std::sin(a1) * 6.0, w.z);
                            lns.push_back({scv(p0, 0.4f), col});
                            lns.push_back({scv(p1, 0.4f), col});
                        }
                        lns.push_back({scv(w, 0.3f), col});
                        lns.push_back({scv(w, 7.0f), col});
                    }
                }
            }
            // Via points: the pending ones while authoring, and the selected path's
            // stored ones, so it is visible which road was picked and why.
            {
                auto drawVia = [&](const Border& v, const glm::vec3& col) {
                    const glm::dvec3 w = fracToWorld(polys, v.trackId, v.frac);
                    if (w.x == 0.0 && w.y == 0.0) return;
                    lns.push_back({scv(w, 0.3f), col});
                    lns.push_back({scv(w, 5.5f), col});
                    pts.push_back({scv(w, 5.5f), col});
                    // a small cross, so a via reads differently from a plain border
                    for (int a2 = 0; a2 < 2; ++a2) {
                        const glm::dvec3 d = a2 ? glm::dvec3(0, 3.0, 0) : glm::dvec3(3.0, 0, 0);
                        lns.push_back({scv(w - d, 5.5f), col});
                        lns.push_back({scv(w + d, 5.5f), col});
                    }
                };
                for (const Border& v : pendingVias) drawVia(v, glm::vec3(1.0f, 0.6f, 0.0f));
                if (selRoute >= 0 && selRoute < static_cast<int>(routes.size()))
                    for (const Border& v : routes[selRoute].vias)
                        drawVia(v, glm::vec3(1.0f, 0.85f, 0.3f));
                if (exitMode && selExitRoute >= 0 &&
                    selExitRoute < static_cast<int>(exitRoutes.size()))
                    for (const Border& v : exitRoutes[selExitRoute].vias)
                        drawVia(v, glm::vec3(1.0f, 0.85f, 0.3f));
            }
            // In-progress start border, highlighted.
            if (pathStart >= 0 && pathStart < static_cast<int>(tc.borders.size())) {
                const glm::dvec3 w = fracToWorld(polys, tc.borders[pathStart].trackId,
                                                 tc.borders[pathStart].frac);
                if (!(w.x == 0.0 && w.y == 0.0)) {
                    const glm::vec3 col(1.0f, 1.0f, 0.3f);
                    pts.push_back({scv(w, 4.5f), col});
                    lns.push_back({scv(w, 0.3f), col}); lns.push_back({scv(w, 4.5f), col});
                }
            }
        }
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
        buildPolys(); // keep circuit polylines (border projection/flood) in sync
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
        tracks.build(paths, glm::vec3(0.0f), data.loadedRadius());
        // The bores first: the terrain needs them to know which of its triangles stand in
        // a tunnel mouth. A geometry edit can move a portal, so both are rebuilt together.
        tunnels.build(data);
        mesh.build(data, &tunnels);
        switchNet.build(data, paths);
        switches.build(switchNet, glm::vec3(0.0f), data.loadedRadius());
        std::vector<TrackVertex> sv = buildings.vertices();
        std::vector<std::uint32_t> si = buildings.indices();
        auto merge = [&](const std::vector<TrackVertex>& v,
                         const std::vector<std::uint32_t>& idx) {
            const std::uint32_t base = static_cast<std::uint32_t>(sv.size());
            sv.insert(sv.end(), v.begin(), v.end());
            for (std::uint32_t k : idx) si.push_back(k + base);
        };
        merge(platforms.vertices(), platforms.indices());
        // Derived from the line speeds, so a geometry edit can move them: rebuilt here
        // rather than once at startup.
        speedSignMesh.build(speedSigns(paths, glm::vec3(0.0f), data.loadedRadius()));
        merge(speedSignMesh.vertices(), speedSignMesh.indices());
        merge(tunnels.vertices(), tunnels.indices());
        merge(switches.vertices(), switches.indices());
        signals.build(allPlacements(), data.sceneOrigin());
        merge(signals.vertices(), signals.indices());
        renderer.updateTerrain(mesh.vertices(), mesh.indices());
        renderer.updateTracks(tracks.vertices(), tracks.indices(),
                              tracks.alwaysIndexCount(), tracks.sleeperChunks());
        renderer.updateStructs(sv, si);
        std::printf("[trackedit] render preview refreshed (%.0f ms)\n",
                    (glfwGetTime() - t0) * 1000.0);
    };
    // Rebuild the switch stands + ground signals into the static struct buffer (buildings
    // + platforms + switches + signals) — the tail of rebuildRenderPreview without the
    // terrain recarve — so a switch-type change or a signal-path edit updates cheaply.
    auto rebuildStructs = [&]() {
        switches.build(switchNet, glm::vec3(0.0f), data.loadedRadius());
        signals.build(allPlacements(), data.sceneOrigin());
        std::vector<TrackVertex> sv = buildings.vertices();
        std::vector<std::uint32_t> si = buildings.indices();
        auto merge = [&](const std::vector<TrackVertex>& v,
                         const std::vector<std::uint32_t>& idx) {
            const std::uint32_t base = static_cast<std::uint32_t>(sv.size());
            sv.insert(sv.end(), v.begin(), v.end());
            for (std::uint32_t k : idx) si.push_back(k + base);
        };
        merge(platforms.vertices(), platforms.indices());
        // Derived from the line speeds, so a geometry edit can move them: rebuilt here
        // rather than once at startup.
        speedSignMesh.build(speedSigns(paths, glm::vec3(0.0f), data.loadedRadius()));
        merge(speedSignMesh.vertices(), speedSignMesh.indices());
        merge(tunnels.vertices(), tunnels.indices());
        merge(switches.vertices(), switches.indices());
        merge(signals.vertices(), signals.indices());
        renderer.updateStructs(sv, si);
    };
    // Try to close an exit route from the pinned start border to the armed exit signal.
    // Unlike a two-click route the destination is already fixed, so this runs both on the
    // click that sets the start and on every V that adds a via - press V on the
    // disambiguating border and an ambiguous route commits on the spot. Returns true when
    // it committed (which disarms); otherwise the start and the arm are kept so the user
    // can keep narrowing it.
    auto tryCommitExitRoute = [&]() -> bool {
        if (armedExit < 0 || armedExit >= static_cast<int>(exitSignals.size())) return false;
        if (pathStart < 0 || pathStart >= static_cast<int>(tc.borders.size())) return false;
        const SignalPath& sig = exitSignals[armedExit];
        std::vector<SectionInterval> route;
        const int n = findSignalRoute(polys, tc.borders[pathStart], sig.start, route,
                                      pendingVias);
        if (n != 1) {
            pathMsg = n == 0 ? "no route to the signal (V on a border adds a via)"
                             : "ambiguous - add a via (V) to pick the road";
            pathMsgUntil = glfwGetTime() + 3.0;
            return false;
        }
        SignalPath r;
        r.id = nextExitRouteId++;
        r.name = "R" + std::to_string(r.id);
        r.start = tc.borders[pathStart];
        r.end = sig.start;
        r.vias = pendingVias;
        r.parts = std::move(route);
        // Arriving at the border is not enough: the route must reach the signal from in
        // front of it, or it would be authority to pass a signal facing the other way.
        if (exitRouteTarget(r, exitSignals, polys) != armedExit) {
            --nextExitRouteId;
            pathMsg = "that route arrives behind the signal";
            pathMsgUntil = glfwGetTime() + 3.0;
            return false;
        }
        r.exitId = sig.id;
        r.type = defaultRouteType(r, sig, switchNet, polys);
        exitRoutes.push_back(std::move(r));
        selExitRoute = static_cast<int>(exitRoutes.size()) - 1;
        selExit = -1;
        pathStart = -1; pendingVias.clear(); armedExit = -1;
        exitRoutesDirty = true;
        const SignalPath& made = exitRoutes[selExitRoute];
        std::printf("[trackedit] exit route %d -> signal %d: %zu interval(s), %zu via(s), "
                    "%s (Ctrl+S to save)\n", made.id, made.exitId, made.parts.size(),
                    made.vias.size(), made.type == RouteType::C2 ? "C2" : "C1");
        beginNaming(NameTarget::ExitRoute, selExitRoute);
        return true;
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
    rebuildStructs(); // bake the ground signals (at loaded signal-path starts) into view
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
                "red = dead end, white = selected.\n"
                "Modes (Esc menu): Geometry edit (above) | Track circuits — click a track "
                "line to drop a border point (insulated joint), X deletes it, hover inside "
                "a border-bounded block and press Enter to save it as a sensing section "
                "(auto-named S1, S2, ...); right-click a section to select it and F2 to "
                "rename it; Ctrl+S writes overlay/track-circuits.txt.\n\n");

    const glm::vec3 sunDir = glm::normalize(glm::vec3(0.4f, -0.5f, 0.75f));
    const char* shotPath = std::getenv("EBANER_SCREENSHOT");
    int frame = 0;
    bool prevEnter = false, prevL = false, prevX = false, prevML = false,
         prevG = false, prevS = false, prevUp = false, prevDown = false,
         prevJ = false, prevN = false, prevR = false, prevK = false, prevC = false,
         prevP = false, prevF2 = false, prevMR = false, prevM = false,
         prevV = false, prevB = false, prevT = false, prevF = false;
    bool prevNameEnter = false, prevNameEsc = false, prevNameBs = false;
    bool prevMenuEnter = false, prevMenuUp = false, prevMenuDown = false;
    const std::vector<std::string> kMenuItems = {"Geometry edit", "Track circuits",
                                                 "Switches", "Signal paths",
                                                 "Exit signals", "Entry signals",
                                                 "Distant signals", "Exit"};
    // Flashing lamps (an entry signal's danger) blink from the push constant rather than by
    // rebuilding the signal mesh - here those vertices share the struct buffer with 32k
    // buildings, so a rebuild twice a second is out of the question.
    auto blinkLit = [] {
        constexpr double kBlinkPeriod = 1.0; // s, half lit
        return std::fmod(glfwGetTime(), kBlinkPeriod) < kBlinkPeriod * 0.5 ? 1.0f : 0.0f;
    };
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
            if (mE && !prevMenuEnter) {
                const std::string& sel = kMenuItems[g_menuSel];
                if (sel == "Exit") {
                    glfwSetWindowShouldClose(window, GLFW_TRUE);
                } else { // pick a mode and close the menu
                    mode = sel == "Track circuits" ? EdMode::Circuits
                           : sel == "Switches"      ? EdMode::Switches
                           : sel == "Signal paths"  ? EdMode::SignalPaths
                           : sel == "Exit signals"  ? EdMode::ExitSignals
                           : sel == "Entry signals" ? EdMode::EntrySignals
                           : sel == "Distant signals" ? EdMode::DistantSignals
                                                    : EdMode::Geometry;
                    selected.clear(); selA = selB = -1; selBorder = -1;
                    selTurnout = -1;
                    pathStart = -1; selPath = -1; selExit = -1; pendingVias.clear();
                    selExitRoute = -1; armedExit = -1; selEntry = -1; selDistant = -1;
                    showPending = false;
                    rebuildOverlay();
                    g_menuOpen = false;
                }
            }
            prevMenuUp = mU; prevMenuDown = mD; prevMenuEnter = mE;

            std::vector<TextVertex> tv;
            appendMenu(tv, "MENU", kMenuItems, g_menuSel, fbw, fbh);
            renderer.setOverlayText(tv);

            const float aspect = static_cast<float>(fbw) / static_cast<float>(fbh);
            PushConstants pc{};
            pc.viewProj = g_camera.projMatrix(aspect) * g_camera.viewMatrix();
            pc.sunDir = glm::vec4(sunDir, data.minElevation());
            pc.camPos = glm::vec4(g_camera.position(), data.maxElevation());
            pc.params = glm::vec4(0.5f, blinkLit(), 0.0f, 0.0f);
            if (shotPath) {
                if (frame == 20) renderer.requestCapture(shotPath);
                if (frame == 24) glfwSetWindowShouldClose(window, GLFW_TRUE);
            }
            renderer.drawFrame(pc);
            ++frame;
            continue;
        }

        if (g_naming) {
            // --- Modal section-name entry: charCallback fills g_nameBuf; here we handle
            // Enter (commit), Esc (cancel), Backspace (delete). All other input is inert. ---
            auto down = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
            const bool e = down(GLFW_KEY_ENTER), esc = down(GLFW_KEY_ESCAPE),
                       bs = down(GLFW_KEY_BACKSPACE);
            // What each target is called and where its dirty flag lives.
            const char* what = namingWhat == NameTarget::Section      ? "section"
                               : namingWhat == NameTarget::Path       ? "signal path"
                               : namingWhat == NameTarget::Exit       ? "exit signal"
                               : namingWhat == NameTarget::ExitRoute  ? "exit route"
                               : namingWhat == NameTarget::Entry      ? "entry signal"
                               : namingWhat == NameTarget::Distant    ? "distant signal"
                                                                      : "";
            const char* pfx = namingWhat == NameTarget::Section      ? "S"
                              : namingWhat == NameTarget::Path       ? "P"
                              : namingWhat == NameTarget::Exit       ? "E"
                              : namingWhat == NameTarget::Entry      ? "N"
                              : namingWhat == NameTarget::Distant    ? "D"
                                                                     : "R";
            if (e && !prevNameEnter) {
                std::vector<SignalPath>* nr = namedRoutes(namingWhat);
                if (namingWhat == NameTarget::Distant && namingIdx >= 0 &&
                    namingIdx < static_cast<int>(distantSignals.size())) {
                    DistantSignal& d = distantSignals[namingIdx];
                    d.name = g_nameBuf.empty() ? (pfx + std::to_string(d.id)) : g_nameBuf;
                    distantDirty = true;
                    std::printf("[trackedit] %s %d named \"%s\" (Ctrl+S to save)\n", what,
                                d.id, d.name.c_str());
                    rebuildOverlay();
                } else if (namingWhat == NameTarget::Section && namingIdx >= 0 &&
                           namingIdx < static_cast<int>(tc.sections.size())) {
                    Section& sec = tc.sections[namingIdx];
                    sec.name = g_nameBuf.empty() ? (pfx + std::to_string(sec.id)) : g_nameBuf;
                    circuitsDirty = true;
                    std::printf("[trackedit] %s %d named \"%s\" (Ctrl+S to save)\n", what,
                                sec.id, sec.name.c_str());
                    rebuildOverlay();
                } else if (nr && namingIdx >= 0 && namingIdx < static_cast<int>(nr->size())) {
                    SignalPath& p = (*nr)[namingIdx];
                    p.name = g_nameBuf.empty() ? (pfx + std::to_string(p.id)) : g_nameBuf;
                    (namingWhat == NameTarget::Path        ? pathsDirty
                     : namingWhat == NameTarget::Exit      ? exitDirty
                     : namingWhat == NameTarget::Entry     ? entryDirty
                                                           : exitRoutesDirty) = true;
                    std::printf("[trackedit] %s %d named \"%s\" (Ctrl+S to save)\n", what,
                                p.id, p.name.c_str());
                    rebuildOverlay();
                }
                g_naming = false; namingWhat = NameTarget::None; namingIdx = -1;
            }
            // Esc leaves the auto default in place, so naming is a prompt, never a barrier.
            if (esc && !prevNameEsc) {
                g_naming = false; namingWhat = NameTarget::None; namingIdx = -1;
            }
            if (bs && !prevNameBs && !g_nameBuf.empty()) g_nameBuf.pop_back();
            prevNameEnter = e; prevNameEsc = esc; prevNameBs = bs;

            const float scn = std::max(2.0f, static_cast<float>(fbh) / 240.0f);
            std::vector<TextVertex> tv;
            char nb[160];
            const std::vector<SignalPath>* nrv = namedRoutes(namingWhat);
            int nid = 0;
            if (nrv && namingIdx >= 0 && namingIdx < static_cast<int>(nrv->size()))
                nid = (*nrv)[namingIdx].id;
            else if (namingWhat == NameTarget::Section && namingIdx >= 0 &&
                     namingIdx < static_cast<int>(tc.sections.size()))
                nid = tc.sections[namingIdx].id;
            else if (namingWhat == NameTarget::Distant && namingIdx >= 0 &&
                     namingIdx < static_cast<int>(distantSignals.size()))
                nid = distantSignals[namingIdx].id;
            std::string title = "NAME ";
            for (const char* c = what; *c; ++c)
                title.push_back(static_cast<char>(std::toupper(*c)));
            std::snprintf(nb, sizeof(nb), "%s %d:  %s_", title.c_str(), nid,
                          g_nameBuf.c_str());
            appendText(tv, nb, fbw * 0.5f - 220.0f, fbh * 0.45f, scn * 1.4f,
                       glm::vec3(1.0f, 0.95f, 0.4f), fbw, fbh);
            appendText(tv, "type a name   Enter = confirm   Esc = cancel   Backspace = edit",
                       fbw * 0.5f - 220.0f, fbh * 0.45f + 34.0f, scn * 0.85f,
                       glm::vec3(0.85f, 0.85f, 0.8f), fbw, fbh);
            renderer.setOverlayText(tv);
            const float aspect = static_cast<float>(fbw) / static_cast<float>(fbh);
            PushConstants pc{};
            pc.viewProj = g_camera.projMatrix(aspect) * g_camera.viewMatrix();
            pc.sunDir = glm::vec4(sunDir, data.minElevation());
            pc.camPos = glm::vec4(g_camera.position(), data.maxElevation());
            pc.params = glm::vec4(0.5f, blinkLit(), 0.0f, 0.0f);
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
        // when the cursor is released (Tab) — captured mode is for mouse-look. Geometry
        // mode only; in circuits mode the continuous "add border" ring is the affordance
        // (a border sits anywhere along the line, not on a data point).
        int pointHover = -1;
        glm::vec2 hoverPx(0.0f);
        if (!g_mouseCaptured && mode == EdMode::Geometry) {
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

        // Circuits mode: pick the nearest track *segment* under the free cursor (a border
        // can sit between geo-points) as trackId + arc-length fraction, plus the nearest
        // existing border for select/delete.
        bool circHit = false;
        std::uint32_t circTrack = 0;
        double circFrac = 0.0;
        glm::dvec3 circWorld(0.0);
        glm::vec2 circPx(0.0f);
        int borderHover = -1;
        int sectionHover = -1; // section whose interval is under the cursor
        // Circuits mode authors borders/sections; Switches mode reuses the same section
        // pick to add/remove a circuit from the selected switch's locking set (L). Every
        // route mode needs it too: a border is the only thing they let you click, and the
        // ring drawn around the hovered one is the sole cue for where that is.
        if ((mode == EdMode::Circuits || mode == EdMode::Switches ||
             mode == EdMode::SignalPaths || mode == EdMode::ExitSignals ||
             mode == EdMode::EntrySignals || mode == EdMode::DistantSignals) &&
            !g_mouseCaptured) {
            double mx = 0.0, my = 0.0;
            glfwGetCursorPos(window, &mx, &my);
            int winw = fbw, winh = fbh;
            glfwGetWindowSize(window, &winw, &winh);
            const glm::vec2 cur(static_cast<float>(mx) * fbw / std::max(winw, 1),
                                static_cast<float>(my) * fbh / std::max(winh, 1));
            const glm::dvec3 o = data.sceneOrigin();
            auto proj = [&](const glm::dvec3& w, glm::vec2& px) -> bool {
                const glm::vec4 clip = viewProj * glm::vec4(float(w.x - o.x), float(w.y - o.y),
                                                            float(w.z - o.z), 1.0f);
                if (clip.w <= 0.0f) return false;
                px = glm::vec2((clip.x / clip.w * 0.5f + 0.5f) * fbw,
                               (clip.y / clip.w * 0.5f + 0.5f) * fbh);
                return true;
            };
            auto clip = [&](const glm::dvec3& w) {
                return viewProj * glm::vec4(float(w.x - o.x), float(w.y - o.y),
                                            float(w.z - o.z), 1.0f);
            };
            float best = 24.0f; // px pick radius (thin lines / sparse points -> forgiving)
            for (const TrackPoly& p : polys) {
                const double total = polyLength(p.pts);
                if (total < 1e-6) continue;
                double acc = 0.0;
                for (std::size_t i = 1; i < p.pts.size(); ++i) {
                    const glm::dvec3 P0 = p.pts[i - 1], P1 = p.pts[i];
                    const double seg = std::hypot(P1.x - P0.x, P1.y - P0.y);
                    // Clip the segment to the near plane so a long (main-line) segment with
                    // one endpoint behind the camera is still pickable; keep the world-param
                    // range [u0,u1] the visible screen part maps back to.
                    glm::vec4 c0 = clip(P0), c1 = clip(P1);
                    constexpr float eps = 0.01f;
                    double u0 = 0.0, u1 = 1.0;
                    if (c0.w < eps && c1.w < eps) { acc += seg; continue; }
                    if (c0.w < eps) { float tt = (eps - c0.w) / (c1.w - c0.w); u0 = tt; c0 = glm::mix(c0, c1, tt); }
                    else if (c1.w < eps) { float tt = (eps - c1.w) / (c0.w - c1.w); u1 = 1.0 - tt; c1 = glm::mix(c1, c0, tt); }
                    const glm::vec2 pa((c0.x / c0.w * 0.5f + 0.5f) * fbw,
                                       (c0.y / c0.w * 0.5f + 0.5f) * fbh);
                    const glm::vec2 pb((c1.x / c1.w * 0.5f + 0.5f) * fbw,
                                       (c1.y / c1.w * 0.5f + 0.5f) * fbh);
                    const glm::vec2 ab = pb - pa;
                    const float L2 = glm::dot(ab, ab);
                    const float st = L2 > 1e-6f ? glm::clamp(glm::dot(cur - pa, ab) / L2,
                                                            0.0f, 1.0f) : 0.0f;
                    const float d = glm::length(cur - (pa + ab * st));
                    if (d < best) {
                        best = d; circHit = true; circTrack = p.id;
                        // Screen param st -> object param must be perspective-correct, else
                        // the border lands offset from the click on a foreshortened segment
                        // (viewing along the track). c0/c1 are the clipped clip-space verts.
                        const double denom = (1.0 - st) / c0.w + st / c1.w;
                        const double localU = denom > 1e-20 ? (st / c1.w) / denom : st;
                        const double u = u0 + (u1 - u0) * localU; // world param on segment
                        circWorld = glm::mix(P0, P1, u);
                        circFrac = (acc + seg * u) / total;
                        circPx = pa + ab * st;
                    }
                    acc += seg;
                }
            }
            float bb = 14.0f; // px, nearest existing border
            for (std::size_t bi = 0; bi < tc.borders.size(); ++bi) {
                const glm::dvec3 w = fracToWorld(polys, tc.borders[bi].trackId,
                                                 tc.borders[bi].frac);
                if (w.x == 0.0 && w.y == 0.0) continue;
                glm::vec2 px;
                if (proj(w, px)) {
                    const float d = glm::length(px - cur);
                    if (d < bb) { bb = d; borderHover = static_cast<int>(bi); }
                }
            }
            // Which section is under the cursor (its interval contains the pick)?
            if (circHit)
                for (std::size_t si = 0; si < tc.sections.size(); ++si)
                    for (const auto& iv : tc.sections[si].parts)
                        if (iv.trackId == circTrack && circFrac >= iv.from - 1e-4 &&
                            circFrac <= iv.to + 1e-4)
                            sectionHover = static_cast<int>(si);
        }

        // Switches mode: pick the nearest working turnout by its world position
        // projected to screen (mirrors the geo-point pick).
        int turnoutHover = -1;
        glm::vec2 turnoutHoverPx(0.0f);
        if (mode == EdMode::Switches && !g_mouseCaptured) {
            double mx = 0.0, my = 0.0;
            glfwGetCursorPos(window, &mx, &my);
            int winw = fbw, winh = fbh;
            glfwGetWindowSize(window, &winw, &winh);
            const glm::vec2 cur(static_cast<float>(mx) * fbw / std::max(winw, 1),
                                static_cast<float>(my) * fbh / std::max(winh, 1));
            const glm::dvec3 o = data.sceneOrigin();
            const auto& tos = switchNet.turnouts();
            // Several switches can share one junction (each branch of a multi-way node
            // has its own). Collect everything under the cursor, then prefer the one
            // *after* the current selection, so clicking repeatedly cycles through them
            // instead of always landing on the same one.
            std::vector<std::pair<float, int>> under; // (pixel distance, turnout)
            glm::vec2 bestPx(0.0f);
            float best = 18.0f; // px pick radius
            for (std::size_t i = 0; i < tos.size(); ++i) {
                if (tos[i].mainPath < 0) continue; // inert crossing
                const glm::vec4 clip = viewProj * glm::vec4(float(tos[i].world.x - o.x),
                                                            float(tos[i].world.y - o.y),
                                                            float(tos[i].world.z - o.z), 1.0f);
                if (clip.w <= 0.0f) continue;
                const glm::vec2 px((clip.x / clip.w * 0.5f + 0.5f) * fbw,
                                   (clip.y / clip.w * 0.5f + 0.5f) * fbh);
                const float d = glm::length(px - cur);
                if (d < 18.0f) under.push_back({d, static_cast<int>(i)});
                if (d < best) { best = d; turnoutHover = static_cast<int>(i); bestPx = px; }
            }
            turnoutHoverPx = bestPx;
            if (!under.empty()) {
                std::sort(under.begin(), under.end());
                // Anything within a few pixels of the nearest is "at the same spot".
                const float near = under.front().first + 6.0f;
                std::vector<int> tied;
                for (const auto& [d, i] : under)
                    if (d <= near) tied.push_back(i);
                if (tied.size() > 1) {
                    const auto it = std::find(tied.begin(), tied.end(), selTurnout);
                    turnoutHover = it != tied.end()
                                       ? tied[(it - tied.begin() + 1) % tied.size()]
                                       : tied.front();
                }
            }
        }

        // Signal-paths mode: pick the nearest signal path (sample its route intervals and
        // project to screen), for right-click select / delete.
        // Signal paths and exit signals are authored identically (a start border, optional
        // vias, a destination), so the handlers below work on whichever collection the
        // current mode edits.
        const bool exitMode = mode == EdMode::ExitSignals;
        const bool entryMode = mode == EdMode::EntrySignals;
        const bool routeMode = exitMode || entryMode || mode == EdMode::SignalPaths;
        std::vector<SignalPath>& routes = exitMode    ? exitSignals
                                          : entryMode ? entrySignals
                                                      : signalPaths;
        bool& routesDirty = exitMode ? exitDirty : entryMode ? entryDirty : pathsDirty;
        int& selRoute = exitMode ? selExit : entryMode ? selEntry : selPath;
        int& nextRouteId = exitMode ? nextExitId : entryMode ? nextEntryId : nextPathId;
        // Exit signals mode carries a second collection: the routes leading up to those
        // signals. Exactly one of a signal and a route is ever selected, and while armed
        // a left-click means "the start of a route to that signal", not "a new signal".
        // Queried rather than snapshotted: committing a route disarms mid-frame, and a
        // stale copy would leave the HUD indexing a signal that is no longer armed.
        auto armed = [&] {
            return exitMode && armedExit >= 0 &&
                   armedExit < static_cast<int>(exitSignals.size());
        };

        int pathHover = -1;
        int exitRouteHover = -1;
        if (routeMode && !g_mouseCaptured) {
            double mx = 0.0, my = 0.0;
            glfwGetCursorPos(window, &mx, &my);
            int winw = fbw, winh = fbh;
            glfwGetWindowSize(window, &winw, &winh);
            const glm::vec2 cur(static_cast<float>(mx) * fbw / std::max(winw, 1),
                                static_cast<float>(my) * fbh / std::max(winh, 1));
            const glm::dvec3 o = data.sceneOrigin();
            float best = 12.0f; // px
            // Both collections compete for the same cursor with one shared `best`, so the
            // nearest wins outright and the highlight always matches what the eye picks out.
            auto pick = [&](const std::vector<SignalPath>& rs, int& hit, int& other) {
                for (std::size_t pi = 0; pi < rs.size(); ++pi)
                    for (const SectionInterval& iv : rs[pi].parts)
                        for (int k = 0; k <= 12; ++k) {
                            const double f = iv.from + (iv.to - iv.from) * k / 12.0;
                            const glm::dvec3 w = fracToWorld(polys, iv.trackId, f);
                            if (w.x == 0.0 && w.y == 0.0) continue;
                            const glm::vec4 clip = viewProj * glm::vec4(
                                float(w.x - o.x), float(w.y - o.y), float(w.z - o.z) + 1.5f,
                                1.0f);
                            if (clip.w <= 0.0f) continue;
                            const glm::vec2 px((clip.x / clip.w * 0.5f + 0.5f) * fbw,
                                               (clip.y / clip.w * 0.5f + 0.5f) * fbh);
                            const float d = glm::length(px - cur);
                            if (d < best) {
                                best = d;
                                hit = static_cast<int>(pi);
                                other = -1;
                            }
                        }
            };
            pick(routes, pathHover, exitRouteHover);
            if (exitMode) pick(exitRoutes, exitRouteHover, pathHover);
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
        const bool kF2 = glfwGetKey(window, GLFW_KEY_F2) == GLFW_PRESS;
        const bool kM = glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS;
        const bool kV = glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS;
        const bool kB = glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS;
        const bool kT = glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS;
        const bool kF = glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS;
        // Save is Ctrl+S (plain S is the backward-movement key).
        const bool kSave = ctrl && glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS;
        if (mode == EdMode::Geometry) {
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
            // R: add a connecting rail between 2 points (builds a switch each end).
            if (kR && !prevR) doAddRail();
            // K: auto-build a slip at the crossing under the selected point.
            if (kK && !prevK) doAutoDiamond();
            // C: build a scissors (double) crossover between 2 selected tracks' points.
            if (kC && !prevC) doScissors();
        } else if (mode == EdMode::Circuits) { // --- Circuits mode ---
            // Enter: seed a section at the hovered track spot and flood-fill it.
            if (kEnter && !prevEnter && circHit) {
                pendingFlood = floodSection(polys, tc.borders, circTrack, circFrac);
                showPending = true;
                if (pendingFlood.enclosed) {
                    Section s;
                    s.id = nextSectionId++;
                    s.name = "S" + std::to_string(s.id); // default name; F2 to rename
                    s.parts = pendingFlood.parts;
                    tc.sections.push_back(s);
                    selSection = static_cast<int>(tc.sections.size()) - 1;
                    circuitsDirty = true;
                    std::printf("[trackedit] section %d: %zu track(s), %.0f m, %d joint(s), "
                                "%d dead-end(s) (Ctrl+S to save)\n", s.id, s.parts.size(),
                                pendingFlood.lengthM, pendingFlood.borderEnds,
                                pendingFlood.deadEnds);
                } else {
                    std::printf("[trackedit] not enclosed: block spans %zu track(s), %.0f m "
                                "with no border — place border points to bound it\n",
                                pendingFlood.parts.size(), pendingFlood.lengthM);
                }
                rebuildOverlay();
            }
            // X: delete the selected (or hovered) border.
            if (kX && !prevX) {
                const int del = selBorder >= 0 ? selBorder : borderHover;
                if (del >= 0 && del < static_cast<int>(tc.borders.size())) {
                    tc.borders.erase(tc.borders.begin() + del);
                    selBorder = -1; moveArmed = false; circuitsDirty = true;
                    std::printf("[trackedit] border removed (Ctrl+S to save)\n");
                    rebuildOverlay();
                }
            }
            // M: arm a move of the selected border - the next click on its track moves it,
            // carrying the sections and signal paths anchored to it.
            if (kM && !prevM) {
                if (selBorder >= 0 && selBorder < static_cast<int>(tc.borders.size())) {
                    moveArmed = !moveArmed;
                    pathMsg = moveArmed ? "click the new spot on this track" : "move cancelled";
                    pathMsgUntil = glfwGetTime() + 3.0;
                } else {
                    pathMsg = "select a border first (click it), then M to move";
                    pathMsgUntil = glfwGetTime() + 3.0;
                }
                rebuildOverlay();
            }
            // F2: rename the selected (or hovered) section — start modal text entry.
            if (kF2 && !prevF2) {
                const int t = selSection >= 0 ? selSection : sectionHover;
                if (t >= 0 && t < static_cast<int>(tc.sections.size())) {
                    beginNaming(NameTarget::Section, t);
                }
            }
        } else if (mode == EdMode::Switches) { // --- Switches mode ---
            // M: toggle the selected switch between manual and motor-driven; the stand
            // visual updates immediately and the change is staged for Ctrl+S. Becoming a
            // motor seeds the default locking set (the circuits it sits within); reverting
            // to manual clears it.
            if (kM && !prevM && selTurnout >= 0) {
                const SwitchType cur = switchNet.type(selTurnout);
                if (cur == SwitchType::Motor) {
                    switchNet.setType(selTurnout, SwitchType::Manual);
                    switchNet.setLock(selTurnout, {});
                } else {
                    switchNet.setType(selTurnout, SwitchType::Motor);
                    switchNet.setLock(selTurnout, defaultLockSections(
                        switchNet.turnouts()[selTurnout], tc, polys));
                }
                switchTypesDirty = true;
                rebuildStructs();  // refresh the 3-D stand (light: no terrain recarve)
                rebuildOverlay();  // refresh the type-coloured marker + lock highlight
                std::printf("[trackedit] switch %d -> %s (Ctrl+S to save)\n", selTurnout,
                            switchNet.type(selTurnout) == SwitchType::Motor ? "motor"
                                                                            : "manual");
            }
            // L: toggle the hovered circuit in/out of the selected motor switch's lock set.
            if (kL && !prevL && selTurnout >= 0 &&
                switchNet.type(selTurnout) == SwitchType::Motor && sectionHover >= 0) {
                const int id = tc.sections[sectionHover].id;
                std::vector<int> lk = switchNet.lock(selTurnout);
                const auto it = std::find(lk.begin(), lk.end(), id);
                const bool removed = it != lk.end();
                if (removed) lk.erase(it);
                else lk.push_back(id);
                switchNet.setLock(selTurnout, std::move(lk));
                switchTypesDirty = true;
                rebuildOverlay();
                std::printf("[trackedit] switch %d %s lock circuit %s (Ctrl+S to save)\n",
                            selTurnout, removed ? "removed" : "added",
                            tc.sections[sectionHover].name.c_str());
            }
        } else if (mode == EdMode::DistantSignals) { // --- Distant signals mode ---
            const bool haveSel = selDistant >= 0 &&
                                 selDistant < static_cast<int>(distantSignals.size());
            // F: turn the selected signal round. Which way it reads is the whole of its
            // configuration, so flipping is the one edit it has.
            if (kF && !prevF) {
                if (haveSel) {
                    DistantSignal& d = distantSignals[selDistant];
                    d.dir = -d.dir;
                    distantDirty = true;
                    pathMsg = std::string("reads toward ") + (d.dir > 0 ? "+frac" : "-frac");
                    rebuildStructs();
                } else {
                    pathMsg = "right-click a distant signal first, then F";
                }
                pathMsgUntil = glfwGetTime() + 3.0;
                rebuildOverlay();
            }
            if (kX && !prevX && haveSel) {
                distantSignals.erase(distantSignals.begin() + selDistant);
                selDistant = -1;
                distantDirty = true;
                std::printf("[trackedit] distant signal removed (Ctrl+S to save)\n");
                rebuildStructs();
                rebuildOverlay();
            }
            if (kF2 && !prevF2 && haveSel) beginNaming(NameTarget::Distant, selDistant);
        } else { // --- Signal-paths / exit-signals modes ---
            // V: pin the hovered border as a via the route must pass through, so an
            // otherwise ambiguous destination can be narrowed to one road.
            if (kV && !prevV) {
                if (pathStart < 0) {
                    pathMsg = "click a start border first, then V to add a via";
                } else if (borderHover < 0) {
                    pathMsg = "hover a border to add it as a via";
                } else if (borderHover == pathStart) {
                    pathMsg = "that is the start border";
                } else if (!pendingVias.empty() &&
                           pendingVias.back().trackId == tc.borders[borderHover].trackId &&
                           pendingVias.back().frac == tc.borders[borderHover].frac) {
                    pathMsg = "already the last via";
                } else {
                    pendingVias.push_back(tc.borders[borderHover]);
                    pathMsg = "via added (" + std::to_string(pendingVias.size()) + ")";
                    // The destination is already known while armed, so retry at once: this
                    // is what lets an ambiguous exit route close on the V that resolves it.
                    if (armed()) tryCommitExitRoute();
                }
                pathMsgUntil = glfwGetTime() + 3.0;
                rebuildOverlay();
            }
            // B: with an exit signal selected, start drawing a route that leads up to it -
            // a main signal's authority begins back at the platform road, not at the mast.
            if (kB && !prevB && exitMode) {
                int target = -1;
                if (selExit >= 0 && selExit < static_cast<int>(exitSignals.size()))
                    target = selExit;
                else if (selExitRoute >= 0 &&
                         selExitRoute < static_cast<int>(exitRoutes.size()))
                    // A route is selected: aim at the signal it already serves, so adding a
                    // second road to the same signal needs no hunting for its ribbon.
                    for (std::size_t i = 0; i < exitSignals.size(); ++i)
                        if (exitSignals[i].id == exitRoutes[selExitRoute].exitId)
                            target = static_cast<int>(i);
                if (armed()) {
                    armedExit = -1; pathStart = -1; pendingVias.clear();
                    pathMsg = "route cancelled";
                } else if (target >= 0) {
                    armedExit = target;
                    selExit = target; selExitRoute = -1;
                    pathStart = -1; pendingVias.clear();
                    pathMsg = "click the start border inside the station";
                } else {
                    pathMsg = "right-click an exit signal or route first, then B";
                }
                pathMsgUntil = glfwGetTime() + 3.0;
                rebuildOverlay();
            }
            // T: what the selected route authorises - C1 clear (two greens) or C2 clear over
            // a deviation (one green). The create-time guess is only a starting point.
            if (kT && !prevT && (exitMode || entryMode)) {
                // In entry mode the selected route is the signal's own record; in exit mode
                // it is the approach route, since an exit signal's type lives there.
                const bool haveEntry = entryMode && selEntry >= 0 &&
                                       selEntry < static_cast<int>(entrySignals.size());
                if (haveEntry || (exitMode && selExitRoute >= 0 &&
                                  selExitRoute < static_cast<int>(exitRoutes.size()))) {
                    SignalPath& r = haveEntry ? entrySignals[selEntry]
                                              : exitRoutes[selExitRoute];
                    r.type = r.type == RouteType::C1 ? RouteType::C2 : RouteType::C1;
                    (haveEntry ? entryDirty : exitRoutesDirty) = true;
                    pathMsg = std::string("type ") +
                              (r.type == RouteType::C2 ? "C2 (deviation)" : "C1 (no restriction)");
                    std::printf("[trackedit] %s %d -> %s (Ctrl+S to save)\n",
                                haveEntry ? "entry signal" : "exit route", r.id,
                                r.type == RouteType::C2 ? "C2" : "C1");
                } else {
                    pathMsg = entryMode ? "right-click an entry route first, then T"
                                        : "right-click an exit route first, then T";
                }
                pathMsgUntil = glfwGetTime() + 3.0;
                rebuildOverlay();
            }
            // X: cancel whatever is in progress, else delete the selected thing.
            if (kX && !prevX) {
                if (armed() || pathStart >= 0) {
                    armedExit = -1; pathStart = -1; pendingVias.clear();
                } else if (exitMode && selExitRoute >= 0 &&
                           selExitRoute < static_cast<int>(exitRoutes.size())) {
                    exitRoutes.erase(exitRoutes.begin() + selExitRoute);
                    selExitRoute = -1; exitRoutesDirty = true;
                    std::printf("[trackedit] exit route removed (Ctrl+S to save)\n");
                } else if (selRoute >= 0 && selRoute < static_cast<int>(routes.size())) {
                    // Deleting an exit signal takes its routes with it: a route that leads
                    // to nothing is not something the sim should ever be handed.
                    int gone = 0;
                    if (exitMode) {
                        const int id = routes[selRoute].id;
                        const auto it = std::remove_if(
                            exitRoutes.begin(), exitRoutes.end(),
                            [&](const SignalPath& r) { return r.exitId == id; });
                        gone = static_cast<int>(std::distance(it, exitRoutes.end()));
                        exitRoutes.erase(it, exitRoutes.end());
                        if (gone > 0) { exitRoutesDirty = true; selExitRoute = -1; }
                    }
                    routes.erase(routes.begin() + selRoute);
                    selRoute = -1; routesDirty = true;
                    const std::string also =
                        gone > 0 ? " with " + std::to_string(gone) + " route(s) that led to it"
                                 : std::string();
                    std::printf("[trackedit] %s removed%s (Ctrl+S to save)\n",
                                exitMode ? "exit signal"
                                : entryMode ? "entry signal" : "signal path",
                                also.c_str());
                    rebuildStructs(); // a start may have lost its signal
                }
                rebuildOverlay();
            }
            // F2: rename whatever is selected (modal text entry, shared with sections).
            if (kF2 && !prevF2) {
                if (exitMode && selExitRoute >= 0)
                    beginNaming(NameTarget::ExitRoute, selExitRoute);
                else if (selRoute >= 0)
                    beginNaming(exitMode    ? NameTarget::Exit
                                : entryMode ? NameTarget::Entry
                                            : NameTarget::Path,
                                selRoute);
            }
        }
        // P: re-render the actual track + terrain (cuttings) + switch stands (both modes).
        if (kP && !prevP) rebuildRenderPreview();
        // Up/Down: raise/lower the selected point(s). Auto-repeats while held (an
        // immediate first step, then throttled) so big changes don't need many taps.
        constexpr double kElevStep = 0.1; // metres per step (auto-repeats while held)
        const bool kUp = glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS;
        const bool kDn = glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS;
        if (mode == EdMode::Geometry && (kUp || kDn) && !selected.empty()) {
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
        if (kSave && !prevS && mode == EdMode::Circuits &&
            (circuitsDirty || pathsDirty || exitDirty || exitRoutesDirty || entryDirty)) {
            // Circuits mode: save the sensing sections to their own overlay file. A border
            // move also rewrites every route anchored to it, so write those too - otherwise
            // the overlays would silently disagree on the next load, and there would be no
            // way to save the exit files from this mode at all.
            if (circuitsDirty) {
                if (writeTrackCircuits(datasetRoot, tc)) {
                    std::printf("[trackedit] saved %zu border(s), %zu section(s) -> "
                                "%s/overlay/track-circuits.txt\n", tc.borders.size(),
                                tc.sections.size(), datasetRoot.c_str());
                    circuitsDirty = false;
                } else {
                    std::fprintf(stderr, "[trackedit] failed to write track-circuits file\n");
                }
            }
            if (pathsDirty) {
                if (writeSignalPaths(datasetRoot, signalPaths)) {
                    std::printf("[trackedit] saved %zu signal path(s) -> "
                                "%s/overlay/signal-paths.txt\n", signalPaths.size(),
                                datasetRoot.c_str());
                    pathsDirty = false;
                } else {
                    std::fprintf(stderr, "[trackedit] failed to write signal-paths file\n");
                }
            }
            if (exitDirty) {
                if (writeExitSignals(datasetRoot, exitSignals)) {
                    std::printf("[trackedit] saved %zu exit signal(s) -> "
                                "%s/overlay/exit-signals.txt\n", exitSignals.size(),
                                datasetRoot.c_str());
                    exitDirty = false;
                } else {
                    std::fprintf(stderr, "[trackedit] failed to write exit signals\n");
                }
            }
            if (exitRoutesDirty) {
                if (writeExitRoutes(datasetRoot, exitRoutes)) {
                    std::printf("[trackedit] saved %zu exit route(s) -> "
                                "%s/overlay/exit-routes.txt\n", exitRoutes.size(),
                                datasetRoot.c_str());
                    exitRoutesDirty = false;
                } else {
                    std::fprintf(stderr, "[trackedit] failed to write exit routes\n");
                }
            }
            if (entryDirty) {
                if (writeEntrySignals(datasetRoot, entrySignals)) {
                    std::printf("[trackedit] saved %zu entry signal(s) -> "
                                "%s/overlay/entry-signals.txt\n", entrySignals.size(),
                                datasetRoot.c_str());
                    entryDirty = false;
                } else {
                    std::fprintf(stderr, "[trackedit] failed to write entry signals\n");
                }
            }
        } else if (kSave && !prevS && mode == EdMode::Geometry &&
                   (!pending.empty() || removals > 0)) {
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
        } else if (kSave && !prevS && mode == EdMode::Switches && switchTypesDirty) {
            // Switches mode: save the manual/motor overrides to their own overlay file.
            const std::vector<SwitchTypeOverride> ovr = collectSwitchOverrides(switchNet);
            if (writeSwitchTypes(datasetRoot, ovr)) {
                std::printf("[trackedit] saved %zu motor switch(es) -> "
                            "%s/overlay/switch-types.txt\n", ovr.size(), datasetRoot.c_str());
                switchTypesDirty = false;
            } else {
                std::fprintf(stderr, "[trackedit] failed to write switch-types file\n");
            }
        } else if (kSave && !prevS && mode == EdMode::ExitSignals &&
                   (exitDirty || exitRoutesDirty)) {
            // The mode edits two collections, so a save covers whichever is dirty.
            if (exitDirty) {
                if (writeExitSignals(datasetRoot, exitSignals)) {
                    std::printf("[trackedit] saved %zu exit signal(s) to "
                                "%s/overlay/exit-signals.txt\n", exitSignals.size(),
                                datasetRoot.c_str());
                    exitDirty = false;
                } else {
                    std::fprintf(stderr, "[trackedit] failed to write exit signals\n");
                }
            }
            if (exitRoutesDirty) {
                if (writeExitRoutes(datasetRoot, exitRoutes)) {
                    std::printf("[trackedit] saved %zu exit route(s) to "
                                "%s/overlay/exit-routes.txt\n", exitRoutes.size(),
                                datasetRoot.c_str());
                    exitRoutesDirty = false;
                } else {
                    std::fprintf(stderr, "[trackedit] failed to write exit routes\n");
                }
            }
        } else if (kSave && !prevS && mode == EdMode::EntrySignals && entryDirty) {
            if (writeEntrySignals(datasetRoot, entrySignals)) {
                std::printf("[trackedit] saved %zu entry signal(s) to "
                            "%s/overlay/entry-signals.txt\n", entrySignals.size(),
                            datasetRoot.c_str());
                entryDirty = false;
            } else {
                std::fprintf(stderr, "[trackedit] failed to write entry signals\n");
            }
        } else if (kSave && !prevS && mode == EdMode::DistantSignals && distantDirty) {
            if (writeDistantSignals(datasetRoot, distantSignals)) {
                std::printf("[trackedit] saved %zu distant signal(s) to "
                            "%s/overlay/distant-signals.txt\n", distantSignals.size(),
                            datasetRoot.c_str());
                distantDirty = false;
            } else {
                std::fprintf(stderr, "[trackedit] failed to write distant signals\n");
            }
        } else if (kSave && !prevS && mode == EdMode::SignalPaths && pathsDirty) {
            // Signal-paths mode: save the mini signal paths to their own overlay file.
            if (writeSignalPaths(datasetRoot, signalPaths)) {
                std::printf("[trackedit] saved %zu signal path(s) -> "
                            "%s/overlay/signal-paths.txt\n", signalPaths.size(),
                            datasetRoot.c_str());
                pathsDirty = false;
            } else {
                std::fprintf(stderr, "[trackedit] failed to write signal-paths file\n");
            }
        }
        prevEnter = kEnter; prevL = kL; prevX = kX; prevG = kG; prevS = kSave;
        prevJ = kJ; prevN = kN; prevR = kR; prevK = kK; prevC = kC; prevP = kP;
        prevF2 = kF2; prevM = kM; prevV = kV; prevB = kB; prevT = kT; prevF = kF;

        // Click to select (cursor freed only). Ctrl+click toggles for multi-select;
        // plain click selects one; clicking empty space clears.
        const bool mL = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        if (!g_mouseCaptured && mL && !prevML && mode == EdMode::Geometry) {
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
        } else if (!g_mouseCaptured && mL && !prevML && mode == EdMode::Circuits) {
            // A move is armed: this click relocates the selected border (and everything
            // anchored to it) rather than dropping a new one.
            if (moveArmed && selBorder >= 0 && selBorder < static_cast<int>(tc.borders.size())) {
                std::string why;
                if (!circHit) {
                    moveArmed = false; // clicked off the track: cancel
                    pathMsg = "move cancelled";
                } else if (const Border& b = tc.borders[selBorder];
                           !canMoveBorder(tc, polys, selBorder, circTrack, circFrac, why) ||
                           // Every collection anchored to this border gets a say: a move
                           // that would collapse or reverse any of their routes is refused.
                           borderMoveBreaksPath(signalPaths, polys, b.trackId, b.frac,
                                                circFrac, why) ||
                           borderMoveBreaksPath(exitSignals, polys, b.trackId, b.frac,
                                                circFrac, why) ||
                           borderMoveBreaksPath(exitRoutes, polys, b.trackId, b.frac,
                                                circFrac, why) ||
                           borderMoveBreaksPath(entrySignals, polys, b.trackId, b.frac,
                                                circFrac, why)) {
                    pathMsg = "cannot move: " + why; // stay armed so the user can retry
                } else {
                    // Everything anchored to this border moves with it: the circuits, the
                    // mini paths, the exit signals standing on it and the routes that lead
                    // up to them. Missing any one of them silently breaks that record.
                    const std::uint32_t t = tc.borders[selBorder].trackId;
                    const double oldF = tc.borders[selBorder].frac;
                    const int nc = moveBorderFrac(tc, polys, t, oldF, circFrac);
                    const int np = moveBorderFrac(signalPaths, polys, t, oldF, circFrac);
                    const int ne = moveBorderFrac(exitSignals, polys, t, oldF, circFrac);
                    const int nr = moveBorderFrac(exitRoutes, polys, t, oldF, circFrac);
                    const int ny = moveBorderFrac(entrySignals, polys, t, oldF, circFrac);
                    circuitsDirty = true;
                    if (np > 0) pathsDirty = true;
                    if (ne > 0) exitDirty = true;
                    if (nr > 0) exitRoutesDirty = true;
                    if (ny > 0) entryDirty = true;
                    moveArmed = false;
                    rebuildStructs(); // signals stand at route starts: re-place them
                    pathMsg = "border moved (" + std::to_string(nc) + " circuit + " +
                              std::to_string(np + ne + nr + ny) + " route value(s))";
                    std::printf("[trackedit] border moved on %#x %.6f -> %.6f (%d circuit, %d "
                                "path, %d exit, %d exit-route, %d entry value(s); "
                                "Ctrl+S to save)\n",
                                t, oldF, circFrac, nc, np, ne, nr, ny);
                }
                pathMsgUntil = glfwGetTime() + 3.0;
                rebuildOverlay();
            }
            // Click a border to select it, else drop a new border on the nearest track.
            else if (borderHover >= 0) {
                selBorder = borderHover;
            } else if (circHit) {
                tc.borders.push_back({circTrack, circFrac});
                selBorder = static_cast<int>(tc.borders.size()) - 1;
                circuitsDirty = true;
                std::printf("[trackedit] border on %#x at %.3f (Ctrl+S to save)\n",
                            circTrack, circFrac);
            }
            rebuildOverlay();
        } else if (!g_mouseCaptured && mL && !prevML && mode == EdMode::Switches) {
            // Click a turnout to select it; clicking empty space clears the selection.
            selTurnout = turnoutHover;
            rebuildOverlay();
        } else if (!g_mouseCaptured && mL && !prevML && mode == EdMode::DistantSignals) {
            // Free placement: any point along any track, not snapped to a border.
            if (circHit) {
                DistantSignal d;
                d.id = nextDistantId++;
                d.name = "D" + std::to_string(d.id);
                d.trackId = circTrack;
                d.frac = circFrac;
                distantSignals.push_back(std::move(d));
                selDistant = static_cast<int>(distantSignals.size()) - 1;
                distantDirty = true;
                rebuildStructs();
                std::printf("[trackedit] distant signal %d on %#x at %.6f (Ctrl+S to save)\n",
                            distantSignals[selDistant].id, circTrack, circFrac);
                beginNaming(NameTarget::Distant, selDistant);
            }
            rebuildOverlay();
        } else if (!g_mouseCaptured && mL && !prevML && armed()) {
            // Drawing a route up to the armed exit signal: the destination is already fixed,
            // so one click on the start border is the whole gesture.
            if (borderHover >= 0) {
                pathStart = borderHover;
                if (!tryCommitExitRoute())
                    std::printf("[trackedit] exit route: %s\n", pathMsg.c_str());
            } else {
                armedExit = -1; pathStart = -1; pendingVias.clear(); // empty space: cancel
            }
            rebuildOverlay();
        } else if (!g_mouseCaptured && mL && !prevML && routeMode) {
            // Click a border to set the start (for an exit signal, where it stands); click
            // a second border to build the route (commit if unique, refuse otherwise).
            if (borderHover >= 0) {
                if (pathStart < 0) {
                    pathStart = borderHover;
                } else if (borderHover == pathStart) {
                    // clicked the same border again: cancel
                    pathStart = -1; pendingVias.clear();
                } else {
                    std::vector<SectionInterval> route;
                    const int n = findSignalRoute(polys, tc.borders[pathStart],
                                                  tc.borders[borderHover], route,
                                                  pendingVias);
                    if (n == 1) {
                        SignalPath p;
                        p.id = nextRouteId++;
                        p.name = std::string(exitMode ? "E" : entryMode ? "N" : "P") +
                                 std::to_string(p.id);
                        p.start = tc.borders[pathStart];
                        p.end = tc.borders[borderHover];
                        p.vias = pendingVias;
                        p.parts = std::move(route);
                        // An entry signal's record is the whole movement, so its authority
                        // follows from the turnouts it takes; T overrides afterwards.
                        if (entryMode) p.type = defaultRouteType(p, switchNet, polys);
                        routes.push_back(std::move(p));
                        selRoute = static_cast<int>(routes.size()) - 1;
                        pathStart = -1; pendingVias.clear(); routesDirty = true;
                        rebuildStructs(); // place the signal at the new route's start
                        std::printf("[trackedit] %s %d: %zu interval(s), %zu via(s) "
                                    "(Ctrl+S to save)\n",
                                    exitMode ? "exit signal"
                                    : entryMode ? "entry signal" : "signal path",
                                    routes[selRoute].id, routes[selRoute].parts.size(),
                                    routes[selRoute].vias.size());
                        // Prompt for a name at once: these label the routes an operator
                        // will pick from, so none should be left as just "P7".
                        beginNaming(exitMode    ? NameTarget::Exit
                                    : entryMode ? NameTarget::Entry
                                                : NameTarget::Path,
                                    selRoute);
                    } else {
                        pathMsg = n == 0 ? "no route (V on a border adds a via)"
                                         : "ambiguous - add a via (V) to pick the road";
                        pathMsgUntil = now + 3.0;
                        std::printf("[trackedit] %s: %s\n",
                                    exitMode ? "exit signal"
                                    : entryMode ? "entry signal" : "signal path",
                                    pathMsg.c_str());
                        // keep pathStart and the vias so another can be added
                    }
                }
            } else {
                pathStart = -1; pendingVias.clear(); // clicked empty space: cancel
            }
            rebuildOverlay();
        }
        prevML = mL;
        // Right-click selects: a section (circuits) or a signal path (signal-paths mode).
        const bool mR = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
        if (!g_mouseCaptured && mR && !prevMR && mode == EdMode::Circuits) {
            selSection = sectionHover;
            rebuildOverlay();
        } else if (!g_mouseCaptured && mR && !prevMR && mode == EdMode::DistantSignals) {
            // Nearest distant to the cursor, in screen terms so it picks what the eye does.
            double mx = 0.0, my = 0.0;
            glfwGetCursorPos(window, &mx, &my);
            int winw = fbw, winh = fbh;
            glfwGetWindowSize(window, &winw, &winh);
            const glm::vec2 cur(static_cast<float>(mx) * fbw / std::max(winw, 1),
                                static_cast<float>(my) * fbh / std::max(winh, 1));
            const glm::dvec3 o = data.sceneOrigin();
            float best = 24.0f; // px
            selDistant = -1;
            for (std::size_t i = 0; i < distantSignals.size(); ++i) {
                const DistantSignal& d = distantSignals[i];
                const glm::dvec3 w = fracToWorld(polys, d.trackId, d.frac);
                if (w.x == 0.0 && w.y == 0.0) continue;
                const glm::vec4 clip = viewProj * glm::vec4(
                    float(w.x - o.x), float(w.y - o.y), float(w.z - o.z) + 2.0f, 1.0f);
                if (clip.w <= 0.0f) continue;
                const glm::vec2 px((clip.x / clip.w * 0.5f + 0.5f) * fbw,
                                   (clip.y / clip.w * 0.5f + 0.5f) * fbh);
                const float dpx = glm::length(px - cur);
                if (dpx < best) { best = dpx; selDistant = static_cast<int>(i); }
            }
            rebuildOverlay();
        } else if (!g_mouseCaptured && mR && !prevMR && routeMode) {
            selRoute = pathHover;
            selExitRoute = exitRouteHover;
            rebuildOverlay();
        }
        prevMR = mR;

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
                const float y1 =
                    40.0f + (mode == EdMode::ExitSignals    ? 10.0f
                             : mode == EdMode::SignalPaths  ? 9.0f
                             : mode == EdMode::EntrySignals ? 10.0f
                                                            : 8.0f) * lh;
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
          if (mode == EdMode::Geometry) {
            std::snprintf(buf, sizeof(buf), "MODE: GEOMETRY (Esc menu to switch)");
            appendText(tv, buf, x, 40.0f + 3 * lh, sc, glm::vec3(0.7f, 0.85f, 1.0f), fbw, fbh);
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
          } else if (mode == EdMode::Circuits) { // --- Circuits mode HUD ---
            appendText(tv, "MODE: TRACK CIRCUITS (Esc menu to switch)", x, 40.0f + 3 * lh,
                       sc, glm::vec3(1.0f, 0.6f, 0.6f), fbw, fbh);
            char selname[48] = "";
            if (selSection >= 0 && selSection < static_cast<int>(tc.sections.size()))
                std::snprintf(selname, sizeof(selname), "  sel: %s",
                              tc.sections[selSection].name.c_str());
            std::snprintf(buf, sizeof(buf), "BORDERS %zu  SECTIONS %zu%s",
                          tc.borders.size(), tc.sections.size(), selname);
            appendText(tv, buf, x, 40.0f + 4 * lh, sc, glm::vec3(0.9f, 0.85f, 0.7f), fbw, fbh);
            const bool dirty = circuitsDirty || pathsDirty;
            std::snprintf(buf, sizeof(buf), "UNSAVED %s   %s", dirty ? "yes" : "no",
                          dirty ? "Ctrl+S to save" : "");
            appendText(tv, buf, x, 40.0f + 5 * lh, sc,
                       dirty ? glm::vec3(1.0f, 0.6f, 0.3f) : glm::vec3(0.6f, 0.9f, 0.6f),
                       fbw, fbh);
            // A move in progress, or the transient result of the last one.
            if (moveArmed)
                appendText(tv, "MOVE ARMED - click the new spot on this track (M cancels)",
                           x, 40.0f + 6 * lh, sc, glm::vec3(1.0f, 0.9f, 0.4f), fbw, fbh);
            else if (glfwGetTime() < pathMsgUntil && !pathMsg.empty())
                appendText(tv, pathMsg, x, 40.0f + 6 * lh, sc,
                           pathMsg.rfind("cannot", 0) == 0 ? glm::vec3(1.0f, 0.55f, 0.4f)
                                                           : glm::vec3(0.7f, 1.0f, 0.75f),
                           fbw, fbh);
            appendText(tv,
                       g_mouseCaptured ? "CIRCUITS: press Tab to free the cursor"
                                       : "CIRCUITS: click=border  M=move  X=delete  "
                                         "Enter=section  right-click=select  F2=rename",
                       x, 40.0f + 7 * lh, sc, glm::vec3(0.85f, 0.85f, 0.7f), fbw, fbh);
            // Section name labels floating at each section's midpoint.
            for (const Section& s : tc.sections) {
                if (s.parts.empty()) continue;
                const auto& iv = s.parts.front();
                const glm::dvec3 w = fracToWorld(polys, iv.trackId, 0.5 * (iv.from + iv.to));
                if (w.x == 0.0 && w.y == 0.0) continue;
                const glm::dvec3 o = data.sceneOrigin();
                const glm::vec4 clip = viewProj * glm::vec4(float(w.x - o.x), float(w.y - o.y),
                                                            float(w.z - o.z) + 2.0f, 1.0f);
                if (clip.w <= 0.0f) continue;
                const float px = (clip.x / clip.w * 0.5f + 0.5f) * fbw;
                const float py = (clip.y / clip.w * 0.5f + 0.5f) * fbh;
                appendText(tv, s.name, px, py, sc * 0.8f, glm::vec3(1.0f, 1.0f, 0.6f), fbw, fbh);
            }
          } else if (mode == EdMode::Switches) { // --- Switches mode HUD ---
            appendText(tv, "MODE: SWITCHES (Esc menu to switch)", x, 40.0f + 3 * lh,
                       sc, glm::vec3(0.5f, 0.85f, 1.0f), fbw, fbh);
            char selty[48] = "";
            if (selTurnout >= 0)
                std::snprintf(selty, sizeof(selty), "  sel: %s",
                              switchNet.type(selTurnout) == SwitchType::Motor ? "MOTOR"
                                                                              : "MANUAL");
            std::snprintf(buf, sizeof(buf), "SWITCHES %zu%s", switchNet.size(), selty);
            appendText(tv, buf, x, 40.0f + 4 * lh, sc,
                       selTurnout >= 0 ? glm::vec3(1.0f, 0.9f, 0.3f)
                                       : glm::vec3(0.9f, 0.85f, 0.7f),
                       fbw, fbh);
            std::snprintf(buf, sizeof(buf), "UNSAVED %s   %s",
                          switchTypesDirty ? "yes" : "no",
                          switchTypesDirty ? "Ctrl+S to save" : "");
            appendText(tv, buf, x, 40.0f + 5 * lh, sc,
                       switchTypesDirty ? glm::vec3(1.0f, 0.6f, 0.3f)
                                        : glm::vec3(0.6f, 0.9f, 0.6f),
                       fbw, fbh);
            // Locking circuits of the selected motor switch (red-orange ribbons in view).
            if (selTurnout >= 0 && switchNet.type(selTurnout) == SwitchType::Motor) {
                std::string names;
                for (int id : switchNet.lock(selTurnout))
                    for (const Section& s : tc.sections)
                        if (s.id == id)
                            names += (names.empty() ? "" : ", ") +
                                     (s.name.empty() || s.name == "-" ? "S" + std::to_string(s.id)
                                                                      : s.name);
                std::snprintf(buf, sizeof(buf), "LOCK circuits: %s",
                              names.empty() ? "(none - always movable)" : names.c_str());
                appendText(tv, buf, x, 40.0f + 6 * lh, sc, glm::vec3(1.0f, 0.55f, 0.3f),
                           fbw, fbh);
            }
            appendText(tv,
                       g_mouseCaptured ? "SWITCHES: press Tab to free the cursor"
                                       : "SWITCHES: click=select  M=manual/motor  "
                                         "hover circuit + L=add/remove lock",
                       x, 40.0f + 7 * lh, sc, glm::vec3(0.85f, 0.85f, 0.7f), fbw, fbh);
          } else if (mode == EdMode::DistantSignals) { // --- Distant signals mode HUD ---
            appendText(tv, "MODE: DISTANT SIGNALS (Esc menu to switch)", x, 40.0f + 3 * lh,
                       sc, glm::vec3(1.0f, 0.75f, 0.3f), fbw, fbh);
            const bool haveSel = selDistant >= 0 &&
                                 selDistant < static_cast<int>(distantSignals.size());
            char selnm[96] = "";
            if (haveSel)
                std::snprintf(selnm, sizeof(selnm), "  sel: %s",
                              distantSignals[selDistant].name.c_str());
            std::snprintf(buf, sizeof(buf), "DISTANTS %zu%s", distantSignals.size(), selnm);
            appendText(tv, buf, x, 40.0f + 4 * lh, sc,
                       haveSel ? glm::vec3(1.0f, 0.9f, 0.3f) : glm::vec3(0.9f, 0.85f, 0.7f),
                       fbw, fbh);
            // What the selected one can see right now. In the editor every main signal is at
            // danger, so this reports *which* signal it reads, not what that signal says.
            if (haveSel) {
                const DistantSignal& d = distantSignals[selDistant];
                const std::vector<SignalPlacement> pl = allPlacements();
                const int hit = firstMainSignalAhead(polys, junctions, switchNet, pl,
                                                     d.trackId, d.frac, d.dir, kDistantReach);
                if (hit < 0)
                    std::snprintf(buf, sizeof(buf),
                                  "reads %s   no main signal within %.0f km",
                                  d.dir > 0 ? "+frac" : "-frac", kDistantReach / 1000.0);
                else
                    std::snprintf(buf, sizeof(buf), "reads %s   sees %s signal",
                                  d.dir > 0 ? "+frac" : "-frac",
                                  pl[hit].kind == SignalKind::Entry ? "an entry" : "an exit");
                appendText(tv, buf, x, 40.0f + 5 * lh, sc,
                           hit < 0 ? glm::vec3(1.0f, 0.55f, 0.4f) : glm::vec3(0.6f, 1.0f, 0.7f),
                           fbw, fbh);
            }
            std::snprintf(buf, sizeof(buf), "UNSAVED %s   %s", distantDirty ? "yes" : "no",
                          distantDirty ? "Ctrl+S to save" : "");
            appendText(tv, buf, x, 40.0f + 6 * lh, sc,
                       distantDirty ? glm::vec3(1.0f, 0.6f, 0.3f) : glm::vec3(0.6f, 0.9f, 0.6f),
                       fbw, fbh);
            if (glfwGetTime() < pathMsgUntil && !pathMsg.empty())
                appendText(tv, pathMsg, x, 40.0f + 7 * lh, sc, glm::vec3(1.0f, 0.55f, 0.4f),
                           fbw, fbh);
            appendText(tv,
                       g_mouseCaptured
                           ? "DISTANT SIGNALS: press Tab to free the cursor"
                           : "DISTANT SIGNALS: click any spot on a track   F=flip",
                       x, 40.0f + 8 * lh, sc, glm::vec3(0.85f, 0.85f, 0.7f), fbw, fbh);
            // Name labels beside each one, since a distant is a bare point otherwise.
            for (const DistantSignal& d : distantSignals) {
                const glm::dvec3 w = fracToWorld(polys, d.trackId, d.frac);
                if (w.x == 0.0 && w.y == 0.0) continue;
                const glm::dvec3 o = data.sceneOrigin();
                const glm::vec4 clip = viewProj * glm::vec4(float(w.x - o.x), float(w.y - o.y),
                                                            float(w.z - o.z) + 5.5f, 1.0f);
                if (clip.w <= 0.0f) continue;
                appendText(tv, d.name.size() > 20 ? d.name.substr(0, 19) + "~" : d.name,
                           (clip.x / clip.w * 0.5f + 0.5f) * fbw + 10.0f,
                           (clip.y / clip.w * 0.5f + 0.5f) * fbh - 4.0f, sc * 0.8f,
                           glm::vec3(1.0f, 0.85f, 0.5f), fbw, fbh);
            }
          } else { // --- Signal-paths / exit-signals mode HUD ---
            appendText(tv, exitMode    ? "MODE: EXIT SIGNALS (Esc menu to switch)"
                           : entryMode ? "MODE: ENTRY SIGNALS (Esc menu to switch)"
                                       : "MODE: SIGNAL PATHS (Esc menu to switch)",
                       x, 40.0f + 3 * lh, sc,
                       exitMode    ? glm::vec3(1.0f, 0.6f, 0.45f)
                       : entryMode ? glm::vec3(0.45f, 1.0f, 1.0f)
                                   : glm::vec3(0.5f, 1.0f, 0.7f),
                       fbw, fbh);
            // Whichever of the two collections this mode edits is selected. Exactly one is.
            const SignalPath* selR =
                exitMode && selExitRoute >= 0 &&
                        selExitRoute < static_cast<int>(exitRoutes.size())
                    ? &exitRoutes[selExitRoute]
                : selRoute >= 0 && selRoute < static_cast<int>(routes.size())
                    ? &routes[selRoute]
                    : nullptr;
            char selnm[96] = "";
            if (selR) std::snprintf(selnm, sizeof(selnm), "  sel: %s", selR->name.c_str());
            char vtxt[40] = "";
            if (!pendingVias.empty())
                std::snprintf(vtxt, sizeof(vtxt), "  vias: %zu", pendingVias.size());
            else if (selR && !selR->vias.empty())
                std::snprintf(vtxt, sizeof(vtxt), "  vias: %zu", selR->vias.size());
            if (exitMode)
                std::snprintf(buf, sizeof(buf), "EXITS %zu  ROUTES %zu%s%s", routes.size(),
                              exitRoutes.size(), selnm, vtxt);
            else
                std::snprintf(buf, sizeof(buf), "%s %zu%s%s",
                              entryMode ? "ENTRY ROUTES" : "PATHS", routes.size(), selnm,
                              vtxt);
            appendText(tv, buf, x, 40.0f + 4 * lh, sc,
                       selR ? glm::vec3(1.0f, 0.9f, 0.3f) : glm::vec3(0.9f, 0.85f, 0.7f),
                       fbw, fbh);
            // Exit mode says what the selection *is*, since a signal and a route look alike
            // in a name alone: a signal plus how many routes reach it, or a route plus the
            // signal it serves and the authority it grants.
            if (exitMode) {
                if (selR && selExitRoute >= 0) {
                    const char* sname = "(no signal)";
                    for (const SignalPath& e : exitSignals)
                        if (e.id == selR->exitId) sname = e.name.c_str();
                    std::snprintf(buf, sizeof(buf), "ROUTE -> %s   type %s   %zu circuit(s)",
                                  sname,
                                  selR->type == RouteType::C2 ? "C2 (deviation)"
                                                              : "C1 (no restriction)",
                                  pathSections(*selR, tc).size());
                } else if (selR) {
                    int lead = 0;
                    for (const SignalPath& r : exitRoutes)
                        if (r.exitId == selR->id) ++lead;
                    std::snprintf(buf, sizeof(buf), "SIGNAL   %d route(s) lead here", lead);
                } else {
                    std::snprintf(buf, sizeof(buf),
                                  "right-click a signal or a route to select it");
                }
                appendText(tv, buf, x, 40.0f + 5 * lh, sc, glm::vec3(0.8f, 0.9f, 1.0f),
                           fbw, fbh);
            }
            // An entry route says what it authorises and how far, the way an exit route
            // does - it is the same question, just answered by one record instead of two.
            if (entryMode) {
                if (selR)
                    std::snprintf(buf, sizeof(buf), "ROUTE   type %s   %zu circuit(s)",
                                  selR->type == RouteType::C2 ? "C2 (deviation)"
                                                              : "C1 (no restriction)",
                                  pathSections(*selR, tc).size());
                else
                    std::snprintf(buf, sizeof(buf),
                                  "several routes from one border are one signal");
                appendText(tv, buf, x, 40.0f + 5 * lh, sc, glm::vec3(0.8f, 0.9f, 1.0f),
                           fbw, fbh);
            }
            const bool anyDirty = routesDirty || (exitMode && exitRoutesDirty);
            const float dl = (exitMode || entryMode) ? 6.0f : 5.0f; // dirty line
            const float fl = dl + 1.0f;              // feedback line
            const float hl = fl + 1.0f;              // hint line
            std::snprintf(buf, sizeof(buf), "UNSAVED %s   %s", anyDirty ? "yes" : "no",
                          anyDirty ? "Ctrl+S to save" : "");
            appendText(tv, buf, x, 40.0f + dl * lh, sc,
                       anyDirty ? glm::vec3(1.0f, 0.6f, 0.3f) : glm::vec3(0.6f, 0.9f, 0.6f),
                       fbw, fbh);
            // Transient feedback (no route / ambiguous), else the in-progress state. While
            // armed this line carries the disambiguation: a left-click means something else.
            if (glfwGetTime() < pathMsgUntil && !pathMsg.empty())
                appendText(tv, pathMsg, x, 40.0f + fl * lh, sc, glm::vec3(1.0f, 0.55f, 0.4f),
                           fbw, fbh);
            else if (armed())
                appendText(tv, "ADDING ROUTE TO " + exitSignals[armedExit].name +
                               ": click the start border",
                           x, 40.0f + fl * lh, sc, glm::vec3(1.0f, 0.9f, 0.5f), fbw, fbh);
            else if (pathStart >= 0)
                appendText(tv, "start set - click the destination border", x,
                           40.0f + fl * lh, sc, glm::vec3(1.0f, 0.9f, 0.5f), fbw, fbh);
            // Two short hint lines rather than one long one: only ~56 characters fit in
            // the panel, so a single line silently lost its tail - which is where the keys
            // that matter were. The second line says what to do next given the selection.
            const glm::vec3 hintCol(0.85f, 0.85f, 0.7f);
            if (g_mouseCaptured) {
                appendText(tv, exitMode    ? "EXIT SIGNALS: press Tab to free the cursor"
                               : entryMode ? "ENTRY SIGNALS: press Tab to free the cursor"
                                           : "SIGNAL PATHS: press Tab to free the cursor",
                           x, 40.0f + hl * lh, sc, hintCol, fbw, fbh);
            } else if (armed()) {
                // The feedback line above already names the signal and the gesture, so
                // this only has to carry the keys.
                appendText(tv, "V=add via (retries)   X or B=cancel",
                           x, 40.0f + hl * lh, sc, glm::vec3(1.0f, 0.9f, 0.5f), fbw, fbh);
            } else if (entryMode) {
                appendText(tv, "ENTRY SIGNALS: click the signal border, then the",
                           x, 40.0f + hl * lh, sc, hintCol, fbw, fbh);
                appendText(tv, "destination   T=C1/C2  V=via  F2=rename  X=cancel/del",
                           x, 40.0f + (hl + 1) * lh, sc, hintCol, fbw, fbh);
            } else if (!exitMode) {
                appendText(tv, "SIGNAL PATHS: click the start border, then the end",
                           x, 40.0f + hl * lh, sc, hintCol, fbw, fbh);
                appendText(tv, "V=via  right-click=select  F2=rename  X=cancel/del",
                           x, 40.0f + (hl + 1) * lh, sc, hintCol, fbw, fbh);
            } else {
                appendText(tv, "NEW SIGNAL: click its border, then the destination",
                           x, 40.0f + hl * lh, sc, hintCol, fbw, fbh);
                const char* second =
                    selExitRoute >= 0 ? "T=C1/C2  B=another route  F2=rename  X=delete"
                    : selExit >= 0    ? "B=add a route up to this signal  F2/X  V=via"
                                      : "ROUTE UP TO A SIGNAL: right-click it, then B";
                appendText(tv, second, x, 40.0f + (hl + 1) * lh, sc,
                           selR ? glm::vec3(1.0f, 0.9f, 0.5f) : hintCol, fbw, fbh);
            }
            // Name labels at each route's midpoint interval. Names may be long now, so
            // they are clipped here - the HUD shows the selected one in full.
            auto label = [&](const SignalPath& p, const glm::vec3& col, bool withType) {
                if (p.parts.empty()) return;
                const SectionInterval& iv = p.parts[p.parts.size() / 2];
                const glm::dvec3 w = fracToWorld(polys, iv.trackId, 0.5 * (iv.from + iv.to));
                if (w.x == 0.0 && w.y == 0.0) return;
                const glm::dvec3 o = data.sceneOrigin();
                const glm::vec4 clip = viewProj * glm::vec4(float(w.x - o.x), float(w.y - o.y),
                                                            float(w.z - o.z) + 2.0f, 1.0f);
                if (clip.w <= 0.0f) return;
                std::string t = p.name.size() > 20 ? p.name.substr(0, 19) + "~" : p.name;
                if (withType) t += p.type == RouteType::C2 ? "  C2" : "  C1";
                appendText(tv, t, (clip.x / clip.w * 0.5f + 0.5f) * fbw,
                           (clip.y / clip.w * 0.5f + 0.5f) * fbh, sc * 0.8f, col, fbw, fbh);
            };
            for (const SignalPath& p : routes)
                label(p,
                      exitMode    ? glm::vec3(1.0f, 0.8f, 0.7f)
                      : entryMode ? glm::vec3(0.7f, 1.0f, 1.0f)
                                  : glm::vec3(0.7f, 1.0f, 0.8f),
                      entryMode); // an entry route states its own authority
            if (exitMode)
                for (const SignalPath& r : exitRoutes)
                    label(r, glm::vec3(0.7f, 0.9f, 1.0f), true); // the authority it grants
          }
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
            // Circuits: a ring at the candidate border spot on the nearest track (or the
            // existing border it would select), so the click target is visible.
            if (mode == EdMode::Circuits && circHit) {
                const bool onBorder = borderHover >= 0;
                const glm::vec3 hc = moveArmed ? glm::vec3(1.0f, 0.9f, 0.4f)
                                     : onBorder ? glm::vec3(1.0f, 1.0f, 0.3f)
                                                : glm::vec3(0.3f, 0.8f, 1.0f);
                const float R = 10.0f, T = 2.0f;
                quad(circPx.x, circPx.y - R, R, T, hc);
                quad(circPx.x, circPx.y + R, R, T, hc);
                quad(circPx.x - R, circPx.y, T, R, hc);
                quad(circPx.x + R, circPx.y, T, R, hc);
                appendText(tv, moveArmed ? "move here" : onBorder ? "select border"
                                                                  : "add border",
                           circPx.x + 13.0f, circPx.y - 4.0f, sc * 0.7f, hc, fbw, fbh);
            }
            // Switches: a ring around the turnout under the cursor, labelled with its type.
            if (mode == EdMode::Switches && turnoutHover >= 0) {
                const bool motor = switchNet.type(turnoutHover) == SwitchType::Motor;
                const glm::vec3 hc = motor ? glm::vec3(0.3f, 0.8f, 1.0f)
                                           : glm::vec3(1.0f, 0.7f, 0.2f);
                const float R = 11.0f, T = 2.0f;
                quad(turnoutHoverPx.x, turnoutHoverPx.y - R, R, T, hc);
                quad(turnoutHoverPx.x, turnoutHoverPx.y + R, R, T, hc);
                quad(turnoutHoverPx.x - R, turnoutHoverPx.y, T, R, hc);
                quad(turnoutHoverPx.x + R, turnoutHoverPx.y, T, R, hc);
                // Say how many switches share this spot, so it is clear that clicking
                // again picks the next one rather than re-picking the same.
                int here = 0;
                const glm::dvec3 oo = data.sceneOrigin();
                const Turnout& ht = switchNet.turnouts()[turnoutHover];
                for (std::size_t k = 0; k < switchNet.size(); ++k) {
                    const Turnout& t = switchNet.turnouts()[k];
                    if (t.mainPath >= 0 &&
                        std::hypot(t.world.x - ht.world.x, t.world.y - ht.world.y) < 3.0)
                        ++here;
                }
                (void)oo;
                char lbl[64];
                if (here > 1)
                    std::snprintf(lbl, sizeof(lbl), "%s  (#%d of %d here - click to cycle)",
                                  motor ? "motor" : "manual", turnoutHover, here);
                else
                    std::snprintf(lbl, sizeof(lbl), "%s", motor ? "motor" : "manual");
                appendText(tv, lbl, turnoutHoverPx.x + 14.0f,
                           turnoutHoverPx.y - 4.0f, sc * 0.7f, hc, fbw, fbh);
            }
            // Signal paths: ring the border under the cursor (the click target).
            if (routeMode && borderHover >= 0 &&
                borderHover < static_cast<int>(tc.borders.size())) {
                const glm::dvec3 o = data.sceneOrigin();
                const glm::dvec3 w = fracToWorld(polys, tc.borders[borderHover].trackId,
                                                 tc.borders[borderHover].frac);
                const glm::vec4 clip = viewProj * glm::vec4(float(w.x - o.x), float(w.y - o.y),
                                                            float(w.z - o.z), 1.0f);
                if (clip.w > 0.0f) {
                    const glm::vec2 bpx((clip.x / clip.w * 0.5f + 0.5f) * fbw,
                                        (clip.y / clip.w * 0.5f + 0.5f) * fbh);
                    const glm::vec3 hc(1.0f, 1.0f, 0.3f);
                    const float R = 10.0f, T = 2.0f;
                    quad(bpx.x, bpx.y - R, R, T, hc);
                    quad(bpx.x, bpx.y + R, R, T, hc);
                    quad(bpx.x - R, bpx.y, T, R, hc);
                    quad(bpx.x + R, bpx.y, T, R, hc);
                    appendText(tv,
                               armed()        ? "route start"
                               : pathStart < 0 ? ((exitMode || entryMode) ? "signal" : "start")
                                               : "end",
                               bpx.x + 13.0f,
                               bpx.y - 4.0f, sc * 0.7f, hc, fbw, fbh);
                }
            }
            renderer.setOverlayText(tv);
        }

        PushConstants pc{};
        pc.viewProj = viewProj;
        pc.sunDir = glm::vec4(sunDir, data.minElevation());
        pc.camPos = glm::vec4(g_camera.position(), data.maxElevation());
        // Ghost the terrain/rails so the raw geo-point network reads clearly on top.
        pc.params = glm::vec4(0.5f, blinkLit(), 0.0f, 0.0f);

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
