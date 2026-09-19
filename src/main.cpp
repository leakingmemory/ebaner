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

#include "BuildingMesh.h"
#include "Camera.h"
#include "Font.h"
#include "PlatformMesh.h"
#include "RoadMesh.h"
#include "SignalMesh.h"
#include "Script.h"
#include "LineBlock.h"
#include "Occupancy.h"
#include "SignalPaths.h"
#include <cassert>

#include "CrossingMesh.h"
#include "AvalancheMesh.h"
#include "LampGeometry.h"
#include "AvalancheSignals.h"
#include "FlagMesh.h"
#include "TxpGraph.h"
#include "TxpNetwork.h"
#include "TxpMesh.h"
#include "TxpPositions.h"
#include "FlagPosts.h"
#include "LevelCrossings.h"
#include "SimpleEntrySignals.h"
#include "SwitchMesh.h"
#include "SwitchNetwork.h"
#include "SwitchTypes.h"
#include "TerrainData.h"
#include "TerrainMesh.h"
#include "Textures.h"
#include "TrackCircuits.h"
#include "TrackGraph.h"
#include "TrackMesh.h"
#include "SpeedLimits.h"
#include "SpeedSignMesh.h"
#include "StationPicker.h"
#include "Stations.h"
#include "Streaming.h"
#include "TrackPath.h"
#include "TrainYard.h"
#include "TunnelMesh.h"
#include "Audio.h"
#include "Consist.h"
#include "Coupling.h"
#include "Vehicle.h"
#include "VehicleMesh.h"
#include "VulkanRenderer.h"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <cstdio>
#include <unordered_set>
#include <cstdlib>
#include <deque>
#include <exception>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

Camera g_camera;
double g_lastX = 0.0, g_lastY = 0.0;
bool g_firstMouse = true;
bool g_mouseCaptured = true;
bool g_chase = false; // chase-cam mode (ride the rail vehicle)
int g_driverPos = -1; // driver camera: -1 off, else cab index along the whole train
int g_cabCount = 2;   // how many cabs there are to cycle through (set on spawn)
float g_driverYaw = 0.0f, g_driverPitch = 0.0f; // look offsets relative to the train
constexpr float kLookSens = 0.0022f; // radians per pixel (matches Camera)
Audio* g_audio = nullptr; // for the M mute toggle in the key callback
bool g_throwSwitch = false; // T pressed: throw the switch under the crosshair
bool g_menuOpen = false;    // Escape menu overlay (pauses the sim)
int g_menuSel = 0;          // highlighted menu item
bool g_mapMode = false;     // traffic-manager 2-D map view
bool g_mapDirty = false;    // (re)build the map overlay this frame
// The Escape menu is a small state machine rather than one flat list: it can now pick a
// train to drive and place new ones, and each of those is its own list. Out here beside the
// other pickers because Escape has to be able to back out of a step, and Escape is handled
// in the key callback.
enum class MenuStep { Root, PickTrain, PickStation, PickVehicle };
MenuStep g_menuStep = MenuStep::Root;
int g_menuSubSel = 0; // highlighted line of whichever sub-list is open

// The traffic manager's route picker, in two steps like the dispatcher's own panel: a
// station's routes are entries and exits together, and at a worked station there are enough
// of both that one flat list is long to work and easy to misread - "NO MO NB T2" leaves the
// station and "NO MO INFN T2" comes into it, and they sat next to each other.
enum class RoutePickStep { None, PickKind, PickList };
RoutePickStep g_routeStep = RoutePickStep::None;
bool g_routeEntry = false;  // which kind the list is showing
int g_routeKindSel = 0;     // highlighted line of the kind step
int g_routePickSel = 0;     // highlighted route in that picker
bool g_signalPick = false;  // traffic manager: the simple-entry-signal picker is open
int g_signalPickSel = 0;    // highlighted signal in that picker
// Asking for a line is three choices - dispatch, where to, what train - so the station
// panel is the one place that is not a flat list. The step lives out here because Escape
// is handled in the key callback and has to back out of one step at a time.
enum class DispatchStep { None, PickDest, PickType };
DispatchStep g_dispatchStep = DispatchStep::None;
int g_dispatchSel = 0;      // highlighted line within the current step
std::string g_dispatchTo;   // the destination, held between the two steps
// Traffic manager: pending station changes, +1 for the next station and -1 for the
// previous. An accumulator rather than an index because the key callback cannot see the
// station list, which lives in main() with everything else the panel works from.
int g_tmStationStep = 0;
float g_mapZoom = 1.0f;     // map zoom: 1 = ~4 km tall, higher = zoomed in
constexpr float kMapZoomMin = 0.5f;  // ~8 km tall (zoomed out)
constexpr float kMapZoomMax = 40.0f; // ~100 m tall (zoomed in)
glm::vec2 g_mapPan(0.0f);   // WASD pan offset from the default centre (scene metres)
std::string g_mapMsg;       // transient map feedback (e.g. a blocked-throw reason)
double g_mapMsgUntil = 0.0; // glfwGetTime() until which g_mapMsg is shown
bool g_mapMsgOk = false;    // whether it reports something done or something refused

void cursorCallback(GLFWwindow*, double x, double y) {
    if (g_menuOpen || g_mapMode) return; // menu/map open: cursor is free, no mouselook
    if (!g_mouseCaptured) { g_firstMouse = true; return; }
    if (g_firstMouse) { g_lastX = x; g_lastY = y; g_firstMouse = false; return; }
    const float dx = static_cast<float>(x - g_lastX);
    const float dy = static_cast<float>(y - g_lastY);
    g_lastX = x;
    g_lastY = y;
    if (g_driverPos >= 0) { // free-look around the cab, relative to the train
        g_driverYaw -= dx * kLookSens;
        g_driverPitch = glm::clamp(g_driverPitch - dy * kLookSens, -1.4f, 1.4f);
    } else {
        g_camera.look(dx, dy);
    }
}

void scrollCallback(GLFWwindow*, double, double yoff) {
    // Mouse wheel zooms the traffic-manager map (ignored in the cab view).
    if (!g_mapMode) return;
    g_mapZoom = glm::clamp(g_mapZoom * std::pow(1.15f, static_cast<float>(yoff)),
                           kMapZoomMin, kMapZoomMax);
}

void keyCallback(GLFWwindow* win, int key, int, int action, int) {
    // Escape closes the route picker if it is open, else toggles the menu overlay; while
    // the menu is open the other hotkeys are inert.
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        // One step at a time out of the dispatch choices, and only then out of the panel:
        // choosing the wrong destination should cost the destination, not the whole thing.
        if (g_dispatchStep == DispatchStep::PickType) {
            g_dispatchStep = DispatchStep::PickDest;
            g_dispatchSel = 0;
        } else if (g_dispatchStep == DispatchStep::PickDest) {
            g_dispatchStep = DispatchStep::None;
            g_dispatchTo.clear();
        } else if (g_signalPick) {
            g_signalPick = false;
        } else if (g_routeStep == RoutePickStep::PickList) {
            g_routeStep = RoutePickStep::PickKind;
            g_routePickSel = 0;
        } else if (g_routeStep == RoutePickStep::PickKind) {
            g_routeStep = RoutePickStep::None;
        } else if (g_menuOpen && g_menuStep != MenuStep::Root) {
            g_menuStep = MenuStep::Root; // back out of a sub-list, not out of the menu
            g_menuSubSel = 0;
        } else {
            g_menuOpen = !g_menuOpen;
            g_menuStep = MenuStep::Root;
            g_menuSubSel = 0;
        }
    }
    if (g_menuOpen) return;
    // Tab toggles mouse capture (handy for debugging).
    if (key == GLFW_KEY_TAB && action == GLFW_PRESS) {
        g_mouseCaptured = !g_mouseCaptured;
        glfwSetInputMode(win, GLFW_CURSOR,
                         g_mouseCaptured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
        g_firstMouse = true;
    }
    // C toggles the chase camera (ride the rail vehicle); leaves the driver view.
    if (key == GLFW_KEY_C && action == GLFW_PRESS) { g_chase = !g_chase; g_driverPos = -1; }
    // V cycles the driver camera through the cab positions, then back to free-fly.
    // Every cab of every set, including the shut-down ones at a coupler: they cannot
    // drive, but they can be sat in and looked out of.
    if (key == GLFW_KEY_V && action == GLFW_PRESS) {
        const int n = std::max(1, g_cabCount);
        g_driverPos = (g_driverPos + 2) % (n + 1) - 1; // -1 -> 0 .. n-1 -> -1
        g_driverYaw = g_driverPitch = 0.0f;      // face forward on entering / switching
        if (g_driverPos >= 0) g_chase = false;
    }
    // M mutes / unmutes the synthesized sound.
    if (key == GLFW_KEY_M && action == GLFW_PRESS && g_audio) g_audio->toggleMuted();
    // T throws the switch stand the crosshair is aimed at.
    if (key == GLFW_KEY_T && action == GLFW_PRESS) g_throwSwitch = true;
    // O toggles the traffic-manager 2-D map (overview).
    if (key == GLFW_KEY_O && action == GLFW_PRESS) {
        g_mapMode = !g_mapMode;
        g_mapDirty = g_mapMode;
        if (g_mapMode) g_mapPan = glm::vec2(0.0f); // start centred on the throat
        // Free the cursor so switches can be clicked; restore mouselook on leaving.
        glfwSetInputMode(win, GLFW_CURSOR,
                         (g_mapMode || !g_mouseCaptured) ? GLFW_CURSOR_NORMAL
                                                         : GLFW_CURSOR_DISABLED);
        g_firstMouse = true;
    }
    // R offers the exit routes of the nearest station, to set a main-signal route. Only in
    // the map: R is the cab reverser, which means nothing from a dispatcher's view.
    if (g_mapMode && key == GLFW_KEY_R && action == GLFW_PRESS) {
        g_routeStep = g_routeStep == RoutePickStep::None ? RoutePickStep::PickKind
                                                         : RoutePickStep::None;
        g_routeKindSel = 0;
        g_routePickSel = 0;
    }
    // E offers the simple entry signals of the nearest station. Only in the map, and
    // only E is free there - the map binds nothing but WASD to pan.
    if (g_mapMode && key == GLFW_KEY_E && action == GLFW_PRESS) {
        g_signalPick = !g_signalPick;
        g_signalPickSel = 0;
        g_dispatchStep = DispatchStep::None; // no half-made choice left behind
        g_dispatchTo.clear();
    }
    // Walk to the next / previous station to work. Only in the map: N is the cab
    // reverser's neutral, which means nothing from a dispatcher's view - the same reason
    // R is free here. A dispatcher works one station at a time and needs to reach the
    // others without hunting for them across the map.
    //
    // Left / Right do the same. They are the obvious keys for stepping along a line and
    // nothing in the map uses them (the pickers take Up / Down), so there is no reason to
    // make someone remember that "back" is B.
    if (g_mapMode && action == GLFW_PRESS) {
        if (key == GLFW_KEY_N || key == GLFW_KEY_RIGHT) ++g_tmStationStep;
        if (key == GLFW_KEY_B || key == GLFW_KEY_LEFT) --g_tmStationStep;
    }
    // Z / X zoom the map in / out (keyboard alternative to the scroll wheel;
    // repeat-enabled so holding the key keeps zooming).
    if (g_mapMode && (action == GLFW_PRESS || action == GLFW_REPEAT)) {
        if (key == GLFW_KEY_Z)
            g_mapZoom = glm::min(g_mapZoom * 1.25f, kMapZoomMax);
        if (key == GLFW_KEY_X)
            g_mapZoom = glm::max(g_mapZoom / 1.25f, kMapZoomMin);
    }
}

VulkanRenderer* g_renderer = nullptr;
void resizeCallback(GLFWwindow*, int, int) {
    if (g_renderer) g_renderer->notifyResize();
}

} // namespace

int main(int argc, char** argv) {
    const std::string datasetRoot = (argc > 1) ? argv[1] : "../norway-rails";

    // Offline audio checks: render a scripted sequence to a WAV and exit.
    if (const char* dump = std::getenv("EBANER_AUDIO_DUMP")) {
        Audio::dumpTest(dump);
        return EXIT_SUCCESS;
    }
    if (const char* dump = std::getenv("EBANER_AUDIO_DUMP_IMPACT")) {
        Audio::dumpImpactTest(dump);
        return EXIT_SUCCESS;
    }
    if (const char* dump = std::getenv("EBANER_AUDIO_DUMP_ENGINE")) {
        Audio::dumpEngineTest(dump);
        return EXIT_SUCCESS;
    }
    if (const char* dump = std::getenv("EBANER_AUDIO_DUMP_CROSSING")) {
        Audio::dumpCrossingTest(dump);
        return EXIT_SUCCESS;
    }
    if (const char* dump = std::getenv("EBANER_AUDIO_DUMP_GRID")) {
        Audio::dumpGridTest(dump);
        return 0;
    }
    if (const char* dump = std::getenv("EBANER_AUDIO_DUMP_ROLLING")) {
        Audio::dumpRollingTest(dump);
        return EXIT_SUCCESS;
    }

    // Which station to start at. The dataset carries them - name, position, and
    // whether it is a station or a stop - so this is a lookup, not a table of ours.
    const std::vector<Station> stations = loadStations(datasetRoot);
    const Station* start = pickStation(stations, (argc > 2) ? argv[2] : "");
    if (!start) return EXIT_FAILURE;

    // --- Window ---
    if (!glfwInit()) {
        std::fprintf(stderr, "glfwInit failed\n");
        return EXIT_FAILURE;
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow* window =
        glfwCreateWindow(1280, 720, "ebaner - Bodo terrain", nullptr, nullptr);
    if (!window) {
        std::fprintf(stderr, "window creation failed (is Vulkan/WSI available?)\n");
        glfwTerminate();
        return EXIT_FAILURE;
    }

    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSetCursorPosCallback(window, cursorCallback);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetFramebufferSizeCallback(window, resizeCallback);

    // --- Land-cover textures ---
    std::vector<std::uint8_t> texPixels = landtex::generate();
    LandTextureData texData;
    texData.pixels = texPixels.data();
    texData.size = landtex::SIZE;
    texData.layers = landtex::LAYERS;
    texData.byteSize = texPixels.size();

    // --- Renderer -----------------------------------------------------------
    // Brought up on an empty world, before a single tile is read. That is what lets
    // the station be chosen on screen: the terrain window is centred on whichever
    // one is picked, so nothing can be read until it is.
    VulkanRenderer renderer;
    g_renderer = &renderer;
    try {
        renderer.init(window, {}, {}, texData, {}, {}, 0, {}, {}, {}, {}, {});
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Vulkan init failed: %s\n", e.what());
        glfwDestroyWindow(window);
        glfwTerminate();
        return EXIT_FAILURE;
    }

    // --- Load terrain data ---
    TerrainData data;
    TerrainMesh mesh;
    TunnelMesh tunnels;
    TrackMesh tracks;
    RoadMesh roads;
    BuildingMesh buildings;
    PlatformMesh platforms;
    SwitchMesh switches;
    SwitchNetwork switchNet;
    std::vector<TrackPath> paths;
    TrackGraph graph; // raw track lines (scene-relative), for the 2-D traffic-manager map

    // --- Start screen, part one: where ---------------------------------------
    // Reading the world takes long enough that it cannot be done speculatively, so the
    // station is settled here and the loading follows. The vehicle is chosen afterwards,
    // once there are paths for it to stand on.
    if (const Station* picked = runStationPicker(window, renderer, stations, start)) {
        start = picked;
    } else {
        renderer.cleanup();
        glfwDestroyWindow(window);
        glfwTerminate();
        return EXIT_SUCCESS; // closed at the picker
    }
    glfwSetWindowTitle(window, ("ebaner - " + start->name).c_str());
    std::printf("[main] starting at %s (%s)\n", start->name.c_str(),
                start->line.c_str());
    drawLoadingNotice(window, renderer, *start);

    try {
        data.load(datasetRoot, start->world);
       
        paths = buildTrackPaths(data);
       
        // The bores first: the terrain needs them to know which of its triangles are
        // standing in a tunnel mouth.
        tunnels.build(data);
        std::printf("[TunnelMesh] %zu bore(s), %.0f m, %zu vertices\n", tunnels.boreCount(),
                    tunnels.totalLength(), tunnels.vertices().size());
        tracks.build(paths, glm::vec3(0.0f), data.loadedRadius());
        roads.build(data);
        buildings.build(data);
        platforms.build(data, paths);
        // turnout detection + routing, minus whatever the overlay says is not a switch
        switchNet.build(data, paths, loadSwitchSuppressions(datasetRoot));
        applySwitchTypes(switchNet, loadSwitchTypes(datasetRoot)); // manual/motor overrides
        switches.build(switchNet, glm::vec3(0.0f), data.loadedRadius());
        graph = buildTrackGraph(data);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Failed to load terrain: %s\n", e.what());
        return EXIT_FAILURE;
    }

    // --- First rail vehicle: a wheelset on the main line near the start ---
    const TrackPath* vpath = nullptr;
    float vs = 0.0f;
    {
        // The start point is on a main line by construction, so the one we want runs
        // within metres of the origin. Reject on the bounding box first: walking every
        // path in the country at 5 m steps is the better part of a million samples.
        float best = 1e30f;
        for (float reach : {2000.0f, -1.0f}) { // then unbounded, if the line is odd here
            for (const TrackPath& p : paths) {
                if (p.trackType() != 0) continue; // main line only
                if (reach > 0.0f && !p.nearXY(glm::vec2(0.0f), reach)) continue;
                for (float s = 0.0f; s <= p.length(); s += 5.0f) {
                    const glm::vec3 q = p.poseAt(s).pos; // nearest point to scene origin
                    const float d2 = q.x * q.x + q.y * q.y;
                    if (d2 < best) { best = d2; vpath = &p; vs = s; }
                }
            }
            if (vpath) break;
        }
        if (!vpath && !paths.empty()) vpath = &paths[0];
    }

    // Every train in the world. The one the driver is on is chosen on the start screen
    // and created on confirm; parting a coupler adds a second, and from then on this
    // loop has to mean "every train" wherever it is talking about occupancy, level
    // crossings, sound or geometry, and "the one I am on" only where it is talking
    // about the controls, the HUD and the cameras.
    //
    // A deque and not a vector, deliberately: `vehicle` points into this, and a vector
    // would move its elements out from under that pointer the moment an uncoupling
    // appended to it. A moved-from Consist has no sets at all, so lead() on one is not
    // a morning anybody wants.
    //
    // Only ever appended to, which is what keeps `vehicle` valid: a deque moves nothing
    // on push_back. A parted portion therefore goes on the end rather than in behind the
    // train it came out of, so the order the sets reach the vehicle mesh in changes -
    // and because the vehicle index buffer is fixed from the moment it is attached, a
    // change of composition has to be followed by attachVehicle and not by another
    // vertex refresh. Under the old indices the trains would draw as a heap of triangles
    // and nothing would say why.
    std::deque<Consist> trains;
    int driverTrain = 0;        // index into `trains` of the one being driven
    Consist* vehicle = nullptr; // &trains[driverTrain]; re-seated whenever that changes
    // The coupler U has been aimed at and is waiting for a second press on: which train,
    // which of its couplers, and when it was armed. -1 is nothing armed. Up here rather
    // than beside the other input state because the marker drawn on it is built in
    // rebuildSignalBuffer, which is defined further down but before the loop.
    int armedTrain = -1, armedCoupler = -1;
    double armedAt = 0.0;
    VehicleMesh vmesh;
    Audio audio;
    g_audio = &audio; // init() is deferred until just before the render loop


    // Start camera at the track-1 terminus, a few metres up, looking down the line.
    glm::vec3 startPos = data.startPos() + glm::vec3(0.0f, 0.0f, 5.0f);
    g_camera.init(startPos, data.startDir());

    // Optional scripted camera for verification:
    // EBANER_CAM="x,y,z,yawDeg,pitchDeg" (scene-relative metres).
    if (const char* cam = std::getenv("EBANER_CAM")) {
        float x, y, z, yaw, pitch;
        if (std::sscanf(cam, "%f,%f,%f,%f,%f", &x, &y, &z, &yaw, &pitch) == 5) {
            g_camera.setPose(glm::vec3(x, y, z), glm::radians(yaw),
                             glm::radians(pitch));
            std::printf("[main] scripted camera: pos=(%.1f,%.1f,%.1f) "
                        "yaw=%.1f pitch=%.1f\n", x, y, z, yaw, pitch);
        }
    }

    // Platforms are the same solid-lit static geometry as buildings and draw
    // identically, so merge them into the building buffers (offsetting the
    // platform indices) rather than adding new renderer plumbing.
    std::vector<TrackVertex> structVerts = buildings.vertices();
    std::vector<std::uint32_t> structIndices = buildings.indices();
    {
        const std::uint32_t vbase =
            static_cast<std::uint32_t>(structVerts.size());
        structVerts.insert(structVerts.end(), platforms.vertices().begin(),
                           platforms.vertices().end());
        structIndices.reserve(structIndices.size() + platforms.indices().size());
        for (std::uint32_t idx : platforms.indices())
            structIndices.push_back(idx + vbase);
    }
    // Speed-increase signs are derived from the line speeds rather than authored, and never
    // change, so they join the static bucket too.
    SpeedSignMesh speedSignMesh;
    {
        const std::vector<SpeedSign> sg = speedSigns(paths, glm::vec3(0.0f), data.loadedRadius());
        speedSignMesh.build(sg);
        const std::uint32_t vbase = static_cast<std::uint32_t>(structVerts.size());
        structVerts.insert(structVerts.end(), speedSignMesh.vertices().begin(),
                           speedSignMesh.vertices().end());
        structIndices.reserve(structIndices.size() + speedSignMesh.indices().size());
        for (std::uint32_t idx : speedSignMesh.indices()) structIndices.push_back(idx + vbase);
        std::printf("[SpeedSigns] %zu sign(s), %zu vertices\n", sg.size(),
                    speedSignMesh.vertices().size());
    }
    // The tunnel bores are static rock, so they belong in the same bucket.
    {
        const std::uint32_t vbase = static_cast<std::uint32_t>(structVerts.size());
        structVerts.insert(structVerts.end(), tunnels.vertices().begin(),
                           tunnels.vertices().end());
        structIndices.reserve(structIndices.size() + tunnels.indices().size());
        for (std::uint32_t idx : tunnels.indices()) structIndices.push_back(idx + vbase);
    }
    // Switch stands go in a dynamic buffer (rebuilt when a switch is thrown), not the
    // static struct bucket — attached just after renderer.init below. The ground signals
    // are dynamic too (their lamps follow the aspect), attached once `polys` exists.

    // --- Hand the built world to the renderer --------------------------------
    // What renderer.init used to do, now that it comes up empty and the world arrives
    // afterwards. The ground goes in per tile, exactly as the streamer will replace it.
    {
        TerrainMesh chunk;
        std::size_t verts = 0;
        for (const auto& [key, t] : data.tiles()) {
            chunk.buildTile(data, *t, &tunnels);
            renderer.setTerrainChunk(key, chunk.vertices(), chunk.indices());
            verts += chunk.vertices().size();
        }
        std::printf("[TerrainMesh] %zu chunk(s), %zu vertices\n",
                    renderer.terrainChunkCount(), verts);
    }
    renderer.updateTracks(tracks.vertices(), tracks.indices(),
                          tracks.alwaysIndexCount(), tracks.sleeperChunks(),
                          tracks.alwaysChunks());
    renderer.updateRoads(roads.vertices(), roads.indices());
    // The buildings' own chunks, and how far into the buffer they reach: what follows
    // them here is platforms, speed signs and bores, which are always drawn.
    renderer.updateStructs(structVerts, structIndices, buildings.chunks(),
                           static_cast<std::uint32_t>(buildings.indices().size()));

    renderer.attachSwitches(switches.vertices(), switches.indices());

    // From here the world follows the camera: tiles are read and dropped, and every mesh
    // built from them is rebuilt, on a worker thread. The rail network is not rebuilt -
    // it is already the whole line - so nothing a vehicle is standing on is disturbed.
    WorldStreamer streamer;
    // The scene point the visual geometry currently follows. Switch stands are rebuilt on
    // their own when one is thrown, and have to be built about the same point.
    glm::vec3 worldCentre(0.0f);
    glm::vec2 elevRange(data.minElevation(), data.maxElevation());
    if (std::getenv("EBANER_NOSTREAM") == nullptr)
        streamer.start(data, paths, switchNet);

    std::printf(
        "\nControls: WASD move, Q/E down/up, mouse look, Shift boost, "
        "C chase vehicle, V driver view (switch cab), I engines start/stop, "
        "Up/Down push vehicle, , / . power/brake lever (the power controller on a "
        "machine with two handles - back past neutral is its electric brake - whose "
        "train brake is K release / L apply and whose independent loco brake is "
        "H release / J apply), Space emergency, "
        "F/N/R reverser, T throw aimed switch, U uncouple, M mute, "
        "Tab release cursor, Esc menu (drive another train, place one)\n\n");

    // Directional sun (scene space): from the south-west, fairly high.
    const glm::vec3 sunDir = glm::normalize(glm::vec3(0.4f, -0.5f, 0.75f));

    // Optional one-shot screenshot: EBANER_SCREENSHOT=path renders a few frames,
    // captures, then exits (used for headless verification).
    const char* shotPath = std::getenv("EBANER_SCREENSHOT");
    int frame = 0;

    // Spawns the chosen vehicle once (create + mesh + attach + log physics).
    auto spawnVehicle = [&](int idx) {
        if (!vpath) return;
        idx = std::clamp(idx, 0, kNumVehicleSpecs - 1);
        const VehicleSpec& sp = kVehicleSpecs[idx];
        // Keep the whole vehicle on the track: the outermost axle sits
        // (bogieSpacing + wheelbase)/2 from the body centre. Nudge the spawn in
        // from the ends (long carriages otherwise straddle the buffer-stop end).
        const float outerHalf =
            0.5f * (sp.bogieSpacing + sp.wheelbase) +
            static_cast<float>(std::max(0, sp.units - 1)) *
                0.5f * (sp.length + Consist::kCouplerGap);
        float startS = vs;
        const float L = vpath->length(), margin = outerHalf + 1.0f;
        if (L > 2.0f * margin) {
            startS = std::clamp(vs, margin, L - margin);
        } else {
            // Nowhere on this path fits the train. Worth saying, because what happens
            // otherwise looks like nothing: the sets past the end are pinned there by
            // the arc-length clamp, the train draws as a heap and derails on the first
            // step with nothing to explain it. Three sets need 119 m of clear road
            // where one needs 35, so a path that was long enough before may not be.
            std::printf("[Vehicle] %s wants %.0f m of clear road and this one is %.0f m "
                        "- it will not fit and will derail at once\n",
                        specTitle(sp), 2.0f * margin, L);
        }
        trains.clear();
        trains.emplace_back(&paths, vpath, sp, startS);
        driverTrain = 0;
        vehicle = &trains.front();
        vehicle->attachNetwork(&paths, &switchNet); // divert at switches
        vmesh.build(trains);
        g_cabCount = drivercam::count(*vehicle);
        renderer.attachVehicle(vmesh.vertices(), vmesh.indices(),
                               vmesh.glassFirstIndex());

        const GravityResolution g = vehicle->gravity();
        float maxGradeDeg = 0.0f, kMax = 0.0f, cantAtMax = 0.0f;
        for (float s = 0.0f; s <= vpath->length(); s += 5.0f) {
            const TrackPose p = vpath->poseAt(s);
            const float gr = glm::degrees(std::asin(glm::clamp(p.tangent.z, -1.0f, 1.0f)));
            if (std::abs(gr) > std::abs(maxGradeDeg)) maxGradeDeg = gr;
            if (std::abs(p.curvature) > kMax) { kMax = std::abs(p.curvature); cantAtMax = p.cant; }
        }
        const glm::vec3 I = vehicle->inertia();
        if (vehicle->unitCount() > 1)
            std::printf("[Vehicle] %d sets coupled: %.1f m over the couplers, %d axles, "
                        "%d cabs (%d drive), %d engines\n",
                        vehicle->unitCount(), vehicle->length(), vehicle->axleCount(),
                        vehicle->cabCount(), 2, vehicle->engineCount());
        std::printf("[Vehicle] %s: mass %.0f kg, dims LxWxH = %.2fx%.2fx%.2f m, "
                    "wheelbase %.2f m; inertia (roll,pitch,yaw) = (%.0f,%.0f,%.0f) "
                    "kg*m^2; CoM %.2f m; Davis %.0f/%.0f/%.0f N @0/10/30 m/s\n",
                    vehicle->name(), vehicle->mass(), vehicle->length(),
                    vehicle->width(), vehicle->height(), vehicle->wheelbase(), I.x,
                    I.y, I.z, vehicle->comHeight(), vehicle->rollingResistance(0.0f),
                    vehicle->rollingResistance(10.0f),
                    vehicle->rollingResistance(30.0f));
        if (kMax > 1e-6f) {
            const TippingLimit tl = vehicle->tippingLimit(kMax, cantAtMax);
            std::printf("[Vehicle] steepest grade %+.2f deg; sharpest curve R=%.0f m "
                        "-> overturn at %.0f km/h\n",
                        maxGradeDeg, 1.0f / kMax, tl.critSpeed * 3.6f);
        }
        (void)g;
    };

    // Where the map is centred (scene-relative): the centroid of the routed turnouts of
    // the station started at - its throat, which is the operational detail the view is
    // about, so zooming stays framed on it. Falls back to the scene origin if there are
    // none.
    //
    // Only the turnouts near the origin count. The rail network is resident and global,
    // so averaging all of them puts the centre in the middle of the country and opens the
    // traffic manager on a station hundreds of kilometres from the train.
    glm::vec2 mapCenter(0.0f);
    {
        constexpr double kThroatM = 4000.0; // the station and its approaches, no more
        const glm::dvec3 org = switchNet.sceneOrigin();
        const auto& tos = switchNet.turnouts();
        glm::dvec2 sum(0.0);
        int n = 0;
        for (const auto& t : tos) {
            if (t.mainPath < 0) continue; // inert crossing: not a working switch
            const glm::dvec2 at(t.world.x - org.x, t.world.y - org.y);
            if (glm::length(at) > kThroatM) continue;
            sum += at;
            ++n;
        }
        if (n > 0) mapCenter = glm::vec2(sum / static_cast<double>(n));
    }

    // Bounds of the whole track network (scene-relative), used to clamp WASD panning
    // so the view can follow the line out of the station but not drift into the void.
    glm::vec2 mapMin(1e30f), mapMax(-1e30f);
    for (const LineVertex& lv : graph.lines) {
        mapMin = glm::min(mapMin, glm::vec2(lv.pos));
        mapMax = glm::max(mapMax, glm::vec2(lv.pos));
    }
    if (graph.lines.empty()) { mapMin = mapMax = mapCenter; }

    // --- Track circuits (sensing sections) + live occupancy -------------------------
    // Sections are authored in the overlay file, anchored to track id + arc-length. We
    // draw each on the map and light it red when a wheelset (axle) is inside it - the
    // same thing that shunts a real track circuit. The section geometry is static, so
    // sample it once into scene-relative polylines; occupancy is recomputed per frame.
    const TrackCircuits circuits = loadTrackCircuits(datasetRoot);
    std::vector<TrackPoly> polys;
    for (std::size_t i = 0; i < graph.pointWorld.size(); ++i) {
        if (polys.empty() || polys.back().id != graph.pointTrack[i])
            polys.push_back({graph.pointTrack[i], {}});
        polys.back().pts.push_back(graph.pointWorld[i]);
    }
    // Mini ground signals (dvergsignal) at the signal-path starts. Their lamps follow the
    // aspect, so they live in a dynamic buffer, rebuilt when an aspect changes.
    const std::vector<SignalPath> signalPaths = loadSignalPaths(datasetRoot);
    // Main signals stand on a border too: an exit protecting the route out of the station,
    // an entry authorising one in. They live in one placement list with the dwarfs, so a
    // pair at the same border shares a pole.
    const std::vector<SignalPath> exitSignals = loadExitSignals(datasetRoot);
    const std::vector<SignalPath> entrySignals = loadEntrySignals(datasetRoot);
    std::vector<SignalPlacement> mainPlacements =
        signalPlacements(exitSignals, polys, SignalKind::Exit);
    {
        // Several entry routes leaving one border are one mast, which signalPlacements
        // already does; they simply join the exit masts in the same list.
        const std::vector<SignalPlacement> entries =
            signalPlacements(entrySignals, polys, SignalKind::Entry);
        mainPlacements.insert(mainPlacements.end(), entries.begin(), entries.end());
    }
    // Block signals out on the line. They carry no route of their own: each governs the
    // signal path already authored from its border facing its way, and that road is what it
    // opens before it clears - so the line block is resolved here, before the placements
    // are built, and its answer decides both.
    const std::vector<BlockSignal> blockSignals = loadBlockSignals(datasetRoot);
    const LineBlocks lineBlocks =
        resolveLineBlocks(blockSignals, signalPaths, polys, circuits);
    for (const std::string& n : lineBlocks.notes) std::printf("[Block] %s\n", n.c_str());
    // Which mini paths belong to a block signal, so nothing else may work them.
    std::vector<char> blockRoad(signalPaths.size(), 0);
    for (const BlockRoad& br : lineBlocks.roads)
        if (br.road >= 0) blockRoad[br.road] = 1;
    std::vector<SignalPlacement> dwarfPlacements = signalPlacements(signalPaths, polys);
    // A block signal's road already has a dwarf built from it, since every mini path gets
    // one. Left in place, opening that road to clear the block signal would light a
    // shunting signal beside it and offer it on the map as something to set by hand.
    dropBlockRoads(dwarfPlacements, blockRoad);
    std::vector<SignalPlacement> sigPlacements =
        mergeSignals(dwarfPlacements, mainPlacements);
    // Appended after the merge like the distants rather than merged as a main: there is
    // nothing at a block signal's border for it to share a pole with any more.
    std::vector<int> blockPlacement(blockSignals.size(), -1);
    for (std::size_t i = 0; i < blockSignals.size(); ++i) {
        if (lineBlocks.roads[i].road < 0) continue; // no road: nothing to stand for
        const SignalPath& road = signalPaths[lineBlocks.roads[i].road];
        SignalPlacement sp;
        if (!routeStartPose(road, polys, sp.world, sp.forward)) continue;
        sp.kind = SignalKind::Block;
        sp.at = blockSignals[i].at;
        sp.side = blockSignals[i].side;
        sp.paths.push_back(lineBlocks.roads[i].road);
        blockPlacement[i] = static_cast<int>(sigPlacements.size());
        sigPlacements.push_back(std::move(sp));
    }
    for (std::size_t li = 0; li < lineBlocks.lines.size(); ++li) {
        const LineBlock& L = lineBlocks.lines[li];
        std::printf("[Block] line %zu %-10s %zu signal(s), %zu section(s)%s\n", li,
                    L.name.c_str(), L.signals.size(), L.sections.size(),
                    L.sound ? "" : "  UNSOUND - held at danger");
    }
    // Distant signals stand at a plain point rather than on a route, so they are placed
    // directly and appended after the merge - one must never fold onto a dwarf's pole.
    const std::vector<DistantSignal> distantSignals = loadDistantSignals(datasetRoot);
    for (std::size_t i = 0; i < distantSignals.size(); ++i) {
        const DistantSignal& d = distantSignals[i];
        const glm::dvec3 w = fracToWorld(polys, d.trackId, d.frac);
        if (w.x == 0.0 && w.y == 0.0) continue; // stale/missing track
        SignalPlacement sp;
        sp.kind = SignalKind::Distant;
        sp.world = w;
        sp.forward = trackTangent(polys, d.trackId, d.frac, d.dir);
        sp.at = {d.trackId, d.frac};
        sp.side = d.side;
        sp.paths.push_back(static_cast<int>(i));
        sigPlacements.push_back(std::move(sp));
    }
    // Simple entry signals: red or green, no circuits, one green per station. Placed
    // like the distants - a plain point on a track - but they govern rather than repeat.
    const std::vector<SimpleEntrySignal> simpleEntries =
        loadSimpleEntrySignals(datasetRoot);
    const std::vector<SignalStation> simpleEntryStation =
        attachStations(simpleEntries, stations, polys);
    // What each station is doing. A station is either unmanned - its signals dark, trains
    // running through without reference to them - or manned, when they show red and one of
    // them may be cleared. Unmanned is the default and the absent case: a line with signals
    // newly placed on it still runs exactly as it did before they were there, rather than
    // stopping dead until someone goes and clears them.
    //
    // Runtime only. Which station is manned belongs in the overlay no more than a switch
    // position does.
    struct StationState {
        bool manned = false;
        int green = -1; // index into simpleEntries, -1 = all red
    };
    std::unordered_map<std::string, StationState> simpleStationState;
    // placement index -> index into simpleEntries, so the aspect can be written back.
    std::vector<int> simpleEntryPlacement(simpleEntries.size(), -1);
    for (std::size_t i = 0; i < simpleEntries.size(); ++i) {
        const SimpleEntrySignal& e = simpleEntries[i];
        const glm::dvec3 w = fracToWorld(polys, e.trackId, e.frac);
        if (w.x == 0.0 && w.y == 0.0) continue; // stale/missing track
        SignalPlacement sp;
        sp.kind = SignalKind::StationEntry;
        sp.world = w;
        sp.forward = trackTangent(polys, e.trackId, e.frac, e.dir);
        sp.at = {e.trackId, e.frac};
        sp.side = e.side;
        sp.paths.push_back(static_cast<int>(i));
        simpleEntryPlacement[i] = static_cast<int>(sigPlacements.size());
        sigPlacements.push_back(std::move(sp));
    }
    if (!simpleEntries.empty())
        std::printf("[SimpleEntry] %zu signal(s)\n", simpleEntries.size());
    // The junction graph is geometry, so it cannot change while the viewer runs: build it
    // once here rather than on every distant-signal read.
    const TrackJunctions junctions = trackJunctions(polys);
    // What every mini path needs of the turnouts and which circuits it runs through, worked
    // out once. Both are fixed by the geometry, and deriving them per frame is what made a
    // train entering a circuit stall the picture - see RouteStatics in SignalPaths.h.
    const RouteStatics pathStatics = routeStatics(signalPaths, switchNet, polys, circuits);
    // The map is a shunting view: it shows the dwarfs and the mini routes they set. An exit
    // signal sharing a dwarf's pole appears as that dwarf; one standing alone has nothing to
    // set there, so the map skips it. These pick the dwarf half of a shared placement.
    auto isMini = [](const SignalPlacement& sp) {
        return sp.kind == SignalKind::Dwarf || sp.withDwarf;
    };
    auto miniPaths = [](const SignalPlacement& sp) -> const std::vector<int>& {
        return sp.kind == SignalKind::Dwarf ? sp.paths : sp.dwarfPaths;
    };
    auto miniAspect = [](const SignalPlacement& sp) {
        return sp.kind == SignalKind::Dwarf ? sp.aspect : sp.dwarfAspect;
    };
    // Level crossings, secured by lights alone. Their detection circuits are their own -
    // arc-length ranges either side of the crossing - so nothing has to be authored for
    // one to work, and none of the track-circuit machinery is involved.
    const std::vector<LevelCrossing> crossings = loadLevelCrossings(datasetRoot);
    const std::vector<CrossingSite> crossingSites =
        resolveCrossings(crossings, paths, polys, data.sceneOrigin());
    std::vector<CrossingState> crossingStates(crossings.size());
    for (std::size_t i = 0; i < crossings.size(); ++i) {
        if (!crossingSites[i].resolved()) {
            std::fprintf(stderr, "[Crossing] \"%s\" is not on any path - stale overlay?\n",
                         crossings[i].name.c_str());
            continue;
        }
        std::size_t on = 0;
        for (const CrossingSite::On& o : crossingSites[i].tracks)
            if (o.path >= 0) ++on;
        std::printf("[Crossing] %-16s %5.0f km/h -> approach %.0f m, inner +/-%.0f m,"
                    " %zu track(s)\n",
                    crossings[i].name.c_str(), crossingSites[i].lineSpeedKmh,
                    crossingSites[i].outerM, crossingSites[i].innerM, on);
        if (on < crossings[i].tracks.size())
            std::fprintf(stderr, "[Crossing] \"%s\": %zu of its track(s) resolved to no"
                                 " path - stale overlay?\n",
                         crossings[i].name.c_str(), crossings[i].tracks.size() - on);
    }
    // Which signals stand in each crossing's approach circuits, facing it. Worked out once:
    // where a signal stands never changes, only what it is showing.
    //
    // Only the circuit-driven signals - the dwarfs and the two mains. A simple station
    // signal has no circuits behind it and goes dark when its station is unmanned, when
    // trains run past it without reference to it at all; reading one as a barrier would
    // break a crossing's detection every time a station was switched off.
    std::vector<std::vector<CrossingGuard>> crossingGuards(crossings.size());
    for (std::size_t ci = 0; ci < crossings.size(); ++ci) {
        const CrossingSite& site = crossingSites[ci];
        if (!site.resolved()) continue;
        const glm::dvec3 org = data.sceneOrigin();
        for (std::size_t k = 0; k < sigPlacements.size(); ++k) {
            const SignalPlacement& sp = sigPlacements[k];
            // A block signal counts with the mains: one at danger inside a crossing's
            // approach holds a train short of it just as a station's signal does, and the
            // crossing has no business ringing for a train that cannot reach it.
            if (sp.kind != SignalKind::Dwarf && sp.kind != SignalKind::Entry &&
                sp.kind != SignalKind::Exit && sp.kind != SignalKind::Block)
                continue;
            float s = 0.0f;
            const int road = crossingTrackUnder(
                site, paths,
                glm::vec2(static_cast<float>(sp.world.x - org.x),
                          static_cast<float>(sp.world.y - org.y)),
                s);
            if (road < 0) continue; // not standing on any of this crossing's roads
            const float rel = s - site.tracks[road].s;
            if (std::abs(rel) <= site.innerM || std::abs(rel) > site.outerM) continue;
            // Facing the crossing: a train passing it carries on toward the crossing rather
            // than away from it. One facing the other way protects the opposite direction
            // and has nothing to say about anything coming here.
            const glm::vec3 t = paths[site.tracks[road].path].poseAt(s).tangent;
            const double toward = rel < 0.0f ? 1.0 : -1.0;
            if ((sp.forward.x * t.x + sp.forward.y * t.y) * toward <= 0.0) continue;
            crossingGuards[ci].push_back({road, static_cast<int>(k), rel});
        }
        if (!crossingGuards[ci].empty())
            std::printf("[Crossing] %-16s %zu signal(s) can break its approach\n",
                        crossings[ci].name.c_str(), crossingGuards[ci].size());
    }

    // Flag posts: the hand signal the station's TXP hangs out.
    const std::vector<FlagPost> flagPosts = loadFlagPosts(datasetRoot);
    const std::vector<SignalStation> flagPostStation =
        attachStations(flagPosts, stations, polys);
    // What each post is displaying, one entry per post and answerable to nothing else.
    // Not part of StationState: a flag is set and taken down on its own, several posts
    // may show different things at once, and a manned station most of the time has no
    // flag out at all. Runtime only, as a switch position is.
    std::vector<FlagColour> flagShown(flagPosts.size(), FlagColour::None);
    if (!flagPosts.empty())
        std::printf("[FlagPost] %zu post(s)\n", flagPosts.size());

    // Avalanche warning signals. They answer to nothing here: no route runs through one,
    // no aspect logic reads one, and no station owns one. What they show is a property of
    // the mountainside.
    const std::vector<AvalancheSignal> avalanches = loadAvalancheSignals(datasetRoot);
    // One aspect per signal, and every one of them is Clear. Nothing in this program can
    // make it anything else - the detector that would raise a warning does not exist. It
    // is a vector rather than a constant so that when one does, it writes here and the
    // rebuild that already runs draws it: no new buffer, no new mesh path, no new plumbing.
    std::vector<AvalancheAspect> avalancheShown(avalanches.size(), AvalancheAspect::Clear);
    if (!avalanches.empty())
        std::printf("[Avalanche] %zu warning signal(s)\n", avalanches.size());

    // Where the TXP stands to give a train permission to leave. One position showing per
    // station: a person really can only be in one place, which is what separates this
    // from the flags in their fixtures. Held outside StationState on purpose - that is
    // about manning, and this does not depend on it.
    const std::vector<TxpPosition> txpPositions = loadTxpPositions(datasetRoot);
    const std::vector<SignalStation> txpStation =
        attachStations(txpPositions, stations, polys);
    std::unordered_map<std::string, int> txpShowingAt; // station -> position, absent none
    // Where a platform puts them above the rail. Static, so found once rather than on
    // every rebuild - it costs a search over every platform and path.
    const std::vector<float> txpLift = txpStandLift(txpPositions, polys, data, paths);
    if (!txpPositions.empty())
        std::printf("[TxpPosition] %zu position(s)\n", txpPositions.size());
    // Who can pass a train order to whom. Derived from the positions above rather than
    // authored: a station is a TXP station because one was placed at it, and the next
    // station along the running line is its neighbour. Nothing to keep in step.
    TxpGraph txpGraph;
    txpGraph.build(txpPositions, txpStation, stations, polys, paths, data.sceneOrigin());
    // Which of them are manned, and the sections they hold between them. Rebuilt as
    // stations open and close rather than authored anywhere.
    TxpNetwork txpNet;

    SignalMesh signals;
    CrossingMesh crossingMesh;
    FlagMesh flagMesh;
    AvalancheMesh avalancheMesh;
    TxpMesh txpMesh;
    // Signals and crossings share one buffer: both change while the sim runs, and the
    // renderer already has an update path for that one. Merged as the struct bucket is.
    std::vector<TrackVertex> signalVerts;
    std::vector<std::uint32_t> signalIdx;
    // What each crossing repeat is standing for: the road the points lead to from where it
    // is, worked out here because this is where the junctions and the switch states are.
    // Re-derived with the mesh, so it follows a thrown switch as well as a phase change.
    // Where each of a crossing's repeat masts stands, worked out once.
    //
    // A repeat is placed in path space, and the walk that decides what it repeats runs in
    // track space, so the mast has to be put back onto a track first. Which track that is,
    // where along it, and which way a train reading the mast is travelling are all fixed by
    // the geometry - only which road the points are set for changes.
    //
    // It used to be redone on every rebuild, and it found its track by walking all 6119 of
    // them and looking each one up *by id*, which walks all 6119 again. Squared, per mast,
    // several times per crossing: 60 ms of it, on every circuit change and on every frame
    // of a barrier sweeping, which is what the driver saw as the picture stopping.
    struct RepeatAnchor {
        bool valid = false;
        std::uint32_t track = 0;
        double frac = 0.0;
        int dir = 1;
    };
    std::vector<std::vector<RepeatAnchor>> repeatAnchors(crossings.size());
    for (std::size_t ci = 0; ci < crossings.size(); ++ci) {
        const CrossingSite& site = crossingSites[ci];
        // Nothing to resolve where the crossing has one track: a repeat there can only ever
        // be about the road it stands on.
        if (crossings[ci].tracks.size() < 2) continue;
        repeatAnchors[ci].assign(2 * site.tracks.size(), RepeatAnchor{});
        for (std::size_t t = 0; t < site.tracks.size(); ++t) {
            if (site.tracks[t].path < 0) continue;
            const TrackPath& p = paths[site.tracks[t].path];
            for (int sideIdx = 0; sideIdx < 2; ++sideIdx) {
                const float side = sideIdx == 0 ? -1.0f : 1.0f;
                const float s = site.tracks[t].s + side * site.distantM;
                if (s < 0.0f || s > p.length()) continue;
                const glm::vec3 at = p.poseAt(s).pos;
                const glm::dvec3 w(at.x + data.sceneOrigin().x,
                                   at.y + data.sceneOrigin().y, 0.0);
                std::uint32_t bestTrack = 0;
                double bestFrac = 0.0, bestD = 6.0;
                for (const TrackPoly& tp : polys) {
                    // On the polyline in hand, never back through its id.
                    if (tp.pts.size() < 2) continue;
                    double frac = 0.0, dist = 0.0;
                    projectOnPolyline(tp.pts, glm::dvec2(w.x, w.y), frac, dist);
                    if (dist < bestD) { bestD = dist; bestTrack = tp.id; bestFrac = frac; }
                }
                if (bestD >= 6.0) continue;
                // Heading toward the crossing: the repeat on the -s side reads a train
                // running in +s, and the walk goes the way that train is going.
                const glm::dvec2 tang = trackTangent(polys, bestTrack, bestFrac, +1);
                const glm::vec3 fwd = p.poseAt(s).tangent;
                RepeatAnchor a;
                a.valid = true;
                a.track = bestTrack;
                a.frac = bestFrac;
                a.dir = (tang.x * fwd.x + tang.y * fwd.y) * side >= 0.0 ? -1 : +1;
                repeatAnchors[ci][2 * t + static_cast<std::size_t>(sideIdx)] = a;
            }
        }
    }

    // What each repeat is currently showing. All that is left to do per rebuild is follow
    // the points, which is the only part of it that can change.
    auto crossingDistantFor = [&]() {
        CrossingMesh::DistantFor out(crossings.size());
        for (std::size_t ci = 0; ci < crossings.size(); ++ci) {
            if (repeatAnchors[ci].empty()) continue;
            const CrossingSite& site = crossingSites[ci];
            out[ci].assign(repeatAnchors[ci].size(), -1);
            for (std::size_t slot = 0; slot < repeatAnchors[ci].size(); ++slot) {
                const RepeatAnchor& a = repeatAnchors[ci][slot];
                if (!a.valid) continue;
                out[ci][slot] =
                    crossingTrackAhead(crossings[ci], polys, junctions, switchNet, a.track,
                                       a.frac, a.dir, site.distantM + 200.0);
            }
        }
        return out;
    };

    // The dynamic buffer is six meshes laid end to end, and an event nearly always touches
    // one of them. A barrier sweeping is the case that matters: it moves every frame, and
    // it changes nothing but its own booms - yet remaking the buffer used to redraw all 220
    // signal masts, every speed board and every avalanche head along with them. So each
    // part is rebuilt only when its caller says that part has moved.
    enum MeshPart : unsigned {
        PartSignals = 1u << 0,   // the masts: their aspects moved
        PartCrossings = 1u << 1, // the crossing geometry: a boom is in motion
        PartRepeats = 1u << 2,   // only re-read what the crossings' repeats show
        PartFlags = 1u << 3,
        PartAvalanche = 1u << 4,
        PartMark = 1u << 5, // the uncoupling mark
        PartTxp = 1u << 6,
        PartAll = 0x7fu,
    };
    // Which of the parts costs what.
    double mSignals = 0, mCross = 0, mFlag = 0, mAval = 0, mTxp = 0, mConcat = 0;
    const bool meshProf = std::getenv("EBANER_PROFILE") != nullptr;
    // The uncoupling mark has no mesh class of its own, so it keeps its geometry here.
    std::vector<TrackVertex> markVerts;
    std::vector<std::uint32_t> markIdx;
    // What the crossings' repeats showed last time they were looked at.
    CrossingMesh::DistantFor lastDistantFor;

    // Returns whether anything was actually remade - if not, there is nothing to upload.
    auto rebuildSignalBuffer = [&](unsigned parts) {
        const bool prof = meshProf;
        auto tick = [&] {
            return prof ? std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now().time_since_epoch())
                              .count()
                        : 0.0;
        };
        bool built = false;
        if (parts & PartSignals) {
            const double t = tick();
            signals.build(sigPlacements, data.sceneOrigin());
            mSignals += tick() - t;
            built = true;
        }
        if (parts & (PartCrossings | PartRepeats)) {
            const double t = tick();
            // What a crossing's repeats show follows the points, so an aspect pass has to
            // look. Looking is cheap; redrawing the crossings is not, and the answer almost
            // never moves - so the geometry is only remade when it does, or when a boom is
            // actually swinging.
            CrossingMesh::DistantFor df = crossingDistantFor();
            const bool repeatsMoved = df != lastDistantFor;
            lastDistantFor = std::move(df);
            if ((parts & PartCrossings) || repeatsMoved) {
                crossingMesh.build(crossings, crossingSites, crossingStates, paths,
                                   data.sceneOrigin(), lastDistantFor);
                built = true;
            }
            mCross += tick() - t;
        }
        if (parts & PartFlags) {
            const double t = tick();
            flagMesh.build(flagPosts, flagShown, polys, data.sceneOrigin());
            mFlag += tick() - t;
            built = true;
        }
        if (parts & PartAvalanche) {
            // The avalanche signals go in the dynamic bucket beside the rest, though
            // nothing changes their aspect yet - which is why nothing but the initial build
            // asks for this part. The day something does move them, it must say so here.
            const double t = tick();
            avalancheMesh.build(avalanches, avalancheShown, polys, data.sceneOrigin());
            mAval += tick() - t;
            built = true;
        }
        if (parts & PartMark) {
            // A mark over the coupler U is armed on, so that "the nearest one to the
            // camera" is something the driver can see rather than something he has to
            // trust. Only the armed one is marked: arming is a keypress and not a hover,
            // so this is rebuilt on an event and not every frame - and a train that may be
            // uncoupled is standing still, so the mark stays over the coupler once placed.
            markVerts.clear();
            markIdx.clear();
            if (armedTrain >= 0 && armedCoupler >= 0 &&
                static_cast<std::size_t>(armedTrain) < trains.size()) {
                const Consist& at = trains[static_cast<std::size_t>(armedTrain)];
                if (armedCoupler + 1 < at.unitCount()) {
                    // Midway between the two sets' centres - see the pick in the loop.
                    const VehicleFrame& a = at.unit(armedCoupler).frame();
                    const glm::vec3 c =
                        0.5f * (a.pos + at.unit(armedCoupler + 1).frame().pos);
                    const glm::vec3 R = a.right, F = a.tangent, U = a.up;
                    const glm::vec3 amber(1.0f, 0.72f, 0.18f);
                    // A post standing out of the roof over the coupler, with a plate across
                    // it: tall enough to clear the bodies and be read from the ground
                    // beside the train, which is where the man doing the uncoupling is.
                    lampgeom::box(markVerts, markIdx, c + U * 3.6f, R, F, U, 0.05f, 0.05f,
                                  1.4f, amber);
                    lampgeom::box(markVerts, markIdx, c + U * 5.1f, R, F, U, 0.45f, 0.06f,
                                  0.16f, amber);
                }
            }
            built = true;
        }
        if (parts & PartTxp) {
            const double t = tick();
            // The TXP appears only where the departure signal is actually being given.
            std::vector<char> txpShowing(txpPositions.size(), 0);
            for (std::size_t i = 0; i < txpPositions.size(); ++i) {
                const auto it = txpShowingAt.find(txpStation[i].name);
                if (it != txpShowingAt.end() && it->second == static_cast<int>(i))
                    txpShowing[i] = 1;
            }
            txpMesh.build(txpPositions, txpShowing, polys, data.sceneOrigin(), txpLift);
            mTxp += tick() - t;
            built = true;
        }
        if (!built) return false;

        // Lay them end to end. Each part's indices are relative to its own vertices, so
        // they are offset by however many vertices came before. Sized once up front: the
        // buffer is a hundred thousand vertices and growing it five times over copied most
        // of it five times.
        const double t = tick();
        struct Part {
            const std::vector<TrackVertex>* v;
            const std::vector<std::uint32_t>* i;
        };
        const Part order[] = {{&signals.vertices(), &signals.indices()},
                              {&crossingMesh.vertices(), &crossingMesh.indices()},
                              {&flagMesh.vertices(), &flagMesh.indices()},
                              {&avalancheMesh.vertices(), &avalancheMesh.indices()},
                              {&markVerts, &markIdx},
                              {&txpMesh.vertices(), &txpMesh.indices()}};
        std::size_t nv = 0, ni = 0;
        for (const Part& p : order) { nv += p.v->size(); ni += p.i->size(); }
        signalVerts.clear();
        signalIdx.clear();
        signalVerts.reserve(nv);
        signalIdx.reserve(ni);
        for (const Part& p : order) {
            const std::uint32_t base = static_cast<std::uint32_t>(signalVerts.size());
            signalVerts.insert(signalVerts.end(), p.v->begin(), p.v->end());
            for (const std::uint32_t k : *p.i) signalIdx.push_back(k + base);
        }
        mConcat += tick() - t;
        return true;
    };
    rebuildSignalBuffer(PartAll);
    renderer.attachSignals(signalVerts, signalIdx);
    // The network overlay belongs to the traffic-manager map, and the map starts shut. The
    // flag defaults on for the editor, which draws the network over the world deliberately.
    renderer.showTrackGraph(false);

    // Route setting (traffic manager): a set route holds its switches and shows its signal
    // clear. It drops as soon as a train enters its circuits (the lock lifts then too; the
    // per-switch occupancy lock guards them from there).
    std::vector<char> routeSet(signalPaths.size(), 0);
    int routeArm = -1; // placement armed by a first click, awaiting its destination
    // What the line blocks are holding, and what their signals last decided. The claim on a
    // line outlives the route that made it: the route is given up as the train enters its
    // last circuit, and the train is then out on the line with nothing else on record
    // saying which way it went.
    std::vector<LineBlockState> lineState(lineBlocks.lines.size());
    BlockOutcome blockOut;

    // --- Main-signal routes ---------------------------------------------------------------
    // Everything a main signal can be asked to authorise, exit or entry alike, resolved once
    // into one list so the interlocking never has to ask which kind it is holding.
    //
    // An exit signal's authority begins back at the platform, so its movement is the exit
    // route joined to the signal's own route beyond. An entry signal's begins at the mast,
    // so its record is already the whole movement - and every one of its circuits is past
    // the signal.
    const std::vector<SignalPath> exitRoutes = loadExitRoutes(datasetRoot);
    // The roads leading up to an entry mast, where any are authored. Absent - which is the
    // usual case and the whole line as it stands - an entry signal's authority begins at
    // the mast, exactly as it always has.
    const std::vector<SignalPath> entryApproaches = loadEntryApproaches(datasetRoot);
    struct MainCandidate {
        std::string name;
        RouteType type = RouteType::C1;
        int placement = -1;      // the mast to light
        int station = -1;
        // Into the station or out of it. The two build loops below are already exactly
        // that split, so this costs nothing to record - and the picker needs it to offer
        // the two apart, which is most of what stops one being mistaken for the other.
        bool entry = false;
        // The station named on the record, if either half named one. Empty = let the
        // geometry decide.
        std::string stationName;
        glm::vec2 anchor{0.0f};  // its in-station end, which is what groups it by station
        SignalPath departure;    // the whole movement
        std::vector<int> beyond; // circuits past the signal
    };
    std::vector<MainCandidate> mainCandidates;
    {
        const glm::dvec3 org = switchNet.sceneOrigin();
        // The mast a route's signal stands at: the placement of that kind whose paths list
        // the signal. `paths` means different things per kind, which is why kind is checked.
        auto mastOf = [&](SignalKind kind, int idx) {
            for (std::size_t k = 0; k < sigPlacements.size(); ++k) {
                if (sigPlacements[k].kind != kind) continue;
                const std::vector<int>& ps = sigPlacements[k].paths;
                if (std::find(ps.begin(), ps.end(), idx) != ps.end())
                    return static_cast<int>(k);
            }
            return -1;
        };
        auto sceneAt = [&](const Border& b) {
            const glm::dvec3 w = fracToWorld(polys, b.trackId, b.frac);
            return glm::vec2(w.x - org.x, w.y - org.y);
        };
        auto named = [](const SignalPath& p, const char* pfx) {
            return p.name.empty() || p.name == "-" ? pfx + std::to_string(p.id) : p.name;
        };
        for (std::size_t ri = 0; ri < exitRoutes.size(); ++ri) {
            int e = -1;
            for (std::size_t k = 0; k < exitSignals.size(); ++k)
                if (exitSignals[k].id == exitRoutes[ri].exitId) e = static_cast<int>(k);
            if (e < 0) {
                std::fprintf(stderr, "[Route] exit route %d names no exit signal (%d)\n",
                             exitRoutes[ri].id, exitRoutes[ri].exitId);
                continue;
            }
            MainCandidate c;
            c.name = named(exitRoutes[ri], "R");
            c.type = exitRoutes[ri].type;
            c.placement = mastOf(SignalKind::Exit, e);
            c.departure = departureRoute(exitRoutes[ri], exitSignals[e]);
            // The signal's own route starts at its border, so its circuits are exactly the
            // ones past the mast - and a circuit cannot straddle a border, so the sets are
            // disjoint rather than merely different.
            c.beyond = pathSections(exitSignals[e], circuits);
            c.entry = false;
            // Either half may name the station; the signal wins only if the route is silent.
            c.stationName = !exitRoutes[ri].station.empty() ? exitRoutes[ri].station
                                                            : exitSignals[e].station;
            if (!exitRoutes[ri].station.empty() && !exitSignals[e].station.empty() &&
                exitRoutes[ri].station != exitSignals[e].station)
                std::fprintf(stderr,
                             "[Route] %s: the route says station \"%s\" and its signal "
                             "says \"%s\"; taking the route's\n", c.name.c_str(),
                             exitRoutes[ri].station.c_str(), exitSignals[e].station.c_str());
            c.anchor = sceneAt(c.departure.start); // the platform end
            mainCandidates.push_back(std::move(c));
        }
        for (std::size_t ei = 0; ei < entrySignals.size(); ++ei) {
            // Which roads lead up to this record's mast, if any are authored. Nearly no
            // mast has one: an entry signal's authority begins where it stands, and only
            // where several roads converge on one mast is there a choice to be made.
            std::vector<int> upTo;
            for (std::size_t ai = 0; ai < entryApproaches.size(); ++ai)
                if (routeTargetSignal(entryApproaches[ai], entrySignals, polys) >= 0 &&
                    entryApproaches[ai].end.trackId == entrySignals[ei].start.trackId &&
                    std::abs(entryApproaches[ai].end.frac - entrySignals[ei].start.frac) <=
                        sameFracTol(polys, entrySignals[ei].start.trackId))
                    upTo.push_back(static_cast<int>(ai));

            if (upTo.empty()) {
                // Untouched, and it must stay so: every entry signal on the line takes this
                // path, and an approach is an addition for the few masts that need one.
                MainCandidate c;
                c.name = named(entrySignals[ei], "E");
                c.type = entrySignals[ei].type;
                c.placement = mastOf(SignalKind::Entry, static_cast<int>(ei));
                c.departure = entrySignals[ei];
                c.beyond = pathSections(c.departure, circuits); // all of it is past the mast
                c.entry = true;
                c.stationName = entrySignals[ei].station;
                c.anchor = sceneAt(c.departure.end);            // the platform end
                mainCandidates.push_back(std::move(c));
                continue;
            }
            for (const int ai : upTo) {
                MainCandidate c;
                // Both halves in the name: which road the train comes in on, and which
                // road it is being let in to. Neither alone says what the operator picked.
                c.name = named(entryApproaches[ai], "A") + " > " + named(entrySignals[ei], "E");
                c.type = entrySignals[ei].type; // the destination decides, not the approach
                c.placement = mastOf(SignalKind::Entry, static_cast<int>(ei));
                c.departure = departureRoute(entryApproaches[ai], entrySignals[ei]);
                // Still only what is past the mast: the approach is the run-up, and a train
                // standing on it is the train being let in rather than one in the way.
                c.beyond = pathSections(entrySignals[ei], circuits);
                c.entry = true;
                c.stationName = !entryApproaches[ai].station.empty()
                                    ? entryApproaches[ai].station
                                    : entrySignals[ei].station;
                c.anchor = sceneAt(c.departure.end); // the platform end
                mainCandidates.push_back(std::move(c));
            }
        }
    }
    // Stations: routes whose in-station ends lie within kStationSpan of one another work one
    // place, so the traffic manager can offer just that place's routes. Anchoring on the
    // platform end rather than on the mast matters - an entry signal can stand a kilometre
    // outside the station it serves, and would otherwise cluster as a station of its own.
    std::vector<glm::vec2> stationAt; // station -> scene-relative centre
    {
        // How far apart two routes' platform ends may lie and still be one place.
        //
        // This is measured against the length of a station, not the gap between them,
        // and it was too small for the former: the widest cluster on the line already
        // reaches 1073 m, and only holds together because a four-track station has
        // routes ending at intermediate points that chain it. A passing loop has no
        // such middle. At Eiterstraum the inbound routes end at opposite ends of an
        // 1081 m loop - 855 m apart - so the station came out as two clusters, and
        // since the picker offers the one nearest the station being worked, the two
        // routes in the other half could never be offered at all.
        //
        // There is a lot of room to be generous: at 900 m and above the line settles
        // at eight stations and the nearest two are 10.7 km apart, so the threshold
        // can sit an order of magnitude below the gap it must not bridge.
        constexpr double kStationSpan = 1500.0; // m
        // A route may name its station outright, and that overrules the geometry. Those
        // are held back from the clustering below and placed afterwards: a branch running
        // two kilometres out to an industrial siding otherwise clusters as a place of its
        // own, and since the picker offers the cluster *nearest the station being worked*,
        // a cluster with no station near it can never be offered - the route is built, has
        // a mast, and is unreachable from the panel.
        const glm::dvec3 orgS = data.sceneOrigin();
        std::vector<int> named(mainCandidates.size(), -1); // -> index into `stations`
        for (std::size_t a = 0; a < mainCandidates.size(); ++a) {
            if (mainCandidates[a].stationName.empty()) continue;
            for (std::size_t i = 0; i < stations.size(); ++i)
                if (stations[i].name == mainCandidates[a].stationName)
                    named[a] = static_cast<int>(i);
            if (named[a] < 0)
                std::fprintf(stderr,
                             "[Route] %s names station \"%s\", which the export does not "
                             "have; falling back to the geometry\n",
                             mainCandidates[a].name.c_str(),
                             mainCandidates[a].stationName.c_str());
            else
                mainCandidates[a].station = -2; // held back
        }
        for (std::size_t a = 0; a < mainCandidates.size(); ++a) {
            if (mainCandidates[a].station != -1) continue;
            const int st = static_cast<int>(stationAt.size());
            std::vector<std::size_t> queue{a};
            mainCandidates[a].station = st;
            glm::vec2 sum(0.0f);
            int n = 0;
            while (!queue.empty()) { // single link: pull in everything within reach
                const std::size_t c = queue.back();
                queue.pop_back();
                sum += mainCandidates[c].anchor;
                ++n;
                for (std::size_t o = 0; o < mainCandidates.size(); ++o) {
                    if (mainCandidates[o].station != -1) continue;
                    if (glm::length(mainCandidates[o].anchor - mainCandidates[c].anchor) >
                        kStationSpan)
                        continue;
                    mainCandidates[o].station = st;
                    queue.push_back(o);
                }
            }
            stationAt.push_back(sum / static_cast<float>(std::max(n, 1)));
        }
        // Now the ones that named a station: each joins the cluster the panel would offer
        // when that station is being worked - the one nearest it - so naming a station and
        // walking to it are the same thing. With nothing to join, the named station becomes
        // a cluster in its own right.
        for (std::size_t a = 0; a < mainCandidates.size(); ++a) {
            if (mainCandidates[a].station != -2) continue;
            const Station& st = stations[static_cast<std::size_t>(named[a])];
            const glm::vec2 at(static_cast<float>(st.world.x - orgS.x),
                               static_cast<float>(st.world.y - orgS.y));
            int best = -1;
            float bestD = std::numeric_limits<float>::max();
            for (std::size_t i = 0; i < stationAt.size(); ++i)
                if (const float d = glm::length(stationAt[i] - at); d < bestD) {
                    bestD = d;
                    best = static_cast<int>(i);
                }
            if (best < 0) {
                best = static_cast<int>(stationAt.size());
                stationAt.push_back(at);
            }
            mainCandidates[a].station = best;
            std::printf("[Route] %s -> station \"%s\" as authored\n",
                        mainCandidates[a].name.c_str(), st.name.c_str());
        }
        // The centres are what the panel measures against, so they have to reflect the
        // final membership rather than the geometric pass alone.
        {
            std::vector<glm::vec2> sum(stationAt.size(), glm::vec2(0.0f));
            std::vector<int> cnt(stationAt.size(), 0);
            for (const MainCandidate& c : mainCandidates)
                if (c.station >= 0) { sum[c.station] += c.anchor; ++cnt[c.station]; }
            for (std::size_t i = 0; i < stationAt.size(); ++i)
                if (cnt[i] > 0) stationAt[i] = sum[i] / static_cast<float>(cnt[i]);
        }
        std::printf("[Route] %zu exit route(s), %zu entry route(s)%s -> %zu main route(s) "
                    "in %zu station(s)\n", exitRoutes.size(), entrySignals.size(),
                    entryApproaches.empty()
                        ? ""
                        : (", " + std::to_string(entryApproaches.size()) +
                           " entry approach(es)").c_str(),
                    mainCandidates.size(), stationAt.size());
        for (const SignalPath& a : entryApproaches)
            if (routeTargetSignal(a, entrySignals, polys) < 0)
                std::fprintf(stderr, "[Route] entry approach %d ends at no entry mast\n",
                             a.id);
        // Every movement the interlocking knows, in full. What a route *is* - the road it
        // covers, what it holds beyond the signal, which mast it lights - is otherwise only
        // visible a line at a time through the picker, so a change meant to leave the
        // existing routes alone could not be shown to have done so.
        if (std::getenv("EBANER_ROUTES")) {
            for (std::size_t ri = 0; ri < mainCandidates.size(); ++ri) {
                const MainCandidate& c = mainCandidates[ri];
                std::printf("[Route] %2zu \"%s\" %s mast=%d station=%d anchor=%.1f,%.1f",
                            ri, c.name.c_str(), c.type == RouteType::C2 ? "C2" : "C1",
                            c.placement, c.station, c.anchor.x, c.anchor.y);
                for (const SectionInterval& iv : c.departure.parts)
                    std::printf(" %x:%.6f:%.6f", iv.trackId, iv.from, iv.to);
                std::printf(" beyond");
                for (const int id : c.beyond) std::printf(" %d", id);
                std::printf("\n");
            }
            for (std::size_t li = 0; li < lineBlocks.lines.size(); ++li) {
                const LineBlock& L = lineBlocks.lines[li];
                std::printf("[Block] line %2zu \"%s\"%s sections", li, L.name.c_str(),
                            L.sound ? "" : " UNSOUND");
                for (const int id : L.sections) std::printf(" %d", id);
                std::printf(" ends");
                for (const Border& b : L.ends) std::printf(" %x:%g", b.trackId, b.frac);
                for (const int i : L.signals)
                    std::printf(" | \"%s\" %+d road \"%s\"", blockSignals[i].name.c_str(),
                                lineBlocks.roads[i].align,
                                signalPaths[lineBlocks.roads[i].road].name.c_str());
                std::printf("\n");
            }
        }
    }
    // A departure the interlocking is holding. Its circuits are locked so no other main
    // route can take them; each is released as a train enters it, so the route unwinds
    // behind the train.
    //
    // Only the circuits *beyond* the signal put it back to danger. The approach circuits
    // between the platform and the mast are the dwarfs' business: the train being
    // dispatched runs over them to reach the signal, and dropping it then would take the
    // authority away just as the driver was about to accept it. Past the signal the rule is
    // absolute - any occupancy there, by any train, and the aspect goes.
    struct MainRoute {
        int route = -1;          // index into mainCandidates
        int placement = -1;      // the mast to light
        SignalPath departure;    // exit route + the signal's own route beyond
        std::vector<int> locked; // section ids still held (approach and beyond alike)
        std::vector<int> beyond; // of those, the ones past the signal
        bool signalClear = true; // false once a circuit beyond the signal was entered
    };
    std::vector<MainRoute> mainRoutes;

    struct SecRun { int section; std::vector<glm::vec2> pts; }; // scene-relative xy run
    std::vector<SecRun> secRuns;
    {
        const glm::dvec3 org = switchNet.sceneOrigin();
        for (std::size_t si = 0; si < circuits.sections.size(); ++si) {
            for (const SectionInterval& iv : circuits.sections[si].parts) {
                SecRun run;
                run.section = static_cast<int>(si);
                for (const glm::dvec3& w : sampleSectionRun(polys, iv))
                    run.pts.push_back(glm::vec2(w.x - org.x, w.y - org.y));
                if (run.pts.size() >= 2) secRuns.push_back(std::move(run));
            }
        }
    }
    std::vector<char> secOccupied(circuits.sections.size(), 0);

    // Where every section lies in the coordinates a train runs in, resolved once. This is
    // what makes occupancy a question about the same road rather than about how near two
    // things look - see Occupancy.h.
    const SectionSpans sectionSpans = resolveSectionSpans(circuits, paths);
    for (const std::string& n : sectionSpans.notes) std::printf("[Circuit] %s\n", n.c_str());
    std::printf("[Circuit] %zu section(s), %zu placed on the roads%s\n",
                circuits.sections.size(), sectionSpans.entries.size(),
                sectionSpans.notes.empty() ? "" : " (see above)");

    // Resolve each motor switch's locking set now that circuits + polys exist (from the
    // authored overlay, else the circuits the switch sits within). Gates remote throws.
    applySwitchLocks(switchNet, loadSwitchTypes(datasetRoot), circuits, polys);

    // The dataset's own script, if it has one. Run here, after every authored overlay has
    // been read and resolved: nothing below this point is world data, so when the script
    // is eventually given something to look at, all of it already exists. The state is
    // kept open for the life of the program - what a script defines has to stay defined.
    Script script;
    script.run(datasetRoot);

    // Which sections hold a train: their road against the trains'.
    //
    // Every train, not just the one being driven. A portion left standing in a section holds
    // it exactly as surely, and everything the interlocking does - releasing a route,
    // clearing a signal, refusing to move a switch - is decided from this occupancy and not
    // from the vehicle, so this is what makes all of it true of a detached portion as well.
    //
    // A train that ran out of rail under part of itself is worth a word, since it means the
    // body reaches past a buffer stop and the circuits cannot see all of it.
    std::vector<PathSpan> trainSpans, oneTrain;
    bool warnedOffTrack = false; // said once, not sixty times a second
    auto updateOccupancy = [&](std::vector<char>& occ) {
        trainSpans.clear();
        bool offTrack = false;
        for (const Consist& t : trains) {
            if (!t.occupiedSpans(oneTrain)) offTrack = true;
            trainSpans.insert(trainSpans.end(), oneTrain.begin(), oneTrain.end());
        }
        if (offTrack && !warnedOffTrack)
            std::printf("[Circuit] part of a train is past the end of the track - the "
                        "circuits cannot see all of it\n");
        warnedOffTrack = offTrack;
        Vehicle::coalesceSpans(trainSpans);
        computeOccupancy(sectionSpans, trainSpans, occ);
    };

    // Traffic-manager 2-D map overlay: the track network (coloured by type) plus a
    // diamond at each working switch, coloured by its current position.
    // The traffic-manager ortho projection (scene-relative -> clip), reused for the
    // rendered map, the HUD labels and click-picking. North up, ~4 km tall at zoom 1,
    // centred on the throat + WASD pan; Y flipped for Vulkan, z mapped into [0,1].
    auto mapOrtho = [&](float aspect) {
        const float halfH = 2000.0f / g_mapZoom, halfW = halfH * aspect;
        const float zn = -2000.0f, zf = 3000.0f;
        glm::mat4 proj(0.0f);
        proj[0][0] = 1.0f / halfW;
        proj[1][1] = -1.0f / halfH;
        proj[2][2] = 1.0f / (zf - zn);
        const glm::vec2 center = mapCenter + g_mapPan;
        proj[3][0] = -center.x / halfW;
        proj[3][1] = center.y / halfH;
        proj[3][2] = -zn / (zf - zn);
        proj[3][3] = 1.0f;
        return proj;
    };

    // Map markers are sized in scene metres but drawn at a roughly constant *screen* size,
    // so they stay clickable whether the view is 8 km or 200 m across.
    auto markerScale = [&]() { return 90.0f / std::max(g_mapZoom, 0.01f); };
    // Where a signal's marker sits: offset to the right of its track so it doesn't sit on
    // the rails. Drawing and picking both use this, so you click exactly what you see.
    auto signalAnchor = [&](const SignalPlacement& sp) {
        const glm::dvec3 o = switchNet.sceneOrigin();
        const glm::vec2 f(float(sp.forward.x), float(sp.forward.y));
        const glm::vec2 r(f.y, -f.x);
        return glm::vec2(float(sp.world.x - o.x), float(sp.world.y - o.y)) +
               r * (markerScale() * 0.42f);
    };
    // What the cursor is over in the map, so it can be highlighted before it is clicked.
    int hoverSignal = -1, hoverDest = -1, hoverTurnout = -1;

    // The map is drawn in two layers. The rail network is the same 200,000 vertices every
    // time and is attached once; everything that changes - the occupancy bands, the switch
    // diamonds, the highlights - is rebuilt here and goes to a buffer meant to be written
    // every frame. Both used to go together, so a train crossing a circuit border copied
    // and re-uploaded the whole 4.8 MB network and waited for the device to go idle.
    bool mapNetworkAttached = false;
    auto buildMapOverlay = [&]() {
        if (!mapNetworkAttached) {
            renderer.attachTrackGraph(graph.lines, {});
            mapNetworkAttached = true;
        }
        std::vector<LineVertex> lines;
        std::vector<LineVertex> points;

        // Track-circuit sections, over the base graph: a 3 m band (two parallel rails)
        // along each section. Each section gets its own hue so the blocks are legible;
        // a section turns red the moment a wheelset is inside it.
        static const glm::vec3 kSecPal[] = {
            {0.40f, 0.70f, 1.00f}, {0.70f, 0.50f, 1.00f}, {0.30f, 0.90f, 0.75f},
            {0.90f, 0.85f, 0.40f}, {0.55f, 0.80f, 0.95f}};
        for (const SecRun& run : secRuns) {
            const bool occ = !secOccupied.empty() && secOccupied[run.section];
            const glm::vec3 col = occ ? glm::vec3(1.0f, 0.2f, 0.2f) // occupied: red
                                      : kSecPal[run.section % 5];   // clear: block hue
            for (std::size_t i = 1; i < run.pts.size(); ++i) {
                const glm::vec2 a = run.pts[i - 1], b = run.pts[i];
                const glm::vec2 d = b - a;
                const float L = glm::length(d);
                const glm::vec2 p = L > 1e-4f ? glm::vec2(-d.y, d.x) / L * 1.5f : glm::vec2(0.0f);
                for (float s : {-1.0f, 1.0f}) {
                    lines.push_back({glm::vec3(a + p * s, 2.0f), col});
                    lines.push_back({glm::vec3(b + p * s, 2.0f), col});
                }
            }
        }

        const glm::dvec3 org = switchNet.sceneOrigin();
        // Main signals: a square at the mast, coloured by aspect. Informational - a main
        // route is set from the R list, not by clicking, so these are not pick targets.
        for (const SignalPlacement& sp : sigPlacements) {
            if (sp.kind == SignalKind::Dwarf) continue; // main and distant signals
            const glm::vec2 f(float(sp.forward.x), float(sp.forward.y));
            const glm::vec2 r(f.y, -f.x); // right of travel: the side it stands on
            const glm::vec2 b(float(sp.world.x - org.x) + r.x * 8.0f,
                              float(sp.world.y - org.y) + r.y * 8.0f);
            const bool distant = sp.kind == SignalKind::Distant;
            const glm::vec3 col =
                distant ? (sp.aspect == SignalAspect::Clear ? glm::vec3(0.2f, 1.0f, 0.3f)
                           : sp.aspect == SignalAspect::ClearReduced
                               ? glm::vec3(0.9f, 0.9f, 0.2f)
                               : glm::vec3(1.0f, 0.7f, 0.1f)) // expect stop: amber
                : sp.aspect == SignalAspect::Dark         ? glm::vec3(0.45f, 0.45f, 0.5f)
                : sp.aspect == SignalAspect::Clear        ? glm::vec3(0.2f, 1.0f, 0.3f)
                : sp.aspect == SignalAspect::ClearReduced ? glm::vec3(0.6f, 1.0f, 0.2f)
                                                          : glm::vec3(1.0f, 0.2f, 0.15f);
            // A distant is drawn smaller: it is a repeater, not a thing you can set.
            const float h = distant ? 4.0f : 6.0f;
            const glm::vec2 c[4] = {b + glm::vec2(-h, -h), b + glm::vec2(h, -h),
                                    b + glm::vec2(h, h), b + glm::vec2(-h, h)};
            for (int i = 0; i < 4; ++i) {
                lines.push_back({glm::vec3(c[i], 3.5f), col});
                lines.push_back({glm::vec3(c[(i + 1) % 4], 3.5f), col});
            }
            // A stem back to the track, so it is obvious which line it belongs to.
            lines.push_back({glm::vec3(b, 3.5f), col});
            lines.push_back({glm::vec3(float(sp.world.x - org.x),
                                       float(sp.world.y - org.y), 3.5f), col});
            points.push_back({glm::vec3(b, 3.5f), col});
        }

        const auto& tos = switchNet.turnouts();
        for (std::size_t i = 0; i < tos.size(); ++i) {
            if (tos[i].mainPath < 0) continue; // inert crossing: no working switch
            const SwitchState st = switchNet.state(static_cast<int>(i));
            const glm::vec3 col = st == SwitchState::Straight   ? glm::vec3(0.2f, 0.9f, 0.3f)
                                  : st == SwitchState::Diverging ? glm::vec3(1.0f, 0.6f, 0.1f)
                                                                 : glm::vec3(1.0f, 0.2f, 0.2f);
            const glm::vec3 c(static_cast<float>(tos[i].world.x - org.x),
                              static_cast<float>(tos[i].world.y - org.y),
                              static_cast<float>(tos[i].world.z - org.z));
            const float r = 18.0f; // m, diamond half-diagonal
            const glm::vec3 n(0, r, 0), s(0, -r, 0), e(r, 0, 0), w(-r, 0, 0);
            auto seg = [&](glm::vec3 a, glm::vec3 b) {
                lines.push_back({c + a, col}); lines.push_back({c + b, col});
            };
            seg(n, e); seg(e, s); seg(s, w); seg(w, n); // diamond outline
            points.push_back({c, col});
            if (static_cast<int>(i) == hoverTurnout) { // "you would throw this"
                const glm::vec3 hc(1.0f, 1.0f, 1.0f);
                const float hr = r * 1.7f;
                lines.push_back({c + glm::vec3(0, hr, 0), hc});
                lines.push_back({c + glm::vec3(hr, 0, 0), hc});
                lines.push_back({c + glm::vec3(hr, 0, 0), hc});
                lines.push_back({c + glm::vec3(0, -hr, 0), hc});
                lines.push_back({c + glm::vec3(0, -hr, 0), hc});
                lines.push_back({c + glm::vec3(-hr, 0, 0), hc});
                lines.push_back({c + glm::vec3(-hr, 0, 0), hc});
                lines.push_back({c + glm::vec3(0, hr, 0), hc});
            }
            // Motor-driven switches get an outer ring so they read as remotely worked,
            // regardless of the state colour.
            if (switchNet.type(static_cast<int>(i)) == SwitchType::Motor) {
                const glm::vec3 ringCol(0.6f, 0.7f, 1.0f);
                const float rr = 30.0f; // m, ring radius (outside the diamond)
                constexpr int kN = 12;
                glm::vec3 prev = c + glm::vec3(rr, 0, 0);
                for (int k = 1; k <= kN; ++k) {
                    const float a = 6.2831853f * k / kN;
                    const glm::vec3 cur = c + glm::vec3(std::cos(a) * rr, std::sin(a) * rr, 0);
                    lines.push_back({prev, ringCol});
                    lines.push_back({cur, ringCol});
                    prev = cur;
                }
            }
        }

        // Set routes: a wide white band along the locked path, so the road that is set
        // reads at a glance. White (not green) keeps it clear of the green "straight"
        // switch diamonds; the band's width and continuity distinguish it from the
        // narrower circuit ribbons underneath.
        for (std::size_t pi = 0; pi < signalPaths.size(); ++pi) {
            if (!routeSet[pi]) continue;
            const glm::vec3 col(1.0f, 1.0f, 1.0f);
            for (const SectionInterval& iv : signalPaths[pi].parts) {
                glm::dvec3 prev = fracToWorld(polys, iv.trackId, iv.from);
                if (prev.x == 0.0 && prev.y == 0.0) continue;
                for (int k = 1; k <= 24; ++k) {
                    const double f = iv.from + (iv.to - iv.from) * k / 24.0;
                    const glm::dvec3 cur = fracToWorld(polys, iv.trackId, f);
                    const glm::vec2 a(float(prev.x - org.x), float(prev.y - org.y));
                    const glm::vec2 b(float(cur.x - org.x), float(cur.y - org.y));
                    const glm::vec2 d = b - a;
                    const float L = glm::length(d);
                    const glm::vec2 p = L > 1e-4f ? glm::vec2(-d.y, d.x) / L * 4.0f
                                                  : glm::vec2(0.0f);
                    for (float s : {-1.0f, 1.0f}) {
                        lines.push_back({glm::vec3(a + p * s, 3.0f), col});
                        lines.push_back({glm::vec3(b + p * s, 3.0f), col});
                    }
                    prev = cur;
                }
            }
        }

        // A ring of `rad` around a scene-relative point (used for hover + destinations).
        const float ms = markerScale();
        auto ring = [&](glm::vec2 c, float rad, const glm::vec3& col) {
            constexpr int kN = 14;
            glm::vec3 prev(c + glm::vec2(rad, 0.0f), 3.5f);
            for (int j = 1; j <= kN; ++j) {
                const float a = 6.2831853f * j / kN;
                const glm::vec3 cur(c + glm::vec2(std::cos(a), std::sin(a)) * rad, 3.5f);
                lines.push_back({prev, col}); lines.push_back({cur, col});
                prev = cur;
            }
        };

        // Signals: a chevron pointing the way the signal faces, coloured by aspect, sitting
        // just off its track. Sized in screen terms so it stays visible (and clickable) at
        // any zoom; the armed one is yellow and its destinations are ringed and labelled.
        for (std::size_t k = 0; k < sigPlacements.size(); ++k) {
            const SignalPlacement& sp = sigPlacements[k];
            if (!isMini(sp)) continue; // exit signal on its own: not a map object yet
            const SignalAspect asp = miniAspect(sp);
            const bool armed = static_cast<int>(k) == routeArm;
            const bool hovered = static_cast<int>(k) == hoverSignal;
            const glm::vec3 col =
                armed ? glm::vec3(1.0f, 1.0f, 0.4f) // armed: yellow, like its destinations
                : asp == SignalAspect::Dark           ? glm::vec3(0.45f, 0.45f, 0.5f)
                      : asp == SignalAspect::Clear        ? glm::vec3(0.3f, 1.0f, 0.4f)
                        : asp == SignalAspect::TrainOnTrack ? glm::vec3(1.0f, 0.75f, 0.15f)
                                                            : glm::vec3(1.0f, 0.25f, 0.2f);
            const glm::vec2 f(float(sp.forward.x), float(sp.forward.y));
            const glm::vec2 r(f.y, -f.x); // right of travel: the side the signal stands on
            const glm::vec2 base = signalAnchor(sp);
            const float L = ms * (armed ? 0.34f : 0.26f), W = ms * (armed ? 0.17f : 0.13f);
            const glm::vec3 tip(base + f * L, 3.5f);
            const glm::vec3 bl(base - f * (L * 0.4f) + r * W, 3.5f);
            const glm::vec3 br(base - f * (L * 0.4f) - r * W, 3.5f);
            lines.push_back({tip, col}); lines.push_back({bl, col});
            lines.push_back({tip, col}); lines.push_back({br, col});
            lines.push_back({bl, col});  lines.push_back({br, col});
            points.push_back({glm::vec3(base, 3.5f), col});
            // A stem back to the track, so it is obvious which line the signal belongs to.
            lines.push_back({glm::vec3(base, 3.5f), col});
            lines.push_back({glm::vec3(float(sp.world.x - org.x),
                                       float(sp.world.y - org.y), 3.5f), col});
            if (hovered) ring(base, ms * 0.34f, glm::vec3(1.0f)); // "you would click this"
            if (!armed) continue;
            for (int pi : miniPaths(sp)) { // ring each destination it can be set to
                const Border& e = signalPaths[pi].end;
                const glm::dvec3 w = fracToWorld(polys, e.trackId, e.frac);
                if (w.x == 0.0 && w.y == 0.0) continue;
                const glm::vec2 c(float(w.x - org.x), float(w.y - org.y));
                const bool dh = pi == hoverDest;
                const glm::vec3 rc = dh ? glm::vec3(1.0f) : glm::vec3(1.0f, 1.0f, 0.4f);
                ring(c, ms * 0.30f, rc);
                if (dh) ring(c, ms * 0.40f, rc);
                points.push_back({glm::vec3(c, 3.5f), rc});
            }
        }
        renderer.setMapOverlay(lines, points);
    };

    // The stations the traffic manager can work, in the order the line runs.
    //
    // A dispatcher works one station at a time, so the panel needs to be told which one
    // rather than inferring it from wherever the map happens to have been panned. The set
    // is the start station's own line: the signals, flags and TXP positions are all
    // loaded whole and the rail network is resident, so any station on the line can be
    // worked, whether or not anything is authored at it yet - offering only the ones that
    // already own something would leave nowhere to switch *to*, which is the whole point.
    // The dataset carries the line name, so this is a filter and not a table of ours.
    struct TmStation {
        std::string name;
        glm::vec2 at{0.0f}; // scene-relative, for centring the map and finding the nearest
    };
    std::vector<TmStation> tmStations;
    {
        const glm::dvec3 org = data.sceneOrigin();
        for (const Station& st : stations) {
            if (st.line != start->line) continue;
            tmStations.push_back({st.name,
                                  glm::vec2(static_cast<float>(st.world.x - org.x),
                                            static_cast<float>(st.world.y - org.y))});
        }
        // Ordered by how far along they are from where the run started, which on a line
        // is the order they come in: stepping through them walks the railway rather than
        // hopping about it alphabetically.
        const glm::vec2 from(static_cast<float>(start->world.x - org.x),
                             static_cast<float>(start->world.y - org.y));
        std::sort(tmStations.begin(), tmStations.end(),
                  [&](const TmStation& a, const TmStation& b) {
                      return glm::length(a.at - from) < glm::length(b.at - from);
                  });
    }
    // Which one the panel is working. Set to the nearest when the map is opened, and
    // moved from there by hand - see the N / B keys below.
    int tmStation = 0;
    // Set when a station was asked for outright, so the first open keeps it instead of
    // snapping to whichever is nearest the train. Cleared once that open has happened.
    bool tmPinned = false;
    auto tmNearest = [&](glm::vec2 to) {
        int best = 0;
        float bestD = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < tmStations.size(); ++i) {
            const float d = glm::length(tmStations[i].at - to);
            if (d < bestD) { bestD = d; best = static_cast<int>(i); }
        }
        return best;
    };
    // Where the station being worked is, for the map and for the route picker.
    auto tmStationAt = [&]() {
        if (tmStations.empty()) return mapCenter;
        return tmStations[std::clamp(tmStation, 0,
                                     static_cast<int>(tmStations.size()) - 1)].at;
    };
    // The simple entry signals belong to a station by name, so the picker is that
    // station's list - the one the traffic manager is working.
    auto simpleStationHere = [&]() -> std::string {
        if (tmStations.empty()) return {};
        return tmStations[std::clamp(tmStation, 0,
                                     static_cast<int>(tmStations.size()) - 1)].name;
    };
    // The map-mode HUD: title, colour legend, and controls. When a vehicle is given
    // (the sim runs live under the map) its speed is shown so the motion is visible.
    auto appendMapHud = [&](std::vector<TextVertex>& tv, int fbw, int fbh,
                            const Consist* veh) {
        const float sc = std::max(2.0f, static_cast<float>(fbh) / 240.0f);
        const float x = 40.0f, lh = 12.0f * sc;
        float y = 40.0f;
        // Name the station being worked, not the one the dataset starts at: every panel
        // below acts on it, so which it is has to be on screen.
        {
            std::string title = "TRAFFIC MANAGER";
            const std::string here = simpleStationHere();
            if (!here.empty()) {
                title += " - " + here;
                if (tmStations.size() > 1)
                    title += "  (" + std::to_string(tmStation + 1) + "/" +
                             std::to_string(tmStations.size()) + ")";
            }
            appendText(tv, title, x, y, sc, glm::vec3(1.0f, 0.95f, 0.5f), fbw, fbh);
        }
        y += lh;
        if (veh) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "train %.0f km/h  (sim live)",
                          veh->speed() * 3.6f);
            appendText(tv, buf, x, y, sc * 0.75f, glm::vec3(0.7f, 1.0f, 0.75f), fbw, fbh);
            y += lh;
        }
        appendText(tv, "main amber / siding cyan / yard magenta", x, y,
                   sc * 0.75f, glm::vec3(0.85f, 0.9f, 1.0f), fbw, fbh);
        y += lh;
        appendText(tv, "switch: straight green / diverging orange / broken red / motor ringed",
                   x, y, sc * 0.75f, glm::vec3(0.85f, 0.9f, 0.85f), fbw, fbh);
        y += lh;
        // Track-circuit legend, and the names of any occupied sections (live).
        if (!secRuns.empty()) {
            std::string occLine = "circuit blocks coloured / occupied red";
            std::string names;
            for (std::size_t si = 0; si < circuits.sections.size(); ++si)
                if (si < secOccupied.size() && secOccupied[si]) {
                    const Section& s = circuits.sections[si];
                    const std::string nm = s.name.empty() || s.name == "-"
                                               ? "S" + std::to_string(s.id)
                                               : s.name;
                    names += (names.empty() ? "" : ", ") + nm;
                }
            if (!names.empty()) occLine += "   OCCUPIED: " + names;
            appendText(tv, occLine, x, y, sc * 0.75f,
                       names.empty() ? glm::vec3(0.6f, 0.8f, 0.95f) : glm::vec3(1.0f, 0.5f, 0.4f),
                       fbw, fbh);
            y += lh;
        }
        char hint[224];
        std::snprintf(hint, sizeof(hint),
                      "O: cab  Esc: menu  scroll/Z-X: zoom  WASD: pan  "
                      "Left/Right or N/B: prev/next station  "
                      "click switch to throw  click signal then destination for a route  "
                      "R: exit routes  E: entry signals  (view %.1f km)",
                      4.0f / g_mapZoom);
        appendText(tv, hint, x, y, sc * 0.75f, glm::vec3(0.7f, 0.85f, 0.7f), fbw, fbh);
        y += lh;
        // Name what the cursor is over, floating beside it, so the click target is obvious.
        {
            const glm::mat4 vp = mapOrtho(static_cast<float>(fbw) / fbh);
            const glm::dvec3 o = switchNet.sceneOrigin();
            auto label = [&](glm::vec2 scene, const std::string& s, const glm::vec3& col) {
                const glm::vec4 clip = vp * glm::vec4(scene.x, scene.y, 0.0f, 1.0f);
                if (clip.w <= 0.0f) return;
                appendText(tv, s, (clip.x / clip.w * 0.5f + 0.5f) * fbw + 14.0f,
                           (clip.y / clip.w * 0.5f + 0.5f) * fbh - 6.0f, sc * 0.7f, col,
                           fbw, fbh);
            };
            if (hoverSignal >= 0 && hoverSignal < static_cast<int>(sigPlacements.size())) {
                const SignalPlacement& sp = sigPlacements[hoverSignal];
                int setHere = -1;
                for (int pi : miniPaths(sp)) if (routeSet[pi]) setHere = pi;
                label(signalAnchor(sp),
                      setHere >= 0 ? "cancel route" : "signal - click to select route",
                      glm::vec3(1.0f));
            } else if (hoverDest >= 0 && hoverDest < static_cast<int>(signalPaths.size())) {
                const Border& e = signalPaths[hoverDest].end;
                const glm::dvec3 w = fracToWorld(polys, e.trackId, e.frac);
                label(glm::vec2(float(w.x - o.x), float(w.y - o.y)),
                      "set route " + signalPaths[hoverDest].name, glm::vec3(1.0f));
            } else if (hoverTurnout >= 0) {
                const Turnout& t = switchNet.turnouts()[hoverTurnout];
                label(glm::vec2(float(t.world.x - o.x), float(t.world.y - o.y)),
                      "switch - click to throw", glm::vec3(1.0f));
            }
            // Where the route is going, once a signal is armed.
            if (routeArm >= 0 && routeArm < static_cast<int>(sigPlacements.size()))
                for (int pi : miniPaths(sigPlacements[routeArm])) {
                    if (pi == hoverDest) continue; // already labelled above
                    const Border& e = signalPaths[pi].end;
                    const glm::dvec3 w = fracToWorld(polys, e.trackId, e.frac);
                    label(glm::vec2(float(w.x - o.x), float(w.y - o.y)),
                          signalPaths[pi].name, glm::vec3(1.0f, 1.0f, 0.5f));
                }
        }
        // Transient feedback from a switch click (thrown / blocked reason).
        if (glfwGetTime() < g_mapMsgUntil && !g_mapMsg.empty()) {
            // Green for something done, orange for something refused. Carried by the
            // setter rather than sniffed from the text: matching a prefix meant every new
            // message was an error by default, so walking to a station announced itself in
            // the same colour as a blocked route and looked like it had failed.
            appendText(tv, g_mapMsg, x, y, sc,
                       g_mapMsgOk ? glm::vec3(0.5f, 1.0f, 0.6f)
                                  : glm::vec3(1.0f, 0.55f, 0.4f),
                       fbw, fbh);
        }
    };

    enum class Mode { Menu, Sim };
    Mode mode = Mode::Menu;
    int menuIndex = 0;
    if (const char* vsel = std::getenv("EBANER_VEHICLE")) {
        menuIndex = std::clamp(std::atoi(vsel), 0, kNumVehicleSpecs - 1);
        spawnVehicle(menuIndex);
        mode = Mode::Sim;
    }
    // Open the traffic manager straight away, and optionally at a named station. The map
    // is otherwise only reachable by pressing keys, which left the one part of the sim
    // that is all panels and no 3-D view impossible to screenshot or check headlessly.
    // Opens the station panel with the map, so the one part of the dispatcher's UI that
    // is otherwise reachable only by keypress can be screenshotted and checked.

    if (const char* msel = std::getenv("EBANER_MAP")) {
        g_mapMode = true;
        g_mapDirty = true;
        mode = Mode::Sim;
        for (std::size_t i = 0; i < tmStations.size(); ++i)
            if (tmStations[i].name == msel) {
                tmStation = static_cast<int>(i);
                tmPinned = true;
                g_mapPan = glm::clamp(tmStations[i].at, mapMin, mapMax) - mapCenter;
            }
    }
    bool prevUp = false, prevDown = false, prevEnter = false;
    bool prevNum[9] = {}; // one per vehicle-select number key
    bool prevBrkDown = false, prevBrkUp = false, prevBrkEmerg = false;
    bool prevBrkRel = false, prevBrkApp = false; // the separate train brake, K / L
    bool prevIndRel = false, prevIndApp = false; // the independent loco brake, H / J
    bool prevSafety = false, prevEngine = false;
    bool prevRevF = false, prevRevN = false, prevRevR = false;
    bool prevUncouple = false;
    int markedTrain = -1, markedCoupler = -1; // what the coupler mark is drawn on
    bool prevMenuEnter = false, prevMenuUp = false, prevMenuDown = false;
    bool prevMenuPgUp = false, prevMenuPgDn = false;
    bool mapAttached = false; // whether the map overlay is currently attached
    bool switchesChanged = true; // a switch moved: re-evaluate the signal aspects
    bool simpleSignalsChanged = true; // a simple entry signal was set (or first frame)
    // A flag moves no aspect, so it needs its own trigger: the rebuild below is gated on
    // an aspect having changed, and a flag would otherwise be set and never drawn.
    bool flagsChanged = false;
    bool prevMapClick = false; // edge-trigger for the map left-click
    bool prevPickUp = false, prevPickDown = false, prevPickEnter = false;
    const std::vector<std::string> kMenuItems = {"Drive another train",
                                                "Place a train here",
                                                "Place a train at a station",
                                                "Traffic manager", "Exit"};
    std::vector<std::string> menuList; // what the open sub-list is showing
    std::string menuNote;              // outcome of the last thing done, shown in the title
    // Where a train being placed is to go. Settled when the place step is entered, used
    // when the vehicle is finally chosen.
    glm::vec2 placeTarget(0.0f);

    auto setMapMsg = [&](const std::string& m, bool ok = false) {
        g_mapMsg = m; g_mapMsgOk = ok; g_mapMsgUntil = glfwGetTime() + 3.0;
    };

    // --- More than one train ------------------------------------------------------------
    //
    // Every train in `trains` is already stepped every frame and already holds its own
    // circuits, so a second train needs no simulation work at all. What it needs is the two
    // ways in: a way to say which one you are driving, and a way to get one onto the line
    // somewhere other than where the session started. Both are in the Escape menu.

    // Take the controls of train `idx`, sitting in cab `cab`.
    //
    // One helper and not four assignments at each site, because `vehicle` is a raw pointer
    // into `trains` and every input, HUD and camera block downstream reads through it
    // alongside `g_driverPos`. Setting some of them and not the others reads one train's
    // handle into another train's cab, and nothing says so.
    auto driveTrain = [&](int idx, int cab) {
        if (idx < 0 || idx >= static_cast<int>(trains.size())) return;
        driverTrain = idx;
        vehicle = &trains[static_cast<std::size_t>(idx)];
        g_cabCount = std::max(1, drivercam::count(*vehicle));
        g_driverPos = std::clamp(cab, 0, g_cabCount - 1);
        g_chase = false;
        g_driverYaw = g_driverPitch = 0.0f;
    };

    // The cab to land in when taking a train over: the one that is in gear if any is, since
    // that is the end it is being driven from and the end its controls are set at. Cab 0
    // otherwise, which is the front as it was laid out.
    // Two trains become one: `gone`'s sets go into `keep` and `gone` leaves the world.
    //
    // This is the one operation that takes a train *out* of `trains`, which the comment on
    // that container says never happens - `vehicle` is a raw pointer into it and
    // driverTrain, armedTrain and markedTrain are raw indices, so every one of them is
    // wrong the moment an element before it goes. Hence one place that does it, in one
    // order, and nothing anywhere else touching the container.
    //
    // Returns the set index of the new joint, for a caller that wants to do something to
    // the couplers there.
    auto coupleTrains = [&](int keep, int gone, bool tail, bool opposed) {
        const int keepSets = trains[static_cast<std::size_t>(keep)].unitCount();
        const int goneSets = trains[static_cast<std::size_t>(gone)].unitCount();

        // Where the driver ends up, worked out before anything moves. He is somewhere in
        // one of these two trains, addressed by a cab index that counts two to a set along
        // the whole train - so it shifts if sets are inserted ahead of him, and his set is
        // renumbered from the other end if his train is the one turned round.
        int newDriver = driverTrain, newCab = g_driverPos;
        const int oldSet = std::max(0, g_driverPos) / 2, oldLocal = std::max(0, g_driverPos) % 2;
        if (driverTrain == gone) {
            newDriver = keep;
            const int within = opposed ? goneSets - 1 - oldSet : oldSet;
            newCab = 2 * (within + (tail ? keepSets : 0)) + oldLocal;
        } else if (driverTrain == keep && !tail) {
            newCab = 2 * (oldSet + goneSets) + oldLocal;
        }

        trains[static_cast<std::size_t>(keep)].absorb(
            std::move(trains[static_cast<std::size_t>(gone)]), tail, opposed);
        trains.erase(trains.begin() + gone);

        // Everything that was an index into the container, now that one is missing.
        auto shift = [&](int& idx) {
            if (idx == gone) idx = -1;
            else if (idx > gone) --idx;
        };
        shift(armedTrain);
        if (armedTrain < 0) armedCoupler = -1;
        shift(markedTrain);
        if (markedTrain < 0) markedCoupler = -1;
        if (newDriver > gone) --newDriver;

        const bool wasOutside = g_driverPos < 0;
        const bool wasChasing = g_chase;
        driveTrain(newDriver, std::max(0, newCab));
        if (wasOutside) g_driverPos = -1; // he was stood beside it, not sitting in it
        g_chase = wasChasing;

        // The composition changed, so the index buffer has to be rebuilt - a vertex
        // refresh under the old indices draws the trains as a heap and says nothing.
        vmesh.build(trains);
        renderer.attachVehicle(vmesh.vertices(), vmesh.indices(), vmesh.glassFirstIndex());
        return tail ? keepSets : goneSets; // the first set on the far side of the joint
    };

    auto cabToSitIn = [](const Consist& t) {
        const int a = t.activeCab();
        return a >= 0 ? a : 0;
    };

    // Where a train is, in the words the signalling uses: the track circuit it stands on if
    // it stands on one, else the nearest station and how far off it is. This is what makes
    // the picker worth having - a list that says "2 sets, 0 km/h" of every train tells you
    // nothing about which is the one behind the block signal.
    auto trainWhere = [&](const Consist& t) {
        std::vector<PathSpan> spans;
        std::string where;
        if (t.occupiedSpans(spans)) {
            std::vector<char> occ;
            computeOccupancy(sectionSpans, spans, occ);
            for (std::size_t i = 0; i < occ.size() && i < circuits.sections.size(); ++i) {
                if (!occ[i]) continue;
                if (!where.empty()) where += "+";
                where += circuits.sections[i].name;
            }
        }
        if (!where.empty()) return where;
        const glm::vec3 p = t.pose().pos;
        const glm::dvec3 w(p.x + data.sceneOrigin().x, p.y + data.sceneOrigin().y, 0.0);
        if (const Station* st = nearestStation(stations, w)) {
            const double dx = st->world.x - w.x, dy = st->world.y - w.y;
            char buf[160];
            std::snprintf(buf, sizeof buf, "%s %+.1f km", st->name.c_str(),
                          std::sqrt(dx * dx + dy * dy) / 1000.0);
            return std::string(buf);
        }
        return std::string("on the line");
    };

    // Which line of the station list to open on. The list is 720 long and runs the length
    // of the country; the one meant is nearly always the nearest.
    auto stationNearCamera = [&]() {
        const glm::vec3 c = g_camera.position();
        const glm::dvec3 w(c.x + data.sceneOrigin().x, c.y + data.sceneOrigin().y, 0.0);
        const Station* st = nearestStation(stations, w);
        return st ? static_cast<int>(st - stations.data()) : 0;
    };

    // The two lists that cannot change, built once. Stations especially: 720 of them, and
    // the menu is redrawn every frame it is open.
    const std::vector<std::string> stationLabels = [&] {
        std::vector<std::string> out;
        out.reserve(stations.size());
        for (const Station& st : stations)
            out.push_back(st.name + (st.isStop() ? "  (stop)" : "") + "  " + st.line);
        return out;
    }();
    // The runtime picker's rows, with a heading wherever the category changes and a map
    // back to the spec each row stands for. A heading maps to -1 and the selection steps
    // over it, so the list reads the same as the start screen's without the caller having
    // to know that some rows are not choices.
    //
    // Labelled by specTitle and not by `name`: a formation keeps its locomotive's name, so
    // this list used to offer two rows both reading "NSB Di 4 (Henschel)".
    std::vector<int> vehicleRow;
    const std::vector<std::string> vehicleLabels = [&vehicleRow] {
        std::vector<std::string> out;
        int lastCat = -1;
        for (int i = 0; i < kNumVehicleSpecs; ++i) {
            if (kVehicleSpecs[i].category != lastCat) {
                lastCat = kVehicleSpecs[i].category;
                out.push_back(std::string("-- ") + categoryName(lastCat) + " --");
                vehicleRow.push_back(-1);
            }
            out.push_back(specTitle(kVehicleSpecs[i]));
            vehicleRow.push_back(i);
        }
        return out;
    }();

    // One line per train for the picker.
    auto trainLabels = [&]() {
        std::vector<std::string> out;
        for (std::size_t i = 0; i < trains.size(); ++i) {
            const Consist& t = trains[i];
            char buf[256];
            std::snprintf(buf, sizeof buf, "%d. %d set%s  %3.0f km/h  %s%s",
                          static_cast<int>(i) + 1, t.unitCount(),
                          t.unitCount() == 1 ? " " : "s", t.speed() * 3.6f,
                          trainWhere(t).c_str(),
                          static_cast<int>(i) == driverTrain ? "   [driving]" : "");
            out.push_back(buf);
        }
        return out;
    };

    // Put a train of `specIdx` on the nearest main line to `targetXY` (scene coordinates).
    // Returns what to tell the driver, either way.
    auto placeTrain = [&](int specIdx, glm::vec2 targetXY) {
        const VehicleSpec& sp = kVehicleSpecs[std::clamp(specIdx, 0, kNumVehicleSpecs - 1)];
        const Placement p = placeTrainNear(paths, targetXY, sp);
        if (p.path == nullptr) return std::string(p.why ? p.why : "nowhere to put it");
        if (!placementClear(trains, p)) return std::string("a train is already standing there");
        // On the end and never in the middle: `vehicle` and `armedTrain` are both indexes
        // into this deque, and a deque moves nothing on push_back.
        trains.emplace_back(&paths, p.path, sp, p.s);
        // Without this the sets never divert at a turnout - and nothing says so until one
        // reaches a switch set against it and runs straight through.
        trains.back().attachNetwork(&paths, &switchNet);
        // The composition changed, so the index buffer has to be rebuilt: a vertex refresh
        // under the old indices draws the trains as a heap and warns nobody.
        vmesh.build(trains);
        renderer.attachVehicle(vmesh.vertices(), vmesh.indices(), vmesh.glassFirstIndex());
        // Placing does not move the driver - you keep whatever you were driving and take
        // the new one over from the picker when you want it. Unless you were driving
        // nothing at all, in which case this is what you came for.
        if (vehicle == nullptr) driveTrain(static_cast<int>(trains.size()) - 1, 0);
        else vehicle = &trains[static_cast<std::size_t>(driverTrain)];
        const std::string where = trainWhere(trains.back());
        std::printf("[Train] placed %s on path %u at %.0f m (%s); %zu train(s) now\n",
                    specTitle(sp), p.path->trackId(), p.s, where.c_str(),
                    trains.size());
        std::fflush(stdout);
        return "placed at " + where;
    };

    // EBANER_TRAINS="Trofors,Svenningdal" puts a train at each named station before the
    // first frame. The signalling scenarios worth testing all need two trains a section
    // apart, and building one of those by hand every time is how they stop being tested.
    // EBANER_VEHICLE chooses what kind; without it, a single Class 93.
    if (const char* names = std::getenv("EBANER_TRAINS")) {
        int spec = menuIndex;
        if (std::getenv("EBANER_VEHICLE") == nullptr)
            for (int i = 0; i < kNumVehicleSpecs; ++i)
                if (kVehicleSpecs[i].body == BodyClass93 && kVehicleSpecs[i].units == 1)
                    spec = i;
        std::istringstream is(names);
        std::string one;
        while (std::getline(is, one, ',')) {
            while (!one.empty() && one.front() == ' ') one.erase(one.begin());
            while (!one.empty() && one.back() == ' ') one.pop_back();
            if (one.empty()) continue;
            const Station* st = findStation(stations, one);
            if (st == nullptr) {
                std::fprintf(stderr, "[Train] EBANER_TRAINS: no station called \"%s\"\n",
                             one.c_str());
                continue;
            }
            const std::string note =
                placeTrain(spec, glm::vec2(st->world.x - data.sceneOrigin().x,
                                           st->world.y - data.sceneOrigin().y));
            std::printf("[Train] %s: %s\n", st->name.c_str(), note.c_str());
        }
        // Placing seats the driver only when nothing was being driven, which is exactly
        // the case here when EBANER_VEHICLE was not also given - so skip the start screen.
        if (vehicle != nullptr) mode = Mode::Sim;
        std::fflush(stdout);
    }

    // EBANER_CAB seats the driver in a cab at start-up, for the same reason as the two
    // below: nothing else in a scripted run ever does. spawnVehicle leaves g_driverPos at
    // -1, and the only two things that seat a driver are a keypress and the train picker -
    // so the inside of a cab could not be screenshotted or checked headlessly at all.
    if (const char* cs = std::getenv("EBANER_CAB")) {
        if (vehicle != nullptr) {
            const int n = std::max(1, drivercam::count(*vehicle));
            g_cabCount = n;
            g_driverPos = std::clamp(std::atoi(cs), 0, n - 1);
            g_chase = false;
            std::printf("[Cab] seated in cab %d of %d\n", g_driverPos, n);
        } else {
            std::puts("[Cab] EBANER_CAB: nothing is being driven");
        }
        std::fflush(stdout);
    }

    // EBANER_MENU opens the Escape menu at a step, for the same reason EBANER_MAP opens the
    // traffic manager: a list that is only reachable by keypress cannot be screenshotted or
    // checked headlessly, and this one is where every train in the world is named.
    //   EBANER_MENU=1        the root menu
    //   EBANER_MENU=train    the train picker
    //   EBANER_MENU=station  the station list, opened where the camera is
    //   EBANER_MENU=vehicle  the what-to-place list
    if (const char* m = std::getenv("EBANER_MENU")) {
        g_menuOpen = true;
        const std::string which(m);
        if (which == "train") g_menuStep = MenuStep::PickTrain;
        else if (which == "station") g_menuStep = MenuStep::PickStation;
        else if (which == "vehicle") g_menuStep = MenuStep::PickVehicle;
        if (g_menuStep == MenuStep::PickTrain)
            g_menuSubSel =
                std::clamp(driverTrain, 0, std::max(0, static_cast<int>(trains.size()) - 1));
        else if (g_menuStep == MenuStep::PickStation)
            g_menuSubSel = stationNearCamera();
    }
    // Whether the line between two TXP stations holds a train. What the neighbours check
    // before agreeing to a station opening between them.
    //
    // Measured from a little way inside each station rather than from the station itself:
    // the section is the line *between* them, and a train standing at a platform is not
    // on it. Without that inset a station could never be opened while anything stood at
    // either of its neighbours.
    auto txpSectionClear = [&](const std::string& a, const std::string& b) {
        if (!vehicle) return true; // nothing running, so nothing in the way
        const TxpStationNode* na = nullptr;
        const TxpStationNode* nb = nullptr;
        for (const TxpStationNode& n : txpGraph.nodes()) {
            if (n.name == a) na = &n;
            if (n.name == b) nb = &n;
        }
        if (!na || !nb || na->path < 0 || na->path != nb->path) return true;
        constexpr float kStationLimitM = 300.0f; // roughly out to the entry signals
        const TrackPath& p = paths[na->path];
        float lo = std::min(na->s, nb->s) + kStationLimitM;
        float hi = std::max(na->s, nb->s) - kStationLimitM;
        if (hi <= lo) return true; // stations closer together than their own limits
        // Any train on the stretch between them, not only the one being driven.
        for (const Consist& t : trains) {
            const std::vector<VehicleFrame> bogies = t.bogieFrames();
            for (float s = lo; s <= hi; s += 25.0f) {
                const glm::vec3 q = p.poseAt(s).pos;
                for (const VehicleFrame& bg : bogies)
                    if (glm::distance(q, bg.pos) < 30.0f) return false;
            }
        }
        return true;
    };
    // Man or unman a station, going through the train-order network. Manning is the one
    // that can be refused: the neighbours have to agree to hand over part of what they
    // hold, and they will not while there is a train in it.
    auto logExchange = [&](const TxpExchange& r) {
        for (const TxpMessage& m : r.exchange) {
            const char* k = m.kind == TxpMsgKind::Connect     ? "CONNECT"
                            : m.kind == TxpMsgKind::Accept    ? "ACCEPT"
                            : m.kind == TxpMsgKind::Reject    ? "REJECT"
                            : m.kind == TxpMsgKind::Request   ? "REQUEST"
                            : m.kind == TxpMsgKind::LineClear ? "LINE CLEAR"
                            : m.kind == TxpMsgKind::OnTrack   ? "ON TRACK"
                            : m.kind == TxpMsgKind::Arrived   ? "ARRIVED"
                                                              : "CANCELLED";
            std::printf("[TXP] %-10s %s -> %s%s%s\n", k, m.from.c_str(), m.to.c_str(),
                        m.reason.empty() ? "" : ": ", m.reason.c_str());
        }
    };
    // The reason a refusal came back, for the message line.
    auto refusal = [&](const TxpExchange& r) {
        for (const TxpMessage& m : r.exchange)
            if (m.kind == TxpMsgKind::Reject && !m.reason.empty()) return m.reason;
        return std::string();
    };
    auto setManned = [&](const std::string& station, bool on) {
        StationState& st = simpleStationState[station];
        if (!on) {
            // Unmanning can be refused too: a station holding a train order cannot walk
            // away from it, because the sections either side are about to become one.
            const TxpExchange c = txpNet.close(txpGraph, station);
            logExchange(c);
            if (!c.accepted) {
                setMapMsg(station + ": cannot close - " + refusal(c));
                return false;
            }
            st.manned = false;
            st.green = -1; // comes back on in a known state, not still showing a green
            setMapMsg(station + ": unmanned, signals off", true);
            return false;
        }
        const TxpExchange r = txpNet.open(txpGraph, station, txpSectionClear);
        logExchange(r);
        if (!r.accepted) {
            setMapMsg(station + ": opening refused - " + refusal(r));
            return false;
        }
        st.manned = true;
        st.green = -1;
        const std::vector<std::string> with = txpNet.linksOf(station);
        std::string note = station + ": manned, signals at danger";
        if (!with.empty()) {
            note += "  (line to ";
            for (std::size_t i = 0; i < with.size(); ++i)
                note += (i ? " and " : "") + with[i];
            note += ")";
        }
        setMapMsg(note, true);
        return true;
    };
    // Scripted starts for the dispatcher's UI, which is otherwise reachable only by
    // keypress and so could not be screenshotted or checked at all.
    //   EBANER_TXP_OPEN=A,B,C   man those stations, through the network as the panel does
    //   EBANER_PANEL=1|dest|type  open the station panel, optionally at a dispatch step
    if (const char* names = std::getenv("EBANER_TXP_OPEN")) {
        std::string all(names), one;
        std::istringstream is(all);
        while (std::getline(is, one, ',')) if (!one.empty()) setManned(one, true);
    }
    if (const char* panel = std::getenv("EBANER_PANEL")) {
        const std::string p(panel);
        // `route` is the other picker the map offers (R), which like the dispatcher's own
        // panel is reachable only by keypress and so could not be looked at headlessly.
        if (p == "route" || p == "entry" || p == "exit") {
            // `route` stops at the kind step; `entry`/`exit` go straight to that list, so
            // either screen can be looked at headlessly.
            g_routeStep = p == "route" ? RoutePickStep::PickKind : RoutePickStep::PickList;
            g_routeEntry = p == "entry";
            g_routeKindSel = p == "exit" ? 1 : 0; // `route` opens on the first line
            g_routePickSel = 0;
        } else {
            g_signalPick = true;
        }
        if (p == "dest") g_dispatchStep = DispatchStep::PickDest;
        else if (p == "type") {
            g_dispatchStep = DispatchStep::PickType;
            const std::vector<std::string> d = txpNet.linksOf(simpleStationHere());
            if (!d.empty()) g_dispatchTo = d.front();
        }
    }
    auto pathName = [&](int pi) {
        return signalPaths[pi].name.empty() || signalPaths[pi].name == "-"
                   ? "P" + std::to_string(signalPaths[pi].id)
                   : signalPaths[pi].name;
    };
    // Names of a path's circuits that currently hold a train ("" when the road is clear).
    auto occupiedIn = [&](int pi) {
        std::string names;
        for (int id : pathSections(signalPaths[pi], circuits))
            for (std::size_t si = 0; si < circuits.sections.size(); ++si)
                if (circuits.sections[si].id == id && si < secOccupied.size() &&
                    secOccupied[si]) {
                    const Section& s = circuits.sections[si];
                    names += (names.empty() ? "" : ", ") +
                             (s.name.empty() || s.name == "-" ? "S" + std::to_string(s.id)
                                                              : s.name);
                }
        return names;
    };
    auto secName = [&](int id) {
        for (const Section& s : circuits.sections)
            if (s.id == id)
                return s.name.empty() || s.name == "-" ? "S" + std::to_string(s.id) : s.name;
        return "S" + std::to_string(id);
    };
    auto secOccupiedById = [&](int id) {
        for (std::size_t si = 0; si < circuits.sections.size(); ++si)
            if (circuits.sections[si].id == id)
                return si < secOccupied.size() && secOccupied[si] != 0;
        return false;
    };
    auto exitRouteName = [&](int ri) { return mainCandidates[ri].name; };
    // Which set route (if any) holds turnout `t`.
    auto routeHolding = [&](int t) {
        for (std::size_t pi = 0; pi < signalPaths.size(); ++pi) {
            if (!routeSet[pi]) continue;
            for (const PathSwitch& ps : pathSwitchRequirements(signalPaths[pi], switchNet, polys))
                if (ps.turnout == t) return static_cast<int>(pi);
        }
        return -1;
    };
    // Which set *main* route (if any) holds turnout `t`, by name. A departure keeps its
    // turnouts for as long as it exists, not only while its first circuit is held.
    auto mainRouteHolding = [&](int t) -> std::string {
        for (const MainRoute& mr : mainRoutes)
            for (const PathSwitch& ps : pathSwitchRequirements(mr.departure, switchNet, polys))
                if (ps.turnout == t) return exitRouteName(mr.route);
        return {};
    };

    // Attempt to throw switch `i` from the map: allowed only for a non-broken motor
    // switch whose locking circuits are all clear. Sets the transient feedback message.
    auto tryMapThrow = [&](int i) {
        auto setMsg = setMapMsg;
        if (const std::string mr = mainRouteHolding(i); !mr.empty()) {
            setMsg("Locked by main route " + mr);
            return;
        }
        if (const int held = routeHolding(i); held >= 0) {
            setMsg("Locked by route " + pathName(held));
            return;
        }
        if (switchNet.type(i) != SwitchType::Motor) {
            setMsg("Manual switch - hand-thrown in the cab");
            return;
        }
        if (switchNet.state(i) == SwitchState::Broken) {
            setMsg("Switch BROKEN - cannot be worked");
            return;
        }
        std::string occNames; // occupied locking circuits, if any
        for (int id : switchNet.lock(i))
            for (std::size_t si = 0; si < circuits.sections.size(); ++si)
                if (circuits.sections[si].id == id && si < secOccupied.size() && secOccupied[si]) {
                    const Section& s = circuits.sections[si];
                    occNames += (occNames.empty() ? "" : ", ") +
                                (s.name.empty() || s.name == "-" ? "S" + std::to_string(s.id)
                                                                 : s.name);
                }
        if (!occNames.empty()) {
            setMsg("BLOCKED: " + occNames + " occupied");
            return;
        }
        switchNet.toggle(i);
        switches.build(switchNet, worldCentre, data.loadedRadius());
        renderer.updateSwitches(switches.vertices(), switches.indices());
        g_mapDirty = true;
        switchesChanged = true;
        setMsg(std::string("Switch thrown -> ") +
                   (switchNet.state(i) == SwitchState::Straight ? "straight" : "diverging"),
               true);
    };

    // Set a route: move its switches into position, lock the path and clear its signal.
    // Everything is validated before anything moves, so a refused route changes nothing.
    auto trySetRoute = [&](int pi) {
        if (pi >= 0 && pi < static_cast<int>(blockRoad.size()) && blockRoad[pi]) {
            setMapMsg("That road is worked by a block signal");
            return;
        }
        if (const std::string occ = occupiedIn(pi); !occ.empty()) {
            setMapMsg("Route " + pathName(pi) + " occupied: " + occ);
            return;
        }
        const std::vector<PathSwitch> reqs =
            pathSwitchRequirements(signalPaths[pi], switchNet, polys);
        std::vector<PathSwitch> toMove;
        for (const PathSwitch& ps : reqs) {
            if (switchNet.state(ps.turnout) == ps.need) continue; // already right
            if (const std::string mr = mainRouteHolding(ps.turnout); !mr.empty()) {
                setMapMsg("Switch held by main route " + mr);
                return;
            }
            if (const int held = routeHolding(ps.turnout); held >= 0) {
                setMapMsg("Switch held by route " + pathName(held));
                return;
            }
            if (switchNet.type(ps.turnout) != SwitchType::Motor) {
                setMapMsg("Route needs a manual switch thrown by hand");
                return;
            }
            if (switchNet.state(ps.turnout) == SwitchState::Broken) {
                setMapMsg("Route blocked: switch BROKEN");
                return;
            }
            for (int id : switchNet.lock(ps.turnout))
                for (std::size_t si = 0; si < circuits.sections.size(); ++si)
                    if (circuits.sections[si].id == id && si < secOccupied.size() &&
                        secOccupied[si]) {
                        const Section& s = circuits.sections[si];
                        setMapMsg("Switch locked: " +
                                  (s.name.empty() || s.name == "-" ? "S" + std::to_string(s.id)
                                                                   : s.name) +
                                  " occupied");
                        return;
                    }
            toMove.push_back(ps);
        }
        for (const PathSwitch& ps : toMove) switchNet.setState(ps.turnout, ps.need);
        if (!toMove.empty()) {
            switches.build(switchNet, worldCentre, data.loadedRadius());
            renderer.updateSwitches(switches.vertices(), switches.indices());
        }
        routeSet[pi] = 1;
        switchesChanged = true;
        g_mapDirty = true;
        setMapMsg("Route " + pathName(pi) + " set" +
                      (toMove.empty() ? ""
                                      : " (" + std::to_string(toMove.size()) +
                                            " switch(es) moved)"),
                  true);
    };
    auto cancelRoute = [&](int pi) {
        if (pi >= 0 && pi < static_cast<int>(blockRoad.size()) && blockRoad[pi]) {
            setMapMsg("That road is worked by a block signal");
            return;
        }
        routeSet[pi] = 0;
        switchesChanged = true;
        g_mapDirty = true;
        setMapMsg("Route " + pathName(pi) + " cancelled", true);
    };

    // --- Setting a main-signal route ------------------------------------------------------
    auto mainRouteFor = [&](int ri) -> MainRoute* {
        for (MainRoute& mr : mainRoutes)
            if (mr.route == ri) return &mr;
        return nullptr;
    };
    // Why exit route `ri` cannot be set right now - `full` empty when it can. `brief` is a
    // word or two for the picker, which has a hard width budget; the full reason goes to the
    // message line when the operator actually tries it.
    struct RouteBlock { std::string brief, full; };
    auto exitRouteBlocked = [&](int ri) -> RouteBlock {
        if (mainCandidates[ri].placement < 0) return {"no signal", "no signal stands there"};
        const SignalPath& dep = mainCandidates[ri].departure;
        // Only the road *beyond* the signal has to be clear. The run-up to it is where the
        // train being cleared is standing, and refusing a departure on that account would
        // refuse very nearly every departure anyone wanted to set. It is the same division
        // the release below already makes: a circuit beyond the signal is the safety one,
        // the approach is not.
        //
        // An entry signal's authority begins at its own mast, so all of its circuits are
        // beyond it and this asks about the whole route - which is right, since a train
        // may not be let in to a platform that is occupied.
        std::string occ;
        for (int id : mainCandidates[ri].beyond)
            if (secOccupiedById(id)) occ += (occ.empty() ? "" : ", ") + secName(id);
        if (!occ.empty()) return {"occupied", "occupied: " + occ};
        // Is the plain line beyond the station spoken for? This is the one test that can
        // see a train the routes have forgotten: a MainRoute is erased as its last circuit
        // is entered, so once a train is out on the line the conflict loop below has
        // nothing left to compare against. Two trains cleared toward each other would not
        // collide - the block signals stop them - but they would come to a stand facing
        // each other with no way out, which is not a state working normally should reach.
        {
            std::string brief;
            const std::string why = lineBlockRefusal(lineBlocks, lineState, dep, signalPaths,
                                                     secOccupiedById, &brief);
            if (!why.empty()) return {brief, why};
        }
        // Where another departure already holds road we want, two things make it a
        // conflict, and sharing by itself is not one of them.
        //
        // A circuit lying beyond *both* signals is: each has authorised a movement onto it,
        // and two trains would be let at the same rails. But a circuit beyond one signal
        // and on the other's run-up is the join between two routes set end to end - an
        // entry route finishes on the platform road that a departure from it begins on, and
        // setting both is how a train is let *through* a station rather than into it, with
        // a green at the entry signal and another at the exit. Which of the two you happen
        // to set first cannot matter, so the test is symmetric in them.
        //
        // Running the other way over rails we share is a conflict wherever it happens: that
        // is two movements facing each other.
        const std::vector<int> ours = pathSections(dep, circuits);
        const std::vector<int>& oursBeyond = mainCandidates[ri].beyond;
        for (const MainRoute& mr : mainRoutes) {
            if (mr.route == ri) continue;
            auto held = [&](int id) {
                return std::find(mr.locked.begin(), mr.locked.end(), id) != mr.locked.end();
            };
            auto beyondBoth = [&](int id) {
                return held(id) &&
                       std::find(oursBeyond.begin(), oursBeyond.end(), id) !=
                           oursBeyond.end() &&
                       std::find(mr.beyond.begin(), mr.beyond.end(), id) != mr.beyond.end();
            };
            if (std::any_of(ours.begin(), ours.end(), beyondBoth))
                return {"conflict", "conflicts with " + exitRouteName(mr.route)};
            if (std::any_of(ours.begin(), ours.end(), held) &&
                routesOppose(dep, mr.departure))
                return {"opposed", "faces " + exitRouteName(mr.route)};
        }
        for (const PathSwitch& ps : pathSwitchRequirements(dep, switchNet, polys)) {
            if (switchNet.state(ps.turnout) == ps.need) continue; // already right
            if (const std::string mr = mainRouteHolding(ps.turnout); !mr.empty())
                return {"switch held", "switch held by " + mr};
            if (const int held = routeHolding(ps.turnout); held >= 0)
                return {"switch held", "switch held by " + pathName(held)};
            if (switchNet.type(ps.turnout) != SwitchType::Motor)
                return {"hand switch", "needs a hand-thrown switch"};
            if (switchNet.state(ps.turnout) == SwitchState::Broken)
                return {"BROKEN", "switch BROKEN"};
            for (int id : switchNet.lock(ps.turnout))
                if (secOccupiedById(id))
                    return {"switch locked", "switch locked: " + secName(id) + " occupied"};
        }
        return {};
    };
    // Set a departure: move its turnouts, open the dwarf paths lying along it, lock every
    // circuit it runs through and clear its signal. Validated in full before anything moves,
    // so a refused route changes nothing.
    auto trySetExitRoute = [&](int ri) {
        if (const RouteBlock why = exitRouteBlocked(ri); !why.full.empty()) {
            setMapMsg("Route " + exitRouteName(ri) + ": " + why.full);
            return;
        }
        const SignalPath& dep = mainCandidates[ri].departure;
        std::vector<PathSwitch> toMove;
        for (const PathSwitch& ps : pathSwitchRequirements(dep, switchNet, polys))
            if (switchNet.state(ps.turnout) != ps.need) toMove.push_back(ps);
        for (const PathSwitch& ps : toMove) switchNet.setState(ps.turnout, ps.need);
        if (!toMove.empty()) {
            switches.build(switchNet, worldCentre, data.loadedRadius());
            renderer.updateSwitches(switches.vertices(), switches.indices());
        }
        // The dwarfs along the way are opened too, and keep their own release logic - a
        // shunting signal standing at danger under a cleared main signal reads as a fault.
        int dwarfs = 0;
        for (std::size_t pi = 0; pi < signalPaths.size(); ++pi) {
            if (routeSet[pi] || !routeContains(dep, signalPaths[pi])) continue;
            routeSet[pi] = 1;
            ++dwarfs;
        }
        MainRoute mr;
        mr.route = ri;
        mr.placement = mainCandidates[ri].placement;
        mr.departure = dep;
        mr.locked = pathSections(dep, circuits);
        mr.beyond = mainCandidates[ri].beyond;
        mainRoutes.push_back(std::move(mr));
        // Whichever line blocks this departure runs onto are now set its way, and stay so
        // until the line is genuinely clear - not until this route ends.
        for (std::size_t li = 0; li < lineBlocks.lines.size(); ++li) {
            if (lineState[li].claimed != 0) continue;
            if (lineBlockDirection(lineBlocks, static_cast<int>(li), dep, signalPaths) == 0)
                continue;
            std::printf("[Block] line %s claimed by %s\n", lineBlocks.lines[li].name.c_str(),
                        exitRouteName(ri).c_str());
        }
        claimLineBlocks(lineBlocks, lineState, dep, signalPaths, exitRouteName(ri));
        switchesChanged = true;
        g_mapDirty = true;
        char buf[96];
        std::snprintf(buf, sizeof(buf), " set (%zu switch(es), %d dwarf(s), %zu circuit(s))",
                      toMove.size(), dwarfs, mainRoutes.back().locked.size());
        setMapMsg("Route " + exitRouteName(ri) + buf);
        std::printf("[Main] %s set: %s\n", exitRouteName(ri).c_str(), buf + 1);
    };
    auto cancelExitRoute = [&](int ri) {
        const MainRoute* mr = mainRouteFor(ri);
        if (!mr) return;
        for (std::size_t pi = 0; pi < signalPaths.size(); ++pi)
            if (routeSet[pi] && routeContains(mr->departure, signalPaths[pi])) routeSet[pi] = 0;
        mainRoutes.erase(mainRoutes.begin() + (mr - mainRoutes.data()));
        // The line block's claim is deliberately not dropped here. Cancelling the route
        // says nothing about whether a train already went; the per-frame step releases the
        // line when it is actually clear, which is the only thing that can know.
        switchesChanged = true;
        g_mapDirty = true;
        setMapMsg("Route " + exitRouteName(ri) + " cancelled");
        std::printf("[Main] %s cancelled\n", exitRouteName(ri).c_str());
    };

    // The exit routes on offer: those of the station nearest what the map is looking at,
    // so a dispatcher panning to a place gets that place's routes.
    // `kind`: -1 every route the station has, 0 the ones leaving it, 1 the ones coming in.
    auto stationRoutes = [&](int kind) {
        std::vector<int> out;
        if (stationAt.empty()) return out;
        // The route clusters are geometric and the panel's stations are named, so these
        // are two different lists; the routes offered are the cluster nearest whichever
        // station is being worked, so both pickers answer about the same place.
        const glm::vec2 at = tmStationAt();
        int best = 0;
        float bestD = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < stationAt.size(); ++i) {
            const float d = glm::length(stationAt[i] - at);
            if (d < bestD) { bestD = d; best = static_cast<int>(i); }
        }
        for (std::size_t ri = 0; ri < mainCandidates.size(); ++ri) {
            if (mainCandidates[ri].station != best) continue;
            if (kind >= 0 && mainCandidates[ri].entry != (kind == 1)) continue;
            out.push_back(static_cast<int>(ri));
        }
        return out;
    };
    // One picker line per route: its name, the authority it grants, and either that it is
    // already set or why it cannot be, so the choice is informed before it is committed.
    auto routePickItems = [&](const std::vector<int>& rs) {
        // The panel is sized from its widest line, so the columns are capped: a long route
        // name must not push it off the screen. The full reason is on the message line.
        constexpr std::size_t kName = 26, kTag = 14;
        auto fit = [](std::string t, std::size_t n) {
            if (t.size() > n) t = t.substr(0, n - 1) + "~";
            t.resize(n, ' ');
            return t;
        };
        std::vector<std::string> items;
        items.reserve(rs.size());
        for (int ri : rs) {
            std::string tag = mainRouteFor(ri) ? "SET - cancel"
                                               : exitRouteBlocked(ri).brief;
            items.push_back(fit(exitRouteName(ri), kName) +
                            (mainCandidates[ri].type == RouteType::C2 ? "  C2  " : "  C1  ") +
                            fit(tag, kTag));
        }
        if (items.empty()) items.push_back("(no exit routes here)");
        return items;
    };
    auto simpleSignalsAt = [&](const std::string& station) {
        std::vector<int> out;
        for (std::size_t i = 0; i < simpleEntries.size(); ++i)
            if (simpleEntryStation[i].name == station) out.push_back(static_cast<int>(i));
        return out;
    };
    auto flagPostsAt = [&](const std::string& station) {
        std::vector<int> out;
        for (std::size_t i = 0; i < flagPosts.size(); ++i)
            if (flagPostStation[i].name == station) out.push_back(static_cast<int>(i));
        return out;
    };
    auto showingHere = [&](const std::string& station) {
        const auto it = txpShowingAt.find(station);
        return it == txpShowingAt.end() ? -1 : it->second;
    };
    auto txpPositionsAt = [&](const std::string& station) {
        std::vector<int> out;
        for (std::size_t i = 0; i < txpPositions.size(); ++i)
            if (txpStation[i].name == station) out.push_back(static_cast<int>(i));
        return out;
    };
    // What this station can do about a train order right now. One entry per line the
    // panel will show, derived here so the drawing and the input sides cannot disagree
    // about which line is which.
    enum class OrderAct { None, OnTrack, Cancel, Arrived };
    struct OrderLine {
        std::string other;   // the station at the far end of the section
        OrderAct act = OrderAct::None;
        std::string text;
    };
    auto orderLinesAt = [&](const std::string& station) {
        std::vector<OrderLine> out;
        for (const TxpSection* s : txpNet.sectionsAt(station)) {
            if (!s->held()) continue;
            const std::string other = s->a == station ? s->b : s->a;
            const std::string what = txpTrainTypeName(s->type);
            const bool mine = s->from == station;
            if (s->state == TxpLineState::Prepared && mine) {
                // The two things the end that asked for the line may do with it.
                out.push_back({other, OrderAct::OnTrack,
                               "Train on track: -> " + other + " (" + what + ")"});
                out.push_back({other, OrderAct::Cancel, "Withdraw order: -> " + other});
            } else if (s->state == TxpLineState::Prepared) {
                out.push_back({other, OrderAct::None,
                               "Expecting " + what + " from " + other});
            } else if (mine) {
                out.push_back({other, OrderAct::None,
                               "On the line -> " + other + " (" + what + ")"});
            } else {
                out.push_back({other, OrderAct::Arrived,
                               "Train arrived: from " + other + " (" + what + ")"});
            }
        }
        return out;
    };
    // Where each line of the station panel sits. Derived once because the panel is drawn
    // in one place and acted on in another, a few hundred lines apart, and an off-by-one
    // between them would quietly act on a different line from the highlighted one.
    struct PanelLayout {
        int dispatchLine = 0; // "Request dispatch", always present so index 0 is fixed
        int orderFirst = 1;   // then whatever this station has booked
        int orderCount = 0;
        int mannedLine = 0;
        int allRedLine = 0;
        int sigFirst = 0;
        int sigCount = 0;  // lines the signals occupy (1 even when there are none)
        int flagFirst = 0; // then one line per flag post, which Enter cycles
        int txpFirst = 0;  // then one line per TXP position, which Enter toggles
        int count = 0;
    };
    auto panelLayout = [](const std::vector<int>& ss, const std::vector<int>& fp,
                          const std::vector<int>& tp, std::size_t orders) {
        PanelLayout L;
        L.orderCount = static_cast<int>(orders);
        L.mannedLine = L.orderFirst + L.orderCount;
        L.allRedLine = L.mannedLine + 1;
        L.sigFirst = L.allRedLine + 1;
        L.sigCount = ss.empty() ? 1 : static_cast<int>(ss.size());
        L.flagFirst = L.sigFirst + L.sigCount;
        L.txpFirst = L.flagFirst + static_cast<int>(fp.size());
        L.count = L.txpFirst + static_cast<int>(tp.size());
        return L;
    };

    // The station's own switch first, then its signals. Off is a station-wide state -
    // unmanned, signals dark, trains through - so it cannot be a per-signal choice. Then
    // all-red, because nothing else clears a green: these hold until told otherwise.
    auto signalPickItems = [&](const std::string& station, const std::vector<int>& ss) {
        constexpr std::size_t kName = 26;
        auto fit = [](std::string t, std::size_t n) {
            if (t.size() > n) t = t.substr(0, n - 1) + "~";
            t.resize(n, ' ');
            return t;
        };
        const auto it = simpleStationState.find(station);
        const bool manned = it != simpleStationState.end() && it->second.manned;
        const int green = manned ? it->second.green : -1;
        // Asking for a line comes first: it is what a dispatcher does most, and keeping
        // it at index 0 means the line the panel is indexed from is never conditional.
        std::vector<std::string> items;
        items.push_back(fit("Request dispatch", kName + 6) + "  >");
        const std::vector<OrderLine> orders = orderLinesAt(station);
        for (const OrderLine& o : orders)
            items.push_back(fit(o.text, kName + 6) +
                            (o.act == OrderAct::None ? "" : "  >"));
        items.push_back(
            fit(manned ? "Switch station off (unmanned)" : "Switch station on (manned)",
                kName + 6) +
            (manned ? "  ON" : "  OFF"));
        items.push_back(fit("All red", kName + 6) + (manned && green < 0 ? "  *" : ""));
        for (int i : ss)
            items.push_back(fit(simpleEntries[i].name, kName + 6) +
                            (!manned ? "  dark" : i == green ? "  GREEN" : "  red"));
        if (ss.empty()) items.push_back("(no simple entry signals here)");

        // One line per flag post, which Enter cycles through none, red and green. Each
        // is its own: nothing here reads or writes another post, or the manned switch.
        const std::vector<int> fp = flagPostsAt(station);
        for (int i : fp) {
            const char* w = flagShown[i] == FlagColour::Red     ? "  RED FLAG"
                            : flagShown[i] == FlagColour::Green ? "  GREEN FLAG"
                                                                : "  no flag";
            items.push_back(fit("Flag: " + flagPosts[i].name, kName + 6) + w);
        }
        // Permission to leave: one line per position, which Enter shows or takes down.
        // Only one at a station can be showing, so the others read as blank.
        const std::vector<int> tp = txpPositionsAt(station);
        const auto tit = txpShowingAt.find(station);
        const int showing = tit == txpShowingAt.end() ? -1 : tit->second;
        for (int i : tp)
            items.push_back(fit("Depart: " + txpPositions[i].name, kName + 6) +
                            (i == showing ? "  SHOWN" : "  -"));

        // The layout the input side indexes by has to be the list actually drawn. Said
        // out loud rather than asserted: the release build is -DNDEBUG, so an assert here
        // would be compiled out and a mismatch would quietly act on a different line from
        // the highlighted one - which is the whole failure this guard exists to catch.
        const int want = panelLayout(ss, fp, tp, orders.size()).count;
        if (static_cast<int>(items.size()) != want) {
            static bool told = false;
            if (!told) {
                std::fprintf(stderr,
                             "[Panel] %zu lines drawn but %d indexed - the panel and its "
                             "layout disagree\n", items.size(), want);
                told = true;
            }
        }
        return items;
    };
    // The destinations this station could send a train to: whoever it works a line with.
    auto dispatchDests = [&](const std::string& station) {
        return txpNet.linksOf(station);
    };
    auto appendSignalPicker = [&](std::vector<TextVertex>& tv, int fbw, int fbh) {
        if (!g_signalPick) return;
        const std::string station = simpleStationHere();
        // The two dispatch steps replace the panel rather than stack on it: appendMenu
        // always centres, so a second panel would sit on top of the first and neither
        // would be readable.
        if (g_dispatchStep == DispatchStep::PickDest) {
            const std::vector<std::string> dests = dispatchDests(station);
            std::vector<std::string> items = dests;
            if (items.empty()) items.push_back("(no line is worked from here)");
            appendMenu(tv, "DISPATCH FROM " + station + " - TO WHERE?  (Esc backs out)",
                       items,
                       std::clamp(g_dispatchSel, 0, static_cast<int>(items.size()) - 1),
                       fbw, fbh);
            return;
        }
        if (g_dispatchStep == DispatchStep::PickType) {
            appendMenu(tv, station + " -> " + g_dispatchTo + " - WHAT TRAIN?  (Esc backs out)",
                       {"Passenger", "Cargo", "Other"}, std::clamp(g_dispatchSel, 0, 2),
                       fbw, fbh);
            return;
        }
        const std::vector<int> ss = simpleSignalsAt(station);
        const std::string title =
            "STATION" + (station.empty() ? "" : " - " + station) +
            "  (Up/Down, Enter, Esc)";
        // Clamp against what is actually drawn: the list is the signals and the flags
        // together now, so the signal count alone would cut the highlight short.
        const std::vector<std::string> items = signalPickItems(station, ss);
        appendMenu(tv, title, items,
                   std::clamp(g_signalPickSel, 0,
                              std::max(0, static_cast<int>(items.size()) - 1)),
                   fbw, fbh);
    };

    // The picker panel, drawn after the map HUD so it sits over the map. Two steps, like
    // the dispatcher's: which kind of movement, then which one. The second panel replaces
    // the first rather than stacking - appendMenu always centres, so two would overlap.
    auto appendRoutePicker = [&](std::vector<TextVertex>& tv, int fbw, int fbh) {
        if (g_routeStep == RoutePickStep::None) return;
        const std::string here = simpleStationHere();
        const std::string at = here.empty() ? std::string() : " - " + here;
        if (g_routeStep == RoutePickStep::PickKind) {
            const std::size_t nIn = stationRoutes(1).size(), nOut = stationRoutes(0).size();
            appendMenu(tv, "SET ROUTE" + at + "  (Up/Down, Enter, Esc)",
                       {"Into the station   (" + std::to_string(nIn) + ")",
                        "Out of the station (" + std::to_string(nOut) + ")"},
                       std::clamp(g_routeKindSel, 0, 1), fbw, fbh);
            return;
        }
        const std::vector<int> rs = stationRoutes(g_routeEntry ? 1 : 0);
        // The kind is in the title, so what is being chosen is on screen at the moment of
        // choosing - which is the half of this that keeps an arrival from being read as a
        // departure. The shorter list is the half that makes it workable.
        appendMenu(tv,
                   std::string(g_routeEntry ? "ROUTES INTO" : "ROUTES OUT OF") +
                       at + "  (Up/Down, Enter, Esc backs out)",
                   routePickItems(rs),
                   rs.empty() ? 0
                              : std::clamp(g_routePickSel, 0,
                                           static_cast<int>(rs.size()) - 1),
                   fbw, fbh);
    };

    // Set routes at startup, named by their place in the picker (1-based, as they read on
    // screen) and separated by commas - in order, so a second one meets the state the first
    // one left. What setting a route does - the switches it moves, the dwarfs it opens, the
    // signal it clears, and what it then refuses - is otherwise reachable only by keypress,
    // so none of it could be looked at headlessly.
    if (const char* pick = std::getenv("EBANER_ROUTE")) {
        // Occupancy is a per-frame reading and no frame has run yet, so take it once here:
        // a route set from this hook has to meet the same state one set by keypress would,
        // or it would be granted roads with a train standing on them.
        updateOccupancy(secOccupied);
        const std::string all(pick);
        std::istringstream is(all);
        std::string one;
        while (std::getline(is, one, ',')) {
            if (one.empty()) continue;
            // A bare number still means what it always did: the nth of everything the
            // station has, in the order EBANER_ROUTES prints. The picker shows those in
            // two lists now, so `e`/`x` index within one of them for anything written
            // against what is on screen.
            const char pfx = one[0] == 'e' || one[0] == 'x' ? one[0] : '\0';
            const int kind = pfx == 'e' ? 1 : pfx == 'x' ? 0 : -1;
            const std::vector<int> rs = stationRoutes(kind);
            const int n = std::atoi(one.c_str() + (pfx ? 1 : 0)) - 1;
            if (n >= 0 && n < static_cast<int>(rs.size())) trySetExitRoute(rs[n]);
            else
                std::fprintf(stderr,
                             "[Main] EBANER_ROUTE=%s: this station offers %zu %sroute(s)\n",
                             one.c_str(), rs.size(),
                             kind < 0 ? "" : kind ? "entry " : "exit ");
        }
    }

    // Open the audio device now that the heavy startup work is done, so the audio
    // thread isn't starved (which causes ALSA under-runs) during loading.
    audio.init();

    // Where the frame goes, when asked. There was no timing in here at all, so "there is a
    // lag" could not be turned into a number - and the two things that caused it were both
    // several times the frame budget while looking, in the source, like ordinary work.
    struct Stat {
        double worst = 0.0, total = 0.0;
        long n = 0;
        void add(double ms) { worst = std::max(worst, ms); total += ms; ++n; }
        void reset() { worst = total = 0.0; n = 0; }
    };
    const bool profile = std::getenv("EBANER_PROFILE") != nullptr;
    Stat pAspects, pDistants, pMesh, pUpload, pMap, pFrame;
    double profileUntil = glfwGetTime() + 1.0;
    auto now_ms = [] {
        return std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    };

    double lastTime = glfwGetTime();
    while (!glfwWindowShouldClose(window)) {
        const double frameT0 = profile ? now_ms() : 0.0;
        glfwPollEvents();
        const double now = glfwGetTime();
        const float dt = static_cast<float>(now - lastTime);
        lastTime = now;

        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(window, &fbw, &fbh);
        if (fbw == 0 || fbh == 0) { continue; } // minimised

        // Stream the world around the camera. Taking a finished build swaps every buffer
        // at once and waits for the GPU to go idle, which is the cost of rebuilding whole
        // rather than per chunk; it happens once every couple of kilometres travelled.
        {
            streamer.update(g_camera.position());
            WorldStreamer::Build nb;
            if (streamer.take(nb)) {
                worldCentre = nb.centre;
                elevRange = glm::vec2(nb.minElev, nb.maxElev);
                // Ground goes in a tile at a time, so only the ring that changed is
                // uploaded; the rest still swaps whole.
                for (const std::uint64_t key : nb.terrainDrop)
                    renderer.removeTerrainChunk(key);
                std::size_t chunkVerts = 0;
                for (const WorldStreamer::Chunk& c : nb.terrainChunks) {
                    renderer.setTerrainChunk(c.key, c.vertices, c.indices);
                    chunkVerts += c.vertices.size();
                }
                renderer.updateTracks(nb.trackV, nb.trackI, nb.trackAlways,
                                      nb.sleeperChunks, nb.trackAlwaysChunks);
                renderer.updateRoads(nb.roadV, nb.roadI);
                renderer.updateStructs(nb.structV, nb.structI, nb.structChunks,
                                       nb.structChunked);
                switches.build(switchNet, worldCentre, data.loadedRadius());
                renderer.updateSwitches(switches.vertices(), switches.indices());
                std::printf("[stream] about scene (%.0f, %.0f): %zu chunk(s) in "
                            "(%zu vertices), %zu out, %zu resident; %zu track, "
                            "%zu struct vertices\n",
                            nb.centre.x, nb.centre.y, nb.terrainChunks.size(),
                            chunkVerts, nb.terrainDrop.size(),
                            renderer.terrainChunkCount(), nb.trackV.size(),
                            nb.structV.size());
                std::fflush(stdout);
            }
        }

        // Live track-circuit occupancy: recomputed every frame (the ground signals read it
        // in the cab view, not just the map). The overlay/aspects are rebuilt only when it
        // changes - a train entering or leaving a section.
        bool occupancyChanged = false;
        {
            std::vector<char> occ(circuits.sections.size(), 0);
            updateOccupancy(occ);
            if (occ != secOccupied) {
                secOccupied = occ;
                occupancyChanged = true;
                if (g_mapMode) g_mapDirty = true;
            }
        }
        // Signal aspects: a signal shows "train on track" (45 deg) when one of the routes
        // it governs has its switches set and a train standing in that route's circuits.
        // Switch throws also change alignment, so re-evaluate on those too.
        // A set route drops as soon as a train enters its circuits: it stops showing clear
        // and its switches are released (the per-switch occupancy lock guards them now).
        if (occupancyChanged) {
            for (std::size_t pi = 0; pi < signalPaths.size(); ++pi) {
                // A block signal's road is written afresh every step from the state of the
                // line; releasing it here as well would only fight that and log a release
                // every frame the train is on it.
                if (blockRoad[pi]) continue;
                if (!routeSet[pi] || occupiedIn(static_cast<int>(pi)).empty()) continue;
                routeSet[pi] = 0;
                switchesChanged = true;
                g_mapDirty = true;
                std::printf("[Route] %s released (train entered)\n",
                            pathName(static_cast<int>(pi)).c_str());
            }
        }
        // Level crossings: each reads its own circuits and runs its own sequence. Every
        // frame rather than only on an occupancy change, because the phases are timed -
        // the 5 s delays and the stuck timeout advance whether or not a train moved.
        if (!crossings.empty()) {
            // Every train's axles together. A crossing counts wheels over its circuits
            // and does not care whose they are, so two portions standing on one approach
            // read as one long train - which is the conservative answer and the right
            // one for a crossing.
            std::vector<VehicleFrame> axles;
            for (const Consist& t : trains) {
                const std::vector<VehicleFrame> a = t.axleFrames();
                axles.insert(axles.end(), a.begin(), a.end());
            }
            bool anyPhaseMoved = false;
            bool anyBarrierMoving = false;
            // Which signals are giving an authority to move, which is what decides how far
            // each crossing's approach circuits reach. The aspects are last frame's - they
            // settle further down the loop - and one frame is nothing against a sequence
            // measured in seconds.
            std::vector<char> signalOpen(sigPlacements.size(), 0);
            for (std::size_t k = 0; k < sigPlacements.size(); ++k)
                signalOpen[k] = signalGivesAuthority(sigPlacements[k]) ? 1 : 0;
            for (std::size_t ci = 0; ci < crossings.size(); ++ci) {
                const CrossingSite& site = crossingSites[ci];
                if (!site.resolved()) continue;
                // The circuits are per track and read independently: a train on one road
                // says nothing about the other, which is what lets two trains be at one
                // crossing on separate roads.
                //
                // Which road an axle is on is decided once, for the axle, rather than by
                // each track asking whether it is near enough. The two roads of a station
                // converge at their turnouts and run a metre apart there, so both would
                // claim a train passing the points and a departure on the main line armed
                // the loop as it went by. An axle is on one road - the nearest - and on
                // no other.
                // How far the approach circuits reach this instant: a signal at danger
                // facing the crossing breaks its circuit there, because nothing beyond one
                // can reach the crossing without first passing it.
                const std::vector<float> reach =
                    crossingReach(site, crossingGuards[ci], signalOpen);
                std::vector<CrossingOccupancy> occ(site.tracks.size());
                for (const VehicleFrame& ax : axles) {
                    float onS = 0.0f;
                    const int under =
                        crossingTrackUnder(site, paths, glm::vec2(ax.pos), onS);
                    if (under < 0) continue; // not on this crossing at all
                    const float rel = onS - site.tracks[under].s;
                    // Where it is, and then where the points are taking it - out on the
                    // approach the roads have not divided and only the second can answer.
                    const int on = crossingRoadAtPoints(site, switchNet, under, onS);
                    // The limit is the road it is physically on, the occupancy the road
                    // the points are taking it to: where a circuit is cut is geometry,
                    // which road's sequence it arms is the points.
                    const std::size_t lim =
                        2 * static_cast<std::size_t>(under) + (rel < 0.0f ? 0u : 1u);
                    const float far = lim < reach.size() ? reach[lim] : site.outerM;
                    if (std::abs(rel) <= site.innerM) occ[on].inner = true;
                    else if (rel < 0.0f && rel >= -far) occ[on].outerA = true;
                    else if (rel > 0.0f && rel <= far) occ[on].outerB = true;
                }
                std::vector<CrossingPhase> was;
                for (const CrossingTrackState& ts : crossingStates[ci].tracks)
                    was.push_back(ts.phase);
                stepCrossing(crossings[ci], crossingStates[ci], occ, now);
                if (crossingStates[ci].tracks.size() != was.size()) anyPhaseMoved = true;
                for (std::size_t t = 0; t < was.size() &&
                                        t < crossingStates[ci].tracks.size(); ++t)
                    if (crossingStates[ci].tracks[t].phase != was[t]) anyPhaseMoved = true;
                if (crossingStates[ci].barrierMoving()) anyBarrierMoving = true;
            }
            // A moving boom is the one thing here that has to be rebuilt every frame. The
            // flashing does not: it is a function of the clock in the shader, baked into
            // the vertices once. A rigid rotation cannot be done that way - it needs a
            // pivot and an axis per vertex and the vertex has no room for them - so while
            // a barrier is in motion the geometry is remade.
            if (anyPhaseMoved || anyBarrierMoving) {
                const double tM = profile ? now_ms() : 0.0;
                const bool made = rebuildSignalBuffer(PartCrossings);
                if (profile) pMesh.add(now_ms() - tM);
                if (made) {
                    const double tU = profile ? now_ms() : 0.0;
                    renderer.updateSignals(signalVerts, signalIdx);
                    if (profile) pUpload.add(now_ms() - tU);
                }
            }
        }

        // Main routes unwind circuit by circuit. Two different rules, deliberately:
        // every circuit's lock is released as that circuit is itself entered, so the route
        // gives up the road behind the train rather than all at once; but only a circuit
        // *beyond* the signal puts it back to danger - whichever one it is and whoever
        // entered it, which is the safety part. Running up to the signal must not cancel
        // the authority the driver is about to act on.
        if (occupancyChanged) {
            for (std::size_t mi = 0; mi < mainRoutes.size();) {
                MainRoute& mr = mainRoutes[mi];
                bool passedSignal = false;
                std::vector<int> still;
                for (int id : mr.locked) {
                    if (secOccupiedById(id)) {
                        if (std::find(mr.beyond.begin(), mr.beyond.end(), id) != mr.beyond.end())
                            passedSignal = true;
                        std::printf("[Main] %s released %s (train entered)\n",
                                    exitRouteName(mr.route).c_str(), secName(id).c_str());
                    } else {
                        still.push_back(id);
                    }
                }
                if (passedSignal && mr.signalClear) {
                    mr.signalClear = false;
                    std::printf("[Main] %s -> DANGER (a circuit beyond the signal entered)\n",
                                exitRouteName(mr.route).c_str());
                }
                if (still.size() != mr.locked.size()) {
                    mr.locked = std::move(still);
                    switchesChanged = true;
                    g_mapDirty = true;
                }
                if (mr.locked.empty()) {
                    std::printf("[Main] %s complete (last circuit entered)\n",
                                exitRouteName(mr.route).c_str());
                    mainRoutes.erase(mainRoutes.begin() + mi);
                } else {
                    ++mi;
                }
            }
        }
        // The line blocks are stepped before the aspect guard rather than inside it. A
        // claim can be released by occupancy alone, with no route and no switch moving, and
        // the route picker reads the claims later in this frame - so deciding them under
        // the guard would hand it a stale answer for one frame.
        bool blocksChanged = false;
        if (!lineBlocks.empty()) {
            const std::vector<int> wasClaimed = [&] {
                std::vector<int> v;
                for (const LineBlockState& st : lineState) v.push_back(st.claimed);
                return v;
            }();
            auto routeInto = [&](int li) {
                for (const MainRoute& mr : mainRoutes)
                    if (lineBlockDirection(lineBlocks, li, mr.departure, signalPaths) != 0)
                        return true;
                return false;
            };
            blocksChanged = stepLineBlocks(lineBlocks, lineState, signalPaths, switchNet,
                                           polys, secOccupiedById, routeInto, blockOut);
            // A block signal's road is its own to open and close, and it does so before the
            // head is given its green - so there is no step, not one, at which a block
            // signal shows clear over a road that is not set.
            for (std::size_t pi = 0; pi < signalPaths.size(); ++pi)
                if (blockRoad[pi]) routeSet[pi] = blockOut.roadSet[pi];
            for (std::size_t li = 0; li < lineState.size(); ++li) {
                if (lineState[li].claimed == wasClaimed[li]) continue;
                if (lineState[li].claimed == 0)
                    std::printf("[Block] line %s released (clear)\n",
                                lineBlocks.lines[li].name.c_str());
            }
            if (blocksChanged) g_mapDirty = true;
        }
        if (occupancyChanged || switchesChanged || simpleSignalsChanged || flagsChanged ||
            blocksChanged) {
            switchesChanged = false;
            simpleSignalsChanged = false;
            // What each main signal shows: danger unless a route it governs is set and has
            // not yet been entered, then C1 (two greens) or C2 (one green) per its type.
            std::vector<SignalAspect> exitAspects(sigPlacements.size(), SignalAspect::Stop);
            for (const MainRoute& mr : mainRoutes) {
                if (!mr.signalClear || mr.placement < 0) continue;
                exitAspects[mr.placement] = mainCandidates[mr.route].type == RouteType::C2
                                                ? SignalAspect::ClearReduced
                                                : SignalAspect::Clear;
            }
            // Dark where the station is unmanned; otherwise red, bar the one cleared.
            for (std::size_t i = 0; i < simpleEntries.size(); ++i) {
                const int pi = simpleEntryPlacement[i];
                if (pi < 0) continue;
                const auto it = simpleStationState.find(simpleEntryStation[i].name);
                const bool manned = it != simpleStationState.end() && it->second.manned;
                const bool green = manned && it->second.green == static_cast<int>(i);
                exitAspects[pi] = !manned  ? SignalAspect::Dark
                                  : green  ? SignalAspect::Clear
                                           : SignalAspect::Stop;
            }
            // The block signals out on the line, decided above from the state of the line
            // rather than from anything a dispatcher did.
            for (std::size_t i = 0; i < blockPlacement.size(); ++i) {
                const int k = blockPlacement[i];
                if (k >= 0 && i < blockOut.aspect.size()) exitAspects[k] = blockOut.aspect[i];
            }
            const double tA = profile ? now_ms() : 0.0;
            bool aspectsMoved = updateSignalAspects(sigPlacements, pathStatics, switchNet,
                                                    secOccupied, routeSet, exitAspects);
            if (profile) pAspects.add(now_ms() - tA);
            const double tD = profile ? now_ms() : 0.0;
            // After the mains have settled, since a distant only repeats what they show.
            // A switch throw reaches here too, and can change what a distant reads without
            // any route having moved.
            if (updateDistantAspects(sigPlacements, polys, junctions, switchNet))
                aspectsMoved = true;
            if (profile) pDistants.add(now_ms() - tD);
            if (aspectsMoved || flagsChanged) {
                // The repeats rather than the crossings: a route being set throws points,
                // and that can change what a repeat reads without any boom moving. The
                // geometry behind it is only remade if the reading actually changed.
                unsigned parts = PartRepeats;
                if (aspectsMoved) parts |= PartSignals;
                // One flag for two meshes: the same keypress path raises a hand flag and
                // shows a TXP, and both are drawn from it.
                if (flagsChanged) parts |= PartFlags | PartTxp;
                flagsChanged = false;
                const double tM = profile ? now_ms() : 0.0;
                const bool made = rebuildSignalBuffer(parts);
                if (profile) pMesh.add(now_ms() - tM);
                if (made) {
                    const double tU = profile ? now_ms() : 0.0;
                    renderer.updateSignals(signalVerts, signalIdx);
                    if (profile) pUpload.add(now_ms() - tU);
                }
            }
        }

        // What the cursor is over in the map, so the overlay can highlight it before it is
        // clicked (and so a click acts on exactly what was highlighted). Signals first,
        // then the armed signal's destinations, then switches.
        if (g_mapMode && !g_menuOpen) {
            double mx = 0.0, my = 0.0;
            glfwGetCursorPos(window, &mx, &my);
            int winw = fbw, winh = fbh;
            glfwGetWindowSize(window, &winw, &winh);
            const glm::vec2 cur(static_cast<float>(mx) * fbw / std::max(winw, 1),
                                static_cast<float>(my) * fbh / std::max(winh, 1));
            const glm::mat4 vp = mapOrtho(static_cast<float>(fbw) / fbh);
            const glm::dvec3 o = switchNet.sceneOrigin();
            auto px2 = [&](glm::vec2 scene, glm::vec2& px) {
                const glm::vec4 clip = vp * glm::vec4(scene.x, scene.y, 0.0f, 1.0f);
                if (clip.w <= 0.0f) return false;
                px = glm::vec2((clip.x / clip.w * 0.5f + 0.5f) * fbw,
                               (clip.y / clip.w * 0.5f + 0.5f) * fbh);
                return true;
            };
            constexpr float kPick = 20.0f; // px
            int hs = -1, hd = -1, ht = -1;
            float best = kPick;
            // Whatever is nearest the cursor wins, so the highlight always matches what the
            // eye picks out. A destination gets a small edge while a signal is armed,
            // because a route's destination often sits at the border where the *next*
            // signal stands and finishing the route is the intent then.
            if (routeArm >= 0)
                for (int pi : miniPaths(sigPlacements[routeArm])) {
                    const Border& e = signalPaths[pi].end;
                    const glm::dvec3 w = fracToWorld(polys, e.trackId, e.frac);
                    if (w.x == 0.0 && w.y == 0.0) continue;
                    glm::vec2 px;
                    if (!px2(glm::vec2(float(w.x - o.x), float(w.y - o.y)), px)) continue;
                    const float d = glm::length(px - cur) - 8.0f; // preference
                    if (d < best) { best = d; hd = pi; hs = ht = -1; }
                }
            for (std::size_t k = 0; k < sigPlacements.size(); ++k) {
                if (!isMini(sigPlacements[k])) continue;
                glm::vec2 px;
                if (!px2(signalAnchor(sigPlacements[k]), px)) continue;
                const float d = glm::length(px - cur);
                if (d < best) { best = d; hs = static_cast<int>(k); hd = ht = -1; }
            }
            const auto& tos = switchNet.turnouts();
            for (std::size_t i = 0; i < tos.size(); ++i) {
                if (tos[i].mainPath < 0) continue; // inert crossing
                glm::vec2 px;
                if (!px2(glm::vec2(float(tos[i].world.x - o.x),
                                   float(tos[i].world.y - o.y)), px)) continue;
                const float d = glm::length(px - cur);
                if (d < best) { best = d; ht = static_cast<int>(i); hs = hd = -1; }
            }
            if (hs != hoverSignal || hd != hoverDest || ht != hoverTurnout) {
                hoverSignal = hs; hoverDest = hd; hoverTurnout = ht;
                g_mapDirty = true; // redraw with the new highlight
            }
        } else if (hoverSignal >= 0 || hoverDest >= 0 || hoverTurnout >= 0) {
            hoverSignal = hoverDest = hoverTurnout = -1;
        }

        // Traffic-manager map overlay: attach on enter/refresh, clear on leave (so the
        // track lines don't bleed into the 3-D view).
        if (g_mapMode && (g_mapDirty || !mapAttached)) {
            // Opening the map lands on the station nearest the train - where the work is
            // - rather than nearest whatever the map was last framed on, and brings the
            // view with it. Nine times out of ten that is the station meant.
            if (!mapAttached && !tmStations.empty() && !tmPinned) {
                tmStation = tmNearest(glm::vec2(g_camera.position()));
                g_mapPan = glm::clamp(tmStations[tmStation].at, mapMin, mapMax) - mapCenter;
            }
            tmPinned = false;
            renderer.showTrackGraph(true);
            const double tMap = profile ? now_ms() : 0.0;
            buildMapOverlay();
            if (profile) pMap.add(now_ms() - tMap);
            mapAttached = true;
            g_mapDirty = false;
        } else if (!g_mapMode && mapAttached) {
            // Only the changing half is dropped. The network stays attached for the life
            // of the program - re-uploading it is what this split exists to avoid - but it
            // has to be hidden, or it goes on being drawn over the cab view.
            renderer.setMapOverlay({}, {});
            renderer.showTrackGraph(false);
            mapAttached = false;
        }

        // Walking to another station brings the map with it: switching to a station and
        // being left looking at the one before would make the panel and the view disagree
        // about where the work is.
        if (g_tmStationStep != 0) {
            if (tmStations.empty()) {
                g_tmStationStep = 0;
            } else {
                const int n = static_cast<int>(tmStations.size());
                tmStation = ((tmStation + g_tmStationStep) % n + n) % n;
                g_tmStationStep = 0;
                g_mapPan = glm::clamp(tmStations[tmStation].at, mapMin, mapMax) - mapCenter;
                setMapMsg("Station: " + tmStations[tmStation].name + "  (" +
                              std::to_string(tmStation + 1) + "/" +
                              std::to_string(tmStations.size()) + ")",
                          true);
            }
        }

        // WASD pans the map. Speed scales with the view height so it feels the same on
        // screen at any zoom; the centre is clamped to the network bounds.
        if (g_mapMode && !g_menuOpen) {
            auto down = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
            glm::vec2 d(0.0f);
            if (down(GLFW_KEY_W)) d.y += 1.0f; // north (up)
            if (down(GLFW_KEY_S)) d.y -= 1.0f; // south (down)
            if (down(GLFW_KEY_D)) d.x += 1.0f; // east (right)
            if (down(GLFW_KEY_A)) d.x -= 1.0f; // west (left)
            if (d.x != 0.0f || d.y != 0.0f) {
                const float halfH = 2000.0f / g_mapZoom;
                g_mapPan += glm::normalize(d) * (halfH * 1.5f) * dt; // ~1.5 heights/s
                g_mapPan = glm::clamp(mapCenter + g_mapPan, mapMin, mapMax) - mapCenter;
            }
        }

        // Map click: throw the motor switch under the cursor if its locking circuits are
        // clear and it isn't broken. Manual switches are hand-thrown in the cab only.
        {
            const bool mL = g_mapMode && !g_menuOpen &&
                            glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
            if (mL && !prevMapClick) {
                // Act on whatever the overlay is highlighting, so a click always does what
                // the cursor said it would.
                if (hoverSignal >= 0) {
                    // A signal: cancel the route it has set, else arm it for a destination.
                    int setHere = -1;
                    for (int pi : miniPaths(sigPlacements[hoverSignal]))
                        if (routeSet[pi]) setHere = pi;
                    if (setHere >= 0) { cancelRoute(setHere); routeArm = -1; }
                    else if (routeArm == hoverSignal) { routeArm = -1; } // click again to disarm
                    else {
                        routeArm = hoverSignal;
                        setMapMsg("Signal armed - click a ringed destination to set the route");
                    }
                    g_mapDirty = true;
                } else if (hoverDest >= 0) {
                    trySetRoute(hoverDest);
                    routeArm = -1;
                } else if (hoverTurnout >= 0) {
                    tryMapThrow(hoverTurnout);
                } else if (routeArm >= 0) {
                    routeArm = -1; // clicked empty space: disarm
                    g_mapDirty = true;
                }
            }
            prevMapClick = mL;
        }

        // Route picker: Up/Down move, Enter chooses, Esc backs out a step (in the key
        // callback, so it works from either). The sim keeps running underneath - this is
        // not the Escape menu and must not pause it.
        if (g_mapMode && !g_menuOpen && g_routeStep != RoutePickStep::None) {
            auto down = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
            const bool pU = down(GLFW_KEY_UP), pD = down(GLFW_KEY_DOWN),
                       pE = down(GLFW_KEY_ENTER);
            if (g_routeStep == RoutePickStep::PickKind) {
                if (pU && !prevPickUp) g_routeKindSel = (g_routeKindSel + 1) % 2;
                if (pD && !prevPickDown) g_routeKindSel = (g_routeKindSel + 1) % 2;
                if (pE && !prevPickEnter) {
                    g_routeEntry = g_routeKindSel == 0; // into the station is the first line
                    g_routeStep = RoutePickStep::PickList;
                    g_routePickSel = 0;
                }
            } else {
                const std::vector<int> rs = stationRoutes(g_routeEntry ? 1 : 0);
                const int n = static_cast<int>(rs.size());
                if (n > 0) {
                    if (pU && !prevPickUp) g_routePickSel = (g_routePickSel + n - 1) % n;
                    if (pD && !prevPickDown) g_routePickSel = (g_routePickSel + 1) % n;
                    g_routePickSel = std::clamp(g_routePickSel, 0, n - 1);
                    if (pE && !prevPickEnter) {
                        const int ri = rs[g_routePickSel];
                        if (mainRouteFor(ri)) cancelExitRoute(ri);
                        else trySetExitRoute(ri);
                    }
                }
            }
            prevPickUp = pU; prevPickDown = pD; prevPickEnter = pE;
        } else if (g_mapMode && !g_menuOpen && g_signalPick &&
                   g_dispatchStep != DispatchStep::None) {
            // Choosing a destination, then a train type. Escape backs out a step; it is
            // handled in the key callback so it works whichever step is showing.
            auto down = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
            const bool pU = down(GLFW_KEY_UP), pD = down(GLFW_KEY_DOWN),
                       pE = down(GLFW_KEY_ENTER);
            const std::string station = simpleStationHere();
            const std::vector<std::string> dests = dispatchDests(station);
            const int n = g_dispatchStep == DispatchStep::PickDest
                              ? std::max<int>(1, static_cast<int>(dests.size()))
                              : 3;
            if (pU && !prevPickUp) g_dispatchSel = (g_dispatchSel + n - 1) % n;
            if (pD && !prevPickDown) g_dispatchSel = (g_dispatchSel + 1) % n;
            g_dispatchSel = std::clamp(g_dispatchSel, 0, n - 1);
            if (pE && !prevPickEnter) {
                if (g_dispatchStep == DispatchStep::PickDest) {
                    if (dests.empty()) {
                        setMapMsg(station + ": no line is worked from here");
                        g_dispatchStep = DispatchStep::None;
                    } else {
                        g_dispatchTo = dests[g_dispatchSel];
                        g_dispatchStep = DispatchStep::PickType;
                        g_dispatchSel = 0;
                    }
                } else {
                    const TxpTrainType type = g_dispatchSel == 0 ? TxpTrainType::Passenger
                                              : g_dispatchSel == 1 ? TxpTrainType::Cargo
                                                                   : TxpTrainType::Other;
                    const TxpExchange r =
                        txpNet.requestDispatch(station, g_dispatchTo, type);
                    logExchange(r);
                    setMapMsg(r.accepted
                                  ? station + " -> " + g_dispatchTo + ": line clear for a " +
                                        txpTrainTypeName(type) + " train"
                                  : station + " -> " + g_dispatchTo + ": refused - " +
                                        refusal(r),
                              r.accepted);
                    g_dispatchStep = DispatchStep::None;
                    g_dispatchTo.clear();
                    g_mapDirty = true;
                }
            }
            prevPickUp = pU; prevPickDown = pD; prevPickEnter = pE;
        } else if (g_mapMode && !g_menuOpen && g_signalPick) {
            auto down = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
            const bool pU = down(GLFW_KEY_UP), pD = down(GLFW_KEY_DOWN),
                       pE = down(GLFW_KEY_ENTER);
            const std::string station = simpleStationHere();
            const std::vector<int> ss = simpleSignalsAt(station);
            const std::vector<int> fp = flagPostsAt(station);
            const std::vector<int> tp = txpPositionsAt(station);
            const std::vector<OrderLine> orders = orderLinesAt(station);
            const PanelLayout L = panelLayout(ss, fp, tp, orders.size());
            const int n = L.count;
            if (pU && !prevPickUp) g_signalPickSel = (g_signalPickSel + n - 1) % n;
            if (pD && !prevPickDown) g_signalPickSel = (g_signalPickSel + 1) % n;
            g_signalPickSel = std::clamp(g_signalPickSel, 0, n - 1);
            if (pE && !prevPickEnter && !station.empty()) {
                StationState& st = simpleStationState[station];
                if (g_signalPickSel == L.dispatchLine) {
                    // Asking for a line: the destination and the train type follow, and
                    // the far end answers automatically once both are chosen.
                    if (!st.manned) {
                        setMapMsg(station + ": cannot dispatch from an unmanned station");
                    } else if (dispatchDests(station).empty()) {
                        setMapMsg(station + ": no line is worked from here");
                    } else {
                        g_dispatchStep = DispatchStep::PickDest;
                        g_dispatchSel = 0;
                    }
                } else if (g_signalPickSel < L.mannedLine) {
                    const OrderLine& o = orders[g_signalPickSel - L.orderFirst];
                    TxpExchange r;
                    if (o.act == OrderAct::OnTrack) r = txpNet.trainOnTrack(station, o.other);
                    else if (o.act == OrderAct::Cancel) r = txpNet.cancelDispatch(station, o.other);
                    else if (o.act == OrderAct::Arrived) r = txpNet.trainArrived(station, o.other);
                    else { setMapMsg(o.text); }
                    if (o.act != OrderAct::None) {
                        logExchange(r);
                        setMapMsg(r.accepted
                                      ? (o.act == OrderAct::OnTrack
                                             ? station + ": train on track to " + o.other
                                         : o.act == OrderAct::Cancel
                                             ? station + ": order to " + o.other + " withdrawn"
                                             : station + ": train arrived, line to " +
                                                   o.other + " clear")
                                      : station + ": refused - " + refusal(r),
                                  r.accepted);
                    }
                } else if (g_signalPickSel == L.mannedLine) {
                    // Manning goes through the train-order network, which can refuse it -
                    // the neighbours will not hand over a section with a train in it.
                    setManned(station, !st.manned);
                } else if (g_signalPickSel == L.allRedLine) {
                    st.green = -1;
                    setMapMsg(station + ": all entry signals at danger");
                } else if (g_signalPickSel < L.flagFirst) {
                    if (!ss.empty()) {
                        // One green per station is the whole interlocking: setting this
                        // one puts every other signal here back to red by construction,
                        // because the station holds a single id rather than a flag per
                        // signal.
                        // Clearing a signal implies manning, and that is the same act:
                        // it must be able to be refused for the same reason.
                        if (!st.manned && !setManned(station, true)) {
                            prevPickEnter = pE;
                            continue;
                        }
                        st.green = ss[g_signalPickSel - L.sigFirst];
                        setMapMsg(simpleEntries[st.green].name + " cleared (" + station +
                                  ": one green at a time)");
                    }
                } else if (g_signalPickSel >= L.txpFirst) {
                    // One position per station shows at a time - a person cannot stand in
                    // two places - so this is a single index, and showing one takes down
                    // whichever was showing before by construction.
                    const int i = tp[g_signalPickSel - L.txpFirst];
                    if (showingHere(station) == i) {
                        txpShowingAt.erase(station);
                        setMapMsg(txpPositions[i].name + ": TXP stands down");
                    } else {
                        txpShowingAt[station] = i;
                        setMapMsg(txpPositions[i].name + ": permission to leave shown");
                    }
                    flagsChanged = true;
                } else {
                    // Cycle this post and only this post: none -> red -> green -> none.
                    const int i = fp[g_signalPickSel - L.flagFirst];
                    flagShown[i] = flagShown[i] == FlagColour::None ? FlagColour::Red
                                   : flagShown[i] == FlagColour::Red ? FlagColour::Green
                                                                     : FlagColour::None;
                    setMapMsg(flagPosts[i].name + ": " +
                              (flagShown[i] == FlagColour::Red     ? "red flag out"
                               : flagShown[i] == FlagColour::Green ? "green flag out"
                                                                   : "flag taken down"));
                    flagsChanged = true;
                }
                simpleSignalsChanged = true;
                g_mapDirty = true;
            }
            prevPickUp = pU; prevPickDown = pD; prevPickEnter = pE;
        } else {
            prevPickUp = prevPickDown = prevPickEnter = false;
        }

        if (g_menuOpen) {
            // --- Escape menu (pauses the sim) ---
            auto down = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
            const bool mU = down(GLFW_KEY_UP), mD = down(GLFW_KEY_DOWN),
                       mE = down(GLFW_KEY_ENTER);
            const bool mPU = down(GLFW_KEY_PAGE_UP), mPD = down(GLFW_KEY_PAGE_DOWN);
            // 720 stations is a long way to travel one line at a time, so the sub-lists
            // take PgUp/PgDn as the start-up station picker does.
            constexpr int kPageStep = 10;

            // The list on screen, and the selection it moves. Only the train list is
            // rebuilt while it is open, because only it can change - the speeds on it are
            // live. The stations and the vehicle names were settled once at load.
            const bool root = g_menuStep == MenuStep::Root;
            std::string title = "MENU";
            if (g_menuStep == MenuStep::PickTrain) title = "DRIVE WHICH TRAIN";
            else if (g_menuStep == MenuStep::PickStation) title = "PLACE A TRAIN AT";
            else if (g_menuStep == MenuStep::PickVehicle) title = "WHICH TRAIN TO PLACE";
            if (root && !menuNote.empty()) title += "  -  " + menuNote;

            if (g_menuStep == MenuStep::PickTrain) menuList = trainLabels();
            const std::vector<std::string>& list =
                g_menuStep == MenuStep::PickStation  ? stationLabels
                : g_menuStep == MenuStep::PickVehicle ? vehicleLabels
                : root                                ? kMenuItems
                                                      : menuList;
            int& sel = root ? g_menuSel : g_menuSubSel;
            const int n = std::max(1, static_cast<int>(list.size()));
            if (mU && !prevMenuUp) sel = (sel + n - 1) % n;
            if (mD && !prevMenuDown) sel = (sel + 1) % n;
            if (mPU && !prevMenuPgUp) sel = (sel + n - kPageStep % n) % n;
            if (mPD && !prevMenuPgDn) sel = (sel + kPageStep) % n;
            sel = std::clamp(sel, 0, n - 1);
            // Step off a heading on to the next real choice, in whichever direction the
            // selection was last moving, so the headings are invisible to use.
            if (g_menuStep == MenuStep::PickVehicle && !vehicleRow.empty()) {
                const int dir = (mU && !prevMenuUp) || (mPU && !prevMenuPgUp) ? -1 : 1;
                for (int guard = 0; guard < n && vehicleRow[sel] < 0; ++guard)
                    sel = (sel + dir + n) % n;
            }

            if (mE && !prevMenuEnter && !list.empty()) {
                if (root) {
                    const std::string& s2 = kMenuItems[g_menuSel];
                    menuNote.clear();
                    if (s2 == "Exit") glfwSetWindowShouldClose(window, GLFW_TRUE);
                    else if (s2 == "Traffic manager") {
                        g_mapMode = true; g_mapDirty = true; g_menuOpen = false;
                        g_mapPan = glm::vec2(0.0f); // start centred on the throat
                        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL); // click switches
                        g_firstMouse = true;
                    } else if (s2 == "Drive another train") {
                        if (trains.empty()) menuNote = "there are no trains";
                        else {
                            g_menuStep = MenuStep::PickTrain;
                            g_menuSubSel = std::clamp(driverTrain, 0,
                                                      static_cast<int>(trains.size()) - 1);
                        }
                    } else if (s2 == "Place a train here") {
                        // Where the camera is standing. In a cab that is the train you are
                        // already driving, which would only ever be refused, so this takes
                        // the free camera's position and says so if you are not using it.
                        if (g_driverPos >= 0 || g_chase)
                            menuNote = "step out of the cab first (V), then place it where "
                                       "you are standing";
                        else {
                            placeTarget = glm::vec2(g_camera.position());
                            g_menuStep = MenuStep::PickVehicle;
                            g_menuSubSel = 0;
                        }
                    } else if (s2 == "Place a train at a station") {
                        g_menuStep = MenuStep::PickStation;
                        g_menuSubSel = stationNearCamera();
                    }
                } else if (g_menuStep == MenuStep::PickTrain) {
                    const int idx = g_menuSubSel;
                    driveTrain(idx, cabToSitIn(trains[static_cast<std::size_t>(idx)]));
                    // Into the cab, which means out of the map if that is where you were.
                    g_mapMode = false;
                    g_menuOpen = false;
                    g_menuStep = MenuStep::Root;
                    glfwSetInputMode(window, GLFW_CURSOR,
                                     g_mouseCaptured ? GLFW_CURSOR_DISABLED
                                                     : GLFW_CURSOR_NORMAL);
                    g_firstMouse = true;
                    std::printf("[Train] now driving %d of %zu (%d set(s)) from cab %d\n",
                                idx + 1, trains.size(), vehicle->unitCount(), g_driverPos);
                    std::fflush(stdout);
                } else if (g_menuStep == MenuStep::PickStation) {
                    const Station& st = stations[static_cast<std::size_t>(g_menuSubSel)];
                    placeTarget = glm::vec2(st.world.x - data.sceneOrigin().x,
                                            st.world.y - data.sceneOrigin().y);
                    g_menuStep = MenuStep::PickVehicle;
                    g_menuSubSel = 0;
                } else if (g_menuStep == MenuStep::PickVehicle) {
                    const int pick = (g_menuSubSel >= 0 &&
                                      g_menuSubSel < static_cast<int>(vehicleRow.size()))
                                         ? vehicleRow[g_menuSubSel]
                                         : -1;
                    if (pick >= 0) { // a heading is not a choice, so Enter does nothing
                        menuNote = placeTrain(pick, placeTarget);
                        // Placed from the start screen, the vehicle picker is still behind
                        // the menu - and choosing from it clears every train there is.
                        // Having put one on the line answers that question, so leave the
                        // start screen.
                        if (vehicle != nullptr) mode = Mode::Sim;
                        g_menuStep = MenuStep::Root;
                        g_menuSubSel = 0;
                    }
                }
            }
            prevMenuUp = mU; prevMenuDown = mD; prevMenuEnter = mE;
            prevMenuPgUp = mPU; prevMenuPgDn = mPD;
            std::vector<TextVertex> tv;
            appendMenu(tv, title, list, sel, fbw, fbh);
            renderer.setOverlayText(tv);
        } else if (g_mapMode && !vehicle) {
            // --- Traffic-manager 2-D map with no vehicle spawned (entered from the
            // start screen). Nothing to simulate; just draw the map + HUD. When a
            // vehicle exists the sim runs live in the Sim branch below. ---
            std::vector<TextVertex> tv;
            appendMapHud(tv, fbw, fbh, nullptr);
            appendRoutePicker(tv, fbw, fbh);
            appendSignalPicker(tv, fbw, fbh);
            renderer.setOverlayText(tv);
        } else if (mode == Mode::Menu) {
            // --- Start screen: pick a vehicle ---
            auto down = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
            const bool kUp = down(GLFW_KEY_UP), kDn = down(GLFW_KEY_DOWN);
            const bool kEnt = down(GLFW_KEY_ENTER);
            if (kUp && !prevUp)
                menuIndex = (menuIndex + kNumVehicleSpecs - 1) % kNumVehicleSpecs;
            if (kDn && !prevDown) menuIndex = (menuIndex + 1) % kNumVehicleSpecs;
            // A number key per vehicle, counted off the list rather than written out
            // one at a time, so the keys follow the menu when it grows.
            for (int i = 0; i < kNumVehicleSpecs && i < 9; ++i) {
                const bool k = down(GLFW_KEY_1 + i);
                if (k && !prevNum[i]) menuIndex = i;
                prevNum[i] = k;
            }
            const bool confirm = kEnt && !prevEnter;
            prevUp = kUp; prevDown = kDn; prevEnter = kEnt;

            if (confirm) {
                spawnVehicle(menuIndex);
                renderer.setOverlayText({});
                mode = Mode::Sim;
            } else {
                std::vector<TextVertex> tv;
                // Sized to the window, and then sized down again if the list would not
                // fit in it: with the category headings the list is 24 rows, and at the
                // old fixed scale the last vehicles and the prompt were drawn below the
                // bottom of a 700-pixel window, where nobody could see them.
                const float rows = static_cast<float>(kNumVehicleSpecs + kVehicleCats + 4);
                const float fit = (static_cast<float>(fbh) - 60.0f) / (12.0f * rows);
                const float sc =
                    std::max(1.5f, std::min(std::max(2.0f, static_cast<float>(fbh) / 240.0f),
                                            fit));
                const float x = 40.0f, lh = 12.0f * sc;
                // Dark backing panel for contrast over the terrain.
                {
                    auto ndc = [&](float px, float py) {
                        return glm::vec2(px / fbw * 2.0f - 1.0f, py / fbh * 2.0f - 1.0f);
                    };
                    const float x1 =
                        std::min(static_cast<float>(fbw) - 20.0f, 40.0f + 30.0f * 8.0f * sc);
                    const float y1 = 40.0f + (kNumVehicleSpecs + kVehicleCats + 4) * lh;
                    const glm::vec3 pc(0.04f, 0.05f, 0.09f);
                    const glm::vec2 a = ndc(20.0f, 20.0f), b = ndc(x1, 20.0f),
                                    c = ndc(x1, y1), d = ndc(20.0f, y1);
                    tv.push_back({a, pc}); tv.push_back({b, pc}); tv.push_back({c, pc});
                    tv.push_back({a, pc}); tv.push_back({c, pc}); tv.push_back({d, pc});
                }
                appendText(tv, "SELECT VEHICLE", x, 40.0f, sc,
                           glm::vec3(1.0f, 0.95f, 0.5f), fbw, fbh);
                // A heading wherever the category changes, so the complete trains are
                // at the top and the bare test shapes are plainly marked at the bottom.
                // Headings take a row of their own, which is why the row is counted
                // separately from the vehicle index.
                int row = 0, lastCat = -1;
                for (int i = 0; i < kNumVehicleSpecs; ++i) {
                    if (kVehicleSpecs[i].category != lastCat) {
                        lastCat = kVehicleSpecs[i].category;
                        appendText(tv, std::string("  ") + categoryName(lastCat), x,
                                   40.0f + (row + 2) * lh, sc * 0.8f,
                                   glm::vec3(1.0f, 0.85f, 0.45f), fbw, fbh);
                        ++row;
                    }
                    const bool hi = (i == menuIndex);
                    std::string line = (hi ? "> " : "  ");
                    line += std::to_string(i + 1) + ". " + specTitle(kVehicleSpecs[i]);
                    appendText(tv, line, x, 40.0f + (row + 2) * lh, sc,
                               hi ? glm::vec3(1.0f) : glm::vec3(0.6f, 0.6f, 0.65f),
                               fbw, fbh);
                    ++row;
                }
                char pick[64];
                std::snprintf(pick, sizeof(pick),
                              "UP/DOWN OR 1-%d TO CHOOSE, ENTER TO START",
                              std::min(kNumVehicleSpecs, 9));
                appendText(tv, pick, x,
                           40.0f + (kNumVehicleSpecs + kVehicleCats + 3) * lh, sc * 0.75f,
                           glm::vec3(0.7f, 0.8f, 0.9f), fbw, fbh);
                renderer.setOverlayText(tv);
            }
        } else if (vehicle) {
            // --- Sim: hand push + physics + camera ---
            // Up/Down hand-push the vehicle, except while the picker has those keys.
            float pushInput = 0.0f;
            if (g_routeStep == RoutePickStep::None) {
                if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) pushInput += 1.0f;
                if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) pushInput -= 1.0f;
            }

            // The driving controls. ',' steps toward power and '.' away from it, as they
            // always have; on a machine whose power and brake are one lever that walks
            // the single combined axis, and on one with two handles it works the power
            // controller alone, with K and L releasing and applying the train brake. Space
            // slams to emergency. All edge-triggered, so a press is a notch.
            //
            // Letters and the two punctuation keys only: [ ] and \ are AltGr combinations
            // on a Norwegian keyboard and cannot be pressed as plain keys at all.
            auto down = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
            // The keyboard drives the cab you are sitting in (driver view V), or
            // the front cab (0) in any other view.
            const int cab = (g_driverPos >= 0) ? g_driverPos : 0;
            const bool split = vehicle->controls() == ControlSeparate;
            const bool bD = down(GLFW_KEY_COMMA), bU = down(GLFW_KEY_PERIOD),
                       bE = down(GLFW_KEY_SPACE);
            const bool kRel = split && down(GLFW_KEY_K), kApp = split && down(GLFW_KEY_L);
            // The independent brake on H / J, immediately left of the train brake's K / L,
            // so the two read left to right along the row: loco brake, then train brake,
            // each with release on the left and apply on the right.
            const bool iRel = vehicle->hasIndependentBrake() && down(GLFW_KEY_H);
            const bool iApp = vehicle->hasIndependentBrake() && down(GLFW_KEY_J);
            const int prevPos = vehicle->handlePosition(cab);
            const int prevPow = vehicle->powerNotch(cab), prevBrk = vehicle->brakeNotch(cab);
            const int prevInd = vehicle->independentNotch(cab);
            if (bD && !prevBrkDown) {
                if (split) vehicle->movePower(cab, +1); else vehicle->moveHandle(cab, -1);
            }
            if (bU && !prevBrkUp) {
                if (split) vehicle->movePower(cab, -1); else vehicle->moveHandle(cab, +1);
            }
            if (kRel && !prevBrkRel) vehicle->moveBrake(cab, -1);
            if (kApp && !prevBrkApp) vehicle->moveBrake(cab, +1);
            if (iRel && !prevIndRel) vehicle->moveIndependent(cab, -1);
            if (iApp && !prevIndApp) vehicle->moveIndependent(cab, +1);
            if (bE && !prevBrkEmerg) {
                vehicle->setPowerNotch(cab, 0);
                vehicle->setBrakeNotch(cab, Vehicle::kEmergencyNotch);
            }
            prevBrkDown = bD; prevBrkUp = bU; prevBrkEmerg = bE;
            prevBrkRel = kRel; prevBrkApp = kApp;
            prevIndRel = iRel; prevIndApp = iApp;
            const bool moved = split ? (vehicle->powerNotch(cab) != prevPow ||
                                        vehicle->brakeNotch(cab) != prevBrk ||
                                        vehicle->independentNotch(cab) != prevInd)
                                     : vehicle->handlePosition(cab) != prevPos;
            if (moved) {
                if (split) {
                    const int np = vehicle->powerNotch(cab);
                    char ctl[16];
                    if (np < 0) std::snprintf(ctl, sizeof(ctl), "E-brake E%d", -np);
                    else std::snprintf(ctl, sizeof(ctl), "power P%d", np);
                    std::printf("[Handle] cab %d %s  brake %s  loco L%d  BP %.1f  "
                                "BC %.1f  MR %.1f bar\n", cab, ctl,
                                vehicle->brakeNotchName(cab),
                                vehicle->independentNotch(cab), vehicle->bpPressure(),
                                vehicle->bcPressure(), vehicle->mrPressure());
                }
                else
                    std::printf("[Handle] cab %d %s  BP %.1f  BC %.1f  MR %.1f bar\n", cab,
                                vehicle->handleName(cab), vehicle->bpPressure(),
                                vehicle->bcPressure(), vehicle->mrPressure());
                std::fflush(stdout);
            }

            // Reverser handle for the same cab: F = Forward, N = Neutral, R =
            // Reverse (edge-triggered). No traction yet, but the R/N/F interlock
            // gates the brakes across both cabs (see Vehicle::effectiveNotch).
            // The reverser is a cab control: in the map R offers the exit routes instead.
            const bool rF = !g_mapMode && down(GLFW_KEY_F),
                       rN = !g_mapMode && down(GLFW_KEY_N),
                       rR = !g_mapMode && down(GLFW_KEY_R);
            const int prevRev = vehicle->reverser(cab);
            if (rF && !prevRevF) vehicle->setReverser(cab, 1);
            if (rN && !prevRevN) vehicle->setReverser(cab, 0);
            if (rR && !prevRevR) vehicle->setReverser(cab, -1);
            prevRevF = rF; prevRevN = rN; prevRevR = rR;
            if (vehicle->reverser(cab) != prevRev) {
                std::printf("[Reverser] cab %d %s%s\n", cab,
                            vehicle->reverserName(cab),
                            vehicle->interlockEmergency() ? "  (interlock: EMERG)" : "");
                std::fflush(stdout);
            }

            // --- Uncoupling ---------------------------------------------------
            // No picker and no list: the coupler acted on is the one nearest the
            // camera, because uncoupling is something done by walking to it. Past
            // kUncoupleReach there is no coupler at all, so "nearest" means "the one I
            // am standing at" rather than "the one on the train down the valley".
            //
            // Where a coupler is: midway between the centres of the two sets it holds.
            // On a curve that sits a little inside the arc, which is nowhere near
            // enough to confuse two couplers 42 m apart, and unlike stepping half a
            // body length off a tangent it cannot come out on the wrong side of a set.
            constexpr float kUncoupleReach = 30.0f; // m from the camera
            constexpr double kUncoupleArmS = 5.0;   // how long an armed coupler waits
            int nearTrain = -1, nearCoupler = -1;
            {
                const glm::vec3 eye = g_camera.position();
                float nearest = kUncoupleReach;
                for (std::size_t ti = 0; ti < trains.size(); ++ti) {
                    const Consist& t = trains[ti];
                    for (int k = 0; k + 1 < t.unitCount(); ++k) {
                        const glm::vec3 c =
                            0.5f * (t.unit(k).frame().pos + t.unit(k + 1).frame().pos);
                        const float d = glm::distance(eye, c);
                        if (d < nearest) {
                            nearest = d;
                            nearTrain = static_cast<int>(ti);
                            nearCoupler = k;
                        }
                    }
                }
            }
            // Walking away from an armed coupler, or simply leaving it, disarms it.
            if (armedCoupler >= 0 &&
                (nearTrain != armedTrain || nearCoupler != armedCoupler ||
                 now - armedAt > kUncoupleArmS))
                armedTrain = armedCoupler = -1;
            // Two presses, and not because two is a nicer number: there is no coupling
            // yet, so parting a train cannot be undone, and the first press is what
            // lets the driver read back which coupler he is about to open.
            const char* uncoupleWhy = "";
            const bool uKey = !g_mapMode && down(GLFW_KEY_U);
            if (uKey && !prevUncouple && nearCoupler >= 0) {
                Consist& t = trains[static_cast<std::size_t>(nearTrain)];
                if (!t.mayUncouple(nearCoupler, uncoupleWhy)) {
                    armedTrain = armedCoupler = -1;
                } else if (armedTrain == nearTrain && armedCoupler == nearCoupler) {
                    const int k = nearCoupler;
                    const int setsBefore = t.unitCount();
                    std::optional<Consist> rear = t.uncoupleAfter(k);
                    armedTrain = armedCoupler = -1;
                    if (rear) {
                        // On the end, never in the middle: a deque moves nothing on
                        // push_back, and `vehicle` is a pointer into it.
                        trains.push_back(std::move(*rear));
                        // The driver keeps the cab he is sitting in, wherever it has
                        // ended up. Not in one, he keeps the portion whose identity was
                        // preserved - the front, which is still where it was. Either way
                        // he can now step into the other portion from the menu.
                        int stay = driverTrain, cab = g_driverPos;
                        if (nearTrain == driverTrain && g_driverPos >= 2 * (k + 1)) {
                            stay = static_cast<int>(trains.size()) - 1;
                            cab = g_driverPos - 2 * (k + 1);
                        }
                        const bool wasChasing = g_chase;
                        const bool wasOutside = g_driverPos < 0;
                        driveTrain(stay, std::max(0, cab));
                        // driveTrain seats the driver, which is right when he was in a
                        // cab and wrong when he was stood beside the train watching.
                        if (wasOutside) g_driverPos = -1;
                        g_chase = wasChasing;
                        // The composition changed, so the index buffer has to be rebuilt
                        // too - a vertex refresh under the old indices draws a heap of
                        // triangles and says nothing about it.
                        vmesh.build(trains);
                        renderer.attachVehicle(vmesh.vertices(), vmesh.indices(),
                                               vmesh.glassFirstIndex());
                        std::printf("[Uncouple] %d sets became %d + %d; driving train %d "
                                    "cab %d\n",
                                    setsBefore, trains[static_cast<std::size_t>(nearTrain)]
                                                    .unitCount(),
                                    trains.back().unitCount(), driverTrain, g_driverPos);
                        std::fflush(stdout);
                    }
                } else {
                    armedTrain = nearTrain;
                    armedCoupler = nearCoupler;
                    armedAt = now;
                }
            }
            prevUncouple = uKey;
            // The mark follows the armed coupler, and is rebuilt only when that changes.
            if (armedTrain != markedTrain || armedCoupler != markedCoupler) {
                markedTrain = armedTrain;
                markedCoupler = armedCoupler;
                const double tM = profile ? now_ms() : 0.0;
                const bool made = rebuildSignalBuffer(PartMark);
                if (profile) pMesh.add(now_ms() - tM);
                if (made) {
                    const double tU = profile ? now_ms() : 0.0;
                    renderer.updateSignals(signalVerts, signalIdx);
                    if (profile) pUpload.add(now_ms() - tU);
                }
            }

            // I: start / stop the diesel engines (both together, edge-triggered).
            const bool iKey = down(GLFW_KEY_I);
            if (iKey && !prevEngine && vehicle->engineCount() > 0) {
                const EngineState es = vehicle->engineState(0);
                const bool wasOff = es == EngineState::Off || es == EngineState::Stopping;
                vehicle->toggleEngines();
                std::printf("[Engine] %s\n", wasOff ? "starting" : "stopping");
                std::fflush(stdout);
            }
            prevEngine = iKey;

            const float simDt = std::min(dt, 0.05f);
            const VehicleState prev = vehicle->state();
            // Every train is stepped; only the one being driven feels the hand push.
            for (Consist& t : trains) t.update(simDt, &t == vehicle ? pushInput : 0.0f);
            // ...and every train is asked whether it forced a switch, the answers OR'd.
            // The flag clears on read, so asking only the driven train would silently
            // lose one a detached portion had run through, and the only symptom would
            // be the stand no longer matching the switch it stands at.
            bool switchForced = false;
            for (Consist& t : trains)
                switchForced = t.consumeSwitchChanged() || switchForced;
            // Trains meeting. Until this, nothing in the simulator noticed another train
            // was there at all: each consist was stepped alone and two driven together
            // passed through each other in silence. A Class 93's Scharfenberg couples by
            // being driven into, so there is nothing to arm and nothing to aim - what
            // decides between a coupling and a wreck is only how fast the gap was closing.
            //
            // One contact per frame. Coupling erases an element and three trains meeting
            // at once would otherwise pull two out from under the same loop.
            for (std::size_t i = 0; i + 1 < trains.size(); ++i) {
                bool met = false;
                for (std::size_t j = i + 1; j < trains.size() && !met; ++j) {
                    const Contact c = findContact(trains[i], trains[j]);
                    if (c.kind == ContactKind::None) continue;
                    met = true;
                    // Heard from where it happened, not through the near train's
                    // envelope: two trains meeting thirty kilometres away are two
                    // trains meeting thirty kilometres away. Carries further than the
                    // brake hiss - it is the loudest thing either train will ever do.
                    const glm::vec3 hit = trains[j].frame().pos;
                    const float d = glm::distance(g_camera.position(), hit);
                    // The square root opens the quiet end out: a coupling at walking
                    // pace is a fraction of a collision in energy but it is not a
                    // fraction of it in audibility, and it should still be a clack.
                    audio.impact(std::sqrt(std::clamp(c.closing / 8.0f, 0.0f, 1.0f)),
                                 glm::clamp((300.0f - d) / 260.0f, 0.0f, 1.0f));
                    if (c.kind == ContactKind::Crash) {
                        std::printf("[Collision] two trains met at %.1f m/s (%.0f km/h) - "
                                    "%d and %d set(s) derailed\n",
                                    c.closing, c.closing * 3.6f, trains[i].unitCount(),
                                    trains[j].unitCount());
                        std::fflush(stdout);
                        trains[i].derail();
                        trains[j].derail();
                        continue; // nothing is coupled and nothing leaves the container
                    }
                    const bool rough = c.kind == ContactKind::Rough;
                    const int sets = trains[i].unitCount() + trains[j].unitCount();
                    const int joint = coupleTrains(static_cast<int>(i), static_cast<int>(j),
                                                   c.aTail, c.opposed);
                    if (rough) {
                        // Taken hard enough to shake the drawgear: the hoses at the joint
                        // part, which every distributor on the train then reads as the
                        // pipe being lost and answers with its own air. Nothing has to
                        // command the brakes on - see the air brake.
                        Consist& t = trains[i];
                        if (joint > 0) t.unit(joint - 1).burstBrakePipe();
                        if (joint < t.unitCount()) t.unit(joint).burstBrakePipe();
                    }
                    std::printf("[Couple] %s at %.2f m/s (%.1f km/h): one train of %d "
                                "set(s)%s\n", rough ? "roughly" : "gently", c.closing,
                                c.closing * 3.6f, sets,
                                rough ? " - the hoses parted at the joint" : "");
                    std::fflush(stdout);
                }
                if (met) break;
            }
            if (switchForced) { // a switch was forced/broken
                switches.build(switchNet, worldCentre, data.loadedRadius());
                renderer.updateSwitches(switches.vertices(), switches.indices());
                g_mapDirty = true; // refresh the map marker if it's open
                switchesChanged = true;
                std::printf("[Switch] forced -> broken (neutral)\n");
            }
            if (vehicle->state() != prev) {
                static const char* kNames[] = {"OnRail", "Derailed", "Stopped"};
                std::printf("[Vehicle] -> %s (speed %.1f m/s)\n",
                            kNames[static_cast<int>(vehicle->state())],
                            vehicle->speed());
                std::fflush(stdout);
            }
            if (vehicle->safetyBrakeActive() && !prevSafety)
                std::printf("[Brake] LOW RESERVOIR (%.1f bar) -> automatic emergency\n",
                            vehicle->mrPressure());
            prevSafety = vehicle->safetyBrakeActive();
            // Fade the sounds by camera distance to their sources: the brake by the
            // nearest bogie, each engine by its own car section (the underfloor
            // engine). Full when close, silent far away.
            const glm::vec3 camPos = g_camera.position();
            float distGain = 0.0f;
            // Wheel on rail comes from the same wheels as the brake does, but it is the
            // loudest thing a running train makes and carries far further than a brake
            // valve: a train is heard passing long before its air is.
            float rollGain = 0.0f;
            // Which train the roar, the brake hiss and the compressor come from: the
            // nearest to the camera, and not necessarily the one being driven. There is
            // one of each of those voices and one set of filters behind them, so exactly
            // one train can have them. Nearest is what lets a detached portion be heard
            // rolling past and still leaves a driver hearing his own train from the cab.
            // It is only audibly wrong with two trains in earshot both working at once,
            // and that is a limit of the synth rather than of the simulation.
            const Consist* sounded = vehicle;
            {
                // A bare wheelset has no bogie at all, so its own frame stands in -
                // without that the single-axle vehicle was silent at any distance,
                // including nose to nose with it.
                float nearest = std::numeric_limits<float>::max();
                for (const Consist& t : trains) {
                    std::vector<VehicleFrame> at = t.bogieFrames();
                    if (at.empty()) at.push_back(t.frame());
                    for (const VehicleFrame& b : at) {
                        const float d = glm::distance(camPos, b.pos);
                        if (d < nearest) { nearest = d; sounded = &t; }
                    }
                }
                std::vector<VehicleFrame> at = sounded->bogieFrames();
                if (at.empty()) at.push_back(sounded->frame());
                for (const VehicleFrame& b : at) {
                    const float d = glm::distance(camPos, b.pos);
                    distGain = std::max(distGain,
                                        glm::clamp((60.0f - d) / 55.0f, 0.0f, 1.0f));
                    rollGain = std::max(rollGain,
                                        glm::clamp((160.0f - d) / 140.0f, 0.0f, 1.0f));
                }
            }
            // One gain per engine, from the car body that engine sits in - the sets
            // are laid out along the train, so on a coupled pair the near engines swell
            // and the far ones recede as you walk the length of it.
            // Engine k sits in car body k: two engines and two car bodies to a set, laid
            // out in that order in both lists, so the index carries straight across.
            // Gathered over every train, because the synth's six slots have no notion
            // of which train a slot belongs to - a portion left idling nearby wants a
            // voice as much as the far end of the train being driven does. In `trains`
            // order and never sorted: a slot carries its own filter state, so shuffling
            // which engine sits in which slot would smear one engine's sound onto the
            // next. Six is the longest Class 93 train there is, so parting one never
            // asks for more slots than it had.
            Audio::EngineVoice voices[Audio::kMaxEngines];
            int nGain = 0;
            for (const Consist& t : trains) {
                const std::vector<VehicleFrame> secs = t.bodySectionFrames();
                const int n =
                    std::min<int>(t.engineCount(), static_cast<int>(secs.size()));
                for (int k = 0; k < n && nGain < Audio::kMaxEngines; ++k) {
                    // The timbre travels with the rpm, from the machine the slot is
                    // sounding. A slot keeps its filter state between frames, so these
                    // must change in step with the engine it belongs to or one machine's
                    // character is smeared onto the next.
                    const Vehicle& u = t.lead();
                    voices[nGain++] = {
                        t.engineRpm(k),
                        glm::clamp((50.0f - glm::distance(camPos, secs[k].pos)) / 38.0f,
                                   0.0f, 1.0f),
                        u.firingsPerRev(), u.engineVolume(), u.engineRumble(),
                        u.engineBright(), u.idleRpm()};
                }
            }
            // The crossing bell: whichever ringing crossing is loudest from here. A bell
            // carries further than the brakes do, and it is the crossing's own sound
            // rather than the train's, so it is placed at the crossing and not the cab.
            float bellGain = 0.0f;
            for (std::size_t ci = 0; ci < crossings.size(); ++ci) {
                const CrossingSite& site = crossingSites[ci];
                if (!site.resolved()) continue;
                if (!crossingBell(crossingStates[ci], now)) continue;
                int on = -1;
                for (std::size_t t = 0; t < site.tracks.size() && on < 0; ++t)
                    if (site.tracks[t].path >= 0) on = static_cast<int>(t);
                const glm::vec3 at =
                    paths[site.tracks[on].path].poseAt(site.tracks[on].s).pos;
                bellGain = std::max(bellGain,
                                    glm::clamp((250.0f - glm::distance(camPos, at)) / 200.0f,
                                               0.0f, 1.0f));
            }
            audio.setCrossingBell(bellGain);
            audio.update(*sounded, simDt, distGain, voices, nGain, rollGain);
            vmesh.build(trains);
            renderer.updateVehicleVertices(vmesh.vertices());

            // Aim: the switch stand nearest the camera's forward ray (the crosshair),
            // in front and within a small cone. T throws it. Only in the cab view -
            // the map is display-only for switches.
            int aimedSwitch = -1;
            if (!g_mapMode) {
                const glm::vec3 cp = g_camera.position();
                const glm::vec3 cd = glm::normalize(g_camera.forward());
                const glm::dvec3 org = switchNet.sceneOrigin();
                float bestAng = 0.06f; // ~3.4 deg
                const auto& tos = switchNet.turnouts();
                for (int i = 0; i < static_cast<int>(tos.size()); ++i) {
                    const glm::vec3 X(static_cast<float>(tos[i].world.x - org.x),
                                      static_cast<float>(tos[i].world.y - org.y),
                                      static_cast<float>(tos[i].world.z - org.z) + 1.6f);
                    const glm::vec3 rel = X - cp;
                    const float t = glm::dot(rel, cd);
                    if (t < 3.0f || t > 250.0f) continue;
                    const float ang = glm::length(rel - cd * t) / t;
                    if (ang < bestAng) { bestAng = ang; aimedSwitch = i; }
                }
            }
            if (g_throwSwitch) {
                g_throwSwitch = false;
                // Motor (point-machine) switches can't be hand-thrown - they're worked
                // remotely (a later feature). Only manual switches respond to T.
                if (!g_mapMode && aimedSwitch >= 0 &&
                    switchNet.type(aimedSwitch) != SwitchType::Motor) {
                    switchNet.toggle(aimedSwitch);
                    switches.build(switchNet, worldCentre, data.loadedRadius());
                    renderer.updateSwitches(switches.vertices(), switches.indices());
                    switchesChanged = true;
                    std::printf("[Switch] %d -> %s\n", aimedSwitch,
                                switchNet.state(aimedSwitch) == SwitchState::Straight
                                    ? "straight" : "diverging");
                }
            }

            // HUD: in map mode, the traffic-manager overlay (with live train speed);
            // otherwise the cab HUD (speed, reservoir/brake pressures, brake notch).
            if (g_mapMode) {
                std::vector<TextVertex> tv;
                appendMapHud(tv, fbw, fbh, vehicle);
                appendRoutePicker(tv, fbw, fbh);
                appendSignalPicker(tv, fbw, fbh);
                renderer.setOverlayText(tv);
            } else {
                std::vector<TextVertex> tv;
                const float sc = std::max(2.0f, static_cast<float>(fbh) / 240.0f);
                const float lh = 12.0f * sc, x = 40.0f;
                char buf[96];
                const int cab = (g_driverPos >= 0) ? g_driverPos : 0;
                float y = 40.0f; // running line cursor
                std::snprintf(buf, sizeof(buf), "SPEED %.0f km/h  (%.1f m/s)",
                              vehicle->speed() * 3.6f, vehicle->speed());
                appendText(tv, buf, x, y, sc, glm::vec3(1.0f, 0.95f, 0.6f), fbw, fbh);
                y += lh;
                // The driver reads his own set's gauges; a coupled train lists every
                // set's, because the whole point of them being separate is that they
                // can differ - and a set low on air is a thing to see coming.
                std::snprintf(buf, sizeof(buf), "BP %.1f   BC %.1f   MR %.1f bar",
                              vehicle->bpPressure(cab), vehicle->bcPressure(cab),
                              vehicle->mrPressure(cab));
                appendText(tv, buf, x, y, sc, glm::vec3(0.8f, 0.9f, 1.0f), fbw, fbh);
                y += lh;
                if (vehicle->unitCount() > 1) {
                    // Set by set along the train, pipe over brake cylinder. Just the
                    // number: spelling out SET each time runs three sets past the width
                    // of a narrow window for nothing the order does not say. A `!` marks
                    // a set whose pipe is open at a parted coupling, which is the one
                    // reading here that is a fault rather than a state.
                    std::string per;
                    for (int u = 0; u < vehicle->unitCount(); ++u) {
                        char one[40];
                        std::snprintf(one, sizeof(one), "%s%d %.1f/%.1f%s", u ? "   " : "",
                                      u + 1, vehicle->unit(u).bpPressure(),
                                      vehicle->unit(u).bcPressure(),
                                      vehicle->unit(u).brakePipeCut() ? "!" : "");
                        per += one;
                    }
                    appendText(tv, per, x, y, sc, glm::vec3(0.7f, 0.8f, 0.95f), fbw, fbh);
                    y += lh;
                }
                std::snprintf(buf, sizeof(buf), "REV %s (cab %d)   %s",
                              vehicle->reverserName(cab), cab,
                              vehicle->cabDrivable(cab) ? "F / N / R"
                                                        : "SHUT DOWN (coupled)");
                appendText(tv, buf, x, y, sc, glm::vec3(0.85f, 0.85f, 0.7f), fbw, fbh);
                y += lh;
                if (vehicle->controls() == ControlSeparate) {
                    // One controller with a range either side of neutral, so the label
                    // has to say which side it is on - otherwise E2 and P2 read alike and
                    // the driver cannot tell pulling from pushing.
                    const int np = vehicle->powerNotch(cab);
                    char ctl[40];
                    if (np < 0)
                        std::snprintf(ctl, sizeof(ctl), "E-BRAKE E%d  %3.0f kN", -np,
                                      vehicle->lead().dynamicBrakeForce() / 1000.0f);
                    else
                        std::snprintf(ctl, sizeof(ctl), "POWER P%d", np);
                    if (vehicle->hasIndependentBrake())
                        std::snprintf(buf, sizeof(buf),
                                      "%-22s , / .   LOCO L%d  H/J   TRAIN %s  K/L / Space",
                                      ctl, vehicle->independentNotch(cab),
                                      vehicle->brakeNotchName(cab));
                    else
                        std::snprintf(buf, sizeof(buf),
                                      "%-22s , / .      BRAKE %s  K rel / L app / Space",
                                      ctl, vehicle->brakeNotchName(cab));
                }
                else
                    std::snprintf(buf, sizeof(buf),
                                  "HANDLE %s   , power / brake . / Space",
                                  vehicle->handleName(cab));
                const bool emerg = vehicle->brakeNotch(cab) >= Vehicle::kEmergencyNotch;
                appendText(tv, buf, x, y, sc,
                           emerg ? glm::vec3(1.0f, 0.4f, 0.35f) : glm::vec3(0.7f, 0.85f, 0.7f),
                           fbw, fbh);
                y += lh;
                // Sitting in a cab that is not the one in charge. Every control here moves
                // and reads back, and the train ignores all of them, because the commands
                // are taken from the cab holding the reverser. Worth a line of its own:
                // without it this is indistinguishable from a broken locomotive, and the
                // way out - take the reverser here - is not guessable.
                const int inCharge = vehicle->activeCab();
                if (inCharge >= 0 && inCharge != cab) {
                    std::snprintf(buf, sizeof(buf),
                                  "!! CAB %d IS NOT DRIVING - cab %d has the reverser "
                                  "(F or R here takes it) !!", cab, inCharge);
                    appendText(tv, buf, x, y, sc, glm::vec3(1.0f, 0.75f, 0.3f), fbw, fbh);
                    y += lh;
                }
                if (vehicle->interlockEmergency()) {
                    appendText(tv, "!! REVERSER INTERLOCK - AUTO EMERGENCY !!", x, y,
                               sc, glm::vec3(1.0f, 0.35f, 0.3f), fbw, fbh);
                    y += lh;
                }
                if (vehicle->safetyBrakeActive()) {
                    // Which set called for it: on a coupled train the driver's own
                    // gauges can be perfectly healthy while the other set stops him.
                    if (vehicle->unitCount() > 1)
                        std::snprintf(buf, sizeof(buf),
                                      "!! SET %d LOW RESERVOIR - AUTO EMERGENCY !!",
                                      vehicle->trippedUnit() + 1);
                    else
                        std::snprintf(buf, sizeof(buf),
                                      "!! LOW RESERVOIR - AUTO EMERGENCY !!");
                    appendText(tv, buf, x, y, sc, glm::vec3(1.0f, 0.35f, 0.3f), fbw, fbh);
                    y += lh;
                }
                // The uncoupling hold, and why the brakes will not come off. Without
                // this the driver of a train that has just come apart sees an emergency
                // with no cause anywhere on his gauges - his own air is perfect.
                if (vehicle->uncoupleHold() != Consist::UncoupleHold::None) {
                    const bool atN =
                        vehicle->uncoupleHold() == Consist::UncoupleHold::AtNeutral;
                    appendText(tv,
                               atN ? "!! UNCOUPLED - REVERSER OUT OF N TO RELEASE !!"
                                   : "!! UNCOUPLED - AUTO EMERGENCY - REVERSER TO N !!",
                               x, y, sc, glm::vec3(1.0f, 0.35f, 0.3f), fbw, fbh);
                    y += lh;
                }
                // The coupler being stood at, and what U would do to it. Named by the
                // sets it holds together, because that is what the driver can see.
                if (nearCoupler >= 0) {
                    const Consist& nt = trains[static_cast<std::size_t>(nearTrain)];
                    const bool armed =
                        armedTrain == nearTrain && armedCoupler == nearCoupler;
                    const char* why = "";
                    if (!nt.mayUncouple(nearCoupler, why))
                        std::snprintf(buf, sizeof(buf), "COUPLER %d | %d   %s",
                                      nearCoupler + 1, nearCoupler + 2, why);
                    else if (armed)
                        std::snprintf(buf, sizeof(buf),
                                      "!! UNCOUPLE SETS %d | %d - PRESS U AGAIN !!",
                                      nearCoupler + 1, nearCoupler + 2);
                    else
                        std::snprintf(buf, sizeof(buf),
                                      "COUPLER BETWEEN SETS %d AND %d   U to uncouple",
                                      nearCoupler + 1, nearCoupler + 2);
                    appendText(tv, buf, x, y, sc,
                               armed ? glm::vec3(1.0f, 0.8f, 0.3f)
                                     : glm::vec3(0.7f, 0.85f, 0.7f),
                               fbw, fbh);
                    y += lh;
                }
                if (vehicle->engineCount() > 0) {
                    const EngineState es = vehicle->engineState(0);
                    if (es == EngineState::Off)
                        std::snprintf(buf, sizeof(buf), "ENG OFF   I to start");
                    else {
                        const char* sn = es == EngineState::Running    ? "RUNNING"
                                         : es == EngineState::Starting ? "STARTING"
                                                                       : "STOPPING";
                        // On a diesel-electric the revs are only half the story: the
                        // load regulator is still winding excitation on after the engine
                        // has arrived, and at low speed the current limit means the diesel
                        // is barely worked however hard the ammeter is pegged. Both are
                        // worth showing, or the lag reads as the locomotive being broken.
                        if (vehicle->lead().drive() == DriveElectric)
                            std::snprintf(buf, sizeof(buf),
                                          "ENG %s  %.0f rpm   exc %3.0f%%  amps %3.0f%%  "
                                          "load %3.0f%%", sn, vehicle->engineRpm(0),
                                          vehicle->lead().loadRegulator() * 100.0f,
                                          vehicle->lead().tractionAmpsFrac() * 100.0f,
                                          vehicle->lead().enginePowerFrac() * 100.0f);
                        else
                            std::snprintf(buf, sizeof(buf), "ENG %s  %.0f rpm", sn,
                                          vehicle->engineRpm(0));
                    }
                    appendText(tv, buf, x, y, sc, glm::vec3(0.7f, 0.85f, 0.7f), fbw, fbh);
                    y += lh;
                }
                // Centre crosshair + the aimed switch's state and throw prompt.
                appendText(tv, "+", fbw * 0.5f - 3.0f * sc, fbh * 0.5f - 6.0f * sc, sc,
                           glm::vec3(1.0f, 1.0f, 1.0f), fbw, fbh);
                if (aimedSwitch >= 0) {
                    const SwitchState ss = switchNet.state(aimedSwitch);
                    const char* sn = ss == SwitchState::Straight    ? "STRAIGHT"
                                     : ss == SwitchState::Diverging  ? "DIVERGING"
                                                                     : "BROKEN";
                    const bool broken = ss == SwitchState::Broken;
                    const bool motor = switchNet.type(aimedSwitch) == SwitchType::Motor;
                    if (motor)
                        std::snprintf(buf, sizeof(buf), "SWITCH %s   MOTOR - no hand throw", sn);
                    else
                        std::snprintf(buf, sizeof(buf), "SWITCH %s   T to throw", sn);
                    const glm::vec3 col = motor    ? glm::vec3(0.6f, 0.7f, 1.0f)
                                          : broken ? glm::vec3(1.0f, 0.5f, 0.3f)
                                                   : glm::vec3(0.6f, 1.0f, 0.8f);
                    appendText(tv, buf, fbw * 0.5f - (motor ? 92.0f : 66.0f) * sc,
                               fbh * 0.5f + 14.0f * sc, sc, col, fbw, fbh);
                }
                renderer.setOverlayText(tv);
            }

            // Camera control only in the cab view; the map uses a fixed ortho
            // projection, so the 3-D camera is left untouched while it's open.
            // The camera keeps following the train even with the map open. It is not only
            // where the view is drawn from: it is the reference point for how loud the
            // train sounds and for where terrain is streamed in. Freezing it while the
            // map is up leaves the train to roll out of earshot of its own brakes and off
            // the loaded ground, which is what happened when the traffic manager grew
            // enough to keep a dispatcher in it for a while.
            //
            // Only the free-look movement stays behind the map gate, because there WASD
            // pans the map instead - and a free camera is meant to stay where it was put.
            float fwd = 0.0f, right = 0.0f, up = 0.0f;
            bool fast = false;
            if (!g_mapMode) {
                if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) fwd += 1.0f;
                if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) fwd -= 1.0f;
                if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) right += 1.0f;
                if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) right -= 1.0f;
                if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) up += 1.0f;
                if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) up -= 1.0f;
                fast = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
            }
            {
                glm::vec3 dEye, dFwd;
                if (g_driverPos >= 0 && drivercam::eyePose(*vehicle, g_driverPos, dEye, dFwd)) {
                    // Seated at the driver's position; the mouse free-looks relative to
                    // the train's heading, so it doesn't drift on curves.
                    const float fYaw = std::atan2(dFwd.y, dFwd.x);
                    const float fPitch = std::asin(glm::clamp(dFwd.z, -1.0f, 1.0f));
                    g_camera.setPose(dEye, fYaw + g_driverYaw,
                                     glm::clamp(fPitch + g_driverPitch, -1.4f, 1.4f));
                } else if (g_chase) {
                    if (g_driverPos >= 0) g_driverPos = -1; // no cab here; fall back
                    const VehicleFrame vp = vehicle->frame();
                    const glm::vec3 axle = vp.pos + vp.up * wheelset::kAxleCentreAboveBed;
                    const glm::vec3 camPos = axle - vp.tangent * 8.0f + vp.up * 3.0f;
                    const glm::vec3 dir = glm::normalize(axle - camPos);
                    g_camera.setPose(camPos, std::atan2(dir.y, dir.x),
                                     std::asin(glm::clamp(dir.z, -1.0f, 1.0f)));
                } else if (!g_mapMode) {
                    if (g_driverPos >= 0) g_driverPos = -1; // no cab here; fall back
                    g_camera.move(fwd, right, up, dt, fast);
                }
            }
        }

        const float aspect = static_cast<float>(fbw) / static_cast<float>(fbh);
        // Flashing lamps (an entry signal's danger) blink from the push constant rather
        // than by rebuilding the signal mesh: in the editor those vertices share a buffer
        // with 32k buildings, so a rebuild twice a second is out of the question there.
        PushConstants pc{};
        pc.params.y = blinkClock(now); // a clock; each lamp blinks on its own period
        if (g_mapMode) {
            pc.viewProj = mapOrtho(aspect); // top-down ortho, centred + zoomed + panned
        } else {
            pc.viewProj = g_camera.projMatrix(aspect) * g_camera.viewMatrix();
        }
        renderer.setMapMode(g_mapMode);
        // .w channels carry the elevation range for the colour ramp.
        pc.sunDir = glm::vec4(sunDir, elevRange.x);
        pc.camPos = glm::vec4(g_camera.position(), elevRange.y);

        if (shotPath) {
            // EBANER_SHOTFRAME delays the capture, so a scripted run can wait for the
            // streamer to have built the world it was pointed at.
            static const int shotAt = std::getenv("EBANER_SHOTFRAME")
                                          ? std::atoi(std::getenv("EBANER_SHOTFRAME"))
                                          : 20;
            if (frame == shotAt) renderer.requestCapture(shotPath);
            if (frame == shotAt + 4) glfwSetWindowShouldClose(window, GLFW_TRUE);
        }
        renderer.drawFrame(pc);
        ++frame;
        if (profile) {
            pFrame.add(now_ms() - frameT0);
            if (glfwGetTime() >= profileUntil) {
                profileUntil = glfwGetTime() + 1.0;
                auto line = [](const char* what, const Stat& st) {
                    if (st.n == 0) return;
                    std::printf("  %-18s %4ld call(s)  worst %7.2f ms  mean %6.2f\n", what,
                                st.n, st.worst, st.total / static_cast<double>(st.n));
                };
                std::printf("[Profile] over the last second:\n");
                line("frame", pFrame);
                line("signal aspects", pAspects);
                line("distant walks", pDistants);
                line("mesh rebuild", pMesh);
                line("mesh upload", pUpload);
                line("map overlay", pMap);
                std::fflush(stdout);
                pFrame.reset(); pAspects.reset(); pDistants.reset();
                pMesh.reset(); pUpload.reset(); pMap.reset();
            }
        }
    }

    if (profile) {
        // Whatever has not been reported yet, so a short headless run still says something.
        auto line = [](const char* what, const Stat& st) {
            if (st.n == 0) return;
            std::printf("  %-18s %4ld call(s)  worst %7.2f ms  mean %6.2f\n", what, st.n,
                        st.worst, st.total / static_cast<double>(st.n));
        };
        std::printf("[Profile] at exit:\n");
        std::printf("  mesh parts (total ms): signals %.1f  crossings %.1f  concat %.1f  "
                    "flags %.1f  avalanche %.1f  txp+rest %.1f\n",
                    mSignals, mCross, mConcat, mFlag, mAval, mTxp);
        line("frame", pFrame);
        line("signal aspects", pAspects);
        line("distant walks", pDistants);
        line("mesh rebuild", pMesh);
        line("mesh upload", pUpload);
        line("map overlay", pMap);
        std::fflush(stdout);
    }
    renderer.waitIdle();
    renderer.cleanup();
    g_renderer = nullptr;
    g_audio = nullptr;
    audio.shutdown();
    script.shutdown();
    glfwDestroyWindow(window);
    glfwTerminate();
    return EXIT_SUCCESS;
}
