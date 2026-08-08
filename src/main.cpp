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
#include "SignalPaths.h"
#include "SwitchMesh.h"
#include "SwitchNetwork.h"
#include "SwitchTypes.h"
#include "TerrainData.h"
#include "TerrainMesh.h"
#include "Textures.h"
#include "TrackCircuits.h"
#include "TrackGraph.h"
#include "TrackMesh.h"
#include "TrackPath.h"
#include "Audio.h"
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
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <optional>
#include <string>
#include <vector>

namespace {

Camera g_camera;
double g_lastX = 0.0, g_lastY = 0.0;
bool g_firstMouse = true;
bool g_mouseCaptured = true;
bool g_chase = false; // chase-cam mode (ride the rail vehicle)
int g_driverPos = -1; // driver camera: -1 off, else cab index (0 front, 1 rear)
float g_driverYaw = 0.0f, g_driverPitch = 0.0f; // look offsets relative to the train
constexpr float kLookSens = 0.0022f; // radians per pixel (matches Camera)
Audio* g_audio = nullptr; // for the M mute toggle in the key callback
bool g_throwSwitch = false; // T pressed: throw the switch under the crosshair
bool g_menuOpen = false;    // Escape menu overlay (pauses the sim)
int g_menuSel = 0;          // highlighted menu item
bool g_mapMode = false;     // traffic-manager 2-D map view
bool g_mapDirty = false;    // (re)build the map overlay this frame
bool g_routePick = false;   // traffic manager: the exit-route picker is open
int g_routePickSel = 0;     // highlighted route in that picker
float g_mapZoom = 1.0f;     // map zoom: 1 = ~4 km tall, higher = zoomed in
constexpr float kMapZoomMin = 0.5f;  // ~8 km tall (zoomed out)
constexpr float kMapZoomMax = 40.0f; // ~100 m tall (zoomed in)
glm::vec2 g_mapPan(0.0f);   // WASD pan offset from the default centre (scene metres)
std::string g_mapMsg;       // transient map feedback (e.g. a blocked-throw reason)
double g_mapMsgUntil = 0.0; // glfwGetTime() until which g_mapMsg is shown

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
        if (g_routePick) g_routePick = false;
        else g_menuOpen = !g_menuOpen;
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
    if (key == GLFW_KEY_V && action == GLFW_PRESS) {
        g_driverPos = (g_driverPos + 2) % 3 - 1; // -1 -> 0 -> 1 -> -1
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
        g_routePick = !g_routePick;
        g_routePickSel = 0;
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
    if (const char* dump = std::getenv("EBANER_AUDIO_DUMP_ENGINE")) {
        Audio::dumpEngineTest(dump);
        return EXIT_SUCCESS;
    }

    // --- Load terrain data ---
    TerrainData data;
    TerrainMesh mesh;
    TrackMesh tracks;
    RoadMesh roads;
    BuildingMesh buildings;
    PlatformMesh platforms;
    SwitchMesh switches;
    SwitchNetwork switchNet;
    std::vector<TrackPath> paths;
    TrackGraph graph; // raw track lines (scene-relative), for the 2-D traffic-manager map
    try {
        data.load(datasetRoot);
        paths = buildTrackPaths(data);
        mesh.build(data);
        tracks.build(paths);
        roads.build(data);
        buildings.build(data);
        platforms.build(data, paths);
        switchNet.build(data, paths);   // turnout detection + routing
        applySwitchTypes(switchNet, loadSwitchTypes(datasetRoot)); // manual/motor overrides
        switches.build(switchNet);
        graph = buildTrackGraph(data);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Failed to load terrain: %s\n", e.what());
        return EXIT_FAILURE;
    }


    // --- First rail vehicle: a wheelset on the main line near the start ---
    const TrackPath* vpath = nullptr;
    float vs = 0.0f;
    {
        float best = 1e30f;
        for (const TrackPath& p : paths) {
            if (p.trackType() != 0) continue; // main line only
            for (float s = 0.0f; s <= p.length(); s += 5.0f) {
                const glm::vec3 q = p.poseAt(s).pos; // nearest point to scene origin
                const float d2 = q.x * q.x + q.y * q.y;
                if (d2 < best) { best = d2; vpath = &p; vs = s; }
            }
        }
        if (!vpath && !paths.empty()) vpath = &paths[0];
    }

    // The vehicle is chosen on the start screen; created (attached) on confirm.
    std::optional<Vehicle> vehicle;
    VehicleMesh vmesh;
    Audio audio;
    g_audio = &audio; // init() is deferred until just before the render loop

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
    // Switch stands go in a dynamic buffer (rebuilt when a switch is thrown), not the
    // static struct bucket — attached just after renderer.init below. The ground signals
    // are dynamic too (their lamps follow the aspect), attached once `polys` exists.

    // --- Renderer ---
    VulkanRenderer renderer;
    g_renderer = &renderer;
    try {
        renderer.init(window, mesh.vertices(), mesh.indices(), texData,
                      tracks.vertices(), tracks.indices(),
                      tracks.alwaysIndexCount(), tracks.sleeperChunks(),
                      roads.vertices(), roads.indices(),
                      structVerts, structIndices);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Vulkan init failed: %s\n", e.what());
        glfwDestroyWindow(window);
        glfwTerminate();
        return EXIT_FAILURE;
    }
    renderer.attachSwitches(switches.vertices(), switches.indices());

    std::printf(
        "\nControls: WASD move, Q/E down/up, mouse look, Shift boost, "
        "C chase vehicle, V driver view (switch cab), I engines start/stop, "
        "Up/Down push vehicle, , / . power/brake lever, Space emergency, "
        "F/N/R reverser, T throw aimed switch, M mute, Tab release cursor, "
        "Esc menu\n\n");

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
            0.5f * (sp.bogieSpacing + sp.wheelbase);
        float startS = vs;
        const float L = vpath->length(), margin = outerHalf + 1.0f;
        if (L > 2.0f * margin) startS = std::clamp(vs, margin, L - margin);
        vehicle.emplace(vpath, sp, startS);
        vehicle->attachNetwork(&paths, &switchNet); // divert at switches
        vmesh.build(*vehicle);
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

    // Where the map is centred (scene-relative): the centroid of the routed turnouts -
    // the Bodo station throat, which is the operational detail the view is about, so
    // zooming stays framed on it. Falls back to the scene origin if there are none.
    glm::vec2 mapCenter(0.0f);
    {
        const glm::dvec3 org = switchNet.sceneOrigin();
        const auto& tos = switchNet.turnouts();
        glm::dvec2 sum(0.0);
        int n = 0;
        for (const auto& t : tos) {
            if (t.mainPath < 0) continue; // inert crossing: not a working switch
            sum += glm::dvec2(t.world.x - org.x, t.world.y - org.y);
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
    std::vector<SignalPlacement> sigPlacements =
        mergeSignals(signalPlacements(signalPaths, polys), mainPlacements);
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
    SignalMesh signals;
    signals.build(sigPlacements, data.sceneOrigin());
    renderer.attachSignals(signals.vertices(), signals.indices());

    // Route setting (traffic manager): a set route holds its switches and shows its signal
    // clear. It drops as soon as a train enters its circuits (the lock lifts then too; the
    // per-switch occupancy lock guards them from there).
    std::vector<char> routeSet(signalPaths.size(), 0);
    int routeArm = -1; // placement armed by a first click, awaiting its destination

    // --- Main-signal routes ---------------------------------------------------------------
    // Everything a main signal can be asked to authorise, exit or entry alike, resolved once
    // into one list so the interlocking never has to ask which kind it is holding.
    //
    // An exit signal's authority begins back at the platform, so its movement is the exit
    // route joined to the signal's own route beyond. An entry signal's begins at the mast,
    // so its record is already the whole movement - and every one of its circuits is past
    // the signal.
    const std::vector<SignalPath> exitRoutes = loadExitRoutes(datasetRoot);
    struct MainCandidate {
        std::string name;
        RouteType type = RouteType::C1;
        int placement = -1;      // the mast to light
        int station = -1;
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
            c.anchor = sceneAt(c.departure.start); // the platform end
            mainCandidates.push_back(std::move(c));
        }
        for (std::size_t ei = 0; ei < entrySignals.size(); ++ei) {
            MainCandidate c;
            c.name = named(entrySignals[ei], "E");
            c.type = entrySignals[ei].type;
            c.placement = mastOf(SignalKind::Entry, static_cast<int>(ei));
            c.departure = entrySignals[ei];
            c.beyond = pathSections(c.departure, circuits); // all of it is past the signal
            c.anchor = sceneAt(c.departure.end);            // the platform end
            mainCandidates.push_back(std::move(c));
        }
    }
    // Stations: routes whose in-station ends lie within kStationSpan of one another work one
    // place, so the traffic manager can offer just that place's routes. Anchoring on the
    // platform end rather than on the mast matters - an entry signal can stand a kilometre
    // outside the station it serves, and would otherwise cluster as a station of its own.
    std::vector<glm::vec2> stationAt; // station -> scene-relative centre
    {
        constexpr double kStationSpan = 800.0; // m
        for (std::size_t a = 0; a < mainCandidates.size(); ++a) {
            if (mainCandidates[a].station >= 0) continue;
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
                    if (mainCandidates[o].station >= 0) continue;
                    if (glm::length(mainCandidates[o].anchor - mainCandidates[c].anchor) >
                        kStationSpan)
                        continue;
                    mainCandidates[o].station = st;
                    queue.push_back(o);
                }
            }
            stationAt.push_back(sum / static_cast<float>(std::max(n, 1)));
        }
        std::printf("[Route] %zu exit route(s), %zu entry route(s) -> %zu main route(s) "
                    "in %zu station(s)\n", exitRoutes.size(), entrySignals.size(),
                    mainCandidates.size(), stationAt.size());
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
                const glm::dvec3 a = fracToWorld(polys, iv.trackId, iv.from);
                if (a.x == 0.0 && a.y == 0.0) continue; // track gone (stale overlay)
                SecRun run;
                run.section = static_cast<int>(si);
                constexpr int kSteps = 32;
                for (int k = 0; k <= kSteps; ++k) {
                    const double f = iv.from + (iv.to - iv.from) * k / kSteps;
                    const glm::dvec3 w = fracToWorld(polys, iv.trackId, f);
                    run.pts.push_back(glm::vec2(w.x - org.x, w.y - org.y));
                }
                if (run.pts.size() >= 2) secRuns.push_back(std::move(run));
            }
        }
    }
    std::vector<char> secOccupied(circuits.sections.size(), 0);

    // Resolve each motor switch's locking set now that circuits + polys exist (from the
    // authored overlay, else the circuits the switch sits within). Gates remote throws.
    applySwitchLocks(switchNet, loadSwitchTypes(datasetRoot), circuits, polys);

    // Squared planar distance from p to segment ab (for occupancy tests).
    auto pointSegDist2 = [](glm::vec2 p, glm::vec2 a, glm::vec2 b) {
        const glm::vec2 ab = b - a;
        const float L2 = glm::dot(ab, ab);
        const float t = L2 > 1e-6f ? glm::clamp(glm::dot(p - a, ab) / L2, 0.0f, 1.0f) : 0.0f;
        const glm::vec2 c = a + ab * t;
        return glm::dot(p - c, p - c);
    };
    // Recompute which sections hold a wheelset. Tolerance keeps an axle on the right
    // track without bleeding onto a parallel one (track centres are >4 m apart).
    auto computeOccupancy = [&](std::vector<char>& occ) {
        std::fill(occ.begin(), occ.end(), 0);
        if (!vehicle) return;
        const std::vector<VehicleFrame> axles = vehicle->axleFrames();
        constexpr float kTol2 = 2.5f * 2.5f;
        for (const SecRun& run : secRuns) {
            if (occ[run.section]) continue;
            bool hit = false;
            for (std::size_t i = 1; i < run.pts.size() && !hit; ++i)
                for (const VehicleFrame& ax : axles)
                    if (pointSegDist2(glm::vec2(ax.pos), run.pts[i - 1], run.pts[i]) < kTol2) {
                        hit = true;
                        break;
                    }
            if (hit) occ[run.section] = 1;
        }
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

    auto buildMapOverlay = [&]() {
        std::vector<LineVertex> lines = graph.lines;
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
            if (sp.kind == SignalKind::Dwarf) continue; // main signals only
            const glm::vec2 f(float(sp.forward.x), float(sp.forward.y));
            const glm::vec2 r(f.y, -f.x); // right of travel: the side it stands on
            const glm::vec2 b(float(sp.world.x - org.x) + r.x * 8.0f,
                              float(sp.world.y - org.y) + r.y * 8.0f);
            const glm::vec3 col =
                sp.aspect == SignalAspect::Clear          ? glm::vec3(0.2f, 1.0f, 0.3f)
                : sp.aspect == SignalAspect::ClearReduced ? glm::vec3(0.6f, 1.0f, 0.2f)
                                                          : glm::vec3(1.0f, 0.2f, 0.15f);
            const float h = 6.0f;
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
        renderer.attachTrackGraph(lines, points);
    };

    // The map-mode HUD: title, colour legend, and controls. When a vehicle is given
    // (the sim runs live under the map) its speed is shown so the motion is visible.
    auto appendMapHud = [&](std::vector<TextVertex>& tv, int fbw, int fbh,
                            const Vehicle* veh) {
        const float sc = std::max(2.0f, static_cast<float>(fbh) / 240.0f);
        const float x = 40.0f, lh = 12.0f * sc;
        float y = 40.0f;
        appendText(tv, "TRAFFIC MANAGER - BODO", x, y, sc,
                   glm::vec3(1.0f, 0.95f, 0.5f), fbw, fbh);
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
                      "O: cab  Esc: menu  scroll/Z-X: zoom  WASD: pan  click switch to throw  "
                      "click signal then destination to set a route  R: exit routes  "
                      "(view %.1f km)",
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
            const bool ok = g_mapMsg.rfind("Switch thrown", 0) == 0;
            appendText(tv, g_mapMsg, x, y, sc,
                       ok ? glm::vec3(0.5f, 1.0f, 0.6f) : glm::vec3(1.0f, 0.55f, 0.4f),
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
    bool prevUp = false, prevDown = false, prevK1 = false, prevK2 = false,
         prevK3 = false, prevK4 = false, prevK5 = false, prevEnter = false;
    bool prevBrkDown = false, prevBrkUp = false, prevBrkEmerg = false;
    bool prevSafety = false, prevEngine = false;
    bool prevRevF = false, prevRevN = false, prevRevR = false;
    bool prevMenuEnter = false, prevMenuUp = false, prevMenuDown = false;
    bool mapAttached = false; // whether the map overlay is currently attached
    bool switchesChanged = true; // a switch moved: re-evaluate the signal aspects
    bool prevMapClick = false; // edge-trigger for the map left-click
    bool prevPickUp = false, prevPickDown = false, prevPickEnter = false;
    const std::vector<std::string> kMenuItems = {"Traffic manager", "Exit"};

    auto setMapMsg = [&](const std::string& m) {
        g_mapMsg = m; g_mapMsgUntil = glfwGetTime() + 3.0;
    };
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
        switches.build(switchNet);
        renderer.updateSwitches(switches.vertices(), switches.indices());
        g_mapDirty = true;
        switchesChanged = true;
        setMsg(std::string("Switch thrown -> ") +
               (switchNet.state(i) == SwitchState::Straight ? "straight" : "diverging"));
    };

    // Set a route: move its switches into position, lock the path and clear its signal.
    // Everything is validated before anything moves, so a refused route changes nothing.
    auto trySetRoute = [&](int pi) {
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
            switches.build(switchNet);
            renderer.updateSwitches(switches.vertices(), switches.indices());
        }
        routeSet[pi] = 1;
        switchesChanged = true;
        g_mapDirty = true;
        setMapMsg("Route " + pathName(pi) + " set" +
                  (toMove.empty() ? "" : " (" + std::to_string(toMove.size()) + " switch(es) moved)"));
    };
    auto cancelRoute = [&](int pi) {
        routeSet[pi] = 0;
        switchesChanged = true;
        g_mapDirty = true;
        setMapMsg("Route " + pathName(pi) + " cancelled");
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
        std::string occ;
        for (int id : pathSections(dep, circuits))
            if (secOccupiedById(id)) occ += (occ.empty() ? "" : ", ") + secName(id);
        if (!occ.empty()) return {"occupied", "occupied: " + occ};
        // A circuit already held by another departure is the conflicting-route case: two
        // main signals must never both authorise a movement over the same track.
        for (const MainRoute& mr : mainRoutes) {
            if (mr.route == ri) continue;
            for (int id : pathSections(dep, circuits))
                if (std::find(mr.locked.begin(), mr.locked.end(), id) != mr.locked.end())
                    return {"conflict", "conflicts with " + exitRouteName(mr.route)};
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
            switches.build(switchNet);
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
        switchesChanged = true;
        g_mapDirty = true;
        setMapMsg("Route " + exitRouteName(ri) + " cancelled");
        std::printf("[Main] %s cancelled\n", exitRouteName(ri).c_str());
    };

    // The exit routes on offer: those of the station nearest what the map is looking at,
    // so a dispatcher panning to a place gets that place's routes.
    auto stationRoutes = [&]() {
        std::vector<int> out;
        if (stationAt.empty()) return out;
        const glm::vec2 at = mapCenter + g_mapPan;
        int best = 0;
        float bestD = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < stationAt.size(); ++i) {
            const float d = glm::length(stationAt[i] - at);
            if (d < bestD) { bestD = d; best = static_cast<int>(i); }
        }
        for (std::size_t ri = 0; ri < mainCandidates.size(); ++ri)
            if (mainCandidates[ri].station == best) out.push_back(static_cast<int>(ri));
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
    // The picker panel, drawn after the map HUD so it sits over the map.
    auto appendRoutePicker = [&](std::vector<TextVertex>& tv, int fbw, int fbh) {
        if (!g_routePick) return;
        const std::vector<int> rs = stationRoutes();
        appendMenu(tv, "SET ROUTE  (Up/Down, Enter, Esc)", routePickItems(rs),
                   rs.empty() ? 0
                              : std::clamp(g_routePickSel, 0,
                                           static_cast<int>(rs.size()) - 1),
                   fbw, fbh);
    };

    // Open the audio device now that the heavy startup work is done, so the audio
    // thread isn't starved (which causes ALSA under-runs) during loading.
    audio.init();

    double lastTime = glfwGetTime();
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        const double now = glfwGetTime();
        const float dt = static_cast<float>(now - lastTime);
        lastTime = now;

        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(window, &fbw, &fbh);
        if (fbw == 0 || fbh == 0) { continue; } // minimised

        // Live track-circuit occupancy: recomputed every frame (the ground signals read it
        // in the cab view, not just the map). The overlay/aspects are rebuilt only when it
        // changes - a train entering or leaving a section.
        bool occupancyChanged = false;
        {
            std::vector<char> occ(circuits.sections.size(), 0);
            computeOccupancy(occ);
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
                if (!routeSet[pi] || occupiedIn(static_cast<int>(pi)).empty()) continue;
                routeSet[pi] = 0;
                switchesChanged = true;
                g_mapDirty = true;
                std::printf("[Route] %s released (train entered)\n",
                            pathName(static_cast<int>(pi)).c_str());
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
        if (occupancyChanged || switchesChanged) {
            switchesChanged = false;
            // What each main signal shows: danger unless a route it governs is set and has
            // not yet been entered, then C1 (two greens) or C2 (one green) per its type.
            std::vector<SignalAspect> exitAspects(sigPlacements.size(), SignalAspect::Stop);
            for (const MainRoute& mr : mainRoutes) {
                if (!mr.signalClear || mr.placement < 0) continue;
                exitAspects[mr.placement] = mainCandidates[mr.route].type == RouteType::C2
                                                ? SignalAspect::ClearReduced
                                                : SignalAspect::Clear;
            }
            if (updateSignalAspects(sigPlacements, signalPaths, switchNet, polys, circuits,
                                    secOccupied, routeSet, exitAspects)) {
                signals.build(sigPlacements, data.sceneOrigin());
                renderer.updateSignals(signals.vertices(), signals.indices());
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
            buildMapOverlay();
            mapAttached = true;
            g_mapDirty = false;
        } else if (!g_mapMode && mapAttached) {
            renderer.attachTrackGraph({}, {});
            mapAttached = false;
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

        // Route picker: Up/Down move, Enter sets (or cancels a set route), Esc closes. The
        // sim keeps running underneath - this is not the Escape menu and must not pause it.
        if (g_mapMode && !g_menuOpen && g_routePick) {
            auto down = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
            const bool pU = down(GLFW_KEY_UP), pD = down(GLFW_KEY_DOWN),
                       pE = down(GLFW_KEY_ENTER);
            const std::vector<int> rs = stationRoutes();
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
            prevPickUp = pU; prevPickDown = pD; prevPickEnter = pE;
        } else {
            prevPickUp = prevPickDown = prevPickEnter = false;
        }

        if (g_menuOpen) {
            // --- Escape menu (pauses the sim) ---
            auto down = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
            const bool mU = down(GLFW_KEY_UP), mD = down(GLFW_KEY_DOWN),
                       mE = down(GLFW_KEY_ENTER);
            const int n = static_cast<int>(kMenuItems.size());
            if (mU && !prevMenuUp) g_menuSel = (g_menuSel + n - 1) % n;
            if (mD && !prevMenuDown) g_menuSel = (g_menuSel + 1) % n;
            if (mE && !prevMenuEnter) {
                const std::string& sel = kMenuItems[g_menuSel];
                if (sel == "Exit") glfwSetWindowShouldClose(window, GLFW_TRUE);
                else if (sel == "Traffic manager") {
                    g_mapMode = true; g_mapDirty = true; g_menuOpen = false;
                    g_mapPan = glm::vec2(0.0f); // start centred on the throat
                    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL); // click switches
                    g_firstMouse = true;
                }
            }
            prevMenuUp = mU; prevMenuDown = mD; prevMenuEnter = mE;
            std::vector<TextVertex> tv;
            appendMenu(tv, "MENU", kMenuItems, g_menuSel, fbw, fbh);
            renderer.setOverlayText(tv);
        } else if (g_mapMode && !vehicle) {
            // --- Traffic-manager 2-D map with no vehicle spawned (entered from the
            // start screen). Nothing to simulate; just draw the map + HUD. When a
            // vehicle exists the sim runs live in the Sim branch below. ---
            std::vector<TextVertex> tv;
            appendMapHud(tv, fbw, fbh, nullptr);
            appendRoutePicker(tv, fbw, fbh);
            renderer.setOverlayText(tv);
        } else if (mode == Mode::Menu) {
            // --- Start screen: pick a vehicle ---
            auto down = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
            const bool kUp = down(GLFW_KEY_UP), kDn = down(GLFW_KEY_DOWN);
            const bool k1 = down(GLFW_KEY_1), k2 = down(GLFW_KEY_2),
                       k3 = down(GLFW_KEY_3), k4 = down(GLFW_KEY_4),
                       k5 = down(GLFW_KEY_5);
            const bool kEnt = down(GLFW_KEY_ENTER);
            if (kUp && !prevUp)
                menuIndex = (menuIndex + kNumVehicleSpecs - 1) % kNumVehicleSpecs;
            if (kDn && !prevDown) menuIndex = (menuIndex + 1) % kNumVehicleSpecs;
            if (k1 && !prevK1) menuIndex = 0;
            if (k2 && !prevK2 && kNumVehicleSpecs > 1) menuIndex = 1;
            if (k3 && !prevK3 && kNumVehicleSpecs > 2) menuIndex = 2;
            if (k4 && !prevK4 && kNumVehicleSpecs > 3) menuIndex = 3;
            if (k5 && !prevK5 && kNumVehicleSpecs > 4) menuIndex = 4;
            const bool confirm = kEnt && !prevEnter;
            prevUp = kUp; prevDown = kDn; prevK1 = k1; prevK2 = k2; prevK3 = k3;
            prevK4 = k4; prevK5 = k5; prevEnter = kEnt;

            if (confirm) {
                spawnVehicle(menuIndex);
                renderer.setOverlayText({});
                mode = Mode::Sim;
            } else {
                std::vector<TextVertex> tv;
                const float sc = std::max(2.0f, static_cast<float>(fbh) / 240.0f);
                const float x = 40.0f, lh = 12.0f * sc;
                // Dark backing panel for contrast over the terrain.
                {
                    auto ndc = [&](float px, float py) {
                        return glm::vec2(px / fbw * 2.0f - 1.0f, py / fbh * 2.0f - 1.0f);
                    };
                    const float x1 =
                        std::min(static_cast<float>(fbw) - 20.0f, 40.0f + 30.0f * 8.0f * sc);
                    const float y1 = 40.0f + (kNumVehicleSpecs + 4) * lh;
                    const glm::vec3 pc(0.04f, 0.05f, 0.09f);
                    const glm::vec2 a = ndc(20.0f, 20.0f), b = ndc(x1, 20.0f),
                                    c = ndc(x1, y1), d = ndc(20.0f, y1);
                    tv.push_back({a, pc}); tv.push_back({b, pc}); tv.push_back({c, pc});
                    tv.push_back({a, pc}); tv.push_back({c, pc}); tv.push_back({d, pc});
                }
                appendText(tv, "SELECT VEHICLE", x, 40.0f, sc,
                           glm::vec3(1.0f, 0.95f, 0.5f), fbw, fbh);
                for (int i = 0; i < kNumVehicleSpecs; ++i) {
                    const bool hi = (i == menuIndex);
                    std::string line = (hi ? "> " : "  ");
                    line += std::to_string(i + 1) + ". " + kVehicleSpecs[i].name;
                    appendText(tv, line, x, 40.0f + (i + 2) * lh, sc,
                               hi ? glm::vec3(1.0f) : glm::vec3(0.6f, 0.6f, 0.65f),
                               fbw, fbh);
                }
                appendText(tv, "UP/DOWN OR 1-5 TO CHOOSE, ENTER TO START", x,
                           40.0f + (kNumVehicleSpecs + 3) * lh, sc * 0.75f,
                           glm::vec3(0.7f, 0.8f, 0.9f), fbw, fbh);
                renderer.setOverlayText(tv);
            }
        } else if (vehicle) {
            // --- Sim: hand push + physics + camera ---
            // Up/Down hand-push the vehicle, except while the picker has those keys.
            float pushInput = 0.0f;
            if (!g_routePick) {
                if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) pushInput += 1.0f;
                if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) pushInput -= 1.0f;
            }

            // Combined power/brake lever: ',' steps toward power (N -> P1..P5), '.'
            // toward brake (N -> B1..B4 -> Emergency), Space slams to emergency
            // (edge-triggered so each press is one notch). These keys are the same
            // physical position on any layout (unlike [ ] \, which are AltGr
            // combinations on e.g. Norwegian keyboards).
            auto down = [&](int k) { return glfwGetKey(window, k) == GLFW_PRESS; };
            // The keyboard drives the cab you are sitting in (driver view V), or
            // the front cab (0) in any other view.
            const int cab = (g_driverPos >= 0) ? g_driverPos : 0;
            const bool bD = down(GLFW_KEY_COMMA), bU = down(GLFW_KEY_PERIOD),
                       bE = down(GLFW_KEY_SPACE);
            const int prevPos = vehicle->handlePosition(cab);
            if (bD && !prevBrkDown) vehicle->moveHandle(cab, -1); // toward power
            if (bU && !prevBrkUp) vehicle->moveHandle(cab, +1);   // toward brake
            if (bE && !prevBrkEmerg) {
                vehicle->setPowerNotch(cab, 0);
                vehicle->setBrakeNotch(cab, Vehicle::kEmergencyNotch);
            }
            prevBrkDown = bD; prevBrkUp = bU; prevBrkEmerg = bE;
            if (vehicle->handlePosition(cab) != prevPos) {
                std::printf("[Handle] cab %d %s  MR %.1f  BC %.1f bar\n", cab,
                            vehicle->handleName(cab), vehicle->mrPressure(),
                            vehicle->bcPressure());
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
            vehicle->update(simDt, pushInput);
            if (vehicle->consumeSwitchChanged()) { // a switch was forced/broken
                switches.build(switchNet);
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
            for (const VehicleFrame& b : vehicle->bogieFrames())
                distGain = std::max(distGain,
                                    glm::clamp((60.0f - glm::distance(camPos, b.pos)) / 55.0f,
                                               0.0f, 1.0f));
            float engGain[2] = {0.0f, 0.0f};
            const std::vector<VehicleFrame> secs = vehicle->bodySectionFrames();
            for (int k = 0; k < 2 && k < static_cast<int>(secs.size()); ++k)
                engGain[k] = glm::clamp((50.0f - glm::distance(camPos, secs[k].pos)) / 38.0f,
                                        0.0f, 1.0f);
            audio.update(*vehicle, simDt, distGain, engGain[0], engGain[1]);
            vmesh.build(*vehicle);
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
                    switches.build(switchNet);
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
                appendMapHud(tv, fbw, fbh, &*vehicle);
                appendRoutePicker(tv, fbw, fbh);
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
                std::snprintf(buf, sizeof(buf), "MR %.1f bar   BC %.1f bar",
                              vehicle->mrPressure(), vehicle->bcPressure());
                appendText(tv, buf, x, y, sc, glm::vec3(0.8f, 0.9f, 1.0f), fbw, fbh);
                y += lh;
                std::snprintf(buf, sizeof(buf), "REV %s (cab %d)   F / N / R",
                              vehicle->reverserName(cab), cab);
                appendText(tv, buf, x, y, sc, glm::vec3(0.85f, 0.85f, 0.7f), fbw, fbh);
                y += lh;
                std::snprintf(buf, sizeof(buf), "HANDLE %s   , power / brake . / Space",
                              vehicle->handleName(cab));
                const bool emerg = vehicle->brakeNotch(cab) >= Vehicle::kEmergencyNotch;
                appendText(tv, buf, x, y, sc,
                           emerg ? glm::vec3(1.0f, 0.4f, 0.35f) : glm::vec3(0.7f, 0.85f, 0.7f),
                           fbw, fbh);
                y += lh;
                if (vehicle->interlockEmergency()) {
                    appendText(tv, "!! REVERSER INTERLOCK - AUTO EMERGENCY !!", x, y,
                               sc, glm::vec3(1.0f, 0.35f, 0.3f), fbw, fbh);
                    y += lh;
                }
                if (vehicle->safetyBrakeActive()) {
                    appendText(tv, "!! LOW RESERVOIR - AUTO EMERGENCY !!", x, y,
                               sc, glm::vec3(1.0f, 0.35f, 0.3f), fbw, fbh);
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
            if (!g_mapMode) {
                float fwd = 0.0f, right = 0.0f, up = 0.0f;
                if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) fwd += 1.0f;
                if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) fwd -= 1.0f;
                if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) right += 1.0f;
                if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) right -= 1.0f;
                if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) up += 1.0f;
                if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) up -= 1.0f;
                const bool fast = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
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
                } else {
                    if (g_driverPos >= 0) g_driverPos = -1; // no cab here; fall back
                    g_camera.move(fwd, right, up, dt, fast);
                }
            }
        }

        const float aspect = static_cast<float>(fbw) / static_cast<float>(fbh);
        // Flashing lamps (an entry signal's danger) blink from the push constant rather
        // than by rebuilding the signal mesh: in the editor those vertices share a buffer
        // with 32k buildings, so a rebuild twice a second is out of the question there.
        constexpr double kBlinkPeriod = 1.0; // s, half lit
        PushConstants pc{};
        pc.params.y = std::fmod(now, kBlinkPeriod) < kBlinkPeriod * 0.5 ? 1.0f : 0.0f;
        if (g_mapMode) {
            pc.viewProj = mapOrtho(aspect); // top-down ortho, centred + zoomed + panned
        } else {
            pc.viewProj = g_camera.projMatrix(aspect) * g_camera.viewMatrix();
        }
        renderer.setMapMode(g_mapMode);
        // .w channels carry the elevation range for the colour ramp.
        pc.sunDir = glm::vec4(sunDir, data.minElevation());
        pc.camPos = glm::vec4(g_camera.position(), data.maxElevation());

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
    g_audio = nullptr;
    audio.shutdown();
    glfwDestroyWindow(window);
    glfwTerminate();
    return EXIT_SUCCESS;
}
