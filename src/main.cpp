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
#include "SwitchMesh.h"
#include "SwitchNetwork.h"
#include "TerrainData.h"
#include "TerrainMesh.h"
#include "Textures.h"
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

void cursorCallback(GLFWwindow*, double x, double y) {
    if (g_menuOpen) return; // menu open: freeze mouselook
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

void keyCallback(GLFWwindow* win, int key, int, int action, int) {
    // Escape toggles the menu overlay; while it is open the other hotkeys are inert.
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) g_menuOpen = !g_menuOpen;
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
    try {
        data.load(datasetRoot);
        paths = buildTrackPaths(data);
        mesh.build(data);
        tracks.build(paths);
        roads.build(data);
        buildings.build(data);
        platforms.build(data, paths);
        switchNet.build(data, paths);   // turnout detection + routing
        switches.build(switchNet);
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
    // static struct bucket — attached just after renderer.init below.

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
    const std::vector<std::string> kMenuItems = {"Exit"};

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

        if (g_menuOpen) {
            // --- Escape menu (pauses the sim) ---
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
            float pushInput = 0.0f;
            if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) pushInput += 1.0f;
            if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) pushInput -= 1.0f;

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
            const bool rF = down(GLFW_KEY_F), rN = down(GLFW_KEY_N),
                       rR = down(GLFW_KEY_R);
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
            // in front and within a small cone. T throws it.
            int aimedSwitch = -1;
            {
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
                if (aimedSwitch >= 0) {
                    switchNet.toggle(aimedSwitch);
                    switches.build(switchNet);
                    renderer.updateSwitches(switches.vertices(), switches.indices());
                    std::printf("[Switch] %d -> %s\n", aimedSwitch,
                                switchNet.state(aimedSwitch) == SwitchState::Straight
                                    ? "straight" : "diverging");
                }
            }

            // HUD: speed, reservoir/brake pressures and the brake notch.
            {
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
                    std::snprintf(buf, sizeof(buf), "SWITCH %s   T to throw", sn);
                    appendText(tv, buf, fbw * 0.5f - 66.0f * sc, fbh * 0.5f + 14.0f * sc,
                               sc,
                               broken ? glm::vec3(1.0f, 0.5f, 0.3f) : glm::vec3(0.6f, 1.0f, 0.8f),
                               fbw, fbh);
                }
                renderer.setOverlayText(tv);
            }

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

        const float aspect = static_cast<float>(fbw) / static_cast<float>(fbh);
        PushConstants pc{};
        pc.viewProj = g_camera.projMatrix(aspect) * g_camera.viewMatrix();
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
