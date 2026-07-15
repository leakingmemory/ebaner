#include "Camera.h"
#include "TerrainData.h"
#include "BuildingMesh.h"
#include "RoadMesh.h"
#include "TerrainMesh.h"
#include "Textures.h"
#include "TrackMesh.h"
#include "TrackPath.h"
#include "Vehicle.h"
#include "VulkanRenderer.h"
#include "WheelsetMesh.h"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

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
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(win, GLFW_TRUE);
    }
    // Tab toggles mouse capture (handy for debugging).
    if (key == GLFW_KEY_TAB && action == GLFW_PRESS) {
        g_mouseCaptured = !g_mouseCaptured;
        glfwSetInputMode(win, GLFW_CURSOR,
                         g_mouseCaptured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
        g_firstMouse = true;
    }
    // C toggles the chase camera (ride the rail vehicle).
    if (key == GLFW_KEY_C && action == GLFW_PRESS) g_chase = !g_chase;
}

VulkanRenderer* g_renderer = nullptr;
void resizeCallback(GLFWwindow*, int, int) {
    if (g_renderer) g_renderer->notifyResize();
}

} // namespace

int main(int argc, char** argv) {
    const std::string datasetRoot = (argc > 1) ? argv[1] : "../norway-rails";

    // --- Load terrain data ---
    TerrainData data;
    TerrainMesh mesh;
    TrackMesh tracks;
    RoadMesh roads;
    BuildingMesh buildings;
    std::vector<TrackPath> paths;
    try {
        data.load(datasetRoot);
        paths = buildTrackPaths(data);
        mesh.build(data);
        tracks.build(paths);
        roads.build(data);
        buildings.build(data);
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

    // Give the vehicle mass + bounding-box dimensions. Qualified guesses for a
    // bare railway wheelset (two wheels + axle, nothing attached): ~1.3 t; ~0.2 m
    // along travel, ~2.2 m axle across, ~0.92 m wheel diameter tall.
    constexpr float kWheelsetMass = 1300.0f;
    constexpr float kWheelsetLength = 0.20f;
    constexpr float kWheelsetWidth = 2.20f;
    constexpr float kWheelsetHeight = 0.92f;
    std::optional<Vehicle> vehicle;
    if (vpath)
        vehicle.emplace(vpath, vs, kWheelsetMass, kWheelsetLength, kWheelsetWidth,
                        kWheelsetHeight);

    WheelsetMesh wheelset;
    if (vehicle) wheelset.build(vehicle->pose());

    // Resolve gravity at the vehicle: along-track (drives acceleration) vs.
    // weight-on-rails (reacted by the rails; basis for future friction).
    if (vehicle) {
        const GravityResolution g = vehicle->gravity();
        float maxGradeDeg = 0.0f;
        for (float s = 0.0f; s <= vpath->length(); s += 10.0f) {
            const float gr = glm::degrees(std::asin(
                glm::clamp(vpath->poseAt(s).tangent.z, -1.0f, 1.0f)));
            if (std::abs(gr) > std::abs(maxGradeDeg)) maxGradeDeg = gr;
        }
        std::printf(
            "[Vehicle] mass %.0f kg; at s=%.0f grade %+.2f deg -> along-track "
            "accel %.3f m/s^2, weight on rails %.0f N; steepest grade on path "
            "%+.2f deg (accel %.3f m/s^2)\n",
            vehicle->mass(), vehicle->s(), glm::degrees(g.gradeRad),
            glm::length(g.alongTrackAccel), glm::length(g.weightOnRails),
            maxGradeDeg,
            9.81f * std::sin(glm::radians(std::abs(maxGradeDeg))));

        // Rotational inertia (box model) and the curve overturning limit.
        const glm::vec3 I = vehicle->inertia();
        std::printf("[Vehicle] dims LxWxH = %.2fx%.2fx%.2f m; inertia "
                    "(roll,pitch,yaw) = (%.0f, %.0f, %.0f) kg*m^2; CoM %.2f m "
                    "above rail\n",
                    vehicle->length(), vehicle->width(), vehicle->height(), I.x,
                    I.y, I.z, vehicle->comHeight());
        float kMax = 0.0f, cantAtMax = 0.0f;
        for (float s = 0.0f; s <= vpath->length(); s += 5.0f) {
            const TrackPose p = vpath->poseAt(s);
            if (std::abs(p.curvature) > kMax) {
                kMax = std::abs(p.curvature);
                cantAtMax = p.cant;
            }
        }
        if (kMax > 1e-6f) {
            const TippingLimit tl = vehicle->tippingLimit(kMax, cantAtMax);
            std::printf("[Vehicle] sharpest curve R=%.0f m (cant %+.1f deg): "
                        "overturn at lateral accel %.2f m/s^2 -> critical speed "
                        "%.0f km/h\n",
                        1.0f / kMax, glm::degrees(cantAtMax), tl.latAccelLimit,
                        tl.critSpeed * 3.6f);
        }
    }

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

    // --- Renderer ---
    VulkanRenderer renderer;
    g_renderer = &renderer;
    try {
        renderer.init(window, mesh.vertices(), mesh.indices(), texData,
                      tracks.vertices(), tracks.indices(),
                      tracks.alwaysIndexCount(), tracks.sleeperChunks(),
                      roads.vertices(), roads.indices(),
                      buildings.vertices(), buildings.indices(),
                      wheelset.vertices(), wheelset.indices());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Vulkan init failed: %s\n", e.what());
        glfwDestroyWindow(window);
        glfwTerminate();
        return EXIT_FAILURE;
    }

    std::printf(
        "\nControls: WASD move, Q/E down/up, mouse look, Shift boost, "
        "C chase vehicle, Tab release cursor, Esc quit\n\n");

    // Directional sun (scene space): from the south-west, fairly high.
    const glm::vec3 sunDir = glm::normalize(glm::vec3(0.4f, -0.5f, 0.75f));

    // Optional one-shot screenshot: EBANER_SCREENSHOT=path renders a few frames,
    // captures, then exits (used for headless verification).
    const char* shotPath = std::getenv("EBANER_SCREENSHOT");
    int frame = 0;

    double lastTime = glfwGetTime();
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        const double now = glfwGetTime();
        const float dt = static_cast<float>(now - lastTime);
        lastTime = now;

        // Movement input.
        float fwd = 0.0f, right = 0.0f, up = 0.0f;
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) fwd += 1.0f;
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) fwd -= 1.0f;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) right += 1.0f;
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) right -= 1.0f;
        if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) up += 1.0f;
        if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) up -= 1.0f;
        const bool fast = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
        if (g_chase && vehicle) {
            // Ride behind + above the wheelset, looking at it. Recomputed from
            // the vehicle pose each frame, so it follows once the vehicle moves.
            const TrackPose vp = vehicle->pose();
            const glm::vec3 axle = vp.pos + vp.up * wheelset::kAxleCentreAboveBed;
            const glm::vec3 camPos =
                axle - vp.tangent * 8.0f + vp.up * 3.0f;
            const glm::vec3 dir = glm::normalize(axle - camPos);
            g_camera.setPose(camPos, std::atan2(dir.y, dir.x),
                             std::asin(glm::clamp(dir.z, -1.0f, 1.0f)));
        } else {
            g_camera.move(fwd, right, up, dt, fast);
        }

        int fbw = 0, fbh = 0;
        glfwGetFramebufferSize(window, &fbw, &fbh);
        if (fbw == 0 || fbh == 0) { continue; } // minimised

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
    glfwDestroyWindow(window);
    glfwTerminate();
    return EXIT_SUCCESS;
}
