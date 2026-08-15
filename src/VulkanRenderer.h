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

#pragma once

#include <vulkan/vulkan.h>
#include <glm/glm.hpp>

#include "Font.h"       // TextVertex
#include "TrackGraph.h" // LineVertex (editor overlay)
#include "TrackMesh.h"  // TrackVertex, TrackDrawChunk

#include <array>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

struct GLFWwindow;
struct Vertex;

// Push-constant block shared with the terrain shaders (must match GLSL layout).
struct PushConstants {
    glm::mat4 viewProj;
    glm::vec4 sunDir;  // xyz = direction to sun (normalised)
    glm::vec4 camPos;  // xyz = scene-relative camera position
    glm::vec4 params{1.0f, 0.0f, 0.0f, 0.0f}; // x = scene alpha (1 = opaque)
};

// Land-cover texture array to upload (RGBA8, layer-major).
struct LandTextureData {
    const std::uint8_t* pixels = nullptr;
    std::uint32_t size = 0;    // per-layer width == height
    std::uint32_t layers = 0;
    std::size_t byteSize = 0;  // layers * size * size * 4
};

// Minimal single-pipeline Vulkan renderer for the terrain mesh.
class VulkanRenderer {
public:
    void init(GLFWwindow* window, const std::vector<Vertex>& vertices,
              const std::vector<std::uint32_t>& indices,
              const LandTextureData& textures,
              const std::vector<TrackVertex>& trackVertices,
              const std::vector<std::uint32_t>& trackIndices,
              std::uint32_t trackAlwaysIndexCount,
              const std::vector<TrackDrawChunk>& sleeperChunks,
              const std::vector<TrackVertex>& roadVertices,
              const std::vector<std::uint32_t>& roadIndices,
              const std::vector<TrackVertex>& buildingVertices,
              const std::vector<std::uint32_t>& buildingIndices);
    void drawFrame(const PushConstants& pc);
    void waitIdle();
    void cleanup();

    // Attach the chosen vehicle's mesh once, after the start-screen selection.
    // `glassFirstIndex` marks where the transparent (glazing) indices begin.
    void attachVehicle(const std::vector<TrackVertex>& vertices,
                       const std::vector<std::uint32_t>& indices,
                       std::uint32_t glassFirstIndex = 0);
    // Attach the movable switch-stand geometry (solid-lit, track pipeline). Unlike
    // the vehicle it changes topology when a switch is thrown, so `updateSwitches`
    // recreates the device-local buffers (a rare, user-triggered event).
    void attachSwitches(const std::vector<TrackVertex>& vertices,
                        const std::vector<std::uint32_t>& indices);
    void updateSwitches(const std::vector<TrackVertex>& vertices,
                        const std::vector<std::uint32_t>& indices);
    // Attach the ground-signal geometry (solid-lit, track pipeline). Like the switch
    // stands it is dynamic: the lamps change with the signal aspect, so `updateSignals`
    // recreates the (small) device-local buffers.
    void attachSignals(const std::vector<TrackVertex>& vertices,
                       const std::vector<std::uint32_t>& indices);
    void updateSignals(const std::vector<TrackVertex>& vertices,
                       const std::vector<std::uint32_t>& indices);
    // --- Terrain chunks -----------------------------------------------------
    // The terrain is held one tile at a time, so streaming replaces only the ground
    // that actually changed instead of the whole world. Passing empty geometry drops
    // the chunk. Buffers a in-flight frame may still be reading are retired rather
    // than destroyed, and freed once those frames have passed.
    void setTerrainChunk(std::uint64_t key, const std::vector<Vertex>& vertices,
                         const std::vector<std::uint32_t>& indices);
    void removeTerrainChunk(std::uint64_t key);
    std::size_t terrainChunkCount() const { return terrainChunks_.size(); }

    // Editor render-preview: recreate the terrain / track / struct (building) buffers
    // to reflect pending edits. Heavy and user-triggered; each waits for GPU idle.
    void updateTerrain(const std::vector<Vertex>& vertices,
                       const std::vector<std::uint32_t>& indices);
    void updateTracks(const std::vector<TrackVertex>& vertices,
                      const std::vector<std::uint32_t>& indices,
                      std::uint32_t alwaysIndexCount,
                      const std::vector<TrackDrawChunk>& sleeperChunks);
    void updateStructs(const std::vector<TrackVertex>& vertices,
                       const std::vector<std::uint32_t>& indices);
    void updateRoads(const std::vector<TrackVertex>& vertices,
                     const std::vector<std::uint32_t>& indices);
    // Set the 2-D text overlay (screen-space triangles) drawn on top each frame.
    void setOverlayText(const std::vector<TextVertex>& vertices);

    // Attach the editor's raw track-network overlay once: `lines` drawn as a line
    // list, `points` as round point sprites (both `LineVertex`). Used by
    // ebaner-trackedit; the viewer never calls it (overlay stays empty).
    void attachTrackGraph(const std::vector<LineVertex>& lines,
                          const std::vector<LineVertex>& points);

    // Traffic-manager 2-D map: when enabled, the frame clears to a dark panel and skips
    // the 3-D meshes, drawing only the overlay lines/points + text (with an orthographic
    // top-down pc.viewProj set by the caller).
    void setMapMode(bool on) { mapMode_ = on; }

    void notifyResize() { framebufferResized_ = true; }

    // Requests that the next rendered frame be written to `path` (PPM).
    void requestCapture(const std::string& path) { capturePath_ = path; }

private:
    static constexpr int kMaxFramesInFlight = 2;

    void createInstance();
    void setupDebugMessenger();
    void createSurface();
    void pickPhysicalDevice();
    void createLogicalDevice();
    void createSwapchain();
    void createImageViews();
    void createRenderPass();
    void createDepthResources();
    void createFramebuffers();
    void createDescriptorSetLayout();
    void createGraphicsPipeline();
    void createTrackPipeline();
    void createOverlayPipelines(); // editor line + point overlay (unused by viewer)
    void createCommandPool();
    void createTextureArray(const LandTextureData& textures);
    void createDescriptorSet();
    void createMeshBuffers(const std::vector<Vertex>& vertices,
                           const std::vector<std::uint32_t>& indices);
    void createTrackBuffers(const std::vector<TrackVertex>& vertices,
                            const std::vector<std::uint32_t>& indices);
    void createRoadBuffers(const std::vector<TrackVertex>& vertices,
                           const std::vector<std::uint32_t>& indices);
    void createBuildingBuffers(const std::vector<TrackVertex>& vertices,
                               const std::vector<std::uint32_t>& indices);
    void createSwitchBuffers(const std::vector<TrackVertex>& vertices,
                             const std::vector<std::uint32_t>& indices);
    void createSignalBuffers(const std::vector<TrackVertex>& vertices,
                             const std::vector<std::uint32_t>& indices);
    void createVehicleBuffers(const std::vector<TrackVertex>& vertices,
                              const std::vector<std::uint32_t>& indices,
                              std::uint32_t glassFirstIndex);
    void createTextResources();
    // (Re)allocate the per-frame overlay-text buffers to hold `bytes`. Called at startup
    // and again whenever the overlay outgrows them - see setOverlayText.
    void allocateTextBuffers(VkDeviceSize bytes);
    void createTextPipeline();

public:
    // Replaces the wheelset vertices for the next frame (fixed topology).
    void updateVehicleVertices(const std::vector<TrackVertex>& vertices);

private:
    void createCommandBuffers();
    void createSyncObjects();

    void recreateSwapchain();
    void cleanupSwapchain();
    void recordCommandBuffer(VkCommandBuffer cmd, uint32_t imageIndex);
    void saveSwapchainImage(uint32_t imageIndex, const std::string& path);

    // Helpers.
    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags props) const;
    void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                      VkMemoryPropertyFlags props, VkBuffer& buffer,
                      VkDeviceMemory& memory) const;
    void copyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size) const;
    void createImage(uint32_t w, uint32_t h, VkFormat format,
                     VkImageTiling tiling, VkImageUsageFlags usage,
                     VkMemoryPropertyFlags props, VkImage& image,
                     VkDeviceMemory& memory) const;
    VkImageView createImageView(VkImage image, VkFormat format,
                                VkImageAspectFlags aspect) const;
    VkFormat findDepthFormat() const;
    VkShaderModule createShaderModule(const std::vector<char>& code) const;
    VkCommandBuffer beginSingleTime() const;
    void endSingleTime(VkCommandBuffer cmd) const;

    GLFWwindow* window_ = nullptr;

    VkInstance instance_ = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT debugMessenger_ = VK_NULL_HANDLE;
    VkSurfaceKHR surface_ = VK_NULL_HANDLE;

    VkPhysicalDevice physicalDevice_ = VK_NULL_HANDLE;
    uint32_t graphicsFamily_ = 0;
    uint32_t presentFamily_ = 0;
    VkDevice device_ = VK_NULL_HANDLE;
    VkQueue graphicsQueue_ = VK_NULL_HANDLE;
    VkQueue presentQueue_ = VK_NULL_HANDLE;

    VkSwapchainKHR swapchain_ = VK_NULL_HANDLE;
    VkFormat swapchainFormat_ = VK_FORMAT_UNDEFINED;
    VkExtent2D swapchainExtent_{};
    std::vector<VkImage> swapchainImages_;
    std::vector<VkImageView> swapchainImageViews_;

    VkFormat depthFormat_ = VK_FORMAT_UNDEFINED;
    VkImage depthImage_ = VK_NULL_HANDLE;
    VkDeviceMemory depthMemory_ = VK_NULL_HANDLE;
    VkImageView depthView_ = VK_NULL_HANDLE;

    VkRenderPass renderPass_ = VK_NULL_HANDLE;
    std::vector<VkFramebuffer> framebuffers_;

    VkDescriptorSetLayout descriptorSetLayout_ = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
    VkDescriptorSet descriptorSet_ = VK_NULL_HANDLE;

    // Land-cover texture array.
    VkImage landImage_ = VK_NULL_HANDLE;
    VkDeviceMemory landMemory_ = VK_NULL_HANDLE;
    VkImageView landView_ = VK_NULL_HANDLE;
    VkSampler landSampler_ = VK_NULL_HANDLE;
    uint32_t landMipLevels_ = 1;

    VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline pipeline_ = VK_NULL_HANDLE;
    VkPipeline trackPipeline_ = VK_NULL_HANDLE; // railway ribbons (reuses layout)
    VkPipeline vehicleGlassPipeline_ = VK_NULL_HANDLE; // translucent vehicle glazing
    VkPipeline overlayLinePipeline_ = VK_NULL_HANDLE;  // editor track-graph links
    VkPipeline overlayPointPipeline_ = VK_NULL_HANDLE; // editor geo-point sprites

    VkCommandPool commandPool_ = VK_NULL_HANDLE;
    std::vector<VkCommandBuffer> commandBuffers_;

    // One tile's ground. Keyed the way TerrainData keys its tiles.
    struct TerrainChunk {
        VkBuffer vbuf = VK_NULL_HANDLE;
        VkDeviceMemory vmem = VK_NULL_HANDLE;
        VkBuffer ibuf = VK_NULL_HANDLE;
        VkDeviceMemory imem = VK_NULL_HANDLE;
        uint32_t indexCount = 0;
    };
    std::unordered_map<std::uint64_t, TerrainChunk> terrainChunks_;
    // A buffer replaced this frame may still be referenced by a command buffer in
    // flight. The alternative is to wait for the device to go idle before touching it,
    // which is a pipeline stall in the middle of a frame - so instead they wait out the
    // frames that can still be reading them, and are destroyed once those have passed.
    struct RetiredBuffer {
        VkBuffer buf = VK_NULL_HANDLE;
        VkDeviceMemory mem = VK_NULL_HANDLE;
        int framesLeft = 0;
    };
    std::vector<RetiredBuffer> retired_;
    // Hands the buffer over to be destroyed later and clears the handles.
    void retireBuffer(VkBuffer& buf, VkDeviceMemory& mem);
    void retire(TerrainChunk& c);
    void sweepRetired(bool force);

    VkBuffer trackVertexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory trackVertexMemory_ = VK_NULL_HANDLE;
    VkBuffer trackIndexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory trackIndexMemory_ = VK_NULL_HANDLE;
    uint32_t trackIndexCount_ = 0;
    uint32_t trackAlwaysIndexCount_ = 0;       // ballast + rails, drawn always
    std::vector<TrackDrawChunk> sleeperChunks_; // distance-culled sleeper runs

    // Roads reuse the track pipeline (flat solid-colour lit ribbons).
    VkBuffer roadVertexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory roadVertexMemory_ = VK_NULL_HANDLE;
    VkBuffer roadIndexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory roadIndexMemory_ = VK_NULL_HANDLE;
    uint32_t roadIndexCount_ = 0;

    VkBuffer buildingVertexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory buildingVertexMemory_ = VK_NULL_HANDLE;
    VkBuffer buildingIndexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory buildingIndexMemory_ = VK_NULL_HANDLE;
    uint32_t buildingIndexCount_ = 0;

    // Movable switch stands (dynamic: recreated when a switch is thrown).
    VkBuffer switchVertexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory switchVertexMemory_ = VK_NULL_HANDLE;
    VkBuffer switchIndexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory switchIndexMemory_ = VK_NULL_HANDLE;
    uint32_t switchIndexCount_ = 0;

    // Ground signals (dynamic: recreated when an aspect changes).
    VkBuffer signalVertexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory signalVertexMemory_ = VK_NULL_HANDLE;
    VkBuffer signalIndexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory signalIndexMemory_ = VK_NULL_HANDLE;
    uint32_t signalIndexCount_ = 0;

    // Editor overlay: static line-list + point-list of the raw track graph.
    VkBuffer overlayLineVertexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory overlayLineVertexMemory_ = VK_NULL_HANDLE;
    uint32_t overlayLineVertexCount_ = 0;
    VkBuffer overlayPointVertexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory overlayPointVertexMemory_ = VK_NULL_HANDLE;
    uint32_t overlayPointVertexCount_ = 0;

    // Vehicle mesh is rebuilt every frame; one host-visible mapped vertex buffer
    // per in-flight frame (written after that frame's fence), static index buffer.
    std::array<VkBuffer, kMaxFramesInFlight> vehicleVertexBuffers_{};
    std::array<VkDeviceMemory, kMaxFramesInFlight> vehicleVertexMemories_{};
    std::array<void*, kMaxFramesInFlight> vehicleVertexMapped_{};
    VkDeviceSize vehicleVertexBytes_ = 0;
    VkBuffer vehicleIndexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory vehicleIndexMemory_ = VK_NULL_HANDLE;
    uint32_t vehicleIndexCount_ = 0;
    uint32_t vehicleGlassFirstIndex_ = 0; // opaque indices [0, this); glass [this, count)
    std::vector<TrackVertex> pendingVehicleVertices_;

    // 2-D text overlay: a 2-D pipeline + one host-visible mapped vertex buffer per
    // in-flight frame, sized to a fixed capacity; the count varies each frame.
    VkPipeline textPipeline_ = VK_NULL_HANDLE;
    std::array<VkBuffer, kMaxFramesInFlight> textVertexBuffers_{};
    std::array<VkDeviceMemory, kMaxFramesInFlight> textVertexMemories_{};
    std::array<void*, kMaxFramesInFlight> textVertexMapped_{};
    VkDeviceSize textCapacityBytes_ = 0;
    std::vector<TextVertex> pendingTextVertices_;
    uint32_t textVertexCount_ = 0;

    std::vector<VkSemaphore> imageAvailable_;
    std::vector<VkSemaphore> renderFinished_;
    std::vector<VkFence> inFlight_;
    uint32_t currentFrame_ = 0;
    bool framebufferResized_ = false;
    bool mapMode_ = false; // traffic-manager 2-D map: dark clear, overlay-only

    PushConstants lastPush_{};
    bool validationEnabled_ = false;
    bool swapchainCanTransferSrc_ = false;
    std::string capturePath_;
};
