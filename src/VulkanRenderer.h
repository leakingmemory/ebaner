#pragma once

#include <vulkan/vulkan.h>
#include <glm/glm.hpp>

#include "TrackMesh.h" // TrackVertex, TrackDrawChunk

#include <cstdint>
#include <string>
#include <vector>

struct GLFWwindow;
struct Vertex;

// Push-constant block shared with the terrain shaders (must match GLSL layout).
struct PushConstants {
    glm::mat4 viewProj;
    glm::vec4 sunDir;  // xyz = direction to sun (normalised)
    glm::vec4 camPos;  // xyz = scene-relative camera position
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
              const std::vector<std::uint32_t>& roadIndices);
    void drawFrame(const PushConstants& pc);
    void waitIdle();
    void cleanup();

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
    void createCommandPool();
    void createTextureArray(const LandTextureData& textures);
    void createDescriptorSet();
    void createMeshBuffers(const std::vector<Vertex>& vertices,
                           const std::vector<std::uint32_t>& indices);
    void createTrackBuffers(const std::vector<TrackVertex>& vertices,
                            const std::vector<std::uint32_t>& indices);
    void createRoadBuffers(const std::vector<TrackVertex>& vertices,
                           const std::vector<std::uint32_t>& indices);
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

    VkCommandPool commandPool_ = VK_NULL_HANDLE;
    std::vector<VkCommandBuffer> commandBuffers_;

    VkBuffer vertexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory vertexMemory_ = VK_NULL_HANDLE;
    VkBuffer indexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory indexMemory_ = VK_NULL_HANDLE;
    uint32_t indexCount_ = 0;

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

    std::vector<VkSemaphore> imageAvailable_;
    std::vector<VkSemaphore> renderFinished_;
    std::vector<VkFence> inFlight_;
    uint32_t currentFrame_ = 0;
    bool framebufferResized_ = false;

    PushConstants lastPush_{};
    bool validationEnabled_ = false;
    bool swapchainCanTransferSrc_ = false;
    std::string capturePath_;
};
