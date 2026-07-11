#pragma once

#include <vulkan/vulkan.h>
#include <glm/glm.hpp>

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

// Minimal single-pipeline Vulkan renderer for the terrain mesh.
class VulkanRenderer {
public:
    void init(GLFWwindow* window, const std::vector<Vertex>& vertices,
              const std::vector<std::uint32_t>& indices);
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
    void createGraphicsPipeline();
    void createCommandPool();
    void createMeshBuffers(const std::vector<Vertex>& vertices,
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

    VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
    VkPipeline pipeline_ = VK_NULL_HANDLE;

    VkCommandPool commandPool_ = VK_NULL_HANDLE;
    std::vector<VkCommandBuffer> commandBuffers_;

    VkBuffer vertexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory vertexMemory_ = VK_NULL_HANDLE;
    VkBuffer indexBuffer_ = VK_NULL_HANDLE;
    VkDeviceMemory indexMemory_ = VK_NULL_HANDLE;
    uint32_t indexCount_ = 0;

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
