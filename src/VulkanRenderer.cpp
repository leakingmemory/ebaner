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

#include "VulkanRenderer.h"

#include "TerrainMesh.h" // Vertex
#include "TrackMesh.h"   // TrackVertex

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>

#ifndef SHADER_DIR
#define SHADER_DIR "shaders"
#endif

namespace {

const std::vector<const char*> kValidationLayers = {
    "VK_LAYER_KHRONOS_validation"};
const std::vector<const char*> kDeviceExtensions = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME};

void check(VkResult r, const char* what) {
    if (r != VK_SUCCESS) {
        throw std::runtime_error(std::string("Vulkan error in ") + what + ": " +
                                 std::to_string(static_cast<int>(r)));
    }
}

std::vector<char> readFile(const std::string& path) {
    std::ifstream f(path, std::ios::ate | std::ios::binary);
    if (!f) throw std::runtime_error("failed to open shader: " + path);
    const std::size_t size = static_cast<std::size_t>(f.tellg());
    std::vector<char> buf(size);
    f.seekg(0);
    f.read(buf.data(), static_cast<std::streamsize>(size));
    return buf;
}

bool validationLayersSupported() {
    uint32_t count = 0;
    vkEnumerateInstanceLayerProperties(&count, nullptr);
    std::vector<VkLayerProperties> available(count);
    vkEnumerateInstanceLayerProperties(&count, available.data());
    for (const char* want : kValidationLayers) {
        bool found = false;
        for (const auto& have : available) {
            if (std::strcmp(want, have.layerName) == 0) { found = true; break; }
        }
        if (!found) return false;
    }
    return true;
}

VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT severity,
    VkDebugUtilsMessageTypeFlagsEXT, const VkDebugUtilsMessengerCallbackDataEXT* data,
    void*) {
    if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
        std::cerr << "[vk] " << data->pMessage << "\n";
    }
    return VK_FALSE;
}

} // namespace

void VulkanRenderer::init(GLFWwindow* window,
                          const std::vector<Vertex>& vertices,
                          const std::vector<std::uint32_t>& indices,
                          const LandTextureData& textures,
                          const std::vector<TrackVertex>& trackVertices,
                          const std::vector<std::uint32_t>& trackIndices,
                          std::uint32_t trackAlwaysIndexCount,
                          const std::vector<TrackDrawChunk>& sleeperChunks,
                          const std::vector<TrackVertex>& roadVertices,
                          const std::vector<std::uint32_t>& roadIndices,
                          const std::vector<TrackVertex>& buildingVertices,
                          const std::vector<std::uint32_t>& buildingIndices,
                          const std::vector<TrackVertex>& vehicleVertices,
                          const std::vector<std::uint32_t>& vehicleIndices) {
    window_ = window;
    validationEnabled_ = validationLayersSupported();
    trackAlwaysIndexCount_ = trackAlwaysIndexCount;
    sleeperChunks_ = sleeperChunks;

    createInstance();
    setupDebugMessenger();
    createSurface();
    pickPhysicalDevice();
    createLogicalDevice();
    createSwapchain();
    createImageViews();
    createRenderPass();
    createDepthResources();
    createFramebuffers();
    createDescriptorSetLayout();
    createGraphicsPipeline();
    createTrackPipeline();
    createCommandPool();
    createTextureArray(textures);
    createDescriptorSet();
    createMeshBuffers(vertices, indices);
    createTrackBuffers(trackVertices, trackIndices);
    createRoadBuffers(roadVertices, roadIndices);
    createBuildingBuffers(buildingVertices, buildingIndices);
    createVehicleBuffers(vehicleVertices, vehicleIndices);
    createCommandBuffers();
    createSyncObjects();
}

void VulkanRenderer::createInstance() {
    VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
    app.pApplicationName = "ebaner";
    app.applicationVersion = VK_MAKE_VERSION(0, 1, 0);
    app.pEngineName = "none";
    app.apiVersion = VK_API_VERSION_1_2;

    uint32_t glfwExtCount = 0;
    const char** glfwExts = glfwGetRequiredInstanceExtensions(&glfwExtCount);
    std::vector<const char*> extensions(glfwExts, glfwExts + glfwExtCount);
    if (validationEnabled_) {
        extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    }

    VkInstanceCreateInfo ci{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
    ci.pApplicationInfo = &app;
    ci.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
    ci.ppEnabledExtensionNames = extensions.data();
    if (validationEnabled_) {
        ci.enabledLayerCount = static_cast<uint32_t>(kValidationLayers.size());
        ci.ppEnabledLayerNames = kValidationLayers.data();
    }

    check(vkCreateInstance(&ci, nullptr, &instance_), "vkCreateInstance");
}

void VulkanRenderer::setupDebugMessenger() {
    if (!validationEnabled_) return;
    auto fn = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
        vkGetInstanceProcAddr(instance_, "vkCreateDebugUtilsMessengerEXT"));
    if (!fn) return;

    VkDebugUtilsMessengerCreateInfoEXT ci{
        VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT};
    ci.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                         VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    ci.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                     VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                     VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    ci.pfnUserCallback = debugCallback;
    fn(instance_, &ci, nullptr, &debugMessenger_);
}

void VulkanRenderer::createSurface() {
    check(glfwCreateWindowSurface(instance_, window_, nullptr, &surface_),
          "glfwCreateWindowSurface");
}

void VulkanRenderer::pickPhysicalDevice() {
    uint32_t count = 0;
    vkEnumeratePhysicalDevices(instance_, &count, nullptr);
    if (count == 0) throw std::runtime_error("no Vulkan physical devices");
    std::vector<VkPhysicalDevice> devices(count);
    vkEnumeratePhysicalDevices(instance_, &count, devices.data());

    auto findQueues = [&](VkPhysicalDevice dev, uint32_t& gfx, uint32_t& present) {
        uint32_t qCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(dev, &qCount, nullptr);
        std::vector<VkQueueFamilyProperties> props(qCount);
        vkGetPhysicalDeviceQueueFamilyProperties(dev, &qCount, props.data());
        bool haveGfx = false, havePresent = false;
        for (uint32_t i = 0; i < qCount; ++i) {
            if (props[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) { gfx = i; haveGfx = true; }
            VkBool32 sup = VK_FALSE;
            vkGetPhysicalDeviceSurfaceSupportKHR(dev, i, surface_, &sup);
            if (sup) { present = i; havePresent = true; }
        }
        return haveGfx && havePresent;
    };

    // Prefer a discrete GPU, but accept any device that can render+present.
    VkPhysicalDevice fallback = VK_NULL_HANDLE;
    uint32_t fbGfx = 0, fbPresent = 0;
    for (VkPhysicalDevice dev : devices) {
        uint32_t gfx = 0, present = 0;
        if (!findQueues(dev, gfx, present)) continue;
        if (fallback == VK_NULL_HANDLE) { fallback = dev; fbGfx = gfx; fbPresent = present; }
        VkPhysicalDeviceProperties p;
        vkGetPhysicalDeviceProperties(dev, &p);
        if (p.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
            physicalDevice_ = dev;
            graphicsFamily_ = gfx;
            presentFamily_ = present;
            break;
        }
    }
    if (physicalDevice_ == VK_NULL_HANDLE) {
        if (fallback == VK_NULL_HANDLE)
            throw std::runtime_error("no suitable GPU (graphics+present)");
        physicalDevice_ = fallback;
        graphicsFamily_ = fbGfx;
        presentFamily_ = fbPresent;
    }

    VkPhysicalDeviceProperties p;
    vkGetPhysicalDeviceProperties(physicalDevice_, &p);
    std::cout << "[vk] using device: " << p.deviceName << "\n";
}

void VulkanRenderer::createLogicalDevice() {
    std::set<uint32_t> families{graphicsFamily_, presentFamily_};
    std::vector<VkDeviceQueueCreateInfo> queueInfos;
    const float priority = 1.0f;
    for (uint32_t fam : families) {
        VkDeviceQueueCreateInfo qi{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        qi.queueFamilyIndex = fam;
        qi.queueCount = 1;
        qi.pQueuePriorities = &priority;
        queueInfos.push_back(qi);
    }

    VkPhysicalDeviceFeatures features{};
    features.fillModeNonSolid = VK_TRUE;  // harmless; allows wireframe if desired
    features.samplerAnisotropy = VK_TRUE; // anisotropic terrain-texture filtering

    VkDeviceCreateInfo ci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
    ci.queueCreateInfoCount = static_cast<uint32_t>(queueInfos.size());
    ci.pQueueCreateInfos = queueInfos.data();
    ci.pEnabledFeatures = &features;
    ci.enabledExtensionCount = static_cast<uint32_t>(kDeviceExtensions.size());
    ci.ppEnabledExtensionNames = kDeviceExtensions.data();
    if (validationEnabled_) {
        ci.enabledLayerCount = static_cast<uint32_t>(kValidationLayers.size());
        ci.ppEnabledLayerNames = kValidationLayers.data();
    }

    check(vkCreateDevice(physicalDevice_, &ci, nullptr, &device_), "vkCreateDevice");
    vkGetDeviceQueue(device_, graphicsFamily_, 0, &graphicsQueue_);
    vkGetDeviceQueue(device_, presentFamily_, 0, &presentQueue_);
}

void VulkanRenderer::createSwapchain() {
    VkSurfaceCapabilitiesKHR caps;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice_, surface_, &caps);

    uint32_t fmtCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice_, surface_, &fmtCount, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(fmtCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice_, surface_, &fmtCount, formats.data());

    VkSurfaceFormatKHR chosen = formats[0];
    for (const auto& f : formats) {
        if (f.format == VK_FORMAT_B8G8R8A8_SRGB &&
            f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            chosen = f;
            break;
        }
    }

    uint32_t pmCount = 0;
    vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice_, surface_, &pmCount, nullptr);
    std::vector<VkPresentModeKHR> modes(pmCount);
    vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice_, surface_, &pmCount, modes.data());
    VkPresentModeKHR present = VK_PRESENT_MODE_FIFO_KHR;
    for (auto m : modes) {
        if (m == VK_PRESENT_MODE_MAILBOX_KHR) { present = m; break; }
    }

    VkExtent2D extent = caps.currentExtent;
    if (extent.width == UINT32_MAX) {
        int w, h;
        glfwGetFramebufferSize(window_, &w, &h);
        extent.width = std::clamp(static_cast<uint32_t>(w),
                                  caps.minImageExtent.width, caps.maxImageExtent.width);
        extent.height = std::clamp(static_cast<uint32_t>(h),
                                   caps.minImageExtent.height, caps.maxImageExtent.height);
    }

    uint32_t imageCount = caps.minImageCount + 1;
    if (caps.maxImageCount > 0 && imageCount > caps.maxImageCount)
        imageCount = caps.maxImageCount;

    VkSwapchainCreateInfoKHR ci{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
    ci.surface = surface_;
    ci.minImageCount = imageCount;
    ci.imageFormat = chosen.format;
    ci.imageColorSpace = chosen.colorSpace;
    ci.imageExtent = extent;
    ci.imageArrayLayers = 1;
    ci.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    swapchainCanTransferSrc_ =
        (caps.supportedUsageFlags & VK_IMAGE_USAGE_TRANSFER_SRC_BIT) != 0;
    if (swapchainCanTransferSrc_) {
        ci.imageUsage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
    }

    uint32_t indices[] = {graphicsFamily_, presentFamily_};
    if (graphicsFamily_ != presentFamily_) {
        ci.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        ci.queueFamilyIndexCount = 2;
        ci.pQueueFamilyIndices = indices;
    } else {
        ci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    }
    ci.preTransform = caps.currentTransform;
    ci.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    ci.presentMode = present;
    ci.clipped = VK_TRUE;

    check(vkCreateSwapchainKHR(device_, &ci, nullptr, &swapchain_),
          "vkCreateSwapchainKHR");

    vkGetSwapchainImagesKHR(device_, swapchain_, &imageCount, nullptr);
    swapchainImages_.resize(imageCount);
    vkGetSwapchainImagesKHR(device_, swapchain_, &imageCount, swapchainImages_.data());
    swapchainFormat_ = chosen.format;
    swapchainExtent_ = extent;
}

void VulkanRenderer::createImageViews() {
    swapchainImageViews_.resize(swapchainImages_.size());
    for (std::size_t i = 0; i < swapchainImages_.size(); ++i) {
        swapchainImageViews_[i] = createImageView(
            swapchainImages_[i], swapchainFormat_, VK_IMAGE_ASPECT_COLOR_BIT);
    }
}

void VulkanRenderer::createRenderPass() {
    depthFormat_ = findDepthFormat();

    VkAttachmentDescription color{};
    color.format = swapchainFormat_;
    color.samples = VK_SAMPLE_COUNT_1_BIT;
    color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    color.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentDescription depth{};
    depth.format = depthFormat_;
    depth.samples = VK_SAMPLE_COUNT_1_BIT;
    depth.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depth.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depth.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depth.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depth.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depth.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference colorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorRef;
    subpass.pDepthStencilAttachment = &depthRef;

    VkSubpassDependency dep{};
    dep.srcSubpass = VK_SUBPASS_EXTERNAL;
    dep.dstSubpass = 0;
    dep.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                       VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.srcAccessMask = 0;
    dep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                       VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    std::array<VkAttachmentDescription, 2> attachments{color, depth};
    VkRenderPassCreateInfo ci{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
    ci.attachmentCount = static_cast<uint32_t>(attachments.size());
    ci.pAttachments = attachments.data();
    ci.subpassCount = 1;
    ci.pSubpasses = &subpass;
    ci.dependencyCount = 1;
    ci.pDependencies = &dep;

    check(vkCreateRenderPass(device_, &ci, nullptr, &renderPass_),
          "vkCreateRenderPass");
}

void VulkanRenderer::createDepthResources() {
    createImage(swapchainExtent_.width, swapchainExtent_.height, depthFormat_,
                VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, depthImage_, depthMemory_);
    depthView_ = createImageView(depthImage_, depthFormat_,
                                 VK_IMAGE_ASPECT_DEPTH_BIT);
}

void VulkanRenderer::createFramebuffers() {
    framebuffers_.resize(swapchainImageViews_.size());
    for (std::size_t i = 0; i < swapchainImageViews_.size(); ++i) {
        std::array<VkImageView, 2> attachments{swapchainImageViews_[i], depthView_};
        VkFramebufferCreateInfo ci{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        ci.renderPass = renderPass_;
        ci.attachmentCount = static_cast<uint32_t>(attachments.size());
        ci.pAttachments = attachments.data();
        ci.width = swapchainExtent_.width;
        ci.height = swapchainExtent_.height;
        ci.layers = 1;
        check(vkCreateFramebuffer(device_, &ci, nullptr, &framebuffers_[i]),
              "vkCreateFramebuffer");
    }
}

void VulkanRenderer::createGraphicsPipeline() {
    auto vertCode = readFile(std::string(SHADER_DIR) + "/terrain.vert.spv");
    auto fragCode = readFile(std::string(SHADER_DIR) + "/terrain.frag.spv");
    VkShaderModule vert = createShaderModule(vertCode);
    VkShaderModule frag = createShaderModule(fragCode);

    VkPipelineShaderStageCreateInfo vs{
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vs.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vs.module = vert;
    vs.pName = "main";
    VkPipelineShaderStageCreateInfo fs{
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fs.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fs.module = frag;
    fs.pName = "main";
    VkPipelineShaderStageCreateInfo stages[] = {vs, fs};

    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.stride = sizeof(Vertex);
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::array<VkVertexInputAttributeDescription, 4> attrs{};
    attrs[0] = {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, pos)};
    attrs[1] = {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, normal)};
    attrs[2] = {2, 0, VK_FORMAT_R32_SFLOAT, offsetof(Vertex, elevation)};
    attrs[3] = {3, 0, VK_FORMAT_R32_SFLOAT, offsetof(Vertex, landcover)};

    VkPipelineVertexInputStateCreateInfo vertexInput{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vertexInput.vertexBindingDescriptionCount = 1;
    vertexInput.pVertexBindingDescriptions = &binding;
    vertexInput.vertexAttributeDescriptionCount = static_cast<uint32_t>(attrs.size());
    vertexInput.pVertexAttributeDescriptions = attrs.data();

    VkPipelineInputAssemblyStateCreateInfo ia{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo vp{
        VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    vp.viewportCount = 1;
    vp.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rs{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode = VK_CULL_MODE_NONE; // terrain lit on both faces
    rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rs.lineWidth = 1.0f;

    VkPipelineMultisampleStateCreateInfo ms{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo ds{
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    ds.depthTestEnable = VK_TRUE;
    ds.depthWriteEnable = VK_TRUE;
    ds.depthCompareOp = VK_COMPARE_OP_LESS;

    VkPipelineColorBlendAttachmentState cba{};
    cba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                         VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    cba.blendEnable = VK_FALSE;
    VkPipelineColorBlendStateCreateInfo cb{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    cb.attachmentCount = 1;
    cb.pAttachments = &cba;

    std::array<VkDynamicState, 2> dynamics{VK_DYNAMIC_STATE_VIEWPORT,
                                           VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dyn{
        VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dyn.dynamicStateCount = static_cast<uint32_t>(dynamics.size());
    dyn.pDynamicStates = dynamics.data();

    VkPushConstantRange pcr{};
    pcr.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pcr.offset = 0;
    pcr.size = sizeof(PushConstants);

    VkPipelineLayoutCreateInfo pl{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
    pl.setLayoutCount = 1;
    pl.pSetLayouts = &descriptorSetLayout_;
    pl.pushConstantRangeCount = 1;
    pl.pPushConstantRanges = &pcr;
    check(vkCreatePipelineLayout(device_, &pl, nullptr, &pipelineLayout_),
          "vkCreatePipelineLayout");

    VkGraphicsPipelineCreateInfo ci{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    ci.stageCount = 2;
    ci.pStages = stages;
    ci.pVertexInputState = &vertexInput;
    ci.pInputAssemblyState = &ia;
    ci.pViewportState = &vp;
    ci.pRasterizationState = &rs;
    ci.pMultisampleState = &ms;
    ci.pDepthStencilState = &ds;
    ci.pColorBlendState = &cb;
    ci.pDynamicState = &dyn;
    ci.layout = pipelineLayout_;
    ci.renderPass = renderPass_;
    ci.subpass = 0;

    check(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &ci, nullptr,
                                    &pipeline_),
          "vkCreateGraphicsPipelines");

    vkDestroyShaderModule(device_, vert, nullptr);
    vkDestroyShaderModule(device_, frag, nullptr);
}

void VulkanRenderer::createTrackPipeline() {
    // Reuses pipelineLayout_ and renderPass_ (created by createGraphicsPipeline).
    // Ribbons are triangles; vertex is position-only; a single solid colour.
    auto vertCode = readFile(std::string(SHADER_DIR) + "/track.vert.spv");
    auto fragCode = readFile(std::string(SHADER_DIR) + "/track.frag.spv");
    VkShaderModule vert = createShaderModule(vertCode);
    VkShaderModule frag = createShaderModule(fragCode);

    VkPipelineShaderStageCreateInfo vs{
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    vs.stage = VK_SHADER_STAGE_VERTEX_BIT;
    vs.module = vert;
    vs.pName = "main";
    VkPipelineShaderStageCreateInfo fs{
        VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO};
    fs.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    fs.module = frag;
    fs.pName = "main";
    VkPipelineShaderStageCreateInfo stages[] = {vs, fs};

    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.stride = sizeof(TrackVertex);
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    std::array<VkVertexInputAttributeDescription, 5> attrs{{
        {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(TrackVertex, pos)},
        {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(TrackVertex, normal)},
        {2, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(TrackVertex, color)},
        {3, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(TrackVertex, uv)},
        {4, 0, VK_FORMAT_R32_SFLOAT, offsetof(TrackVertex, texLayer)},
    }};

    VkPipelineVertexInputStateCreateInfo vertexInput{
        VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
    vertexInput.vertexBindingDescriptionCount = 1;
    vertexInput.pVertexBindingDescriptions = &binding;
    vertexInput.vertexAttributeDescriptionCount =
        static_cast<uint32_t>(attrs.size());
    vertexInput.pVertexAttributeDescriptions = attrs.data();

    VkPipelineInputAssemblyStateCreateInfo ia{
        VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo vp{
        VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
    vp.viewportCount = 1;
    vp.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo rs{
        VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
    rs.polygonMode = VK_POLYGON_MODE_FILL;
    rs.cullMode = VK_CULL_MODE_NONE;
    rs.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
    rs.lineWidth = 1.0f;

    VkPipelineMultisampleStateCreateInfo ms{
        VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo ds{
        VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO};
    ds.depthTestEnable = VK_TRUE;
    ds.depthWriteEnable = VK_TRUE;
    ds.depthCompareOp = VK_COMPARE_OP_LESS;

    VkPipelineColorBlendAttachmentState cba{};
    cba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                         VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    cba.blendEnable = VK_FALSE;
    VkPipelineColorBlendStateCreateInfo cb{
        VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
    cb.attachmentCount = 1;
    cb.pAttachments = &cba;

    std::array<VkDynamicState, 2> dynamics{VK_DYNAMIC_STATE_VIEWPORT,
                                           VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dyn{
        VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
    dyn.dynamicStateCount = static_cast<uint32_t>(dynamics.size());
    dyn.pDynamicStates = dynamics.data();

    VkGraphicsPipelineCreateInfo ci{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
    ci.stageCount = 2;
    ci.pStages = stages;
    ci.pVertexInputState = &vertexInput;
    ci.pInputAssemblyState = &ia;
    ci.pViewportState = &vp;
    ci.pRasterizationState = &rs;
    ci.pMultisampleState = &ms;
    ci.pDepthStencilState = &ds;
    ci.pColorBlendState = &cb;
    ci.pDynamicState = &dyn;
    ci.layout = pipelineLayout_;
    ci.renderPass = renderPass_;
    ci.subpass = 0;

    check(vkCreateGraphicsPipelines(device_, VK_NULL_HANDLE, 1, &ci, nullptr,
                                    &trackPipeline_),
          "vkCreateGraphicsPipelines(track)");

    vkDestroyShaderModule(device_, vert, nullptr);
    vkDestroyShaderModule(device_, frag, nullptr);
}

void VulkanRenderer::createCommandPool() {
    VkCommandPoolCreateInfo ci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
    ci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    ci.queueFamilyIndex = graphicsFamily_;
    check(vkCreateCommandPool(device_, &ci, nullptr, &commandPool_),
          "vkCreateCommandPool");
}

void VulkanRenderer::createDescriptorSetLayout() {
    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    binding.descriptorCount = 1;
    binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo ci{
        VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
    ci.bindingCount = 1;
    ci.pBindings = &binding;
    check(vkCreateDescriptorSetLayout(device_, &ci, nullptr, &descriptorSetLayout_),
          "vkCreateDescriptorSetLayout");
}

VkCommandBuffer VulkanRenderer::beginSingleTime() const {
    VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    ai.commandPool = commandPool_;
    ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandBufferCount = 1;
    VkCommandBuffer cmd;
    vkAllocateCommandBuffers(device_, &ai, &cmd);
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &bi);
    return cmd;
}

void VulkanRenderer::endSingleTime(VkCommandBuffer cmd) const {
    vkEndCommandBuffer(cmd);
    VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &cmd;
    vkQueueSubmit(graphicsQueue_, 1, &submit, VK_NULL_HANDLE);
    vkQueueWaitIdle(graphicsQueue_);
    vkFreeCommandBuffers(device_, commandPool_, 1, &cmd);
}

void VulkanRenderer::createTextureArray(const LandTextureData& t) {
    const uint32_t S = t.size;
    const uint32_t L = t.layers;
    landMipLevels_ = static_cast<uint32_t>(std::floor(std::log2(S))) + 1;
    const VkDeviceSize bytes = static_cast<VkDeviceSize>(t.byteSize);

    // Staging buffer with all layers (layer-major RGBA8).
    VkBuffer staging;
    VkDeviceMemory stagingMem;
    createBuffer(bytes, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                     VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 staging, stagingMem);
    void* mapped;
    vkMapMemory(device_, stagingMem, 0, bytes, 0, &mapped);
    std::memcpy(mapped, t.pixels, static_cast<std::size_t>(bytes));
    vkUnmapMemory(device_, stagingMem);

    // Array image with a full mip chain.
    VkImageCreateInfo ici{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    ici.imageType = VK_IMAGE_TYPE_2D;
    ici.extent = {S, S, 1};
    ici.mipLevels = landMipLevels_;
    ici.arrayLayers = L;
    ici.format = VK_FORMAT_R8G8B8A8_UNORM;
    ici.tiling = VK_IMAGE_TILING_OPTIMAL;
    ici.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    ici.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT |
                VK_IMAGE_USAGE_SAMPLED_BIT;
    ici.samples = VK_SAMPLE_COUNT_1_BIT;
    ici.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    check(vkCreateImage(device_, &ici, nullptr, &landImage_), "vkCreateImage(land)");

    VkMemoryRequirements req;
    vkGetImageMemoryRequirements(device_, landImage_, &req);
    VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    mai.allocationSize = req.size;
    mai.memoryTypeIndex =
        findMemoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    check(vkAllocateMemory(device_, &mai, nullptr, &landMemory_), "alloc(land)");
    vkBindImageMemory(device_, landImage_, landMemory_, 0);

    // Upload mip 0 (all layers) then generate the rest by successive blits.
    VkCommandBuffer cmd = beginSingleTime();

    VkImageMemoryBarrier barrier{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
    barrier.image = landImage_;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = L;

    // All mips: UNDEFINED -> TRANSFER_DST.
    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = landMipLevels_;
    barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.srcAccessMask = 0;
    barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                         VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr,
                         1, &barrier);

    VkBufferImageCopy region{};
    region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, L};
    region.imageExtent = {S, S, 1};
    vkCmdCopyBufferToImage(cmd, staging, landImage_,
                           VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

    barrier.subresourceRange.levelCount = 1;
    int32_t mipW = static_cast<int32_t>(S), mipH = static_cast<int32_t>(S);
    for (uint32_t i = 1; i < landMipLevels_; ++i) {
        // Source mip i-1: TRANSFER_DST -> TRANSFER_SRC.
        barrier.subresourceRange.baseMipLevel = i - 1;
        barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        barrier.newLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                             VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0,
                             nullptr, 1, &barrier);

        VkImageBlit blit{};
        blit.srcOffsets[1] = {mipW, mipH, 1};
        blit.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, i - 1, 0, L};
        blit.dstOffsets[1] = {mipW > 1 ? mipW / 2 : 1, mipH > 1 ? mipH / 2 : 1, 1};
        blit.dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, i, 0, L};
        vkCmdBlitImage(cmd, landImage_, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                       landImage_, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit,
                       VK_FILTER_LINEAR);

        // Source mip i-1 done: -> SHADER_READ_ONLY.
        barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
        barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                             VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr,
                             0, nullptr, 1, &barrier);

        if (mipW > 1) mipW /= 2;
        if (mipH > 1) mipH /= 2;
    }

    // Last mip: TRANSFER_DST -> SHADER_READ_ONLY.
    barrier.subresourceRange.baseMipLevel = landMipLevels_ - 1;
    barrier.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0,
                         nullptr, 1, &barrier);

    endSingleTime(cmd);
    vkDestroyBuffer(device_, staging, nullptr);
    vkFreeMemory(device_, stagingMem, nullptr);

    // Array image view.
    VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    vi.image = landImage_;
    vi.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
    vi.format = VK_FORMAT_R8G8B8A8_UNORM;
    vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, landMipLevels_, 0, L};
    check(vkCreateImageView(device_, &vi, nullptr, &landView_), "landView");

    // Sampler (repeat, trilinear, anisotropic).
    VkPhysicalDeviceProperties props;
    vkGetPhysicalDeviceProperties(physicalDevice_, &props);
    VkSamplerCreateInfo sci{VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO};
    sci.magFilter = VK_FILTER_LINEAR;
    sci.minFilter = VK_FILTER_LINEAR;
    sci.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    sci.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sci.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sci.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sci.anisotropyEnable = VK_TRUE;
    sci.maxAnisotropy = props.limits.maxSamplerAnisotropy;
    sci.minLod = 0.0f;
    sci.maxLod = static_cast<float>(landMipLevels_);
    check(vkCreateSampler(device_, &sci, nullptr, &landSampler_), "landSampler");
}

void VulkanRenderer::createDescriptorSet() {
    VkDescriptorPoolSize poolSize{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1};
    VkDescriptorPoolCreateInfo pci{VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
    pci.poolSizeCount = 1;
    pci.pPoolSizes = &poolSize;
    pci.maxSets = 1;
    check(vkCreateDescriptorPool(device_, &pci, nullptr, &descriptorPool_),
          "descriptorPool");

    VkDescriptorSetAllocateInfo ai{VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
    ai.descriptorPool = descriptorPool_;
    ai.descriptorSetCount = 1;
    ai.pSetLayouts = &descriptorSetLayout_;
    check(vkAllocateDescriptorSets(device_, &ai, &descriptorSet_),
          "allocDescriptorSet");

    VkDescriptorImageInfo img{};
    img.sampler = landSampler_;
    img.imageView = landView_;
    img.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    VkWriteDescriptorSet w{VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET};
    w.dstSet = descriptorSet_;
    w.dstBinding = 0;
    w.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    w.descriptorCount = 1;
    w.pImageInfo = &img;
    vkUpdateDescriptorSets(device_, 1, &w, 0, nullptr);
}

void VulkanRenderer::createMeshBuffers(const std::vector<Vertex>& vertices,
                                       const std::vector<std::uint32_t>& indices) {
    indexCount_ = static_cast<uint32_t>(indices.size());

    auto upload = [&](const void* data, VkDeviceSize size, VkBufferUsageFlags usage,
                      VkBuffer& buffer, VkDeviceMemory& memory) {
        VkBuffer staging;
        VkDeviceMemory stagingMem;
        createBuffer(size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                         VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                     staging, stagingMem);
        void* mapped;
        vkMapMemory(device_, stagingMem, 0, size, 0, &mapped);
        std::memcpy(mapped, data, static_cast<std::size_t>(size));
        vkUnmapMemory(device_, stagingMem);

        createBuffer(size, VK_BUFFER_USAGE_TRANSFER_DST_BIT | usage,
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, buffer, memory);
        copyBuffer(staging, buffer, size);

        vkDestroyBuffer(device_, staging, nullptr);
        vkFreeMemory(device_, stagingMem, nullptr);
    };

    upload(vertices.data(), sizeof(Vertex) * vertices.size(),
           VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, vertexBuffer_, vertexMemory_);
    upload(indices.data(), sizeof(std::uint32_t) * indices.size(),
           VK_BUFFER_USAGE_INDEX_BUFFER_BIT, indexBuffer_, indexMemory_);
}

void VulkanRenderer::createTrackBuffers(
    const std::vector<TrackVertex>& vertices,
    const std::vector<std::uint32_t>& indices) {
    trackIndexCount_ = static_cast<uint32_t>(indices.size());
    if (indices.empty()) return; // no tracks in the loaded area

    auto upload = [&](const void* data, VkDeviceSize size, VkBufferUsageFlags usage,
                      VkBuffer& buffer, VkDeviceMemory& memory) {
        VkBuffer staging;
        VkDeviceMemory stagingMem;
        createBuffer(size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                         VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                     staging, stagingMem);
        void* mapped;
        vkMapMemory(device_, stagingMem, 0, size, 0, &mapped);
        std::memcpy(mapped, data, static_cast<std::size_t>(size));
        vkUnmapMemory(device_, stagingMem);

        createBuffer(size, VK_BUFFER_USAGE_TRANSFER_DST_BIT | usage,
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, buffer, memory);
        copyBuffer(staging, buffer, size);

        vkDestroyBuffer(device_, staging, nullptr);
        vkFreeMemory(device_, stagingMem, nullptr);
    };

    upload(vertices.data(), sizeof(TrackVertex) * vertices.size(),
           VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, trackVertexBuffer_,
           trackVertexMemory_);
    upload(indices.data(), sizeof(std::uint32_t) * indices.size(),
           VK_BUFFER_USAGE_INDEX_BUFFER_BIT, trackIndexBuffer_, trackIndexMemory_);
}

void VulkanRenderer::createRoadBuffers(
    const std::vector<TrackVertex>& vertices,
    const std::vector<std::uint32_t>& indices) {
    roadIndexCount_ = static_cast<uint32_t>(indices.size());
    if (indices.empty()) return; // no roads in the loaded area

    auto upload = [&](const void* data, VkDeviceSize size, VkBufferUsageFlags usage,
                      VkBuffer& buffer, VkDeviceMemory& memory) {
        VkBuffer staging;
        VkDeviceMemory stagingMem;
        createBuffer(size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                         VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                     staging, stagingMem);
        void* mapped;
        vkMapMemory(device_, stagingMem, 0, size, 0, &mapped);
        std::memcpy(mapped, data, static_cast<std::size_t>(size));
        vkUnmapMemory(device_, stagingMem);

        createBuffer(size, VK_BUFFER_USAGE_TRANSFER_DST_BIT | usage,
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, buffer, memory);
        copyBuffer(staging, buffer, size);

        vkDestroyBuffer(device_, staging, nullptr);
        vkFreeMemory(device_, stagingMem, nullptr);
    };

    upload(vertices.data(), sizeof(TrackVertex) * vertices.size(),
           VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, roadVertexBuffer_, roadVertexMemory_);
    upload(indices.data(), sizeof(std::uint32_t) * indices.size(),
           VK_BUFFER_USAGE_INDEX_BUFFER_BIT, roadIndexBuffer_, roadIndexMemory_);
}

void VulkanRenderer::createBuildingBuffers(
    const std::vector<TrackVertex>& vertices,
    const std::vector<std::uint32_t>& indices) {
    buildingIndexCount_ = static_cast<uint32_t>(indices.size());
    if (indices.empty()) return; // no buildings in the loaded area

    auto upload = [&](const void* data, VkDeviceSize size, VkBufferUsageFlags usage,
                      VkBuffer& buffer, VkDeviceMemory& memory) {
        VkBuffer staging;
        VkDeviceMemory stagingMem;
        createBuffer(size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                         VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                     staging, stagingMem);
        void* mapped;
        vkMapMemory(device_, stagingMem, 0, size, 0, &mapped);
        std::memcpy(mapped, data, static_cast<std::size_t>(size));
        vkUnmapMemory(device_, stagingMem);

        createBuffer(size, VK_BUFFER_USAGE_TRANSFER_DST_BIT | usage,
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, buffer, memory);
        copyBuffer(staging, buffer, size);

        vkDestroyBuffer(device_, staging, nullptr);
        vkFreeMemory(device_, stagingMem, nullptr);
    };

    upload(vertices.data(), sizeof(TrackVertex) * vertices.size(),
           VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, buildingVertexBuffer_,
           buildingVertexMemory_);
    upload(indices.data(), sizeof(std::uint32_t) * indices.size(),
           VK_BUFFER_USAGE_INDEX_BUFFER_BIT, buildingIndexBuffer_,
           buildingIndexMemory_);
}

void VulkanRenderer::createVehicleBuffers(
    const std::vector<TrackVertex>& vertices,
    const std::vector<std::uint32_t>& indices) {
    vehicleIndexCount_ = static_cast<uint32_t>(indices.size());
    if (indices.empty() || vertices.empty()) return; // no vehicle

    // Static index buffer (fixed topology) — staged upload.
    {
        const VkDeviceSize size = sizeof(std::uint32_t) * indices.size();
        VkBuffer staging;
        VkDeviceMemory stagingMem;
        createBuffer(size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                         VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                     staging, stagingMem);
        void* mapped;
        vkMapMemory(device_, stagingMem, 0, size, 0, &mapped);
        std::memcpy(mapped, indices.data(), static_cast<std::size_t>(size));
        vkUnmapMemory(device_, stagingMem);
        createBuffer(size,
                     VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                         VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, vehicleIndexBuffer_,
                     vehicleIndexMemory_);
        copyBuffer(staging, vehicleIndexBuffer_, size);
        vkDestroyBuffer(device_, staging, nullptr);
        vkFreeMemory(device_, stagingMem, nullptr);
    }

    // One host-visible, persistently-mapped vertex buffer per in-flight frame,
    // seeded with the initial wheelset vertices.
    vehicleVertexBytes_ = sizeof(TrackVertex) * vertices.size();
    for (int i = 0; i < kMaxFramesInFlight; ++i) {
        createBuffer(vehicleVertexBytes_, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                         VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                     vehicleVertexBuffers_[i], vehicleVertexMemories_[i]);
        vkMapMemory(device_, vehicleVertexMemories_[i], 0, vehicleVertexBytes_, 0,
                    &vehicleVertexMapped_[i]);
        std::memcpy(vehicleVertexMapped_[i], vertices.data(),
                    static_cast<std::size_t>(vehicleVertexBytes_));
    }
    pendingVehicleVertices_ = vertices;
}

void VulkanRenderer::updateVehicleVertices(
    const std::vector<TrackVertex>& vertices) {
    // Stored now; copied into the current frame's mapped buffer in drawFrame,
    // after that frame's fence (so the GPU has finished reading it).
    if (vehicleIndexCount_ > 0) pendingVehicleVertices_ = vertices;
}

void VulkanRenderer::createCommandBuffers() {
    commandBuffers_.resize(kMaxFramesInFlight);
    VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    ai.commandPool = commandPool_;
    ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandBufferCount = static_cast<uint32_t>(commandBuffers_.size());
    check(vkAllocateCommandBuffers(device_, &ai, commandBuffers_.data()),
          "vkAllocateCommandBuffers");
}

void VulkanRenderer::createSyncObjects() {
    imageAvailable_.resize(kMaxFramesInFlight);
    inFlight_.resize(kMaxFramesInFlight);
    renderFinished_.resize(swapchainImages_.size());

    VkSemaphoreCreateInfo si{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    fi.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    for (int i = 0; i < kMaxFramesInFlight; ++i) {
        check(vkCreateSemaphore(device_, &si, nullptr, &imageAvailable_[i]), "sem");
        check(vkCreateFence(device_, &fi, nullptr, &inFlight_[i]), "fence");
    }
    for (std::size_t i = 0; i < renderFinished_.size(); ++i) {
        check(vkCreateSemaphore(device_, &si, nullptr, &renderFinished_[i]), "sem");
    }
}

void VulkanRenderer::recordCommandBuffer(VkCommandBuffer cmd, uint32_t imageIndex) {
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    check(vkBeginCommandBuffer(cmd, &bi), "vkBeginCommandBuffer");

    std::array<VkClearValue, 2> clears{};
    clears[0].color = {{0.55f, 0.70f, 0.85f, 1.0f}}; // sky
    clears[1].depthStencil = {1.0f, 0};

    VkRenderPassBeginInfo rp{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    rp.renderPass = renderPass_;
    rp.framebuffer = framebuffers_[imageIndex];
    rp.renderArea.extent = swapchainExtent_;
    rp.clearValueCount = static_cast<uint32_t>(clears.size());
    rp.pClearValues = clears.data();

    vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout_,
                            0, 1, &descriptorSet_, 0, nullptr);

    VkViewport viewport{};
    viewport.width = static_cast<float>(swapchainExtent_.width);
    viewport.height = static_cast<float>(swapchainExtent_.height);
    viewport.maxDepth = 1.0f;
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    VkRect2D scissor{{0, 0}, swapchainExtent_};
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    vkCmdPushConstants(cmd, pipelineLayout_,
                       VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                       0, sizeof(PushConstants), &lastPush_);

    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &vertexBuffer_, &offset);
    vkCmdBindIndexBuffer(cmd, indexBuffer_, 0, VK_INDEX_TYPE_UINT32);
    vkCmdDrawIndexed(cmd, indexCount_, 1, 0, 0, 0);

    // Roads reuse the track pipeline (flat solid-colour lit ribbons). Drawn before
    // the railway so tracks sit on top at level crossings.
    if (roadIndexCount_ > 0) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, trackPipeline_);
        vkCmdBindVertexBuffers(cmd, 0, 1, &roadVertexBuffer_, &offset);
        vkCmdBindIndexBuffer(cmd, roadIndexBuffer_, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, roadIndexCount_, 1, 0, 0, 0);
    }

    // Buildings — extruded prisms on the same track pipeline.
    if (buildingIndexCount_ > 0) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, trackPipeline_);
        vkCmdBindVertexBuffers(cmd, 0, 1, &buildingVertexBuffer_, &offset);
        vkCmdBindIndexBuffer(cmd, buildingIndexBuffer_, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, buildingIndexCount_, 1, 0, 0, 0);
    }

    // Rail vehicle (wheelset) — same track pipeline; per-frame dynamic buffer.
    if (vehicleIndexCount_ > 0) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, trackPipeline_);
        vkCmdBindVertexBuffers(cmd, 0, 1, &vehicleVertexBuffers_[currentFrame_],
                               &offset);
        vkCmdBindIndexBuffer(cmd, vehicleIndexBuffer_, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, vehicleIndexCount_, 1, 0, 0, 0);
    }

    // Railway geometry — same layout/push constants; the descriptor set stays
    // bound (the track shaders sample the ballast layer from it). Ballast + rails
    // are always drawn; sleeper boxes only for chunks near the camera (the ballast
    // texture stands in for sleepers at distance).
    if (trackIndexCount_ > 0) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, trackPipeline_);
        vkCmdBindVertexBuffers(cmd, 0, 1, &trackVertexBuffer_, &offset);
        vkCmdBindIndexBuffer(cmd, trackIndexBuffer_, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, trackAlwaysIndexCount_, 1, 0, 0, 0);

        constexpr float kSleeperLODRadius = 230.0f; // metres (ties fade in ~here)
        const glm::vec3 cam = glm::vec3(lastPush_.camPos);
        for (const TrackDrawChunk& c : sleeperChunks_) {
            if (glm::distance(cam, c.centroid) < kSleeperLODRadius)
                vkCmdDrawIndexed(cmd, c.indexCount, 1, c.firstIndex, 0, 0);
        }
    }

    vkCmdEndRenderPass(cmd);
    check(vkEndCommandBuffer(cmd), "vkEndCommandBuffer");
}

void VulkanRenderer::saveSwapchainImage(uint32_t imageIndex,
                                        const std::string& path) {
    if (!swapchainCanTransferSrc_) {
        std::cerr << "[capture] swapchain does not support TRANSFER_SRC; "
                     "cannot screenshot\n";
        return;
    }

    const uint32_t w = swapchainExtent_.width;
    const uint32_t h = swapchainExtent_.height;
    const VkDeviceSize size = static_cast<VkDeviceSize>(w) * h * 4;

    VkBuffer buffer;
    VkDeviceMemory memory;
    createBuffer(size, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                     VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 buffer, memory);

    VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    ai.commandPool = commandPool_;
    ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandBufferCount = 1;
    VkCommandBuffer cmd;
    vkAllocateCommandBuffers(device_, &ai, &cmd);
    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &bi);

    auto barrier = [&](VkImageLayout from, VkImageLayout to,
                       VkAccessFlags srcAccess, VkAccessFlags dstAccess) {
        VkImageMemoryBarrier b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        b.oldLayout = from;
        b.newLayout = to;
        b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = swapchainImages_[imageIndex];
        b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        b.srcAccessMask = srcAccess;
        b.dstAccessMask = dstAccess;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                             VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0,
                             nullptr, 1, &b);
    };

    barrier(VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
            VK_ACCESS_MEMORY_READ_BIT, VK_ACCESS_TRANSFER_READ_BIT);

    VkBufferImageCopy region{};
    region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    region.imageExtent = {w, h, 1};
    vkCmdCopyImageToBuffer(cmd, swapchainImages_[imageIndex],
                           VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, buffer, 1, &region);

    barrier(VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
            VK_ACCESS_TRANSFER_READ_BIT, VK_ACCESS_MEMORY_READ_BIT);

    vkEndCommandBuffer(cmd);
    VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &cmd;
    vkQueueSubmit(graphicsQueue_, 1, &submit, VK_NULL_HANDLE);
    vkQueueWaitIdle(graphicsQueue_);
    vkFreeCommandBuffers(device_, commandPool_, 1, &cmd);

    // Swapchain format is B8G8R8A8; write RGB PPM.
    const uint8_t* pixels = nullptr;
    vkMapMemory(device_, memory, 0, size, 0,
                reinterpret_cast<void**>(const_cast<uint8_t**>(&pixels)));
    std::ofstream out(path, std::ios::binary);
    out << "P6\n" << w << " " << h << "\n255\n";
    for (uint32_t i = 0; i < w * h; ++i) {
        const uint8_t* px = pixels + static_cast<std::size_t>(i) * 4;
        out.put(static_cast<char>(px[2])); // R
        out.put(static_cast<char>(px[1])); // G
        out.put(static_cast<char>(px[0])); // B
    }
    out.close();
    vkUnmapMemory(device_, memory);

    vkDestroyBuffer(device_, buffer, nullptr);
    vkFreeMemory(device_, memory, nullptr);
    std::cout << "[capture] wrote " << path << " (" << w << "x" << h << ")\n";
}

void VulkanRenderer::drawFrame(const PushConstants& pc) {
    lastPush_ = pc;

    vkWaitForFences(device_, 1, &inFlight_[currentFrame_], VK_TRUE, UINT64_MAX);

    uint32_t imageIndex = 0;
    VkResult acq = vkAcquireNextImageKHR(device_, swapchain_, UINT64_MAX,
                                         imageAvailable_[currentFrame_],
                                         VK_NULL_HANDLE, &imageIndex);
    if (acq == VK_ERROR_OUT_OF_DATE_KHR) {
        recreateSwapchain();
        return;
    }
    if (acq != VK_SUCCESS && acq != VK_SUBOPTIMAL_KHR) {
        throw std::runtime_error("vkAcquireNextImageKHR failed");
    }

    vkResetFences(device_, 1, &inFlight_[currentFrame_]);

    // This frame slot's previous GPU read is done (fence waited above), so it is
    // safe to refresh its mapped vehicle vertex buffer for this frame.
    if (vehicleIndexCount_ > 0 && !pendingVehicleVertices_.empty()) {
        const VkDeviceSize bytes =
            sizeof(TrackVertex) * pendingVehicleVertices_.size();
        if (bytes == vehicleVertexBytes_ && vehicleVertexMapped_[currentFrame_])
            std::memcpy(vehicleVertexMapped_[currentFrame_],
                        pendingVehicleVertices_.data(),
                        static_cast<std::size_t>(bytes));
    }

    vkResetCommandBuffer(commandBuffers_[currentFrame_], 0);
    recordCommandBuffer(commandBuffers_[currentFrame_], imageIndex);

    VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submit.waitSemaphoreCount = 1;
    submit.pWaitSemaphores = &imageAvailable_[currentFrame_];
    submit.pWaitDstStageMask = &waitStage;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &commandBuffers_[currentFrame_];
    submit.signalSemaphoreCount = 1;
    submit.pSignalSemaphores = &renderFinished_[imageIndex];
    check(vkQueueSubmit(graphicsQueue_, 1, &submit, inFlight_[currentFrame_]),
          "vkQueueSubmit");

    VkPresentInfoKHR present{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
    present.waitSemaphoreCount = 1;
    present.pWaitSemaphores = &renderFinished_[imageIndex];
    present.swapchainCount = 1;
    present.pSwapchains = &swapchain_;
    present.pImageIndices = &imageIndex;
    if (!capturePath_.empty()) {
        vkQueueWaitIdle(graphicsQueue_);
        saveSwapchainImage(imageIndex, capturePath_);
        capturePath_.clear();
    }

    VkResult pres = vkQueuePresentKHR(presentQueue_, &present);
    if (pres == VK_ERROR_OUT_OF_DATE_KHR || pres == VK_SUBOPTIMAL_KHR ||
        framebufferResized_) {
        framebufferResized_ = false;
        recreateSwapchain();
    } else if (pres != VK_SUCCESS) {
        throw std::runtime_error("vkQueuePresentKHR failed");
    }

    currentFrame_ = (currentFrame_ + 1) % kMaxFramesInFlight;
}

void VulkanRenderer::recreateSwapchain() {
    int w = 0, h = 0;
    glfwGetFramebufferSize(window_, &w, &h);
    while (w == 0 || h == 0) {
        glfwGetFramebufferSize(window_, &w, &h);
        glfwWaitEvents();
    }
    vkDeviceWaitIdle(device_);

    cleanupSwapchain();

    createSwapchain();
    createImageViews();
    createDepthResources();
    createFramebuffers();

    // Per-image render-finished semaphores must match the new image count.
    for (VkSemaphore s : renderFinished_) vkDestroySemaphore(device_, s, nullptr);
    renderFinished_.resize(swapchainImages_.size());
    VkSemaphoreCreateInfo si{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    for (std::size_t i = 0; i < renderFinished_.size(); ++i) {
        check(vkCreateSemaphore(device_, &si, nullptr, &renderFinished_[i]), "sem");
    }
}

void VulkanRenderer::cleanupSwapchain() {
    vkDestroyImageView(device_, depthView_, nullptr);
    vkDestroyImage(device_, depthImage_, nullptr);
    vkFreeMemory(device_, depthMemory_, nullptr);
    for (VkFramebuffer fb : framebuffers_) vkDestroyFramebuffer(device_, fb, nullptr);
    for (VkImageView v : swapchainImageViews_) vkDestroyImageView(device_, v, nullptr);
    vkDestroySwapchainKHR(device_, swapchain_, nullptr);
}

void VulkanRenderer::waitIdle() {
    if (device_) vkDeviceWaitIdle(device_);
}

void VulkanRenderer::cleanup() {
    if (!device_) return;
    vkDeviceWaitIdle(device_);

    cleanupSwapchain();

    vkDestroyPipeline(device_, trackPipeline_, nullptr);
    vkDestroyPipeline(device_, pipeline_, nullptr);
    vkDestroyPipelineLayout(device_, pipelineLayout_, nullptr);
    vkDestroyRenderPass(device_, renderPass_, nullptr);

    vkDestroyBuffer(device_, indexBuffer_, nullptr);
    vkFreeMemory(device_, indexMemory_, nullptr);
    vkDestroyBuffer(device_, vertexBuffer_, nullptr);
    vkFreeMemory(device_, vertexMemory_, nullptr);
    vkDestroyBuffer(device_, trackIndexBuffer_, nullptr);
    vkFreeMemory(device_, trackIndexMemory_, nullptr);
    vkDestroyBuffer(device_, trackVertexBuffer_, nullptr);
    vkFreeMemory(device_, trackVertexMemory_, nullptr);
    vkDestroyBuffer(device_, roadIndexBuffer_, nullptr);
    vkFreeMemory(device_, roadIndexMemory_, nullptr);
    vkDestroyBuffer(device_, roadVertexBuffer_, nullptr);
    vkFreeMemory(device_, roadVertexMemory_, nullptr);
    vkDestroyBuffer(device_, vehicleIndexBuffer_, nullptr);
    vkFreeMemory(device_, vehicleIndexMemory_, nullptr);
    for (int i = 0; i < kMaxFramesInFlight; ++i) {
        if (vehicleVertexMemories_[i]) vkUnmapMemory(device_, vehicleVertexMemories_[i]);
        vkDestroyBuffer(device_, vehicleVertexBuffers_[i], nullptr);
        vkFreeMemory(device_, vehicleVertexMemories_[i], nullptr);
    }
    vkDestroyBuffer(device_, buildingIndexBuffer_, nullptr);
    vkFreeMemory(device_, buildingIndexMemory_, nullptr);
    vkDestroyBuffer(device_, buildingVertexBuffer_, nullptr);
    vkFreeMemory(device_, buildingVertexMemory_, nullptr);

    vkDestroySampler(device_, landSampler_, nullptr);
    vkDestroyImageView(device_, landView_, nullptr);
    vkDestroyImage(device_, landImage_, nullptr);
    vkFreeMemory(device_, landMemory_, nullptr);
    vkDestroyDescriptorPool(device_, descriptorPool_, nullptr);
    vkDestroyDescriptorSetLayout(device_, descriptorSetLayout_, nullptr);

    for (VkSemaphore s : renderFinished_) vkDestroySemaphore(device_, s, nullptr);
    for (VkSemaphore s : imageAvailable_) vkDestroySemaphore(device_, s, nullptr);
    for (VkFence f : inFlight_) vkDestroyFence(device_, f, nullptr);

    vkDestroyCommandPool(device_, commandPool_, nullptr);
    vkDestroyDevice(device_, nullptr);

    if (debugMessenger_) {
        auto fn = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(instance_, "vkDestroyDebugUtilsMessengerEXT"));
        if (fn) fn(instance_, debugMessenger_, nullptr);
    }
    vkDestroySurfaceKHR(instance_, surface_, nullptr);
    vkDestroyInstance(instance_, nullptr);
    device_ = VK_NULL_HANDLE;
}

// ---- Helpers --------------------------------------------------------------

uint32_t VulkanRenderer::findMemoryType(uint32_t typeFilter,
                                        VkMemoryPropertyFlags props) const {
    VkPhysicalDeviceMemoryProperties mem;
    vkGetPhysicalDeviceMemoryProperties(physicalDevice_, &mem);
    for (uint32_t i = 0; i < mem.memoryTypeCount; ++i) {
        if ((typeFilter & (1u << i)) &&
            (mem.memoryTypes[i].propertyFlags & props) == props) {
            return i;
        }
    }
    throw std::runtime_error("no suitable memory type");
}

void VulkanRenderer::createBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                                  VkMemoryPropertyFlags props, VkBuffer& buffer,
                                  VkDeviceMemory& memory) const {
    VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bi.size = size;
    bi.usage = usage;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    check(vkCreateBuffer(device_, &bi, nullptr, &buffer), "vkCreateBuffer");

    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(device_, buffer, &req);
    VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = findMemoryType(req.memoryTypeBits, props);
    check(vkAllocateMemory(device_, &ai, nullptr, &memory), "vkAllocateMemory");
    vkBindBufferMemory(device_, buffer, memory, 0);
}

void VulkanRenderer::copyBuffer(VkBuffer src, VkBuffer dst,
                                VkDeviceSize size) const {
    VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
    ai.commandPool = commandPool_;
    ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandBufferCount = 1;
    VkCommandBuffer cmd;
    vkAllocateCommandBuffers(device_, &ai, &cmd);

    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &bi);
    VkBufferCopy copy{0, 0, size};
    vkCmdCopyBuffer(cmd, src, dst, 1, &copy);
    vkEndCommandBuffer(cmd);

    VkSubmitInfo submit{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &cmd;
    vkQueueSubmit(graphicsQueue_, 1, &submit, VK_NULL_HANDLE);
    vkQueueWaitIdle(graphicsQueue_);
    vkFreeCommandBuffers(device_, commandPool_, 1, &cmd);
}

void VulkanRenderer::createImage(uint32_t w, uint32_t h, VkFormat format,
                                 VkImageTiling tiling, VkImageUsageFlags usage,
                                 VkMemoryPropertyFlags props, VkImage& image,
                                 VkDeviceMemory& memory) const {
    VkImageCreateInfo ci{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
    ci.imageType = VK_IMAGE_TYPE_2D;
    ci.extent = {w, h, 1};
    ci.mipLevels = 1;
    ci.arrayLayers = 1;
    ci.format = format;
    ci.tiling = tiling;
    ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    ci.usage = usage;
    ci.samples = VK_SAMPLE_COUNT_1_BIT;
    ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    check(vkCreateImage(device_, &ci, nullptr, &image), "vkCreateImage");

    VkMemoryRequirements req;
    vkGetImageMemoryRequirements(device_, image, &req);
    VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    ai.allocationSize = req.size;
    ai.memoryTypeIndex = findMemoryType(req.memoryTypeBits, props);
    check(vkAllocateMemory(device_, &ai, nullptr, &memory), "vkAllocateMemory(image)");
    vkBindImageMemory(device_, image, memory, 0);
}

VkImageView VulkanRenderer::createImageView(VkImage image, VkFormat format,
                                            VkImageAspectFlags aspect) const {
    VkImageViewCreateInfo ci{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    ci.image = image;
    ci.viewType = VK_IMAGE_VIEW_TYPE_2D;
    ci.format = format;
    ci.subresourceRange.aspectMask = aspect;
    ci.subresourceRange.levelCount = 1;
    ci.subresourceRange.layerCount = 1;
    VkImageView view;
    check(vkCreateImageView(device_, &ci, nullptr, &view), "vkCreateImageView");
    return view;
}

VkFormat VulkanRenderer::findDepthFormat() const {
    for (VkFormat f : {VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT,
                       VK_FORMAT_D24_UNORM_S8_UINT}) {
        VkFormatProperties props;
        vkGetPhysicalDeviceFormatProperties(physicalDevice_, f, &props);
        if (props.optimalTilingFeatures &
            VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) {
            return f;
        }
    }
    throw std::runtime_error("no supported depth format");
}

VkShaderModule VulkanRenderer::createShaderModule(
    const std::vector<char>& code) const {
    VkShaderModuleCreateInfo ci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
    ci.codeSize = code.size();
    ci.pCode = reinterpret_cast<const uint32_t*>(code.data());
    VkShaderModule module;
    check(vkCreateShaderModule(device_, &ci, nullptr, &module),
          "vkCreateShaderModule");
    return module;
}
