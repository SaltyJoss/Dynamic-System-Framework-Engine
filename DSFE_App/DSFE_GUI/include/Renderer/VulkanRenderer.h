#pragma once

#include <volk.h>

#include <fstream>
#include <sstream>
#include <string>
#include <array>
#include <shaderc/shaderc.hpp>

#include "VulkanContext.h"
#include "VulkanSwapchain.h"

#include "Platform/Logger.h"

namespace renderer {
    // Structure to hold per-frame resources for Vulkan rendering
    struct FrameResources {
        VkCommandPool command_pool = nullptr;
        VkCommandBuffer command_buffer = nullptr;
        VkSemaphore image_acquired_semaphore = nullptr;
    };

    class VulkanRenderer {
        constexpr static uint32_t MAX_FRAMES_IN_FLIGHT{ 2 };
        constexpr static VkFormat SWAPCHAIN_FORMAT{  VK_FORMAT_B8G8R8A8_SRGB };
        constexpr static VkFormat DEPTH_FORMAT{ VK_FORMAT_D32_SFLOAT };

        public:
            bool init(void* windowHandle);
            void shutdown();
            void render();
            void resize(uint32_t width, uint32_t height);
            void wait_idle();

        private:
            // Methods for Vulkan rendering setup
            bool create_shaders();
            static std::string load_shader_source(const std::string& filename);
            VkShaderModule compile_shader(const std::string& source, const std::string& debug_name, shaderc_shader_kind kind, const std::string& entry_point) const;
            VkPipeline create_graphics_pipeline();
            bool create_sync_resources();
            bool create_command_buffers();

            bool create_depth_resources();
            void destroy_depth_resources();

            // Internal state
            VulkanContext* _context = nullptr;
            VulkanSwapchain* _swapchain = nullptr;

            // Frame resources for double/triple buffering
            uint32_t _width = 1920;
            uint32_t _height = 1080;
            uint64_t _frame_idx = 0; // Current frame index for double/triple buffering
            uint64_t _next_signal_val = MAX_FRAMES_IN_FLIGHT - 1;

            // Depth buffer variables
            VkImage _depth_image = VK_NULL_HANDLE; // Vulkan depth image handle
            VkImageView _depth_image_view = VK_NULL_HANDLE; // Vulkan depth image view handle
            VmaAllocation _depth_image_allocation = VK_NULL_HANDLE; // Vulkan depth image allocation

            // Shader variables (For GLSL to SPIR-V compilation, however later I will be using HLSL for shader compilation)
            VkShaderModule _vert_shader = VK_NULL_HANDLE;
	        VkShaderModule _frag_shader = VK_NULL_HANDLE;

            // Graphics Pipeline variables
            VkPipeline _pipeline = VK_NULL_HANDLE; // Vulkan graphics pipeline handle
            VkPipelineLayout _pipeline_layout = VK_NULL_HANDLE; // Vulkan pipeline layout handle

            // Synchronisation variables
            VkSemaphore _timeline_semaphore = VK_NULL_HANDLE; // Vulkan timeline semaphore for synchronisation
            std::array<FrameResources, MAX_FRAMES_IN_FLIGHT> _frame_resources; // Vector of frame resources for each frame
    };
} // namespace renderer
