#pragma once

#include <volk.h>
#include <string>
#include <vector>
#include <array>
#include <shaderc/shaderc.hpp>

struct VmaAllocator_T;
typedef struct VmaAllocator_T* VmaAllocator;

namespace renderer {
    // VulkanRenderer class implementing the IRenderer interface for Vulkan rendering
    class VulkanContext {
        constexpr static uint32_t VK_VERSION{ VK_API_VERSION_1_4 };

        public:
            bool init(void* windowHandle);
            void shutdown();

            // Accessors for Vulkan handles
            VkDevice device() const { return _device; }
            VkPhysicalDevice physical_device() const { return _phys_device; }
            VkInstance instance() const { return _instance; }
            VkSurfaceKHR surface() const { return _surface; }
            VkQueue graphics_queue() const { return _gfx_queue; }

        private:
            bool create_instance();
            bool create_surface(void* native_handle)
            VkPhysicalDevice find_physical_device();
            bool find_graphics_queue();
            bool create_device(VkPhysicalDevice p_dev);
            bool initialise_vma();

            // Vulkan Core
            VkInstance _instance = VK_NULL_HANDLE; // Vulkan instance handle
            VkSurfaceKHR _surface = VK_NULL_HANDLE; // Vulkan surface handle
            VkPhysicalDevice _phys_device = VK_NULL_HANDLE; // Vulkan physical device handle
            VkDevice _device = VK_NULL_HANDLE; // Vulkan device handle
            VmaAllocator _allocator = nullptr; // Vulkan Memory Allocator handle

            // Queues
            uint32_t _gfx_queue_fam_idx = UINT32_MAX; // Vulkan graphics queue family index
            VkQueue _gfx_queue = VK_NULL_HANDLE; // Vulkan graphics queue handle

    };
} // namespace renderer