// DSFE_GUI Renderer/VulkanSwapchain.h
#pragma once

#include <volk.h>
#include <vector>
#include <cstdint>

#include "Platform/Logger.h"

namespace renderer {
    class VulkanContext;

    // VulkanSwapchain class for managing the Vulkan swapchain and its associated resources
    class VulkanSwapchain {
        public:
            bool create(VulkanContext& context, uint32_t width, uint32_t height, VkFormat format = VK_FORMAT_B8G8R8A8_SRGB);
            void destroy();

            // Recreate the swapchain with new dimensions, returning true if successful
            bool recreate(uint32_t width, uint32_t height );

            // Acquire and Present methods for Vulkan swapchain images
            VkResult acquire(VkSemaphore image_acquired, uint32_t& out_index);
            VkResult present(VkQueue queue, uint32_t image_index);

            // Accessors for swapchain properties and resources
            VkSwapchainKHR handle()  const { return _swapchain; }
            VkFormat format()  const { return _swapchain_format; }
            VkExtent2D extent()  const { return { _swapchain_width, _swapchain_height }; }
            uint32_t image_count() const { return static_cast<uint32_t>(_swapchain_images.size()); }
            VkImage image(uint32_t i)      const { return _swapchain_images[i]; }
            VkImageView image_view(uint32_t i) const { return _swapchain_image_views[i]; }
            VkSemaphore render_complete_semaphore(uint32_t i) const { return _render_complete_semaphores[i]; }
            bool needs_recreate() const { return _require_swapchain_recreate; }

        private:
            // Swapchain variables
            VulkanContext* _context = nullptr; // Pointer to the Vulkan context
            VkSwapchainKHR _swapchain = VK_NULL_HANDLE; // Vulkan swapchain handle
            VkFormat _swapchain_format = VK_FORMAT_B8G8R8A8_SRGB; // Format of the swapchain images
            std::vector<VkImage> _swapchain_images; // Vulkan swapchain images
            std::vector<VkImageView> _swapchain_image_views; // Vulkan swapchain
            std::vector<VkSemaphore> _render_complete_semaphores; // Vulkan semaphores for render completion
            bool _require_swapchain_recreate = false; // Flag to indicate if swapchain recreation is required
            uint32_t _swapchain_width = 0; // Swapchain width
            uint32_t _swapchain_height = 0; // Swapchain height
    };
} // namespace renderer