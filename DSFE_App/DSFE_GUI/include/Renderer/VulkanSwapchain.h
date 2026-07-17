#pragma once

#include <volk.h>
#include <vector>
#include <cstdint>

namespace renderer {
    class VulkanContext;

    // VulkanSwapchain class for managing the Vulkan swapchain and its associated resources
    class VulkanSwapchain {
        public:
            bool create(VulkanContext& context, uint32_t width, uint32_t height );
            void destroy();

            // Accessors
            VkSwapchainKHR handle() const { return _swapchain; }
            const std::vector<VkImage>& images() const { return _swapchain_images; }
            const std::vector<VkImageView>& image_views() const { return _swapchain_image_views; }

        private:
            // Swapchain variables
            VkSwapchainKHR _swapchain = VK_NULL_HANDLE; // Vulkan swapchain handle
            std::vector<VkImage> _swapchain_images; // Vulkan swapchain images
            std::vector<VkImageView> _swapchain_image_views; // Vulkan swapchain
            std::vector<VkSemaphore> _render_complete_sempahores; // Vulkan semaphores for render completion
            bool require_swapchain_recreate = false; // Flag to indicate if swapchain recreation is required
            uint32_t _swapchain_width = 0; // Swapchain width
            uint32_t _swapchain_height = 0; // Swapchain height
    };
} // namespace renderer