// DSFE_GUI Renderer/VulkanSwapchain.cpp
#include "Renderer/VulkanSwapchain.h"
#include "Renderer/VulkanContext.h"

#include "EngineLib/LogMacros.h"

#include <algorithm>

namespace renderer {
    // Creates a Vulkan swapchain with the specified width, height, and format. Returns true if successful.
    bool VulkanSwapchain::create(VulkanContext& context, uint32_t width, uint32_t height, VkFormat format) {
        _context = &context;

        VkPhysicalDevice phys = _context->physical_device();
        VkDevice dev = _context->device();
        VkSurfaceKHR surface = _context->surface();

        // --- Surface capabilities ---
        VkSurfaceCapabilitiesKHR caps{};
        if (vkGetPhysicalDeviceSurfaceCapabilitiesKHR(phys, surface, &caps) != VK_SUCCESS) {
            LOG_ERROR("vkGetPhysicalDeviceSurfaceCapabilitiesKHR failed");
            return false;
        }

        // currentExtent == 0xFFFFFFFF means the surface defers to us
        VkExtent2D extent{};
        const bool caller_size_valid = (width > 1 && height > 1);
        if (caller_size_valid) {
            extent.width  = std::clamp(width,  caps.minImageExtent.width,  caps.maxImageExtent.width);
            extent.height = std::clamp(height, caps.minImageExtent.height, caps.maxImageExtent.height);
        }
        else if (caps.currentExtent.width != UINT32_MAX) {
            extent = caps.currentExtent;
        } else {
            extent.width  = std::clamp(width,  caps.minImageExtent.width,  caps.maxImageExtent.width);
            extent.height = std::clamp(height, caps.minImageExtent.height, caps.maxImageExtent.height);
        }
        // If the extent is zero, it likely means the window is minimised or not yet ready for rendering
        if (extent.width == 0 || extent.height == 0) {
            LOG_WARN("Swapchain extent is zero (window minimised?) — deferring creation");
            _require_swapchain_recreate = true;
            return false;
        }

        // --- Format ---
        uint32_t format_count = 0;
        vkGetPhysicalDeviceSurfaceFormatsKHR(phys, surface, &format_count, nullptr);
        if (format_count == 0) { LOG_ERROR("No surface formats available"); return false; }

        std::vector<VkSurfaceFormatKHR> formats(format_count);
        vkGetPhysicalDeviceSurfaceFormatsKHR(phys, surface, &format_count, formats.data());

        VkSurfaceFormatKHR chosen = formats.front(); // fallback
        for (const auto& f : formats) {
            if (f.format == format && f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) { chosen = f; break; }
        }
        if (chosen.format != format) {
            LOG_WARN("Requested swapchain format %d unsupported; using %d", format, chosen.format);
        }
        _swapchain_format = chosen.format;

        // --- Present mode: FIFO is always supported and is the sane default ---
        VkPresentModeKHR present_mode = VK_PRESENT_MODE_FIFO_KHR;
        // --- Image count ---
        uint32_t image_count = caps.minImageCount + 1;
        if (caps.maxImageCount > 0 && image_count > caps.maxImageCount) { image_count = caps.maxImageCount; }

        VkSwapchainKHR old_swapchain = _swapchain;

        VkSwapchainCreateInfoKHR info{
            .sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR,
            .surface = surface,
            .minImageCount = image_count,
            .imageFormat = chosen.format,
            .imageColorSpace = chosen.colorSpace,
            .imageExtent = extent,
            .imageArrayLayers = 1,
            .imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
            .imageSharingMode = VK_SHARING_MODE_EXCLUSIVE,
            .preTransform = caps.currentTransform,
            .compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR,
            .presentMode = present_mode,
            .clipped = VK_TRUE,
            .oldSwapchain = old_swapchain
        };

        VkSwapchainKHR new_swapchain = VK_NULL_HANDLE;
        VkResult result = vkCreateSwapchainKHR(dev, &info, nullptr, &new_swapchain);
        if (result != VK_SUCCESS) {
            LOG_ERROR("vkCreateSwapchainKHR failed: %d", result); return false;
        }

        // Tear down the old views/semaphores now that creation succeeded.
        // NOTE: destroy() nulls _swapchain, so old_swapchain is destroyed explicitly below.
        for (VkImageView view : _swapchain_image_views) { vkDestroyImageView(dev, view, nullptr); }
        _swapchain_image_views.clear();
        for (VkSemaphore s : _render_complete_semaphores) { vkDestroySemaphore(dev, s, nullptr); }
        _render_complete_semaphores.clear();
        if (old_swapchain != VK_NULL_HANDLE) { vkDestroySwapchainKHR(dev, old_swapchain, nullptr); }

        _swapchain = new_swapchain;
        _swapchain_width = extent.width;
        _swapchain_height = extent.height;

        // --- Images ---
        uint32_t actual_count = 0;
        vkGetSwapchainImagesKHR(dev, _swapchain, &actual_count, nullptr);
        _swapchain_images.resize(actual_count);
        vkGetSwapchainImagesKHR(dev, _swapchain, &actual_count, _swapchain_images.data());

        // --- Image views ---
        _swapchain_image_views.resize(actual_count);
        for (uint32_t i = 0; i < actual_count; ++i) {
            VkImageViewCreateInfo view_info{
                .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
                .image = _swapchain_images[i],
                .viewType = VK_IMAGE_VIEW_TYPE_2D,
                .format = _swapchain_format,
                .components = { VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY,
                                VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY },
                .subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 }
            };
            if (vkCreateImageView(dev, &view_info, nullptr, &_swapchain_image_views[i]) != VK_SUCCESS) {
                LOG_ERROR("vkCreateImageView failed for swapchain image %u", i); return false;
            }
        }

        // --- Per-image render-complete semaphores ---
        // Per image, not per frame-in-flight: acquire can hand back images out of order.
        VkSemaphoreCreateInfo sem_info{ .sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
        _render_complete_semaphores.resize(actual_count);
        for (uint32_t i = 0; i < actual_count; ++i) {
            if (vkCreateSemaphore(dev, &sem_info, nullptr, &_render_complete_semaphores[i]) != VK_SUCCESS) {
                LOG_ERROR("render-complete semaphore creation failed for image %u", i); return false;
            }
        }

        _require_swapchain_recreate = false;
        LOG_INFO("Swapchain created: %ux%u, %u images, format %d", _swapchain_width, _swapchain_height, actual_count, _swapchain_format);
        return true;
    }

    // Recreates the Vulkan swapchain with new dimensions, returning true if successful. If the swapchain is not yet created, this will create it.
    bool VulkanSwapchain::recreate(uint32_t width, uint32_t height) {
        if (!_context) { LOG_ERROR("recreate() called before create()"); return false; }
        return create(*_context, width, height, _swapchain_format);
    }

    // Destroys the Vulkan swapchain and its associated resources, including image views and semaphores.
    void VulkanSwapchain::destroy() {
        if (!_context) { return; }
        VkDevice dev = _context->device();
        for (VkSemaphore s : _render_complete_semaphores) { vkDestroySemaphore(dev, s, nullptr); }
        _render_complete_semaphores.clear();
        for (VkImageView view : _swapchain_image_views) { vkDestroyImageView(dev, view, nullptr); }
        _swapchain_image_views.clear();
        // Swapchain images are owned by the swapchain — do not destroy them individually.
        _swapchain_images.clear();
        if (_swapchain != VK_NULL_HANDLE) {
            vkDestroySwapchainKHR(dev, _swapchain, nullptr);
            _swapchain = VK_NULL_HANDLE;
        }
        _swapchain_width = 0;
        _swapchain_height = 0;
        _context = nullptr;
    }

    // Acquires the next available swapchain image, signalling the provided semaphore when ready
    VkResult VulkanSwapchain::acquire(VkSemaphore image_acquired, uint32_t& out_index) {
        VkResult result = vkAcquireNextImageKHR(_context->device(), _swapchain, UINT64_MAX, image_acquired, VK_NULL_HANDLE, &out_index);
        if (result == VK_ERROR_OUT_OF_DATE_KHR) {
            // Nothing was signalled - safe to recreate before the next frame.
            _require_swapchain_recreate = true;
        } else if (result == VK_SUBOPTIMAL_KHR) {
            // The semaphore WAS signalled; the frame must complete. Flag for next frame.
            _require_swapchain_recreate = true;
        }
        return result;
    }

    // Presents the specified swapchain image to the given queue, waiting on the corresponding render-complete semaphore
    VkResult VulkanSwapchain::present(VkQueue queue, uint32_t image_index) {
        VkSemaphore wait = _render_complete_semaphores[image_index];
        VkPresentInfoKHR info{
            .sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR,
            .waitSemaphoreCount = 1,
            .pWaitSemaphores = &wait,
            .swapchainCount = 1,
            .pSwapchains = &_swapchain,
            .pImageIndices = &image_index
        };
        VkResult result = vkQueuePresentKHR(queue, &info);
        if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) {
            _require_swapchain_recreate = true;
        }
        return result;
    }

} // namespace renderer