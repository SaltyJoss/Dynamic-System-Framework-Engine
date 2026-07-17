#include "Renderer/VulkanRenderer.h"
#include "EngineLib/LogMacros.h"

namespace renderer {

    static void image_barrier(VkCommandBuffer cmd, VkImage image,
        VkImageLayout old_layout, VkImageLayout new_layout,
        VkPipelineStageFlags2 src_stage, VkAccessFlags2 src_access,
        VkPipelineStageFlags2 dst_stage, VkAccessFlags2 dst_access
    ) {
        VkImageMemoryBarrier2 barrier {
            .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2,
            .srcStageMask = src_stage, .srcAccessMask = src_access,
            .dstStageMask = dst_stage, .dstAccessMask = dst_access,
            .oldLayout = old_layout, .newLayout = new_layout,
            .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .image = image,
            .subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 }
        };
        VkDependencyInfo dep_info {
            .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
            .imageMemoryBarrierCount = 1,
            .pImageMemoryBarriers = &barrier
        };
        vkCmdPipelineBarrier2(cmd, &dep_info);
    }

    bool VulkanRenderer::create_command_buffers() {
        VkDevice dev = _context->device();
        for (auto& f : _frame_resources) {
            VkCommandPoolCreateInfo pool_info{
                .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
                .flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT,
                .queueFamilyIndex = _context->graphics_queue_family()
            };
            if (vkCreateCommandPool(dev, &pool_info, nullptr, &f.command_pool) != VK_SUCCESS) {
                LOG_ERROR("vkCreateCommandPool failed"); return false;
            }
            VkCommandBufferAllocateInfo alloc{
                .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
                .commandPool = f.command_pool,
                .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
                .commandBufferCount = 1
            };
            if (vkAllocateCommandBuffers(dev, &alloc, &f.command_buffer) != VK_SUCCESS) {
                LOG_ERROR("vkAllocateCommandBuffers failed"); return false;
            }
        }
        return true;
    }

    bool VulkanRenderer::create_sync_resources() {
        VkDevice dev = _context->device();

        VkSemaphoreTypeCreateInfo type_info{
            .sType = VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO,
            .semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE,
            .initialValue = 0
        };
        VkSemaphoreCreateInfo timeline_info{
            .sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO,
            .pNext = &type_info
        };
        if (vkCreateSemaphore(dev, &timeline_info, nullptr, &_timeline_semaphore) != VK_SUCCESS) {
            LOG_ERROR("timeline semaphore creation failed"); return false;
        }

        // Only the per-frame acquire semaphores live here.
        // Render-complete semaphores are per swapchain image and owned by VulkanSwapchain.
        VkSemaphoreCreateInfo binary_info{ .sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
        for (auto& f : _frame_resources) {
            if (vkCreateSemaphore(dev, &binary_info, nullptr, &f.image_acquired_semaphore) != VK_SUCCESS) {
                LOG_ERROR("image-acquired semaphore creation failed"); return false;
            }
        }
        return true;
    }

    void VulkanRenderer::render() {
        VkDevice dev = _context->device();

        // Handle a pending recreate at the START of the frame, never mid-frame.
        if (_swapchain->needs_recreate()) { resize(_width, _height); }

        const uint32_t slot = static_cast<uint32_t>(_frame_idx % MAX_FRAMES_IN_FLIGHT);
        FrameResources& f = _frame_resources[slot];

        // Throttle CPU: wait until the frame that last used this slot has retired.
        if (_frame_idx >= MAX_FRAMES_IN_FLIGHT) {
            const uint64_t wait_val = _frame_idx - MAX_FRAMES_IN_FLIGHT + 1;
            VkSemaphoreWaitInfo wait{
                .sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO,
                .semaphoreCount = 1,
                .pSemaphores = &_timeline_semaphore,
                .pValues = &wait_val
            };
            vkWaitSemaphores(dev, &wait, UINT64_MAX);
        }

        uint32_t image_index = 0;
        VkResult acq = _swapchain->acquire(f.image_acquired_semaphore, image_index);
        if (acq == VK_ERROR_OUT_OF_DATE_KHR) { return; } // flagged; recreated next frame
        if (acq != VK_SUCCESS && acq != VK_SUBOPTIMAL_KHR) {
            LOG_ERROR("acquire failed: %d", acq); return;
        }

        vkResetCommandPool(dev, f.command_pool, 0);
        VkCommandBufferBeginInfo begin{
            .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
            .flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT
        };
        vkBeginCommandBuffer(f.command_buffer, &begin);

        image_barrier(f.command_buffer, _swapchain->image(image_index),
                      VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                      VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                      VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                      VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

        VkRenderingAttachmentInfo colour{
            .sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
            .imageView = _swapchain->image_view(image_index),
            .imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            .loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR,
            .storeOp = VK_ATTACHMENT_STORE_OP_STORE,
            .clearValue = {{{ 0.18f, 0.18f, 0.20f, 1.0f }}}
        };
        VkRenderingInfo rendering{
            .sType = VK_STRUCTURE_TYPE_RENDERING_INFO,
            .renderArea = {{0, 0}, _swapchain->extent()},
            .layerCount = 1,
            .colorAttachmentCount = 1,
            .pColorAttachments = &colour
        };
        vkCmdBeginRendering(f.command_buffer, &rendering);
        // TODO: bind _pipeline and draw once create_graphics_pipeline() lands
        vkCmdEndRendering(f.command_buffer);

        image_barrier(f.command_buffer, _swapchain->image(image_index),
                      VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                      VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                      VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                      VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT, 0);

        vkEndCommandBuffer(f.command_buffer);

        VkSemaphoreSubmitInfo wait_sem{
            .sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
            .semaphore = f.image_acquired_semaphore,
            .stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT
        };
        VkSemaphoreSubmitInfo signal_sems[2]{
            {
                .sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
                .semaphore = _swapchain->render_complete_semaphore(image_index),
                .stageMask = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT
            },
            {
                .sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO,
                .semaphore = _timeline_semaphore,
                .value = _frame_idx + 1,
                .stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT
            }
        };
        VkCommandBufferSubmitInfo cmd_info{
            .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO,
            .commandBuffer = f.command_buffer
        };
        VkSubmitInfo2 submit{
            .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2,
            .waitSemaphoreInfoCount = 1,   .pWaitSemaphoreInfos = &wait_sem,
            .commandBufferInfoCount = 1,   .pCommandBufferInfos = &cmd_info,
            .signalSemaphoreInfoCount = 2, .pSignalSemaphoreInfos = signal_sems
        };
        if (vkQueueSubmit2(_context->graphics_queue(), 1, &submit, VK_NULL_HANDLE) != VK_SUCCESS) {
            LOG_ERROR("vkQueueSubmit2 failed"); return;
        }

        _swapchain->present(_context->graphics_queue(), image_index);
        ++_frame_idx;
    }

    void VulkanRenderer::resize(uint32_t width, uint32_t height) {
        if (width == 0 || height == 0) { return; }
        _width = width; _height = height;
        wait_idle();
        _swapchain->recreate(_width, _height);
        // TODO: recreate depth image here once it exists
    }

    void VulkanRenderer::wait_idle() {
        if (_context && _context->device()) { vkDeviceWaitIdle(_context->device()); }
    }

    bool VulkanRenderer::init(void* windowHandle) {
        _context = new VulkanContext();
        if (!_context->init(windowHandle)) { LOG_ERROR("VulkanContext init failed"); return false; }

        _swapchain = new VulkanSwapchain();
        if (!_swapchain->create(*_context, _width, _height, SWAPCHAIN_FORMAT)) {
            LOG_ERROR("VulkanSwapchain create failed"); return false;
        }

        if (!create_command_buffers()) { return false; }
        if (!create_sync_resources()) { return false; }

        LOG_INFO("Vulkan renderer initialised (%ux%u)", _width, _height);
        return true;
    }

    void VulkanRenderer::shutdown() {
        if (!_context) { return; }
        wait_idle();
        VkDevice dev = _context->device();

        for (auto& f : _frame_resources) {
            if (f.image_acquired_semaphore) { vkDestroySemaphore(dev, f.image_acquired_semaphore, nullptr); }
            if (f.command_pool) { vkDestroyCommandPool(dev, f.command_pool, nullptr); }
            f = {};
        }
        if (_timeline_semaphore) { vkDestroySemaphore(dev, _timeline_semaphore, nullptr); _timeline_semaphore = VK_NULL_HANDLE; }
        if (_pipeline)           { vkDestroyPipeline(dev, _pipeline, nullptr); _pipeline = VK_NULL_HANDLE; }
        if (_pipeline_layout)    { vkDestroyPipelineLayout(dev, _pipeline_layout, nullptr); _pipeline_layout = VK_NULL_HANDLE; }
        if (_vert_shader)        { vkDestroyShaderModule(dev, _vert_shader, nullptr); _vert_shader = VK_NULL_HANDLE; }
        if (_frag_shader)        { vkDestroyShaderModule(dev, _frag_shader, nullptr); _frag_shader = VK_NULL_HANDLE; }

        _swapchain->destroy();
        delete _swapchain; _swapchain = nullptr;
        _context->shutdown();
        delete _context; _context = nullptr;
    }

} // namespace renderer