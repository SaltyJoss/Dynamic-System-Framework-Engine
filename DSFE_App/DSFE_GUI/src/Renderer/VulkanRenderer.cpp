#include "Renderer/VulkanRenderer.h"

#include "Assets/VertexHolder.h" 
#include "EngineLib/LogMacros.h"
#include "Platform/Paths.h"

#include <cstring>
namespace renderer {
    // Helper function to create an image memory barrier for Vulkan command buffers
    static void image_barrier(VkCommandBuffer cmd, VkImage image,
        VkImageLayout old_layout, VkImageLayout new_layout,
        VkPipelineStageFlags2 src_stage, VkAccessFlags2 src_access,
        VkPipelineStageFlags2 dst_stage, VkAccessFlags2 dst_access,
        VkImageAspectFlags aspect = VK_IMAGE_ASPECT_COLOR_BIT
    ) {
        VkImageMemoryBarrier2 barrier {
            .sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2,
            .srcStageMask = src_stage, .srcAccessMask = src_access,
            .dstStageMask = dst_stage, .dstAccessMask = dst_access,
            .oldLayout = old_layout, .newLayout = new_layout,
            .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
            .image = image,
            .subresourceRange = { aspect, 0, 1, 0, 1 }
        };
        VkDependencyInfo dep_info {
            .sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO,
            .imageMemoryBarrierCount = 1,
            .pImageMemoryBarriers = &barrier
        };
        vkCmdPipelineBarrier2(cmd, &dep_info);
    }

    // Helper

    // Reads a shader source file whole. Returns empty on failure — the caller logs.
    std::string VulkanRenderer::load_shader_source(const std::string& filename) {
        const std::string path = (paths::assets() / "shaders" / filename).string();

        std::ifstream file(path);
        if (!file) {
            LOG_ERROR("Shader not found: %s", path.c_str());
            return {};
        }
        std::stringstream ss;
        ss << file.rdbuf();
        return ss.str();
    }

    // Compiles one entry point out of already-loaded source. The same source string
    // can be compiled for several stages — HLSL has no per-stage file split.
    VkShaderModule VulkanRenderer::compile_shader(const std::string& source, const std::string& debug_name, shaderc_shader_kind kind, const std::string& entry_point) const {
        shaderc::Compiler compiler;
        shaderc::CompileOptions options;
        options.SetSourceLanguage(shaderc_source_language_glsl);
        options.SetTargetEnvironment(shaderc_target_env_vulkan, shaderc_env_version_vulkan_1_3);
        options.SetOptimizationLevel(shaderc_optimization_level_performance);

        shaderc::SpvCompilationResult result = compiler.CompileGlslToSpv(
            source, kind, debug_name.c_str(), entry_point.c_str(), options);

        if (result.GetCompilationStatus() != shaderc_compilation_status_success) {
            LOG_ERROR("Shader compile failed (%s:%s): %s", debug_name.c_str(), entry_point.c_str(), result.GetErrorMessage().c_str());
            return VK_NULL_HANDLE;
        }

        const std::vector<uint32_t> spirv(result.cbegin(), result.cend());

        VkShaderModuleCreateInfo info{
            .sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO,
            .codeSize = spirv.size() * sizeof(uint32_t),
            .pCode = spirv.data()
        };

        VkShaderModule module = VK_NULL_HANDLE;
        if (vkCreateShaderModule(_context->device(), &info, nullptr, &module) != VK_SUCCESS) {
            LOG_ERROR("vkCreateShaderModule failed (%s:%s)", debug_name.c_str(), entry_point.c_str());
            return VK_NULL_HANDLE;
        }
        return module;
    }

    // Compiles the vertex and fragment shaders, creating shader modules for each.
    ShaderModules VulkanRenderer::create_shaders(const std::string& vertFile, const std::string& fragFile) {
        const std::string vsrc = load_shader_source(vertFile);
        const std::string fsrc = load_shader_source(fragFile);
        ShaderModules modules{ VK_NULL_HANDLE, VK_NULL_HANDLE };

        if (vsrc.empty() || fsrc.empty()) { return { VK_NULL_HANDLE, VK_NULL_HANDLE }; }
        modules.vert = compile_shader(vsrc, vertFile, shaderc_vertex_shader, "main");
        if (modules.vert == VK_NULL_HANDLE) { return { VK_NULL_HANDLE, VK_NULL_HANDLE }; }
        modules.frag = compile_shader(fsrc, fragFile, shaderc_fragment_shader, "main");
        if (modules.frag == VK_NULL_HANDLE) {
            vkDestroyShaderModule(_context->device(), modules.vert, nullptr);
            return { VK_NULL_HANDLE, VK_NULL_HANDLE };
        }

        LOG_INFO("Compiled shaders: vert=%p frag=%p", (void*)modules.vert, (void*)modules.frag);
        return modules;
    }

    VkPipelineLayout VulkanRenderer::create_pipeline_layout() {
        VkPushConstantRange push_range{
            .stageFlags = VK_SHADER_STAGE_VERTEX_BIT,
            .offset = 0,
            .size = sizeof(glm::mat4)
        };
        VkPipelineLayoutCreateInfo layout_info{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
            .pushConstantRangeCount = 1,
            .pPushConstantRanges = &push_range
        };
        VkPipelineLayout layout = VK_NULL_HANDLE;
        if (vkCreatePipelineLayout(_context->device(), &layout_info, nullptr, &layout) != VK_SUCCESS) {
            LOG_ERROR("vkCreatePipelineLayout failed");
            return VK_NULL_HANDLE;
        }
        return layout;
    }

    // Builds the graphics pipeline
    VkPipeline VulkanRenderer::create_graphics_pipeline(VkPipelineLayout layout, ShaderModules shaders) {
        VkDevice dev = _context->device();

        VkPipelineShaderStageCreateInfo stages[2]{
            {
                .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                .stage = VK_SHADER_STAGE_VERTEX_BIT,
                .module = shaders.vert,
                .pName = "main"
            },
            {
                .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                .stage = VK_SHADER_STAGE_FRAGMENT_BIT,
                .module = shaders.frag,
                .pName = "main"
            }
        };

        // Vertices come from constant arrays indexed by SV_VertexID
        VkVertexInputBindingDescription binding{
            .binding = 0,
            .stride = sizeof(assets::VertexHolder),
            .inputRate = VK_VERTEX_INPUT_RATE_VERTEX
        };
        VkVertexInputAttributeDescription attrs[3]{
            { .location = 0, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = offsetof(assets::VertexHolder, _pos) },
            { .location = 1, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = offsetof(assets::VertexHolder, _normal) },
            { .location = 2, .binding = 0, .format = VK_FORMAT_R32G32_SFLOAT,    .offset = offsetof(assets::VertexHolder, _texCoord) }
        };
        VkPipelineVertexInputStateCreateInfo vertex_input{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,
            .vertexBindingDescriptionCount = 1,
            .pVertexBindingDescriptions = &binding,
            .vertexAttributeDescriptionCount = 3,
            .pVertexAttributeDescriptions = attrs
        };

        VkPipelineInputAssemblyStateCreateInfo input_assembly{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
            .topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST
        };

        // Viewport/scissor dynamic so the pipeline survives a resize
        VkPipelineViewportStateCreateInfo viewport_state{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
            .viewportCount = 1,
            .scissorCount = 1
        };

        VkPipelineRasterizationStateCreateInfo raster{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
            .polygonMode = VK_POLYGON_MODE_FILL,
            .cullMode = VK_CULL_MODE_NONE,
            .frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE,
            .lineWidth = 1.0f
        };

        VkPipelineMultisampleStateCreateInfo multisample{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
            .rasterizationSamples = VK_SAMPLE_COUNT_1_BIT
        };

        VkPipelineDepthStencilStateCreateInfo depth_stencil{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,
            .depthTestEnable = VK_TRUE,
            .depthWriteEnable = VK_TRUE,
            .depthCompareOp = VK_COMPARE_OP_LESS,
            .depthBoundsTestEnable = VK_FALSE,
            .stencilTestEnable = VK_FALSE,
            .minDepthBounds = 0.0f,
            .maxDepthBounds = 1.0f
        };

        VkPipelineColorBlendAttachmentState blend_attachment{
            .blendEnable = VK_FALSE,
            .colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                              VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT
        };
        VkPipelineColorBlendStateCreateInfo blend{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
            .attachmentCount = 1,
            .pAttachments = &blend_attachment
        };

        VkDynamicState dynamic_states[]{ VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        VkPipelineDynamicStateCreateInfo dynamic{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
            .dynamicStateCount = 2,
            .pDynamicStates = dynamic_states
        };

        const VkFormat colour_format = _swapchain->format();
        VkPipelineRenderingCreateInfo rendering_info{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,
            .colorAttachmentCount = 1,
            .pColorAttachmentFormats = &colour_format,
            .depthAttachmentFormat = DEPTH_FORMAT
        };

        VkGraphicsPipelineCreateInfo info{
            .sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,
            .pNext = &rendering_info,
            .stageCount = 2,
            .pStages = stages,
            .pVertexInputState = &vertex_input,
            .pInputAssemblyState = &input_assembly,
            .pViewportState = &viewport_state,
            .pRasterizationState = &raster,
            .pMultisampleState = &multisample,
            .pDepthStencilState = &depth_stencil,
            .pColorBlendState = &blend,
            .pDynamicState = &dynamic,
            .layout = layout,
            .renderPass = VK_NULL_HANDLE
        };

        VkPipeline pipeline = VK_NULL_HANDLE;
        if (vkCreateGraphicsPipelines(dev, VK_NULL_HANDLE, 1, &info, nullptr, &pipeline) != VK_SUCCESS) {
            LOG_ERROR("vkCreateGraphicsPipelines failed"); return VK_NULL_HANDLE;
        }
        return pipeline;
    }

    // Create command pools and allocate command buffers for each frame in flight
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

    // Create Vulkan synchronisation resources, including a timeline semaphore and per-frame binary semaphores for image acquisition
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

        // Only the per-frame acquire semaphores live here
        VkSemaphoreCreateInfo binary_info{ .sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
        for (auto& f : _frame_resources) {
            if (vkCreateSemaphore(dev, &binary_info, nullptr, &f.image_acquired_semaphore) != VK_SUCCESS) {
                LOG_ERROR("image-acquired semaphore creation failed"); return false;
            }
        }
        return true;
    }

    // Create a depth image and its associated image view for depth testing in the graphics pipeline
    bool VulkanRenderer::create_depth_resources() {
        VkDevice dev = _context->device();
        const VkExtent2D extent = _swapchain->extent();

        VkImageCreateInfo image_info{
            .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
            .imageType = VK_IMAGE_TYPE_2D,
            .format = DEPTH_FORMAT,
            .extent = { extent.width, extent.height, 1 },
            .mipLevels = 1,
            .arrayLayers = 1,
            .samples = VK_SAMPLE_COUNT_1_BIT,
            .tiling = VK_IMAGE_TILING_OPTIMAL,
            .usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
            .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
            .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED
        };

        VmaAllocationCreateInfo alloc_info{
            .flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT,
            .usage = VMA_MEMORY_USAGE_AUTO,
            .priority = 1.0f
        };

        if (vmaCreateImage(_context->allocator(), &image_info, &alloc_info, &_depth_image, &_depth_image_allocation, nullptr) != VK_SUCCESS) {
            LOG_ERROR("Depth image creation failed"); return false;
        }

        VkImageViewCreateInfo view_info{
            .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
            .image = _depth_image,
            .viewType = VK_IMAGE_VIEW_TYPE_2D,
            .format = DEPTH_FORMAT,
            .subresourceRange = { VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1 }
        };

        if (vkCreateImageView(dev, &view_info, nullptr, &_depth_image_view) != VK_SUCCESS) {
            LOG_ERROR("Depth image view creation failed"); return false;
        }

        LOG_INFO("Depth resources created: %ux%u", extent.width, extent.height);
        return true;
    }

    // Destroys the depth image and its associated image view, freeing the allocated memory
    void VulkanRenderer::destroy_depth_resources() {
        VkDevice dev = _context->device();
        if (_depth_image_view != VK_NULL_HANDLE) {
            vkDestroyImageView(dev, _depth_image_view, nullptr);
            _depth_image_view = VK_NULL_HANDLE;
        }
        if (_depth_image != VK_NULL_HANDLE) {
            vmaDestroyImage(_context->allocator(), _depth_image, _depth_image_allocation);
            _depth_image = VK_NULL_HANDLE;
            _depth_image_allocation = VK_NULL_HANDLE;
        }
    }

    void VulkanRenderer::draw(VkCommandBuffer command_buffer, const renderer::Pipeline& pipeline, 
        const renderer::VulkanRenderer::GpuMesh& mesh, const VkViewport& viewport, const VkRect2D& scissor,
        const glm::mat4& mvp
    ) {
        vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline.pipeline);
        vkCmdSetViewport(command_buffer, 0, 1, &viewport);
        vkCmdSetScissor(command_buffer, 0, 1, &scissor);
        vkCmdPushConstants(command_buffer, pipeline.layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(glm::mat4), &mvp);

        VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(command_buffer, 0, 1, &mesh.vertices.buffer, &offset);
        vkCmdBindIndexBuffer(command_buffer, mesh.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(command_buffer, mesh.index_count, 1, 0, 0, 0);
    }
    

    // Renders a single frame, handling synchronization, command buffer recording, and presentation. This function is called once per frame.
    void VulkanRenderer::render() {
        VkDevice dev = _context->device();

        // Handle a pending recreate at the START of the frame, never mid-frame.
        if (_swapchain->needs_recreate()) { resize(_width, _height); }

        const uint32_t slot = static_cast<uint32_t>(_frame_idx % MAX_FRAMES_IN_FLIGHT);
        FrameResources& f = _frame_resources[slot];

        // Throttle CPU -> wait until the frame that last used this slot has retired.
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

        // Swapchain image barrier
        image_barrier(f.command_buffer, _swapchain->image(image_index),
                      VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                      VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                      VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                      VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
        // Depth image barrier
        image_barrier(f.command_buffer, _depth_image,
                      VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                      VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                      VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT,
                      VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                      VK_IMAGE_ASPECT_DEPTH_BIT);
        // Colour attachment
        VkRenderingAttachmentInfo colour{
            .sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
            .imageView = _swapchain->image_view(image_index),
            .imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            .loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR,
            .storeOp = VK_ATTACHMENT_STORE_OP_STORE,
            .clearValue = {{{ 0.18f, 0.18f, 0.20f, 1.0f }}}
        };
        // Depth attachment
        VkRenderingAttachmentInfo depth{
            .sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
            .imageView = _depth_image_view,
            .imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
            .loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR,
            .storeOp = VK_ATTACHMENT_STORE_OP_STORE,
            .clearValue = {{{ 1.0f, 0 }}}
        };
        // Begin rendering with the specified attachments and render area
        VkRenderingInfo rendering{
            .sType = VK_STRUCTURE_TYPE_RENDERING_INFO,
            .renderArea = {{0, 0}, _swapchain->extent()},
            .layerCount = 1,
            .colorAttachmentCount = 1,
            .pColorAttachments = &colour,
            .pDepthAttachment = &depth
        };
        vkCmdBeginRendering(f.command_buffer, &rendering);

        const VkExtent2D extent = _swapchain->extent();
        VkViewport viewport{
            .x = 0.0f, .y = 0.0f,
            .width  = static_cast<float>(extent.width),
            .height = static_cast<float>(extent.height),
            .minDepth = 0.0f, .maxDepth = 1.0f
        };
        VkRect2D scissor{ {0, 0}, extent };

        // MVP: perspective * view * model. Column-major glm, mul(M,v) in HLSL.
        float aspect = static_cast<float>(extent.width) / static_cast<float>(extent.height);
        glm::mat4 proj = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);
        proj[1][1] *= -1.0f;   // Vulkan clip-space Y is flipped vs glm's GL convention
        glm::mat4 view = glm::lookAt(glm::vec3(2.5f, 2.0f, 3.0f), glm::vec3(0.0f), glm::vec3(0, 1, 0));
        static float t = 0.0f; t += 0.01f;
        glm::mat4 model = glm::rotate(glm::mat4(1.0f), t, glm::vec3(0.3f, 1.0f, 0.0f));
        glm::mat4 mvp = proj * view * model;

        draw(f.command_buffer, _cube_pipeline, _cube, viewport, scissor, mvp);     

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
        destroy_depth_resources();
        create_depth_resources();
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
        if (!create_depth_resources()) { return false; }
        if (!create_command_buffers()) { return false; }
        if (!create_sync_resources()) { return false; }

        // Create the graphics pipeline for rendering (hardcoded to cube for now)
        _cube_pipeline.shaders = create_shaders("cube.vert.glsl", "cube.frag.glsl");
        if (_cube_pipeline.shaders.vert == VK_NULL_HANDLE || _cube_pipeline.shaders.frag == VK_NULL_HANDLE) { return false; }
        _cube_pipeline.layout = create_pipeline_layout();
        if (_cube_pipeline.layout == VK_NULL_HANDLE) { return false; }
        _cube_pipeline.pipeline = create_graphics_pipeline(_cube_pipeline.layout, _cube_pipeline.shaders);
        if (_cube_pipeline.pipeline == VK_NULL_HANDLE) { return false; }

        if (!create_cube()) { return false; }

        LOG_INFO("Vulkan renderer initialised (%ux%u)", _width, _height);
        return true;
    }

    void VulkanRenderer::destroy_pipeline(Pipeline& pipeline) {
        VkDevice dev = _context->device();
        if (pipeline.pipeline) { vkDestroyPipeline(dev, pipeline.pipeline, nullptr); pipeline.pipeline = VK_NULL_HANDLE; }
        if (pipeline.layout)   { vkDestroyPipelineLayout(dev, pipeline.layout, nullptr); pipeline.layout = VK_NULL_HANDLE; }
        if (pipeline.shaders.vert) { vkDestroyShaderModule(dev, pipeline.shaders.vert, nullptr); pipeline.shaders.vert = VK_NULL_HANDLE; }
        if (pipeline.shaders.frag) { vkDestroyShaderModule(dev, pipeline.shaders.frag, nullptr); pipeline.shaders.frag = VK_NULL_HANDLE; }
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

        // Cube pipeline and buffers
        destroy_pipeline(_cube_pipeline);
        destroy_buffer(_cube.vertices);
        destroy_buffer(_cube.indices);

        destroy_depth_resources();
        _swapchain->destroy();
        delete _swapchain; _swapchain = nullptr;
        _context->shutdown();
        delete _context; _context = nullptr;
    }

    // Creates a GPU buffer of the specified size, usage, and memory usage, returning a GpuBuffer struct containing the Vulkan buffer handle and its associated VMA allocation.
    VulkanRenderer::GpuBuffer VulkanRenderer::create_buffer(VkDeviceSize size, VkBufferUsageFlags usage, VmaMemoryUsage memory_usage) {
        VulkanRenderer::GpuBuffer gpu_buffer{};
        GpuBuffer out;

        VkBufferCreateInfo buffer_info{
            .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
            .size = size,
            .usage = usage,
            .sharingMode = VK_SHARING_MODE_EXCLUSIVE
        };
        VmaAllocationCreateInfo alloc_info{
            .flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT,
            .usage = memory_usage
        };
        VkResult r = vmaCreateBuffer(_context->allocator(), &buffer_info, &alloc_info, &out.buffer, &out.allocation, nullptr);
        if (r != VK_SUCCESS) { LOG_ERROR("Failed to create GPU buffer of size %llu (VkResult=%d)", size, r); }
        return out;
    }

    // Destroys a GPU buffer and frees its associated memory allocation
    void VulkanRenderer::destroy_buffer(GpuBuffer& buffer) {
        if (buffer.buffer != VK_NULL_HANDLE) {
            vmaDestroyBuffer(_context->allocator(), buffer.buffer, buffer.allocation);
            buffer.buffer = VK_NULL_HANDLE;
            buffer.allocation = VK_NULL_HANDLE;
        }
    }

    bool VulkanRenderer::create_cube() {
        using assets::VertexHolder;
        const glm::vec3 P[8] = {
            {-0.5f,-0.5f,-0.5f}, {0.5f,-0.5f,-0.5f}, {0.5f,0.5f,-0.5f}, {-0.5f,0.5f,-0.5f},
            {-0.5f,-0.5f, 0.5f}, {0.5f,-0.5f, 0.5f}, {0.5f,0.5f, 0.5f}, {-0.5f,0.5f, 0.5f}
        };
        auto V = [](glm::vec3 p, glm::vec3 n){ return VertexHolder(p, n, {0,0}); };

        std::vector<VertexHolder> verts = {
            V(P[0],{0,0,-1}),V(P[1],{0,0,-1}),V(P[2],{0,0,-1}),V(P[3],{0,0,-1}), // back
            V(P[4],{0,0, 1}),V(P[5],{0,0, 1}),V(P[6],{0,0, 1}),V(P[7],{0,0, 1}), // front
            V(P[0],{-1,0,0}),V(P[3],{-1,0,0}),V(P[7],{-1,0,0}),V(P[4],{-1,0,0}), // left
            V(P[1],{1,0,0}), V(P[2],{1,0,0}), V(P[6],{1,0,0}), V(P[5],{1,0,0}),  // right
            V(P[0],{0,-1,0}),V(P[1],{0,-1,0}),V(P[5],{0,-1,0}),V(P[4],{0,-1,0}), // bottom
            V(P[3],{0,1,0}), V(P[2],{0,1,0}), V(P[6],{0,1,0}), V(P[7],{0,1,0})   // top
        };
        std::vector<uint32_t> indices;
        for (uint32_t f = 0; f < 6; ++f) {
            uint32_t b = f*4;
            indices.insert(indices.end(), { b,b+1,b+2, b,b+2,b+3 });
        }

        const VkDeviceSize vsize = verts.size() * sizeof(VertexHolder);
        const VkDeviceSize isize = indices.size() * sizeof(uint32_t);

        _cube.vertices = create_buffer(vsize, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO);
        _cube.indices  = create_buffer(isize, VK_BUFFER_USAGE_INDEX_BUFFER_BIT,  VMA_MEMORY_USAGE_AUTO);
        _cube.index_count = static_cast<uint32_t>(indices.size());

        // MAPPED_BIT means the allocation's pointer is ready in allocationInfo.
        VmaAllocationInfo vi, ii;
        vmaGetAllocationInfo(_context->allocator(), _cube.vertices.allocation, &vi);
        vmaGetAllocationInfo(_context->allocator(), _cube.indices.allocation, &ii);
        memcpy(vi.pMappedData, verts.data(), vsize);
        memcpy(ii.pMappedData, indices.data(), isize);

        LOG_INFO("Cube uploaded: %zu verts, %u indices", verts.size(), _cube.index_count);
        return true;
    }

} // namespace renderer