#include "Renderer/VulkanRenderer.h"

#include "Assets/VertexHolder.h"
#include "Simulation/SimulationScene.h"
#include "EngineLib/LogMacros.h"
#include "Platform/Paths.h"

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
            .stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
            .offset = 0,
            .size = sizeof(PushConstants)
        };
        VkPipelineLayoutCreateInfo layout_info{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO,
            .setLayoutCount = 1,
            .pSetLayouts = &_camera_set_layout,
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
    VkPipeline VulkanRenderer::create_graphics_pipeline(
        VkPipelineLayout layout, ShaderModules shaders,
        bool alpha_blend, bool depth_write,
        VkSampleCountFlagBits samples
    ) {
        VkDevice dev = _context->device();
        // Shader stages: vertex and fragment
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
        // Input assembly: triangle list, no primitive restart
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
        // Rasterization state: fill polygons, no culling, counter-clockwise front face
        VkPipelineRasterizationStateCreateInfo raster{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
            .polygonMode = VK_POLYGON_MODE_FILL,
            .cullMode = VK_CULL_MODE_NONE,
            .frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE,
            .lineWidth = 1.0f
        };
        // Multisample state: enable MSAA if requested, with a minimum sample shading of 0.25
        VkPipelineMultisampleStateCreateInfo multisample{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
            .rasterizationSamples = samples,
            .sampleShadingEnable = (samples > VK_SAMPLE_COUNT_1_BIT) ? VK_TRUE : VK_FALSE,
            .minSampleShading = 0.25f
        };
        // Depth/stencil state: enable depth testing, optionally enable depth writes, no stencil
        VkPipelineDepthStencilStateCreateInfo depth_stencil{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,
            .depthTestEnable = VK_TRUE,
            .depthWriteEnable = depth_write ? VK_TRUE : VK_FALSE,
            .depthCompareOp = VK_COMPARE_OP_LESS,
            .depthBoundsTestEnable = VK_FALSE,
            .stencilTestEnable = VK_FALSE,
            .minDepthBounds = 0.0f,
            .maxDepthBounds = 1.0f
        };
        // Color blending state: enable alpha blending if requested, otherwise overwrite
        VkPipelineColorBlendAttachmentState blend_attachment{
            .blendEnable = alpha_blend ? VK_TRUE : VK_FALSE,
            .srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA,
            .dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,
            .colorBlendOp = VK_BLEND_OP_ADD,
            .srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE,
            .dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,
            .alphaBlendOp = VK_BLEND_OP_ADD,
            .colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                              VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT
        };
        VkPipelineColorBlendStateCreateInfo blend{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
            .attachmentCount = 1,
            .pAttachments = &blend_attachment
        };
        // Dynamic state: viewport and scissor are dynamic so the pipeline survives a resize
        VkDynamicState dynamic_states[]{ VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        VkPipelineDynamicStateCreateInfo dynamic{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
            .dynamicStateCount = 2,
            .pDynamicStates = dynamic_states
        };
        // Pipeline rendering info: specify the color and depth formats for the pipeline
        const VkFormat colour_format = _swapchain->format();
        VkPipelineRenderingCreateInfo rendering_info{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,
            .colorAttachmentCount = 1,
            .pColorAttachmentFormats = &colour_format,
            .depthAttachmentFormat = DEPTH_FORMAT
        };
        // Create the graphics pipeline with all the specified states and shader stages
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
            .samples = MSAA_SAMPLES,
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

    // Creates the shadow map resources
    bool VulkanRenderer::create_shadow_resources() {
        VkDevice dev = _context->device();
        VkImageCreateInfo image_info{
            .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
            .imageType = VK_IMAGE_TYPE_2D,
            .format = SHADOW_FORMAT,
            .extent = { SHADOW_MAP_SIZE, SHADOW_MAP_SIZE, 1 },
            .mipLevels = 1,
            .arrayLayers = 1,
            .samples = VK_SAMPLE_COUNT_1_BIT,
            .tiling = VK_IMAGE_TILING_OPTIMAL,
            .usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
            .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED
        };
        VmaAllocationCreateInfo alloc_info{ .usage = VMA_MEMORY_USAGE_AUTO };
        if (vmaCreateImage(_context->allocator(), &image_info, &alloc_info, &_shadow_image, &_shadow_alloc, nullptr) != VK_SUCCESS) {
            LOG_ERROR("Shadow image creation failed"); return false;
        }
        VkImageViewCreateInfo view_info{
            .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
            .image = _shadow_image,
            .viewType = VK_IMAGE_VIEW_TYPE_2D,
            .format = SHADOW_FORMAT,
            .subresourceRange = { VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1 }
        };
        if (vkCreateImageView(dev, &view_info, nullptr, &_shadow_view) != VK_SUCCESS) {
            LOG_ERROR("Shadow image view creation failed"); return false;
        }

        VkSamplerCreateInfo sampler_info{
            .sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO,
            .magFilter = VK_FILTER_LINEAR,
            .minFilter = VK_FILTER_LINEAR,
            .mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR,
            .addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
            .addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
            .addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
            .compareEnable = VK_TRUE,
            .compareOp = VK_COMPARE_OP_LESS_OR_EQUAL // Outside the map = lit, inside the map = shadowed
        };
        if (vkCreateSampler(dev, &sampler_info, nullptr, &_shadow_sampler) != VK_SUCCESS) {
            LOG_ERROR("Shadow sampler creation failed"); return false;
        }
        return true;
    }

    // Destroys the shadow map resources, freeing the allocated memory
    void VulkanRenderer::destroy_shadow_resources() {
        VkDevice dev = _context->device();
        if (_shadow_sampler != VK_NULL_HANDLE) {
            vkDestroySampler(dev, _shadow_sampler, nullptr);
            _shadow_sampler = VK_NULL_HANDLE;
        }
        if (_shadow_view != VK_NULL_HANDLE) {
            vkDestroyImageView(dev, _shadow_view, nullptr);
            _shadow_view = VK_NULL_HANDLE;
        }
        if (_shadow_image != VK_NULL_HANDLE) {
            vmaDestroyImage(_context->allocator(), _shadow_image, _shadow_alloc);
            _shadow_image = VK_NULL_HANDLE;
            _shadow_alloc = VK_NULL_HANDLE;
        }
    }

    //  creates the light space matrix for shadow mapping
    glm::mat4 VulkanRenderer::light_space_matrix() const {
        const glm::vec3 light_dir = glm::normalize(glm::vec3(-0.4f, -1.0f, -1.3f));
        const glm::vec3 light_pos = -light_dir * 20.0f;
        glm::mat4 light_view = glm::lookAt(light_pos, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        glm::mat4 light_proj = glm::ortho(-12.0f, 12.0f, -12.0f, 12.0f, 0.1f, 45.0f);
        light_proj[1][1] *= -1.0f; // Vulkan clip space has inverted Y
        return light_proj * light_view;
    }
    // Returns a matrix that mirrors geometry across the Y-axis, useful for reflection rendering.
    glm::mat4 VulkanRenderer::mirror_y_matrix() {
        glm::mat4 m(1.0f);
        m[1][1] = -1.0f;
        return m;
    }

    // Creates a simple grid mesh for rendering a ground plane or reference grid in the scene.
    bool VulkanRenderer::create_grid() {
        using assets::VertexHolder;
        const float S = 100.0f;
        std::vector<VertexHolder> verts = {
            VertexHolder({-S, 0.0f, -S}, {0,1,0}, {0,0}),
            VertexHolder({ S, 0.0f, -S}, {0,1,0}, {1,0}),
            VertexHolder({ S, 0.0f,  S}, {0,1,0}, {1,1}),
            VertexHolder({-S, 0.0f,  S}, {0,1,0}, {0,1})
        };
        std::vector<uint32_t> indices = { 0,1,2, 0,2,3 };

        const VkDeviceSize vsize = verts.size() * sizeof(VertexHolder);
        const VkDeviceSize isize = indices.size() * sizeof(uint32_t);
        _grid_quad.vertices = create_buffer(vsize, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO);
        _grid_quad.indices = create_buffer(isize, VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO);
        _grid_quad.index_count = static_cast<uint32_t>(indices.size());

        VmaAllocationInfo vi, ii;
        vmaGetAllocationInfo(_context->allocator(), _grid_quad.vertices.allocation, &vi);
        vmaGetAllocationInfo(_context->allocator(), _grid_quad.indices.allocation, &ii);
        memcpy(vi.pMappedData, verts.data(), vsize);
        memcpy(ii.pMappedData, indices.data(), isize);

        _grid_pipeline.shaders = create_shaders("grid.vert.glsl", "grid.frag.glsl");
        if (_grid_pipeline.shaders.vert == VK_NULL_HANDLE || _grid_pipeline.shaders.frag == VK_NULL_HANDLE) {
            LOG_ERROR("Failed to create grid shaders"); return false;
        }
        _grid_pipeline.layout = create_pipeline_layout();
        if (_grid_pipeline.layout == VK_NULL_HANDLE) { LOG_ERROR("Failed to create grid pipeline layout"); return false; }
        _grid_pipeline.pipeline = create_graphics_pipeline(_grid_pipeline.layout, _grid_pipeline.shaders, /*alpha_blend=*/false, /*depth_write=*/true);
        return _grid_pipeline.pipeline != VK_NULL_HANDLE;
    }

    // Creates a graphics pipeline specifically for rendering debug lines, using the provided pipeline layout and shader modules.
    VkPipeline VulkanRenderer::create_line_pipeline(VkPipelineLayout layout, ShaderModules shaders) {
        VkDevice dev = _context->device();
        VkPipelineShaderStageCreateInfo stages[2] {
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
        VkVertexInputBindingDescription binding {
            .binding = 0,
            .stride = sizeof(DebugLineVertex),
            .inputRate = VK_VERTEX_INPUT_RATE_VERTEX
        };
        VkVertexInputAttributeDescription attrs[2] {
            { .location = 0, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = offsetof(DebugLineVertex, pos) },
            { .location = 1, .binding = 0, .format = VK_FORMAT_R32G32B32_SFLOAT, .offset = offsetof(DebugLineVertex, colour) }
        };
        VkPipelineVertexInputStateCreateInfo vertex_input {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO,
            .vertexBindingDescriptionCount = 1, .pVertexBindingDescriptions = &binding,
            .vertexAttributeDescriptionCount = 2, .pVertexAttributeDescriptions = attrs
        };
        VkPipelineInputAssemblyStateCreateInfo input_assembly {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO,
            .topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST
        };
        VkPipelineViewportStateCreateInfo viewport_state {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO,
            .viewportCount = 1,
            .scissorCount = 1
        };
        VkPipelineRasterizationStateCreateInfo raster {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO,
            .polygonMode = VK_POLYGON_MODE_FILL,
            .cullMode = VK_CULL_MODE_NONE,
            .frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE,
            .lineWidth = 1.0f
        };
        VkPipelineMultisampleStateCreateInfo multisample {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO,
            .rasterizationSamples = MSAA_SAMPLES
        };
        VkPipelineDepthStencilStateCreateInfo depth_stencil {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO,
            .depthTestEnable = VK_TRUE,
            .depthWriteEnable = VK_TRUE,
            .depthCompareOp = VK_COMPARE_OP_LESS,
            .minDepthBounds = 0.0f,
            .maxDepthBounds = 1.0f
        };
        VkPipelineColorBlendAttachmentState blend_attachment {
            .blendEnable = VK_FALSE,
            .colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT
        };
        VkPipelineColorBlendStateCreateInfo blend {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
            .attachmentCount = 1, .pAttachments = &blend_attachment
        };
        VkDynamicState dynamic_states[] { VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR };
        VkPipelineDynamicStateCreateInfo dynamic {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO,
            .dynamicStateCount = 2, .pDynamicStates = dynamic_states
        };
        const VkFormat colour_format = _swapchain->format();
        VkPipelineRenderingCreateInfo rendering_info {
            .sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO,
            .colorAttachmentCount = 1, .pColorAttachmentFormats = &colour_format,
            .depthAttachmentFormat = DEPTH_FORMAT
        };
        VkGraphicsPipelineCreateInfo info {
            .sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO,
            .pNext = &rendering_info,
            .stageCount = 2, .pStages = stages,
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
            LOG_ERROR("vkCreateGraphicsPipelines (line) failed"); return VK_NULL_HANDLE;
        }
        return pipeline;
    }
    // Creates a graphics pipeline specifically for rendering debug lines, using the provided pipeline layout and shader modules.
    bool VulkanRenderer::create_debug_line_pipeline() {
        _debug_line_pipeline.shaders = create_shaders("debug_line.vert.glsl", "debug_line.frag.glsl");
        if (_debug_line_pipeline.shaders.vert == VK_NULL_HANDLE || _debug_line_pipeline.shaders.frag == VK_NULL_HANDLE) {
            LOG_ERROR("Failed to create debug line shaders"); return false;
        }
        _debug_line_pipeline.layout = create_pipeline_layout();
        if (_debug_line_pipeline.layout == VK_NULL_HANDLE) { LOG_ERROR("Failed to create debug line pipeline layout"); return false; }
        _debug_line_pipeline.pipeline = create_line_pipeline(_debug_line_pipeline.layout, _debug_line_pipeline.shaders);
        return _debug_line_pipeline.pipeline != VK_NULL_HANDLE;
    }
    // Grow the per-frame line vertex buffer to accommodate the specified number of vertices, reallocating and copying existing data if necessary.
    void VulkanRenderer::ensure_debug_line_capacity(uint32_t slot, uint32_t vertexCount) {
        if (vertexCount <= _debug_line_capacity[slot]) { return; }
        wait_idle();
        destroy_buffer(_debug_line_buffers[slot]);
        const uint32_t cap = std::max(vertexCount, 256u);
        _debug_line_buffers[slot] = create_buffer(cap * sizeof(DebugLineVertex), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO);
        _debug_line_capacity[slot] = cap;
    }

    // Draws a mesh using the specified pipeline, command buffer, viewport, scissor rectangle, push constants, and camera descriptor set.
    void VulkanRenderer::draw(VkCommandBuffer command_buffer, const renderer::Pipeline& pipeline, 
        const renderer::VulkanRenderer::GpuMesh& mesh, const VkViewport& viewport, const VkRect2D& scissor,
        const PushConstants& pc, VkDescriptorSet camera_set
    ) {
        vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline.pipeline);
        vkCmdSetViewport(command_buffer, 0, 1, &viewport);
        vkCmdSetScissor(command_buffer, 0, 1, &scissor);
        vkCmdBindDescriptorSets(command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline.layout, 0, 1, &camera_set, 0, nullptr);
        vkCmdPushConstants(command_buffer, pipeline.layout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushConstants), &pc);

        VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(command_buffer, 0, 1, &mesh.vertices.buffer, &offset);
        vkCmdBindIndexBuffer(command_buffer, mesh.indices.buffer, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(command_buffer, mesh.index_count, 1, 0, 0, 0);
    }
    
    // Renders a single frame, handling synchronization, command buffer recording, and presentation. This function is called once per frame.
    void VulkanRenderer::render(const gui::SimulationScene& scene, const glm::mat4& view, const glm::mat4& proj) {
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

        // Shadow Pass
        image_barrier(
            f.command_buffer, _shadow_image,
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
            VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT,
            VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
            VK_IMAGE_ASPECT_DEPTH_BIT
        );
        VkRenderingAttachmentInfo shadow_depth{
            .sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
            .imageView = _shadow_view,
            .imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
            .loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR,
            .storeOp = VK_ATTACHMENT_STORE_OP_STORE,
            .clearValue = {{{ 1.0f, 0 }}}
        };
        VkRenderingInfo shadow_rendering{
            .sType = VK_STRUCTURE_TYPE_RENDERING_INFO,
            .renderArea = {{0, 0}, {SHADOW_MAP_SIZE, SHADOW_MAP_SIZE}},
            .layerCount = 1,
            .colorAttachmentCount = 0,
            .pDepthAttachment = &shadow_depth  
        };
        vkCmdBeginRendering(f.command_buffer, &shadow_rendering);
        {
            VkViewport svp{ 0, 0, (float)SHADOW_MAP_SIZE, (float)SHADOW_MAP_SIZE, 0.0f, 1.0f };
            VkRect2D ssc{ {0, 0}, {SHADOW_MAP_SIZE, SHADOW_MAP_SIZE} };
            const glm::mat4 lsm = light_space_matrix();
            const uint32_t ubo_slot_s = static_cast<uint32_t>(_frame_idx % MAX_FRAMES_IN_FLIGHT);
            for (const gui::Renderable& r : scene.renderables()) {
                if (const GpuMesh* m = get_mesh(r.mesh_id)) {
                    PushConstants pc{};
                    pc.mvp   = lsm * r.transform;
                    //pc.model = r.transform;
                    draw(f.command_buffer, _shadow_pipeline, *m, svp, ssc, pc, _camera_sets[ubo_slot_s]);
                }
            }
        }
        vkCmdEndRendering(f.command_buffer);
        // Transition the shadow map to a read-only layout for sampling in the main pass
        image_barrier(
            f.command_buffer, _shadow_image,
            VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
            VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT,
            VK_IMAGE_ASPECT_DEPTH_BIT
        );

        // Reflection Pass
        image_barrier(
            f.command_buffer, _refl_image,
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT
        );
        image_barrier(
            f.command_buffer, _refl_depth_image,
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
            VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT,
            VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
            VK_IMAGE_ASPECT_DEPTH_BIT
        );
        VkRenderingAttachmentInfo refl_colour{
            .sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
            .imageView = _refl_view,
            .imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            .loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR,
            .storeOp = VK_ATTACHMENT_STORE_OP_STORE,
            .clearValue = {{{ 0.0f, 0.0f, 0.0f, 0.0f }}}   // alpha 0 = "nothing reflected here" mask
        };
        VkRenderingAttachmentInfo refl_depth{
            .sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
            .imageView = _refl_depth_view,
            .imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
            .loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR,
            .storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
            .clearValue = {{{ 1.0f, 0 }}}
        };
        VkRenderingInfo refl_rendering{
            .sType = VK_STRUCTURE_TYPE_RENDERING_INFO,
            .renderArea = {{0, 0}, _refl_extent},
            .layerCount = 1,
            .colorAttachmentCount = 1,
            .pColorAttachments = &refl_colour,
            .pDepthAttachment = &refl_depth
        };
        vkCmdBeginRendering(f.command_buffer, &refl_rendering);
        {
            VkViewport rvp{ 0, 0, (float)_refl_extent.width, (float)_refl_extent.height, 0.0f, 1.0f };
            VkRect2D rsc{ {0, 0}, _refl_extent };
            const glm::mat4 refl_view_proj = proj * view * mirror_y_matrix();
            const uint32_t ubo_slot_r = static_cast<uint32_t>(_frame_idx % MAX_FRAMES_IN_FLIGHT);
            for (const gui::Renderable& r : scene.renderables()) {
                if (const GpuMesh* m = get_mesh(r.mesh_id)) {
                    PushConstants pc{};
                    pc.mvp      = refl_view_proj * r.transform;
                    pc.model    = mirror_y_matrix() * r.transform;   // mirrored world pos/normals for lighting
                    pc.albedo   = r.albedo;
                    pc.material = r.material;
                    draw(f.command_buffer, _refl_mesh_pipeline, *m, rvp, rsc, pc, _camera_sets[ubo_slot_r]);
                }
            }
            // No grid draw — the floor doesn't reflect itself.
        }
        vkCmdEndRendering(f.command_buffer);
        // Transition the reflection colour image to a read-only layout for sampling in the main pass
        image_barrier(
            f.command_buffer, _refl_image,
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
            VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
            VK_ACCESS_2_SHADER_SAMPLED_READ_BIT
        );

        // MSAA colour target barrier

        // MSAA image barrier
        image_barrier(
            f.command_buffer, _msaa_image,
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT
        );

        // Swapchain image barrier
        image_barrier(
            f.command_buffer, _swapchain->image(image_index),
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT
        );
        // Depth image barrier
        image_barrier(
            f.command_buffer, _depth_image,
            VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
            VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
            VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT,
            VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
            VK_IMAGE_ASPECT_DEPTH_BIT
        );
                      
        // Colour attachment
        VkRenderingAttachmentInfo colour{
            .sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO,
            .imageView = _msaa_view,
            .imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            .resolveMode = VK_RESOLVE_MODE_AVERAGE_BIT,
            .resolveImageView = _swapchain->image_view(image_index),
            .resolveImageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
            .loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR,
            .storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE,
            .clearValue = {{{ 0.02f, 0.02f, 0.025f, 1.0f }}}
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

        const glm::mat4 view_proj = proj * view;

        const uint32_t ubo_slot = static_cast<uint32_t>(_frame_idx % MAX_FRAMES_IN_FLIGHT);
        const glm::vec3 cam_pos = glm::vec3(glm::inverse(view)[3]);
        update_camera_ubo(ubo_slot, view, proj, cam_pos);

        for (const gui::Renderable& r : scene.renderables()) {
            if (const GpuMesh* m = get_mesh(r.mesh_id)) {
                PushConstants pc{};
                pc.mvp      = view_proj * r.transform;
                pc.model    = r.transform;
                pc.albedo   = r.albedo;
                pc.material = r.material;
                draw(f.command_buffer, _mesh_pipeline, *m, viewport, scissor, pc, _camera_sets[ubo_slot]);
            }
        }

        // Draw the grid on the XZ plane
        {
            PushConstants pc{};
            pc.mvp   = view_proj;            // world-space quad, identity model
            pc.model = glm::mat4(1.0f);
            draw(f.command_buffer, _grid_pipeline, _grid_quad, viewport, scissor, pc, _camera_sets[ubo_slot]);
        }
        if (!_debug_lines.empty()) {
            const uint32_t vcount = static_cast<uint32_t>(_debug_lines.size());
            ensure_debug_line_capacity(ubo_slot, vcount);
            VmaAllocationInfo ai;
            vmaGetAllocationInfo(_context->allocator(), _debug_line_buffers[ubo_slot].allocation, &ai);
            memcpy(ai.pMappedData, _debug_lines.data(), vcount * sizeof(DebugLineVertex));
            vkCmdBindPipeline(f.command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, _debug_line_pipeline.pipeline);
            vkCmdSetViewport(f.command_buffer, 0, 1, &viewport);
            vkCmdSetScissor(f.command_buffer, 0, 1, &scissor);
            vkCmdBindDescriptorSets(f.command_buffer, VK_PIPELINE_BIND_POINT_GRAPHICS, _debug_line_pipeline.layout, 0, 1, &_camera_sets[ubo_slot], 0, nullptr);
            PushConstants pc{};
            pc.mvp = view_proj;
            vkCmdPushConstants(f.command_buffer, _debug_line_pipeline.layout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(PushConstants), &pc);
            VkDeviceSize offset = 0;
            vkCmdBindVertexBuffers(f.command_buffer, 0, 1, &_debug_line_buffers[ubo_slot].buffer, &offset);
            vkCmdDraw(f.command_buffer, vcount, 1, 0, 0);
        }
        vkCmdEndRendering(f.command_buffer);
        
        // Swapchain image barrier for presentation
        image_barrier(
            f.command_buffer, _swapchain->image(image_index),
            VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
            VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
            VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
            VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT, 0
        );

        VkResult end_res = vkEndCommandBuffer(f.command_buffer);
        if (end_res != VK_SUCCESS) { LOG_ERROR("vkEndCommandBuffer failed: %d", end_res); return; }

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
        VkResult sub_res = vkQueueSubmit2(_context->graphics_queue(), 1, &submit, VK_NULL_HANDLE);
        if (sub_res != VK_SUCCESS) {
            LOG_ERROR("vkQueueSubmit2 failed: %d", sub_res);
            return;
        }

        _swapchain->present(_context->graphics_queue(), image_index);
        ++_frame_idx;
    }

    /*
    Descriptors
    */
    // Creates descriptor set layouts, descriptor pools, and allocates descriptor sets for camera uniform buffers and shadow/reflection textures.
    bool VulkanRenderer::create_descriptors() {
        VkDevice dev = _context->device();
        VkDescriptorSetLayoutBinding bindings[3]{
            {
                .binding = 0, // Camera UBO
                .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                .descriptorCount = 1,
                .stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT
            },
            {
                .binding = 1, // Shadow map
                .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                .descriptorCount = 1,
                .stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT
            },
            {
                .binding = 2, // Reflection colour
                .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                .descriptorCount = 1,
                .stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT
            }
        };
        VkDescriptorSetLayoutCreateInfo layout_info{
            .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO,
            .bindingCount = 3,
            .pBindings = bindings
        };
        if (vkCreateDescriptorSetLayout(dev, &layout_info, nullptr, &_camera_set_layout) != VK_SUCCESS) {
            LOG_ERROR("vkCreateDescriptorSetLayout failed"); return false;
        }

        VkDescriptorPoolSize pool_sizes[2]{
            {
                .type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                .descriptorCount = MAX_FRAMES_IN_FLIGHT
            },
            {
                .type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                .descriptorCount = MAX_FRAMES_IN_FLIGHT * 2 // shadow + reflection
            }
        };
        VkDescriptorPoolCreateInfo pool_info{
            .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO,
            .maxSets = MAX_FRAMES_IN_FLIGHT,
            .poolSizeCount = 2,
            .pPoolSizes = pool_sizes
        };
        if (vkCreateDescriptorPool(dev, &pool_info, nullptr, &_descriptor_pool) != VK_SUCCESS) {
            LOG_ERROR("vkCreateDescriptorPool failed"); return false;
        }

        for (uint32_t i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i) {
            _camera_ubos[i] = create_buffer(sizeof(CameraUBO), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO);

            VkDescriptorSetAllocateInfo alloc_info{
                .sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO,
                .descriptorPool = _descriptor_pool,
                .descriptorSetCount = 1,
                .pSetLayouts = &_camera_set_layout
            };
            if (vkAllocateDescriptorSets(dev, &alloc_info, &_camera_sets[i]) != VK_SUCCESS) {
                LOG_ERROR("vkAllocateDescriptorSets failed"); return false;
            }

            // Write: bind this UBO to binding 0 of this set.
            VkDescriptorBufferInfo buffer_info{
                .buffer = _camera_ubos[i].buffer,
                .offset = 0,
                .range = sizeof(CameraUBO)
            };
            // Write: bind the shadow map to binding 1 of this set.
            VkDescriptorImageInfo shadow_info{
                .sampler = _shadow_sampler,
                .imageView = _shadow_view,
                .imageLayout = VK_IMAGE_LAYOUT_DEPTH_READ_ONLY_OPTIMAL
            };
            // Write:
            VkDescriptorImageInfo refl_info{
                .sampler = _refl_sampler,
                .imageView = _refl_view,
                .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
            };
            // Write: bind the UBO and shadow map to the descriptor set
            VkWriteDescriptorSet writes[3]{
                {
                    .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                    .dstSet = _camera_sets[i],
                    .dstBinding = 0,
                    .dstArrayElement = 0,
                    .descriptorCount = 1,
                    .descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                    .pBufferInfo = &buffer_info
                },
                {
                    .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                    .dstSet = _camera_sets[i],
                    .dstBinding = 1,
                    .dstArrayElement = 0,
                    .descriptorCount = 1,
                    .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    .pImageInfo = &shadow_info
                },
                {
                    .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                    .dstSet = _camera_sets[i],
                    .dstBinding = 2,
                    .dstArrayElement = 0,
                    .descriptorCount = 1,
                    .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                    .pImageInfo = &refl_info
                }
            };
            vkUpdateDescriptorSets(dev, 3, writes, 0, nullptr);
        }
        LOG_INFO("Descriptors created (%u camera UBOs)", MAX_FRAMES_IN_FLIGHT);
        return true;
    }
    // Updates the camera uniform buffer object (UBO) for the specified frame slot with the provided view and projection matrices, as well as the camera position.
    void VulkanRenderer::update_camera_ubo(uint32_t frame_slot, const glm::mat4& view, const glm::mat4& proj, const glm::vec3& cam_pos) {
        CameraUBO data{ view, proj, light_space_matrix(), glm::vec4(cam_pos, 1.0f) };
        VmaAllocationInfo info;
        vmaGetAllocationInfo(_context->allocator(), _camera_ubos[frame_slot].allocation, &info);
        memcpy(info.pMappedData, &data, sizeof(CameraUBO));
    }

    // Destroys the descriptor sets, descriptor pool, and descriptor set layout.
    void VulkanRenderer::destroy_descriptors() {
        VkDevice dev = _context->device();
        for (uint32_t i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i) {
            destroy_buffer(_camera_ubos[i]);
            _camera_sets[i] = VK_NULL_HANDLE;
        }
        if (_descriptor_pool) { vkDestroyDescriptorPool(dev, _descriptor_pool, nullptr); _descriptor_pool = VK_NULL_HANDLE; }
        if (_camera_set_layout) { vkDestroyDescriptorSetLayout(dev, _camera_set_layout, nullptr); _camera_set_layout = VK_NULL_HANDLE; }
    }

    // Create the shadow map graphics pipeline
    bool VulkanRenderer::create_shadow_pipeline() {
        _shadow_pipeline.shaders = create_shaders("shadow.vert.glsl", "shadow.frag.glsl");
        if (_shadow_pipeline.shaders.vert == VK_NULL_HANDLE || _shadow_pipeline.shaders.frag == VK_NULL_HANDLE) {
            LOG_ERROR("Failed to create shadow shaders"); return false;
        }
        _shadow_pipeline.layout = create_pipeline_layout();
        if (_shadow_pipeline.layout == VK_NULL_HANDLE) { LOG_ERROR("Failed to create shadow pipeline layout"); return false; }

        VkDevice dev = _context->device();

        VkPipelineShaderStageCreateInfo stages[2]{
            {
                .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                .stage = VK_SHADER_STAGE_VERTEX_BIT,
                .module = _shadow_pipeline.shaders.vert,
                .pName = "main"
            },
            {
                .sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                .stage = VK_SHADER_STAGE_FRAGMENT_BIT,
                .module = _shadow_pipeline.shaders.frag,
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
            .depthBiasEnable = VK_TRUE,
            .depthBiasConstantFactor = 1.25f,
            .depthBiasSlopeFactor = 1.75f,
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

        VkPipelineColorBlendStateCreateInfo blend{
            .sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO,
            .attachmentCount = 0,
            .pAttachments = nullptr
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
            .colorAttachmentCount = 0,
            .depthAttachmentFormat = SHADOW_FORMAT
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
            .layout = _shadow_pipeline.layout,
            .renderPass = VK_NULL_HANDLE
        };
        if (vkCreateGraphicsPipelines(dev, VK_NULL_HANDLE, 1, &info, nullptr, &_shadow_pipeline.pipeline) != VK_SUCCESS) {
            LOG_ERROR("Failed to create shadow graphics pipeline"); return false;
        }
        return true;
    }

    // Creates the resources needed for MSAA rendering, including an image and image view.
    bool VulkanRenderer::create_msaa_resources() {
        VkDevice dev = _context->device();
        const VkExtent2D extent = _swapchain->extent();
        VkImageCreateInfo image_info{
            .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
            .imageType = VK_IMAGE_TYPE_2D,
            .format = _swapchain->format(),
            .extent = { extent.width, extent.height, 1 },
            .mipLevels = 1,
            .arrayLayers = 1,
            .samples = MSAA_SAMPLES,
            .tiling = VK_IMAGE_TILING_OPTIMAL,
            .usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT,
            .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED
        };
        VmaAllocationCreateInfo alloc_info{ .usage = VMA_MEMORY_USAGE_AUTO };
        if (vmaCreateImage(_context->allocator(), &image_info, &alloc_info, &_msaa_image, &_msaa_alloc, nullptr) != VK_SUCCESS) {
            LOG_ERROR("Failed to create MSAA image"); return false;
        }
        VkImageViewCreateInfo view_info{
            .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
            .image = _msaa_image,
            .viewType = VK_IMAGE_VIEW_TYPE_2D,
            .format = _swapchain->format(),
            .subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 }
        };
        if (vkCreateImageView(dev, &view_info, nullptr, &_msaa_view) != VK_SUCCESS) {
            LOG_ERROR("Failed to create MSAA image view"); return false;
        }
        LOG_INFO("MSAA resources created (%ux samples, %ux%u)", (uint32_t)MSAA_SAMPLES, extent.width, extent.height);
        return true;
    }
    // Destroys the MSAA resources.
    void VulkanRenderer::destroy_msaa_resources() {
        VkDevice dev = _context->device();
        if (_msaa_view) { vkDestroyImageView(dev, _msaa_view, nullptr); _msaa_view = VK_NULL_HANDLE; }
        if (_msaa_image) { vmaDestroyImage(_context->allocator(), _msaa_image, _msaa_alloc); _msaa_image = VK_NULL_HANDLE; _msaa_alloc = VK_NULL_HANDLE; }
    }

    // Creates the resources needed for reflection rendering.
    bool VulkanRenderer::create_reflection_resources() {
        VkDevice dev = _context->device();
        const VkExtent2D sc = _swapchain->extent();
        _refl_extent = {
            std::max(1u, sc.width / REFLECTION_DIVISOR),
            std::max(1u, sc.height / REFLECTION_DIVISOR)
        };

        // Colour image for reflection rendering
        VkImageCreateInfo colour_info{
            .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
            .imageType = VK_IMAGE_TYPE_2D,
            .format = _swapchain->format(),
            .extent = { _refl_extent.width, _refl_extent.height, 1 },
            .mipLevels = 1,
            .arrayLayers = 1,
            .samples = VK_SAMPLE_COUNT_1_BIT,
            .tiling = VK_IMAGE_TILING_OPTIMAL,
            .usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
            .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED
        };
        VmaAllocationCreateInfo alloc_info{ .usage = VMA_MEMORY_USAGE_AUTO };
        if (vmaCreateImage(_context->allocator(), &colour_info, &alloc_info, &_refl_image, &_refl_alloc, nullptr) != VK_SUCCESS) {
            LOG_ERROR("Failed to create reflection colour image"); return false;
        }
        // Create an image view for the reflection colour image
        VkImageViewCreateInfo colour_view_info{
            .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
            .image = _refl_image,
            .viewType = VK_IMAGE_VIEW_TYPE_2D,
            .format = _swapchain->format(),
            .subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 }
        };
        if (vkCreateImageView(dev, &colour_view_info, nullptr, &_refl_view) != VK_SUCCESS) {
            LOG_ERROR("Failed to create reflection colour image view"); return false;
        }

        // Depth image for reflection rendering
        VkImageCreateInfo depth_info{
            .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
            .imageType = VK_IMAGE_TYPE_2D,
            .format = DEPTH_FORMAT,
            .extent = { _refl_extent.width, _refl_extent.height, 1 },
            .mipLevels = 1,
            .arrayLayers = 1,
            .samples = VK_SAMPLE_COUNT_1_BIT,
            .tiling = VK_IMAGE_TILING_OPTIMAL,
            .usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
            .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED
        };
        if (vmaCreateImage(_context->allocator(), &depth_info, &alloc_info, &_refl_depth_image, &_refl_depth_alloc, nullptr) != VK_SUCCESS) {
            LOG_ERROR("Failed to create reflection depth image"); return false;
        }
        // Create an image view for the reflection depth image
        VkImageViewCreateInfo depth_view{
            .sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO,
            .image = _refl_depth_image,
            .viewType = VK_IMAGE_VIEW_TYPE_2D,
            .format = DEPTH_FORMAT,
            .subresourceRange = { VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1 }
        };
        if (vkCreateImageView(dev, &depth_view, nullptr, &_refl_depth_view) != VK_SUCCESS) {
            LOG_ERROR("Failed to create reflection depth image view"); return false;
        }

        // Plain linear sampler
        VkSamplerCreateInfo sampler_info{
            .sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO,
            .magFilter = VK_FILTER_LINEAR,
            .minFilter = VK_FILTER_LINEAR,
            .mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR,
            .addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
            .addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
            .addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
        };
        if (vkCreateSampler(dev, &sampler_info, nullptr, &_refl_sampler) != VK_SUCCESS) {
            LOG_ERROR("Failed to create reflection sampler"); return false;
        }
        LOG_INFO("Reflection resources created (%ux%u)", _refl_extent.width, _refl_extent.height);
        return true;
    }
    // Destroys the reflection resources, including the sampler, colour image and view, and depth image and view.
    void VulkanRenderer::destroy_reflection_resources() {
        VkDevice dev = _context->device();
        if (_refl_sampler) { vkDestroySampler(dev, _refl_sampler, nullptr); _refl_sampler = VK_NULL_HANDLE; }
        if (_refl_view) { vkDestroyImageView(dev, _refl_view, nullptr); _refl_view = VK_NULL_HANDLE; }
        if (_refl_image) { vmaDestroyImage(_context->allocator(), _refl_image, _refl_alloc); _refl_image = VK_NULL_HANDLE; _refl_alloc = VK_NULL_HANDLE; }
        if (_refl_depth_view) { vkDestroyImageView(dev, _refl_depth_view, nullptr); _refl_depth_view = VK_NULL_HANDLE; }
        if (_refl_depth_image) { vmaDestroyImage(_context->allocator(), _refl_depth_image, _refl_depth_alloc); _refl_depth_image = VK_NULL_HANDLE; _refl_depth_alloc = VK_NULL_HANDLE; }
    }

    // Rewrites the reflection descriptor set for each frame in flight, updating the binding for the reflection colour image and sampler.
    void VulkanRenderer::rewrite_reflection_descriptor() {
        VkDescriptorImageInfo refl_info{
            .sampler = _refl_sampler,
            .imageView = _refl_view,
            .imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
        };
        for (uint32_t i=0; i < MAX_FRAMES_IN_FLIGHT; ++i) {
            VkWriteDescriptorSet write{
                .sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET,
                .dstSet = _camera_sets[i],
                .dstBinding = 2,
                .dstArrayElement = 0,
                .descriptorCount = 1,
                .descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                .pImageInfo = &refl_info
            };
            vkUpdateDescriptorSets(_context->device(), 1, &write, 0, nullptr);
        }
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

    /*
     * Resizing andf waiting for idle
     */
    // Resizes the Vulkan renderer to the specified width and height, recreating the swapchain and associated resources as needed.
    void VulkanRenderer::resize(uint32_t width, uint32_t height) {
        if (width == 0 || height == 0) { return; }
        _width = width; _height = height;
        wait_idle();
        destroy_reflection_resources();
        destroy_msaa_resources();
        destroy_depth_resources();
        _swapchain->recreate(_width, _height);
        create_depth_resources();
        create_msaa_resources();
        create_reflection_resources();
        rewrite_reflection_descriptor(); // otherwise sets point at the old view
    }
    // Waits for the Vulkan device to become idle, ensuring that all pending operations are completed before proceeding.
    void VulkanRenderer::wait_idle() {
        if (_context && _context->device()) { vkDeviceWaitIdle(_context->device()); }
    }

    /*
    Initialisation and Destruction
    */

    bool VulkanRenderer::init(const NativeWindow& win, uint32_t width, uint32_t height) {
        _width = width;
        _height = height;
        _context = new VulkanContext();
        if (!_context->init(win)) { LOG_ERROR("VulkanContext init failed"); return false; }
        _swapchain = new VulkanSwapchain();
        if (!_swapchain->create(*_context, _width, _height, SWAPCHAIN_FORMAT)) {
            LOG_ERROR("VulkanSwapchain create failed"); return false;
        }
        if (!create_depth_resources()) { return false; }
        if (!create_msaa_resources()) { return false; }
        if (!create_command_buffers()) { return false; }
        if (!create_sync_resources()) { return false; }
        if (!create_shadow_resources()) { return false; }
        if (!create_reflection_resources()) { return false; }
        if (!create_descriptors()) { return false; }
        if (!create_shadow_pipeline()) { return false; }
        if (!create_grid()) { return false; }
        if (!create_debug_line_pipeline()) { return false; }

        // Create the graphics pipeline for rendering (hardcoded to cube for now)
        _mesh_pipeline.shaders = create_shaders("lit.vert.glsl", "lit.frag.glsl");
        if (_mesh_pipeline.shaders.vert == VK_NULL_HANDLE || _mesh_pipeline.shaders.frag == VK_NULL_HANDLE) { return false; }
        _mesh_pipeline.layout = create_pipeline_layout();
        if (_mesh_pipeline.layout == VK_NULL_HANDLE) { return false; }
        _mesh_pipeline.pipeline = create_graphics_pipeline(_mesh_pipeline.layout, _mesh_pipeline.shaders);
        if (_mesh_pipeline.pipeline == VK_NULL_HANDLE) { return false; }

        _refl_mesh_pipeline.shaders = create_shaders("lit.vert.glsl", "lit.frag.glsl");
        if (_refl_mesh_pipeline.shaders.vert == VK_NULL_HANDLE || _refl_mesh_pipeline.shaders.frag == VK_NULL_HANDLE) { return false; }
        _refl_mesh_pipeline.layout = create_pipeline_layout();
        if (_refl_mesh_pipeline.layout == VK_NULL_HANDLE) { return false; }
        _refl_mesh_pipeline.pipeline = create_graphics_pipeline(_refl_mesh_pipeline.layout, _refl_mesh_pipeline.shaders, false, true, VK_SAMPLE_COUNT_1_BIT);
        if (_refl_mesh_pipeline.pipeline == VK_NULL_HANDLE) { return false; }

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

        destroy_pipeline(_refl_mesh_pipeline);
        destroy_pipeline(_shadow_pipeline);
        destroy_shadow_resources();
        destroy_descriptors();
        for (auto& f : _frame_resources) {
            if (f.image_acquired_semaphore) { vkDestroySemaphore(dev, f.image_acquired_semaphore, nullptr); }
            if (f.command_pool) { vkDestroyCommandPool(dev, f.command_pool, nullptr); }
            f = {};
        }

        if (_timeline_semaphore) { vkDestroySemaphore(dev, _timeline_semaphore, nullptr); _timeline_semaphore = VK_NULL_HANDLE; }

        // Destroy the graphics pipeline and its associated resources
        destroy_pipeline(_mesh_pipeline);
        for (auto& m : _meshes) { destroy_buffer(m.vertices); destroy_buffer(m.indices); }
        _meshes.clear();
        destroy_pipeline(_grid_pipeline);
        destroy_buffer(_grid_quad.vertices);
        destroy_buffer(_grid_quad.indices);
        destroy_pipeline(_debug_line_pipeline);
        for (auto& b : _debug_line_buffers) { destroy_buffer(b); }

        destroy_reflection_resources();
        destroy_msaa_resources();
        destroy_depth_resources();
        _swapchain->destroy();
        delete _swapchain; _swapchain = nullptr;
        _context->shutdown();
        delete _context; _context = nullptr;
    }

    /*
    Mesh Uploading
    */

    uint32_t VulkanRenderer::upload_mesh(const std::vector<assets::VertexHolder>& vertices, const std::vector<uint32_t>& indices) {
        VulkanRenderer::GpuMesh mesh;

        const VkDeviceSize vsize = vertices.size() * sizeof(assets::VertexHolder);
        const VkDeviceSize isize = indices.size() * sizeof(uint32_t);

        mesh.vertices = create_buffer(vsize, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO);
        mesh.indices  = create_buffer(isize, VK_BUFFER_USAGE_INDEX_BUFFER_BIT,  VMA_MEMORY_USAGE_AUTO);
        mesh.index_count = static_cast<uint32_t>(indices.size());

        VmaAllocationInfo vi, ii;
        vmaGetAllocationInfo(_context->allocator(), mesh.vertices.allocation, &vi);
        vmaGetAllocationInfo(_context->allocator(), mesh.indices.allocation, &ii);
        memcpy(vi.pMappedData, vertices.data(), vsize);
        memcpy(ii.pMappedData, indices.data(), isize);

        const uint32_t mesh_id = static_cast<uint32_t>(_meshes.size());
        _meshes.push_back(mesh);
        LOG_INFO("Uploaded mesh %u: %zu verts, %u indices", mesh_id, vertices.size(), mesh.index_count);
        return mesh_id;
    }

    // Destroys all uploaded meshes, freeing their associated GPU buffers and clearing the mesh registry.
    void VulkanRenderer::destroy_all_meshes() {
        wait_idle();
        for (auto& m : _meshes) {
            destroy_buffer(m.vertices);
            destroy_buffer(m.indices);
        }
        _meshes.clear();
        LOG_INFO("GPU mesh registry cleared");
    }
} // namespace renderer