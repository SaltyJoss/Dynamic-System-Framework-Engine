#pragma once

#include "Renderer/NativeWindow.h"

#include <volk.h>

#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <array>
#include <vector>
#include <shaderc/shaderc.hpp>

#include "VulkanContext.h"
#include "VulkanSwapchain.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/glm.hpp>

#include "Platform/Logger.h"

namespace assets { class VertexHolder; }
namespace gui { class SimulationScene; struct Renderable; }

namespace renderer {
    // Structure to hold per-frame resources for Vulkan rendering
    struct FrameResources {
        VkCommandPool command_pool = nullptr;
        VkCommandBuffer command_buffer = nullptr;
        VkSemaphore image_acquired_semaphore = nullptr;
    };

    struct ShaderModules {
        VkShaderModule vert = VK_NULL_HANDLE;
        VkShaderModule frag = VK_NULL_HANDLE;
    };

    struct Pipeline {
        ShaderModules shaders;
        VkPipeline pipeline = VK_NULL_HANDLE;
        VkPipelineLayout layout = VK_NULL_HANDLE;
    };

    class VulkanRenderer {
        // Constants for Vulkan rendering
        constexpr static uint32_t MAX_FRAMES_IN_FLIGHT{ 2 };
        constexpr static VkFormat SWAPCHAIN_FORMAT{  VK_FORMAT_B8G8R8A8_SRGB };
        constexpr static VkFormat DEPTH_FORMAT{ VK_FORMAT_D32_SFLOAT };

        // Push constants structure for passing data to shaders
        struct PushConstants {
            glm::mat4 mvp;
            glm::mat4 model;          // for world-space normals + position in the fragment shader
            glm::vec4 albedo;         // xyz colour, w unused
            glm::vec4 material;       // x=metallic, y=roughness, z=ao, w unused
        };

        public:
            bool init(const NativeWindow& win, uint32_t width, uint32_t height);
            void shutdown();
            void render(const gui::SimulationScene& scene, const glm::mat4& view, const glm::mat4& proj);
            void resize(uint32_t width, uint32_t height);
            void wait_idle();

        private:
            // Internal state
            VulkanContext* _context = nullptr;
            VulkanSwapchain* _swapchain = nullptr;

            // Frame resources for double/triple buffering
            uint32_t _width = 1920;
            uint32_t _height = 1080;
            uint64_t _frame_idx = 0; // Current frame index for double/triple buffering
            uint64_t _next_signal_val = MAX_FRAMES_IN_FLIGHT - 1;

            // Shader variables (For GLSL to SPIR-V compilation, however later I will be using HLSL for shader compilation)
            VkShaderModule _vert_shader = VK_NULL_HANDLE;
	        VkShaderModule _frag_shader = VK_NULL_HANDLE;
            ShaderModules create_shaders(const std::string& vertFile, const std::string& fragFile);
            static std::string load_shader_source(const std::string& filename);
            VkShaderModule compile_shader(const std::string& source, const std::string& debug_name, shaderc_shader_kind kind, const std::string& entry_point) const;

            // Graphics Pipeline variables
            VkPipeline _pipeline = VK_NULL_HANDLE; // Vulkan graphics pipeline handle
            VkPipelineLayout _pipeline_layout = VK_NULL_HANDLE; // Vulkan pipeline layout handle
            VkPipelineLayout create_pipeline_layout();
            VkPipeline create_graphics_pipeline(
                VkPipelineLayout layout, ShaderModules shaders,
                bool alpha_blend = false, bool depth_write = true,
                VkSampleCountFlagBits samples = MSAA_SAMPLES
            );
            void destroy_pipeline(Pipeline& pipeline);

            // Depth buffer variables
            VkImage _depth_image = VK_NULL_HANDLE; // Vulkan depth image handle
            VkImageView _depth_image_view = VK_NULL_HANDLE; // Vulkan depth image view handle
            VmaAllocation _depth_image_allocation = VK_NULL_HANDLE; // Vulkan depth image allocation
            bool create_depth_resources();
            void destroy_depth_resources();

            // Shadow mapping variables
            static constexpr uint32_t SHADOW_MAP_SIZE = 4096;
            static constexpr VkFormat SHADOW_FORMAT = VK_FORMAT_D32_SFLOAT;
            VkImage _shadow_image = VK_NULL_HANDLE; // Vulkan shadow map image handle
            VmaAllocation _shadow_alloc = VK_NULL_HANDLE; // Vulkan shadow map image allocation
            VkImageView _shadow_view = VK_NULL_HANDLE; // Vulkan shadow map image view
            VkSampler _shadow_sampler = VK_NULL_HANDLE; // Vulkan shadow map sampler
            Pipeline _shadow_pipeline; // Vulkan graphics pipeline for shadow mapping
            bool create_shadow_resources();
            void destroy_shadow_resources();
            bool create_shadow_pipeline();
            // Methods for calculating light space matrix and mirror transformation
            glm::mat4 light_space_matrix() const;
            static glm::mat4 mirror_y_matrix();

            // MSAA variables
            static constexpr VkSampleCountFlagBits MSAA_SAMPLES = VK_SAMPLE_COUNT_4_BIT;
            VkImage _msaa_image = VK_NULL_HANDLE; // Vulkan MSAA image handle
            VmaAllocation _msaa_alloc = VK_NULL_HANDLE; // Vulkan MSAA image allocation
            VkImageView _msaa_view = VK_NULL_HANDLE; // Vulkan MSAA image view
            bool create_msaa_resources();
            void destroy_msaa_resources();

            // Descriptor set methods
            bool create_descriptors();
            void destroy_descriptors();
            void update_camera_ubo(uint32_t frame_slot, const glm::mat4& view, const glm::mat4& proj, const glm::vec3& cam_pos);

            // Planar floor reflection variables
            static constexpr uint32_t REFLECTION_DIVISOR = 2; // Half swapchain resolution for reflection rendering
            VkImage _refl_image = VK_NULL_HANDLE; // Vulkan reflection image handle
            VmaAllocation _refl_alloc = VK_NULL_HANDLE; // Vulkan reflection image allocation
            VkImageView _refl_view = VK_NULL_HANDLE; // Vulkan reflection image view
            VkImage _refl_depth_image = VK_NULL_HANDLE; // Vulkan reflection depth image handle
            VmaAllocation _refl_depth_alloc = VK_NULL_HANDLE; // Vulkan reflection depth image
            VkImageView _refl_depth_view = VK_NULL_HANDLE; // Vulkan reflection depth image view
            VkSampler _refl_sampler = VK_NULL_HANDLE; // Vulkan reflection image sampler
            VkExtent2D _refl_extent{};
            Pipeline _refl_mesh_pipeline;
            bool create_reflection_resources();
            void destroy_reflection_resources();
            void rewrite_reflection_descriptor();

            // Synchronisation variables
            VkSemaphore _timeline_semaphore = VK_NULL_HANDLE; // Vulkan timeline semaphore for synchronisation
            std::array<FrameResources, MAX_FRAMES_IN_FLIGHT> _frame_resources; // Vector of frame resources for each frame
            bool create_sync_resources();
            bool create_command_buffers();

        private:
            // GPU buffer structure containing Vulkan buffer and VMA allocation
            struct GpuBuffer {
                VkBuffer buffer = VK_NULL_HANDLE;
                VmaAllocation allocation = nullptr;
            };
            // Debug line vertex structure containing position and colour for rendering debug lines
            struct DebugLineVertex {
                glm::vec3 pos;
                glm::vec3 colour;
            };
            // GPU mesh structure containing vertex and index buffers
            struct GpuMesh {
                GpuBuffer vertices;
                GpuBuffer indices;
                uint32_t index_count = 0;
            };
            // Methods for creating and destroying GPU buffers
            void draw(VkCommandBuffer cmdB, const Pipeline& pipeline, 
                const VulkanRenderer::GpuMesh& mesh, const VkViewport& viewport, const VkRect2D& scissor,
                const PushConstants& pc, VkDescriptorSet camera_set
            );
            GpuBuffer create_buffer(VkDeviceSize size, VkBufferUsageFlags usage, VmaMemoryUsage mem_usage);
            void destroy_buffer(GpuBuffer& buf);

            std::vector<GpuMesh> _meshes;
            Pipeline _mesh_pipeline;
            Pipeline _grid_pipeline;
            GpuMesh  _grid_quad{};
            bool create_grid();

            // Debug line rendering
            Pipeline _debug_line_pipeline;
            std::array<GpuBuffer, MAX_FRAMES_IN_FLIGHT> _debug_line_buffers;
            std::array<uint32_t, MAX_FRAMES_IN_FLIGHT> _debug_line_capacity{};
            std::vector<DebugLineVertex> _debug_lines;
            bool create_debug_line_pipeline();
            VkPipeline create_line_pipeline(VkPipelineLayout layout, ShaderModules shaders);
            void ensure_debug_line_capacity(uint32_t slot, uint32_t vertexCount);

            // Camera uniform buffer object structure for passing camera data to shaders
            struct CameraUBO {
                glm::mat4 view;
                glm::mat4 proj;
                glm::mat4 light_space; // for shadow mapping
                glm::vec4 cam_pos;   // xyz used; vec4 for std140 alignment
            };
            
            // Descriptor set layout and pool for camera uniform buffer
            VkDescriptorSetLayout _camera_set_layout = VK_NULL_HANDLE;
            VkDescriptorPool      _descriptor_pool    = VK_NULL_HANDLE;

            // One UBO + descriptor set per frame-in-flight (can't touch a UBO the GPU is reading)
            std::array<GpuBuffer, MAX_FRAMES_IN_FLIGHT>      _camera_ubos;
            std::array<VkDescriptorSet, MAX_FRAMES_IN_FLIGHT> _camera_sets{};

        public:
            static constexpr uint32_t INVALID_MESH_ID = UINT32_MAX;

            uint32_t upload_mesh(const std::vector<assets::VertexHolder>& vertices, const std::vector<uint32_t>& indices);
            void destroy_all_meshes();
            const GpuMesh* get_mesh(uint32_t id) const { return id < _meshes.size() ? &_meshes[id] : nullptr; }

            void debug_lines_clear() { _debug_lines.clear(); }
            void debug_line(const glm::vec3& start, const glm::vec3& end, const glm::vec3& colour) {
                _debug_lines.push_back({ start, colour });
                _debug_lines.push_back({ end,   colour });
            }
    };
} // namespace renderer
