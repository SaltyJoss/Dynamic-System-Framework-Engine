// DSFE_GUI Simulation/SimulationRenderer.cpp
#include "Simulation/SimulationRenderer.h"
#include "Renderer/VulkanRenderer.h"
#include "Assets/MeshLoader.h"
#include "Scene/Mesh.h"

#include "EngineLib/LogMacros.h"

namespace gui {
    SimulationRenderer::SimulationRenderer(renderer::VulkanRenderer& renderer) : _renderer(renderer) {}

    uint32_t SimulationRenderer::load_mesh(const std::string& path) {
        assets::MeshLoader loader;
        auto meshes = loader.load(path);
        if (meshes.empty() || meshes.front()->_vertices.empty()) {
            LOG_ERROR("SimulationRenderer::loadMesh: no geometry in %s", path.c_str());
            return renderer::VulkanRenderer::INVALID_MESH_ID;
        }
        const scene::Mesh& src = *meshes.front();
        std::vector<uint32_t> idx(src._indices.begin(), src._indices.end());
        return _renderer.upload_mesh(src._vertices, idx);
    }
} // namespace gui 