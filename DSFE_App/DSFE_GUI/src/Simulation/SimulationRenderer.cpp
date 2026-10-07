// DSFE_GUI Simulation/SimulationRenderer.cpp
#include "Simulation/SimulationRenderer.h"
#include "Renderer/VulkanRenderer.h"

#include "EngineLib/LogMacros.h"

namespace gui {
    SimulationRenderer::SimulationRenderer(renderer::VulkanRenderer& renderer) : _renderer(renderer) {}

    uint32_t SimulationRenderer::upload(const std::vector<assets::VertexHolder>& vertices, const std::vector<uint32_t>& indices) {
        return _renderer.upload_mesh(vertices, indices);
    }
} // namespace gui 