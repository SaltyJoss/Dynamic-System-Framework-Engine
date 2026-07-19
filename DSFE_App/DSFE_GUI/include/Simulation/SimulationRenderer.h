// DSFE_GUI Simulation/SimulationRenderer.h
#pragma once

#include "Assets/VertexHolder.h"
#include <vector>
#include <cstdint>

namespace renderer { class VulkanRenderer; }

namespace gui {
    class SimulationRenderer {
        public:
            explicit SimulationRenderer(renderer::VulkanRenderer& renderer);
            uint32_t upload(const std::vector<assets::VertexHolder>& vertices, const std::vector<uint32_t>& indices);

        private:
            renderer::VulkanRenderer& _renderer;
    };
} // namespace gui