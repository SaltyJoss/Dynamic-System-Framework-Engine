// DSFE_GUI Simulation/SimulationRenderer.h
#pragma once

#include <string>
#include <cstring>

namespace renderer { class VulkanRenderer; }

namespace gui {
    class SimulationRenderer {
        public:
            explicit SimulationRenderer(renderer::VulkanRenderer& renderer);
            uint32_t load_mesh(const std::string& path);

        private:
            renderer::VulkanRenderer& _renderer;
    };
} // namespace gui