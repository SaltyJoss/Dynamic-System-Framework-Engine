// DSFE_GUI Simulation/SimulationScene.h
#pragma once 

#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

namespace gui {
    struct Renderable {
        uint32_t mesh_id;
        glm::mat4 transform;
    };

    class SimulationScene {
        public:
            uint32_t add_renderable(uint32_t mesh_id, const glm::mat4& transform);
            void set_transform(uint32_t idx, const glm::mat4& transform);
            const std::vector<Renderable>& renderables() const { return _renderables; }
            void clear() { _renderables.clear(); }

        private:
            std::vector<Renderable> _renderables;
    };
} // namespace gui