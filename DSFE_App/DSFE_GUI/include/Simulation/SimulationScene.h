// DSFE_GUI Simulation/SimulationScene.h
#pragma once 

#include <glm/glm.hpp>
#include <vector>
#include <cstdint>

namespace gui {
    struct Renderable {
        uint32_t mesh_id;
        glm::mat4 transform;
        glm::vec4 albedo{ 0.55f, 0.55f, 0.58f, 1.0f };
        glm::vec4 material{ 0.2f, 0.5f, 1.0f, 0.0f }; // x=metallic, y=roughness, z=ao
    };

    class SimulationScene {
        public:
            uint32_t add_renderable(uint32_t mesh_id, const glm::mat4& transform);
            void set_material(uint32_t idx, const glm::vec3& albedo, float metallic, float roughness, float ao=1.0f);
            void set_transform(uint32_t idx, const glm::mat4& transform);
            const std::vector<Renderable>& renderables() const { return _renderables; }
            void clear() { _renderables.clear(); }

        private:
            std::vector<Renderable> _renderables;
    };
} // namespace gui