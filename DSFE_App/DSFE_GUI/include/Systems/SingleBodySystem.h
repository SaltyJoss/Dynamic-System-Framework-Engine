// DSFE_GUI Systems/SingleBodySystem.h
#pragma once

#include "Systems/ISimulationSystem.h"
#include <core/Types.h>

#include <string>
#include <functional>
#include <cstdint>

#include "Platform/Logger.h"

namespace single_body_system { class SingleBodySystem; }  // Core

namespace gui {
    class MeshStore;
    class SimulationRenderer;

    class SingleBodySystem : public ISimulationSystem {
        public:
            SingleBodySystem(const std::string& mesh_path,
                single_body_system::SingleBodySystem& core_body,
                float scale, MeshStore& mesh_store, SimulationRenderer& renderer);
            
            void build(SimulationScene& scene) override;
            void update(SimulationScene& scene) override;
            void clear(SimulationScene& scene) override;

        private:
            const std::string _mesh_path;
            single_body_system::SingleBodySystem& _core_body;
            float _scale;
            MeshStore& _meshStore;
            SimulationRenderer& _renderer;
            uint32_t _renderable = UINT32_MAX;
    };

} // namespace gui