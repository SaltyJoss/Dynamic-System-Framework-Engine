// DSFE_GUI Systems/MultiBodySystem.h
#pragma once

#include "Systems/ISimulationSystem.h"
#include "Systems/RigidBodyBinding.h"
#include <core/Types.h>

#include <string>
#include <vector>
#include <cstdint>
#include <functional>

#include "Platform/Logger.h"

namespace systems { struct RigidBodyModel; }
namespace assets { class MeshLoader; }

namespace gui {
    class MeshStore;
    class SimulationRenderer;

    class RigidBodySystem : public ISimulationSystem {
        public:
            RigidBodySystem(const robots::RigidBodyModel& model, 
                std::function<const std::vector<mathlib::Mat4>&()> world_src,
                MeshStore& mesh_store, SimulationRenderer& renderer);
            
            void build(SimulationScene& scene) override;
            void update(SimulationScene& scene) override;
            void clear(SimulationScene& scene) override;

        private:
            const robots::RigidBodyModel& _model;
            std::function<const std::vector<mathlib::Mat4>&()> _world_src;
            MeshStore& _meshStore;
            SimulationRenderer& _renderer;
            RobotBinding _binding;
    };
} // namespace gui

