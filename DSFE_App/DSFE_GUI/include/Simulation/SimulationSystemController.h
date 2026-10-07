// DSFE_GUI Simulation/SimulationSystemController.h
#pragma once

#include "Systems/ISimulationSystem.h"
#include <memory>
#include <vector>

namespace gui {
    class SimulationScene;

    class SimulationSystemController {
        public:
            ISimulationSystem* add(std::unique_ptr<ISimulationSystem> system, SimulationScene& scene);
            void update_all(SimulationScene& scene);
            void clear_all(SimulationScene& scene);

        private:
            std::vector<std::unique_ptr<ISimulationSystem>> _systems;
    };
} // namespace gui