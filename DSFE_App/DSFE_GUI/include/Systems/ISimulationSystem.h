// DSFE_GUI Simulation/ISimulationSystem.h
#pragma once

namespace gui {
    class SimulationScene;
    class ISimulationSystem {
        public:
            virtual ~ISimulationSystem() = default;
            virtual void build(SimulationScene& scene) = 0;
            virtual void update(SimulationScene& scene) = 0;
            virtual void clear(SimulationScene& scene) = 0;
    };
} // namespace gui