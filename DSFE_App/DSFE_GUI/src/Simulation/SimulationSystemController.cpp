// DSFE_GUI SimulationSystemController.cpp
#include "Simulation/SimulationSystemController.h"

namespace gui {
    ISimulationSystem* SimulationSystemController::add(std::unique_ptr<ISimulationSystem> system, SimulationScene& scene) {
        if (!system) { return nullptr; }
        system->build(scene);
        ISimulationSystem* ptr = system.get();
        _systems.push_back(std::move(system));
        return ptr;
    }

    void SimulationSystemController::update_all(SimulationScene& scene) {
        for (auto& system : _systems) { system->update(scene); }
    }

    void SimulationSystemController::clear_all(SimulationScene& scene) {
        for (auto& system : _systems) { system->clear(scene); }
        _systems.clear();
    }
} // namespace gui