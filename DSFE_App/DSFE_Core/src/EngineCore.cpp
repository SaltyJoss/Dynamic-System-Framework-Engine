#include "pch.h"
// File:   EngineCore.cpp
// GitHub: SaltyJoss
#include "EngineCore.h"
#include "Scene/SimulationCore.h"

#include "Physics/PhysicsSystem.h"
#include "Robots/RobotSystem.h"
#include "Robots/TrajectoryManager.h"
#include "Scene/Object.h"

#include <vector>
#include <memory>

// Factory function definitions
extern "C" {
	// Factory function to create a SimulationCore instance
    DSFE_API core::ISimulationCore* CreateSimulationCore_v1() {
        try { 
            auto* core = new core::SimulationCore();

            auto* physics = new physics::PhysicsSystem();

            // Headless object container
            auto* objects = new std::vector<std::unique_ptr<scene::Object>>();
            // Robot system must be constructible WITHOUT OpenGL
            auto* robot = new robots::RobotSystem(
                *objects, [](const std::string&) {
                    return std::vector<scene::Object*>{};
                }
            );
			// Trajectory manager must also be constructible without OpenGL
            auto* traj = new control::TrajectoryManager();

            core->setPhysicsSystem(physics);
            core->setRobotSystem(robot);
            core->setObjects(objects);
            core->setTrajectoryManager(traj);

            return core;
        }
        catch (...) { return nullptr; /*Avoids throwing exceptions across C ABI boundary by return null on failure*/ }
    }
	// Destroys a SimulationCore instance
    DSFE_API void DestroySimulationCore(core::ISimulationCore* p) {
        delete p; // makes sure that deletion happens in the dll that created it.
    }
} // extern "C"