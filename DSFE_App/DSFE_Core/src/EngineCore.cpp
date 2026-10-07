// DSFE_CORE EngineCore.cpp
#include "pch.h"
#include "EngineCore.h"
#include "Scene/SimulationCore.h"

#include "Systems/RigidBodySystem.h"
#include "Systems/TrajectoryManager.h"

#include <vector>
#include <memory>

// Factory function definitions
extern "C" {
	// Factory function to create a SimulationCore instance
    DSFE_API core::ISimulationCore* CreateSimulationCore_v1() {
        try { return new core::SimulationCore(); }
        catch (...) { return nullptr; }
    }
	// Destroys a SimulationCore instance
    DSFE_API void DestroySimulationCore(core::ISimulationCore* p) {
        delete p; // makes sure that deletion happens in the dll that created it.
    }
} // extern "C"