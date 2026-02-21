#include "pch.h"
// File:   EngineCore.cpp
// GitHub: SaltyJoss
#include "EngineCore.h"
#include "Scene/SimulationCore.h" // contains core::SimulationCore and core::ISimulationCore

extern "C" {
	// Factory function to create a SimulationCore instance
    ENGINE_API core::ISimulationCore* CreateSimulationCore_v1() {
        try { return new core::SimulationCore(); }
        catch (...) { return nullptr; /*Avoids throwing exceptions across C ABI boundary by return null on failure*/ }
    }
	// Destroys a SimulationCore instance
    ENGINE_API void DestroySimulationCore(core::ISimulationCore* p) {
        delete p; // makes sure that deletion happens in the dll that created it.
    }
} // extern "C"