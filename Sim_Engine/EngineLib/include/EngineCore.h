#pragma once
// File:   EngineCore.h
// GitHub: SaltyJoss

// Conditional compilation for cross-platform symbol export/import
#ifdef _WIN32
  #ifdef ENGINE_LIB_BUILD
    #define ENGINE_API __declspec(dllexport)
  #else
    #define ENGINE_API __declspec(dllimport)
  #endif
#else
  #define ENGINE_API
#endif

// Forward declarations for core components
namespace core { struct ISimulationCore; }

// Factory functions
extern "C" {
    // Create / destroy factory declarations (no definitions in header)
    ENGINE_API core::ISimulationCore* CreateSimulationCore_v1();
    ENGINE_API void DestroySimulationCore(core::ISimulationCore* p);
}