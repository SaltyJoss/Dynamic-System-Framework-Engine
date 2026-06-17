// DSFE_Core EngineCore.h
#pragma once

// Conditional compilation for cross-platform symbol export/import
#ifdef _WIN32
    #ifdef DSFE_CORE_EXPORTS
        #define DSFE_API __declspec(dllexport)
    #else
        #define DSFE_API __declspec(dllimport)
    #endif
#elif defined(__GNUC__) || defined(__clang__)
    #define DSFE_API __attribute__((visibility("default")))
#else
    #define DSFE_API
#endif

// Forward declarations for core components
namespace core { struct ISimulationCore; }

// Factory functions
extern "C" {
    // Create / destroy factory declarations (no definitions in header)
    DSFE_API core::ISimulationCore* CreateSimulationCore_v1();
    DSFE_API void DestroySimulationCore(core::ISimulationCore* p);
}