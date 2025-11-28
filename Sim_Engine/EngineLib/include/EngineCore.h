#pragma once

// =============================================
//            File: EngineCore.h
// =============================================
// Core definitions and macros for the EngineLib.
//
// Summary:
// ============================================
// 
// Definitions:
// --------------------------------------------
// #ifdef _WIN32
//      -> Checks if the platform is Windows.
// #ifdef ENGINE_LIB_BUILD
//      -> Checks if the ENGINE_LIB_BUILD macro is defined for building the library.
// #define ENGINE_API __declspec(dllexport)
//      -> Defines ENGINE_API for exporting symbols when building the library.
// #define ENGINE_API __declspec(dllimport)
//      -> Defines ENGINE_API for importing symbols when using the library.
// #define ENGINE_API
//      -> Defines ENGINE_API as empty for non-Windows platforms.
// --------------------------------------------
//
// ============================================
//              GitHub: SaltyJoss
// ============================================

#ifdef _WIN32
  #ifdef ENGINE_LIB_BUILD
    #define ENGINE_API __declspec(dllexport)
  #else
    #define ENGINE_API __declspec(dllimport)
  #endif
#else
  #define ENGINE_API
#endif