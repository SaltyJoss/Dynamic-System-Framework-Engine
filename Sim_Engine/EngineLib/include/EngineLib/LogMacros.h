#pragma once

// ============================================
//            File: LogMacros.h
// ============================================
// Macros for logging information, warnings, and errors with automatic file and function context.
//
// Summary:
// ============================================
//
// Exception Handling Macros:
// ===========================================
// 
// Global logging macros:
// --------------------------------------------
// LOG_INFO(fmt, ...)
//      -> Logs an informational message with file and function context.
// LOG_WARN(fmt, ...)
//      -> Logs a warning message with file and function context.
// LOG_ERROR(fmt, ...)
//      -> Logs an error message with file and function context.
// --------------------------------------------
// 
// Debug panel logging macros:
// --------------------------------------------
// D_TRACE(fmt, ...)
//      -> Logs a trace message to the debug panel.
// D_DEBUG(fmt, ...)
// 	    -> Logs a debug message to the debug panel.
// D_INFO(fmt, ...)
//      -> Logs an informational message to the debug panel.
// D_WARN(fmt, ...)
// 	    -> Logs a warning message to the debug panel.
// D_ERROR(fmt, ...)
//      -> Logs an error message to the debug panel.
// D_SUCCESS(fmt, ...)
//      -> Logs a success message to the debug panel.
// D_FAIL(fmt, ...)
//      -> Logs a failure message to the debug panel.
// D_RUNTIME(fmt, ...)
//      -> Logs a runtime message to the debug panel (e.g., performance, sim time).
// D_OUTPUT(fmt, ...)
// 	    -> Logs a general output message to the debug panel.
// --------------------------------------------
// ============================================
// 
// Exception Handling Macros - Once Variants:
// ===========================================
// 
// Global logging macros:
// --------------------------------------------
// LOG_INFO_ONCE(fmt, ...)
//      -> Logs an informational message only once, with file and function context.
// LOG_WARN_ONCE(fmt, ...)
//      -> Logs a warning message only once, with file and function context.
// LOG_ERROR_ONCE(fmt, ...)
//      -> Logs an error message only once, with file and function context.
// ---------------------------------------------
// 
// Debug panel logging macros:
// --------------------------------------------
// D_TRACE_ONCE(fmt, ...)
//      -> Logs a trace message to the debug panel only once.
// D_DEBUG_ONCE(fmt, ...)
//    -> Logs a debug message to the debug panel only once.
// D_INFO_ONCE(fmt, ...)
//      -> Logs an informational message to the debug panel only once.
// D_WARN_ONCE(fmt, ...)
// 	    -> Logs a warning message to the debug panel only once.
// D_ERROR_ONCE(fmt, ...)
//      -> Logs an error message to the debug panel only once.
// D_OK_ONCE(fmt, ...)
//      -> Logs a success message to the debug panel only once.
// D_FAIL_ONCE(fmt, ...)
// 	    -> Logs a failure message to the debug panel only once.
// D_RUNTIME_ONCE(fmt, ...)
//      -> Logs a runtime message to the debug panel only once.
// D_OUTPUT_ONCE(fmt, ...)
// 	    -> Logs a general output message to the debug panel only once.
// --------------------------------------------
// ===========================================
// 
// MACRO LOG LEVEL SUMMARY:
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// -----------
// TRACE    -> Detailed trace messages for debugging (e.g., function entry/exit, step-by-step execution, etc.) ~ acts as TRACE level messages, useful for tracing code execution flow, currently not used in global logging (but may be in future as it is standard practice).
// -----------
// DEBUG    -> Debug-level messages for development (e.g., variable values, state changes, etc.) ~ acts as DEBUG level messages, useful for developers during debugging, currently not used in global logging (but may be in future as it is standard practice).
// -----------
// INFO     -> General informational messages (e.g., process milestones, status updates, etc.) ~ acts as INFO level messages, generally useful runtime information for the user to see in cmdline and .log files.
// -----------
// WARN     -> Warning messages indicating potential issues (e.g., deprecated usage, recoverable errors, etc.) ~ acts as WARN level messages, generally non-critical but noteworthy.
// -----------
// ERROR    -> Error messages indicating failures (E.g., exceptions, critical failures, etc.) ~ acts as ERROR and FATAL (would like to introduce CRITICAL later as its own level for commandline specific errors).
// -----------
// SUCCESS  -> Messages indicating successful operations (e.g., completed tasks, successful connections, etc.) ~ acts as OK messages, aims to ONLY be used in the debug panel for specific runtime completion confirmations.
// -----------
// FAIL     -> Messages indicating failed operations (e.g., failed tasks, unsuccessful connections, etc.) ~ acts as FAIL messages, aims to also only be used in the debug panel for specific runtime failure confirmations.
// -----------
// RUNTIME  -> Messages related to runtime performance or simulation time (e.g., frame time, simulation step time, etc.) ~ used for performance monitoring and sim time updates, like a more specific INFO level but focused on runtime metrics.
// -----------
// OUTPUT   -> General output messages not fitting other categories (e.g., user output, data dumps, etc.) ~ used for general output that doesn't fit into other categories, useful for user-facing messages or data outputs.
// -----------
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// 
// ============================================
//			  GitHub: saltyjoss
// ============================================

#include "EngineCore.h"

// Forward declaration of Debug class
class Debug;
extern ENGINE_API Debug gLog;

// ============================================
//         GLOBAL EXCEPTION HANDLING
// ============================================
// 
// --------------------------------------------
// Global logging macros
// --------------------------------------------
#define LOG_INFO(fmt, ...) gLog.logInfo((std::string(strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__) + std::string("::") + __func__).c_str(), fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  gLog.logWarning((std::string(strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__) + std::string("::") + __func__).c_str(), fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) gLog.logError((std::string(strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__) + std::string("::") + __func__).c_str(), fmt, ##__VA_ARGS__)
// --------------------------------------------
// Once variants for global logging macros
// --------------------------------------------
#define LOG_INFO_ONCE(fmt, ...) \
    do { \
        static bool _logged = false; \
        if (!_logged) { \
            LOG_INFO(fmt, ##__VA_ARGS__); \
            _logged = true; \
        } \
    } while(0)
// -----
#define LOG_WARN_ONCE(fmt, ...) \
    do { \
        static bool _warned = false; \
        if (!_warned) { \
            LOG_WARN(fmt, ##__VA_ARGS__); \
            _warned = true; \
        } \
    } while(0)
// -----
#define LOG_ERROR_ONCE(fmt, ...) \
    do { \
        static bool _errored = false; \
        if (!_errored) { \
            LOG_ERROR(fmt, ##__VA_ARGS__); \
            _errored = true; \
        } \
    } while(0)
// --------------------------------------------
//
// ============================================
//        DEBUG PANEL EXCEPTION HANDLING
// ===========================================
// 
// --------------------------------------------
// Debug panel logging macros
// --------------------------------------------
#define D_TRACE(fmt, ...)   Debug::Instance().dLog(LogLevel::Trace,   fmt, ##__VA_ARGS__)
#define D_DEBUG(fmt, ...)   Debug::Instance().dLog(LogLevel::Debug,   fmt, ##__VA_ARGS__)
#define D_INFO(fmt,  ...)   Debug::Instance().dLog(LogLevel::Info,    fmt, ##__VA_ARGS__)
#define D_WARN(fmt,  ...)   Debug::Instance().dLog(LogLevel::Warning, fmt, ##__VA_ARGS__)
#define D_ERROR(fmt, ...)   Debug::Instance().dLog(LogLevel::Error,   fmt, ##__VA_ARGS__)
#define D_SUCCESS(fmt, ...) Debug::Instance().dLog(LogLevel::Success,    fmt, ##__VA_ARGS__) // Using Info level for OK messages
#define D_FAIL(fmt,  ...)   Debug::Instance().dLog(LogLevel::Fail, fmt, ##__VA_ARGS__) // Using Info level for FAIL messages
#define D_RUNTIME(fmt, ...) Debug::Instance().dLog(LogLevel::Runtime, fmt, ##__VA_ARGS__) // Using Runtime level for runtime messages (e.g. performance, sim time)
#define D_OUTPUT(fmt, ...)  Debug::Instance().dLog(LogLevel::Output,    fmt, ##__VA_ARGS__) // Using Info level for general output messages
// --------------------------------------------
// Once variants for debug panel logging macros
// --------------------------------------------
#define D_TRACE_ONCE(fmt, ...) \
    do { \
        static bool _traced = false; \
        if (!_traced) { \
            D_TRACE(fmt, ##__VA_ARGS__); \
            _traced = true; \
        } \
    } while(0)
// -----
#define D_DEBUG_ONCE(fmt, ...) \
    do { \
        static bool _debugged = false; \
        if (!_debugged) { \
            D_DEBUG(fmt, ##__VA_ARGS__); \
            _debugged = true; \
        } \
    } while(0)
// -----
#define D_INFO_ONCE(fmt, ...) \
    do { \
        static bool _logged = false; \
        if (!_logged) { \
            D_INFO(fmt, ##__VA_ARGS__); \
            _logged = true; \
        } \
    } while(0)
// -----
#define D_WARN_ONCE(fmt, ...) \
    do { \
        static bool _warned = false; \
        if (!_warned) { \
            D_WARN(fmt, ##__VA_ARGS__); \
            _warned = true; \
        } \
    } while(0)
// -----
#define D_ERROR_ONCE(fmt, ...) \
    do { \
        static bool _errored = false; \
        if (!_errored) { \
            D_ERROR(fmt, ##__VA_ARGS__); \
            _errored = true; \
        } \
    } while(0)
// -----
#define D_SUCCESS_ONCE(fmt, ...) \
    do { \
        static bool _succeeded = false; \
        if (!_succeeded) { \
            D_SUCCESS(fmt, ##__VA_ARGS__); \
            _succeeded = true; \
        } \
    } while(0)
// -----
#define D_FAIL_ONCE(fmt, ...) \
    do { \
        static bool _failed = false; \
        if (!_failed) { \
            D_FAIL(fmt, ##__VA_ARGS__); \
            _failed = true; \
        } \
    } while(0)
// -----
#define D_RUNTIME_ONCE(fmt, ...) \
    do { \
        static bool _runtimed = false; \
        if (!_runtimed) { \
            D_RUNTIME(fmt, ##__VA_ARGS__); \
            _runtimed = true; \
        } \
    } while(0)
// -----
#define D_OUTPUT_ONCE(fmt, ...) \
    do { \
        static bool _outputted = false; \
        if (!_outputted) { \
            D_OUTPUT(fmt, ##__VA_ARGS__); \
            _outputted = true; \
        } \
    } while(0)
// --------------------------------------------
//
// ============================================
//          SIMULATION ENGINE LOG MACROS 
// ============================================
//
// --------------------------------------------
// Simulation engine specific logging macros
// --------------------------------------------
#define SIM_ERROR(fmt, ...)     Debug::Instance().simLog(simLogLevel::Error,     fmt, ##__VA_ARGS__) // Using Error level for simulation engine errors
#define SIM_FAIL(fmt,  ...)     Debug::Instance().simLog(simLogLevel::Fail,      fmt, ##__VA_ARGS__) // Using Fail level for FAIL messages
#define SIM_SUCCESS(fmt, ...)   Debug::Instance().simLog(simLogLevel::Success,   fmt, ##__VA_ARGS__) // Using Success level for SUCCESSFUL messages
#define SIM_RUNTIME(fmt, ...)   Debug::Instance().simLog(simLogLevel::Runtime,   fmt, ##__VA_ARGS__) // Using Runtime level for runtime messages (e.g. performance, sim time)
#define SIM_ROTATE(fmt, ...)    Debug::Instance().simLog(simLogLevel::Rotate,    fmt, ##__VA_ARGS__) // Using Debug level for rotation related messages
#define SIM_TRANSLATE(fmt, ...) Debug::Instance().simLog(simLogLevel::Translate, fmt, ##__VA_ARGS__) // Using Debug level for translation related messages
// --------------------------------------------

// END OF FILE