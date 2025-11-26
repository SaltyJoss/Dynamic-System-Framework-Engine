#pragma once

// ============================================
//            File: LogMacros.h
// ============================================
// Macros for logging information, warnings, and errors with automatic file and function context.
//
// Summary:
// ============================================
//
// defined macros:
// --------------------------------------------
// File logging macros:
// ---
// LOG_INFO(fmt, ...)
//      -> Logs an informational message with file and function context.
// LOG_WARN(fmt, ...)
//      -> Logs a warning message with file and function context.
// LOG_ERROR(fmt, ...)
//      -> Logs an error message with file and function context.
// LOG_INFO_ONCE(fmt, ...)
//      -> Logs an informational message only once, with file and function context.
// LOG_WARN_ONCE(fmt, ...)
//      -> Logs a warning message only once, with file and function context.
// LOG_ERROR_ONCE(fmt, ...)
//      -> Logs an error message only once, with file and function context.
// 
// Debug panel logging macros:
// ---
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
// D_OK(fmt, ...)
//      -> Logs a success message to the debug panel.
// D_FAIL(fmt, ...)
//      -> Logs a failure message to the debug panel.
// D_RUNTIME(fmt, ...)
//      -> Logs a runtime message to the debug panel (e.g., performance, sim time).
// D_OUTPUT(fmt, ...)
// 	    -> Logs a general output message to the debug panel.
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
// --------------------------------------------
// 
// ============================================

#include "EngineCore.h"

// Forward declaration of Debug class
class Debug;
extern ENGINE_API Debug gLog;

// Macros for automatic file and function info
#define LOG_INFO(fmt, ...) gLog.logInfo((std::string(strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__) + std::string("::") + __func__).c_str(), fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)  gLog.logWarning((std::string(strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__) + std::string("::") + __func__).c_str(), fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...) gLog.logError((std::string(strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__) + std::string("::") + __func__).c_str(), fmt, ##__VA_ARGS__)

#define LOG_INFO_ONCE(fmt, ...) \
    do { \
        static bool _logged = false; \
        if (!_logged) { \
            LOG_INFO(fmt, ##__VA_ARGS__); \
            _logged = true; \
        } \
    } while(0)

#define LOG_WARN_ONCE(fmt, ...) \
    do { \
        static bool _warned = false; \
        if (!_warned) { \
            LOG_WARN(fmt, ##__VA_ARGS__); \
            _warned = true; \
        } \
    } while(0)

#define LOG_ERROR_ONCE(fmt, ...) \
    do { \
        static bool _errored = false; \
        if (!_errored) { \
            LOG_ERROR(fmt, ##__VA_ARGS__); \
            _errored = true; \
        } \
    } while(0)

// Debug panel logging macros
#define D_TRACE(fmt, ...) Debug::Instance().dLog(LogLevel::Trace,   fmt, ##__VA_ARGS__)
#define D_DEBUG(fmt, ...) Debug::Instance().dLog(LogLevel::Debug,   fmt, ##__VA_ARGS__)
#define D_INFO(fmt,  ...) Debug::Instance().dLog(LogLevel::Info,    fmt, ##__VA_ARGS__)
#define D_WARN(fmt,  ...) Debug::Instance().dLog(LogLevel::Warning, fmt, ##__VA_ARGS__)
#define D_ERROR(fmt, ...) Debug::Instance().dLog(LogLevel::Error,   fmt, ##__VA_ARGS__)
#define D_OK(fmt,    ...) Debug::Instance().dLog(LogLevel::Info,    fmt, ##__VA_ARGS__) // Using Info level for OK messages
#define D_FAIL(fmt,  ...) Debug::Instance().dLog(LogLevel::Warning, fmt, ##__VA_ARGS__) // Using Info level for FAIL messages
#define D_RUNTIME(fmt, ...) Debug::Instance().dLog(LogLevel::Runtime, fmt, ##__VA_ARGS__) // Using Runtime level for runtime messages (e.g. performance, sim time)
#define D_OUTPUT(fmt, ...) Debug::Instance().dLog(LogLevel::Output,    fmt, ##__VA_ARGS__) // Using Info level for general output messages

#define D_DEBUG_ONCE(fmt, ...) \
    do { \
        static bool _debugged = false; \
        if (!_debugged) { \
            D_DEBUG(fmt, ##__VA_ARGS__); \
            _debugged = true; \
        } \
    } while(0)

#define D_INFO_ONCE(fmt, ...) \
    do { \
        static bool _logged = false; \
        if (!_logged) { \
            D_INFO(fmt, ##__VA_ARGS__); \
            _logged = true; \
        } \
    } while(0)

#define D_WARN_ONCE(fmt, ...) \
    do { \
        static bool _warned = false; \
        if (!_warned) { \
            D_WARN(fmt, ##__VA_ARGS__); \
            _warned = true; \
        } \
    } while(0)

#define D_ERROR_ONCE(fmt, ...) \
    do { \
        static bool _errored = false; \
        if (!_errored) { \
            D_ERROR(fmt, ##__VA_ARGS__); \
            _errored = true; \
        } \
    } while(0)

#define D_OK_ONCE(fmt, ...) \
    do { \
        static bool _okayed = false; \
        if (!_okayed) { \
            D_OK(fmt, ##__VA_ARGS__); \
            _okayed = true; \
        } \
    } while(0)

#define D_FAIL_ONCE(fmt, ...) \
    do { \
        static bool _failed = false; \
        if (!_failed) { \
            D_FAIL(fmt, ##__VA_ARGS__); \
            _failed = true; \
        } \
    } while(0)