#pragma once

// =============================================
//            File: Logger.h
// =============================================
// Class responsible for logging messages to a file and console with different severity levels.
//
// Summary:
// =============================================
//
// structs / enumerations:
// --------------------------------------------
// LogLevel
//      -> Enumeration of log severity levels (Info, Warning, Error).
// LogEntry
//      -> Struct representing a log entry with level, type, and message.
// --------------------------------------------
//
// public:
// --------------------------------------------
// Debug()
//      -> Constructor that initializes the logger and creates a log file.
// ~Debug()
//      -> Destructor that closes the log file.
// void logError(const char* type, const char* format, ...)
//      -> Logs an error message with the specified type and formatted message.
// void logInfo(const char* type, const char* format, ...)
//      -> Logs an informational message with the specified type and formatted message.
// void logWarning(const char* type, const char* format, ...)
//      -> Logs a warning message with the specified type and formatted message.
// void logDebug(const char* type, const char* format, ...)
//      -> Logs a debug message with the specified type and formatted message, used to parse logs seperately to the debug panel in the application
// --------------------------------------------
//
// private:
// --------------------------------------------
// std::mutex _mutex
//      -> Mutex for thread-safe logging.
// std::ofstream _file
//      -> Output file stream for the log file.
// void logCentral(const char* level, const char* type, const char* format, va_list args)
//      -> Centralized logging function that handles formatting and writing log entries.
// --------------------------------------------
//
// =============================================

#include "EngineCore.h"

#include <filesystem>
#include <stdarg.h>
#include <chrono>
#include <iomanip>
#include <mutex>
#include <sstream>

enum class LogLevel { Trace, Debug, Info, Warning, Error, Ok, Fail, Runtime, Output };

struct LogEntry {
    LogLevel level;
    std::string type;
    std::string message;
};

class ENGINE_API Debug {
public:
    Debug() {
        try { std::filesystem::create_directory("Log"); }
        catch (const std::filesystem::filesystem_error& e)  {
            std::cerr << "Failed to create Log directory: " << e.what() << std::endl;
        }

        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm tm_data;
        localtime_s(&tm_data, &now_time);

        std::ostringstream oss;
        oss << "Log/session_" << std::put_time(&tm_data, "%Y%m%d_%H%M%S") << ".txt";

        try {
            _file.open(oss.str(), std::ios::app);
            if (!_file.is_open())
                std::cerr << "Failed to open log file: " << oss.str() << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "Exception opening log file: " << e.what() << std::endl;
        }

    }

    ~Debug() {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_file.is_open()) _file.close();
    }

    static Debug& Instance() {
        static Debug inst;
        return inst;
    }

    const std::vector<LogEntry>& Entries() const { return _entries; }

	// General logging functions
    void logError(const char* type, const char* format, ...) {
        va_list args;
        va_start(args, format);
        logCentral("ERROR", type, format, args);
        va_end(args);
    }

    void logInfo(const char* type, const char* format, ...) {
        va_list args;
        va_start(args, format);
        logCentral("INFO", type, format, args);
        va_end(args);
    }

    void logWarning(const char* type, const char* format, ...) {
        va_list args;
        va_start(args, format);
        logCentral("WARN", type, format, args);
        va_end(args);
    }

    // Centeralised logging function for debug panel logs only, outputted to debug panel console only
    void dLog(LogLevel level, const char* format, ...) {
        char buffer[1024];

        va_list args;
        va_start(args, format);
        std::vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        LogEntry e;
        e.level = level;
        e.message = buffer;

        _entries.push_back(std::move(e));
    }

    void clear() {
        std::lock_guard<std::mutex> lock(_mutex);
        _entries.clear();
    }

private:
    std::mutex _mutex;
    std::ofstream _file;
    std::vector<LogEntry> _entries;

	// Centralised logging function for global logs outputted to file and console
    void logCentral(const char* level, const char* type, const char* format, va_list args) {
        char buffer[1024];
        vsnprintf(buffer, sizeof(buffer), format, args);

        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm tm_data;
        localtime_s(&tm_data, &now_time);

        std::ostringstream oss;
        oss << "[" << std::put_time(&tm_data, "%Y-%m-%d %H:%M:%S") << "] "
              << "[" << level << " / " << type << "]: "
              << buffer;

		// Lock for thread safety - construct log line outside lock to minimize lock time
        std::string logLine = oss.str(); // construct outside lock
        {
            std::lock_guard<std::mutex> lock(_mutex);
            if (_file.is_open()) { _file << oss.str() << std::endl; }
            std::cout << oss.str() << std::endl;
        }
    }
};

// Global logger instance
extern ENGINE_API Debug gLog;

