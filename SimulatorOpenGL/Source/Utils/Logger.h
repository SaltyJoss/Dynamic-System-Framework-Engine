#include "ch.h"

#include <filesystem>
#include <stdarg.h>
#include <chrono>
#include <iomanip>
#include <ctime>

class Debug {
public:
    Debug() {
        std::filesystem::create_directory("Log");

        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm tm_data;
        localtime_s(&tm_data, &now_time);

        std::ostringstream oss;
        oss << "Log/session_" << std::put_time(&tm_data, "%Y%m%d_%H%M%S") << ".txt";
        _file.open(oss.str(), std::ios::app);
    }

    ~Debug() {
        if (_file.is_open()) _file.close();
    }

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

private:
    std::ofstream _file;

    void logCentral(const char* level, const char* type, const char* format, va_list args) {
        if (!_file.is_open()) return;

        char buffer[1024];
        vsnprintf(buffer, sizeof(buffer), format, args);

        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm tm_data;
        localtime_s(&tm_data, &now_time);

        _file << "[" << std::put_time(&tm_data, "%Y-%m-%d %H:%M:%S") << "] "
            << "[" << level << " / " << type << "]: "
            << buffer << std::endl;
    }
};
