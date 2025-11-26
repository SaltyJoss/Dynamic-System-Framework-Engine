#pragma once

#include "EngineCore.h"
#include <string>
#include <chrono>

enum class logLevel {Info, Warn, Error, Debug};

struct logEntry {
	logLevel level;
	std::string type;
	std::string message;
	std::string fileFunc;
	std::chrono::system_clock::time_point timestamp;
};

class ENGINE_API DebugPanelLog {

};

// Global logger instance
extern ENGINE_API DebugPanelLog dLog;
