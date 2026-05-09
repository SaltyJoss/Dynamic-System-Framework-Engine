// DSFE_GUI DebugLogEntry.h
#pragma once

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

class DebugPanelLog {};

// Global logger instance
extern DebugPanelLog dLog;
