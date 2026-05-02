#pragma once
// File:    DebugLogEntry.h
// GitHub:  SaltyJoss
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

class DSFE_API DebugPanelLog {};

// Global logger instance
extern DSFE_API DebugPanelLog dLog;
