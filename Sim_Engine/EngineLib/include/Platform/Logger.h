#pragma once

// =============================================
//            File: Logger.h
// =============================================
// Class responsible for logging messages to a file and console with different severity levels.
//
// structs / enumerations:
// --------------------------------------------
// LogLevel
//      -> Enumeration of log severity levels (Trace, Debug, Info, Warning, Error, Success, Fail, Runtime, Output).
// LogEntry
//      -> Struct representing a log entry with level, type, and message.
// LogType
//      -> Enumeration of log types (General, Simulation).
// simLogLevel
// 	    -> Enumeration of simulation log severity levels (Error, Fail, Success, Runtime, Rotate, Translate).
// simEntry
//      -> Struct representing a simulation log entry with level, type, and message.
// DataEntry
//      -> Struct representing a data entry with a data string.
// --------------------------------------------
//
// ============================================
//			  GitHub: SaltyJoss
// ============================================

#include "EngineCore.h"

#include <filesystem>
#include <stdarg.h>
#include <chrono>
#include <iomanip>
#include <mutex>
#include <sstream>

enum class LogLevel { Trace, Debug, Info, Warning, Error, Success, Fail, Runtime, Output };
enum class simLogLevel { Error, Fail, Success, Runtime, Rotate, Translate };
enum class LogType { General, Simulation };

struct LogEntry {
	LogLevel level = LogLevel::Info;
	std::string type;
	std::string message;
};

struct ENGINE_API simEntry {
	simLogLevel level = simLogLevel::Runtime;
	std::string type;
	std::string message;
};

struct ENGINE_API DataEntry {
	std::string data;
};

class ENGINE_API Debug {
public:
	Debug() {
		std::cerr << "CWD: " << std::filesystem::current_path().string() << "\n";

		try { std::filesystem::create_directory("Log"); }
		catch (const std::filesystem::filesystem_error& e) {
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

	std::vector<LogEntry>& Entries() { return _entries; }
	const std::vector<LogEntry>& Entries() const { return _entries; }

	std::vector<simEntry>& SimEntries() { return _simEntries; }
	const std::vector<simEntry>& SimEntries() const { return _simEntries; }

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

	void simLog(simLogLevel level, const char* format, ...) {
		char buffer[1024];

		va_list args;
		va_start(args, format);
		std::vsnprintf(buffer, sizeof(buffer), format, args);
		va_end(args);

		simEntry e;
		e.level = level;
		e.message = buffer;
		_simEntries.push_back(std::move(e));
	}

	void clear() {
		std::lock_guard<std::mutex> lock(_mutex);
		_entries.clear();
	}

	void clearSimLog() { _simEntries.clear(); }

private:
	std::mutex _mutex;
	std::ofstream _file;
	std::vector<LogEntry> _entries;
	std::vector<simEntry> _simEntries;

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

enum DataType { Simulation, Reference };

class ENGINE_API DataCapture {
public:
	DataCapture() {
		if (_dataType == DataType::Simulation) { _path = "Simulation"; }
		if (_dataType == DataType::Reference) { _path = "Reference"; }

		std::cerr << "CWD: " << std::filesystem::current_path().string() << "\n";
		
		auto dir = std::filesystem::path("Runs") / _path;
		try { std::filesystem::create_directories(dir); }
		catch (const std::filesystem::filesystem_error& e) {
			std::cerr << "create_directories failed: " << e.what() << std::endl;
		}

		auto now = std::chrono::system_clock::now();
		std::time_t now_time = std::chrono::system_clock::to_time_t(now);
		std::tm tm_data;
		localtime_s(&tm_data, &now_time);
		std::ostringstream oss;
		oss << dir.string() << "/dsfe_run_" << std::put_time(&tm_data, "%Y%m%d_%H%M%S") << ".csv";

		try {
			_file.open(oss.str(), std::ios::app);
			if (!_file.is_open())
				std::cerr << "Failed to open data capture file: " << oss.str() << std::endl;
		}
		catch (const std::exception& e) {
			std::cerr << "Exception opening data capture file: " << e.what() << std::endl;
		}

	}

	~DataCapture() {
		std::lock_guard<std::mutex> lock(_mutex);
		if (_file.is_open()) _file.close();
	}

	std::vector<DataEntry>& DataEntries() { return _dataEntries; }
	const std::vector<DataEntry>& DataEntries() const { return _dataEntries; }

	static DataCapture& Instance() {
		static DataCapture inst;
		return inst;
	}

	// General logging functions
	void logData(DataType dataType, const char* fmt, ...) {
		_dataType = dataType;
		va_list args;
		va_start(args, fmt);
		logCentral(fmt, args);
		va_end(args);
	}

	void clear() {
		std::lock_guard<std::mutex> lock(_mutex);
		_dataEntries.clear();
	}

private:
	std::mutex _mutex;
	std::ofstream _file;
	std::vector<DataEntry> _dataEntries;

	std::string _path = "Simulation";

	DataType _dataType = DataType::Simulation;

	// Centralised logging function for global logs outputted to file and console
	void logCentral(const char* fmt, va_list args) {
		char buffer[1024];
		vsnprintf(buffer, sizeof(buffer), fmt, args);

		std::ostringstream oss;
		oss << buffer;

		// Lock for thread safety - construct log line outside lock to minimize lock time
		std::string logLine = oss.str();

		// construct outside lock
		{
			std::lock_guard<std::mutex> lock(_mutex);
			if (_file.is_open()) { _file << oss.str() << std::endl; }
		}
	}
};

// Global logger instance
extern ENGINE_API Debug gLog;
extern ENGINE_API DataCapture gData;


