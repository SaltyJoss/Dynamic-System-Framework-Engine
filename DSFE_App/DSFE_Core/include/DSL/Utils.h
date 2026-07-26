/*
 * File: DSL/Utils.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include "DSL/SimFwd.h"
#include <optional>

#include "Platform/Logger.h"

namespace utils {
	// Struct for operation result
	struct DSFE_API OpResult {
		bool ok = true;
		std::string message;
		bool done = false;

		static OpResult Success(bool done=false) { return { true, {}, done}; }
		static OpResult Failure(const std::string& msg) { return OpResult{ false, msg, false}; }
	};

	// Struct for axis mask
	struct DSFE_API AxisMask {
		bool x = false;
		bool y = false;
		bool z = false;

		bool any() const { return x || y || z; }
	};

	enum class AngularUnits {
		DegPerSec,
		RadPerSec
	};

	// --- String Utilities ---
	std::vector<std::string> split(const std::string_view s, const std::string_view delimiters);
	std::string_view trim(std::string_view str);
	std::string toLower(std::string_view str);
	std::string toUpper(std::string_view str);
	std::string stripBraces(std::string s);
	void ignoreCaseCompare(std::string& str);
	bool startsWith(const std::string& str, const std::string& prefix);
	bool endsWith(const std::string& str, const std::string& suffix);
	bool contains(const std::string& str, const std::string& substr);

	// --- Type Checking Utilities ---
	bool isInteger(const std::string_view s);
	bool isFloat(const std::string_view s);
	bool isDouble(const std::string_view s);
	bool isBoolean(const std::string_view s);
	
	// --- Conversion Utilities ---
	std::optional<bool> toBoolean(const std::string_view s);

	// --- Command Utilities ---
	double parseDouble(const std::string_view s);
	float parseFloat(const std::string s);
	mathlib::Vec3 parseVec3(const std::string& str);
	AxisMask parseAxisMask(const std::string& s);
	//bool tryParseObjID(const std::string& s, scene::ObjectID& out);

	// --- Unit Conversion Utilities ---
	double degToRad(double degrees);
	mathlib::Vec3 degToRad(mathlib::Vec3& degrees);

	double radToDeg(double radians);
	mathlib::Vec3 radToDeg(mathlib::Vec3& radians);
} // namespace commands