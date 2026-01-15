#pragma once

#include <core/Types.h>
#include <string>
#include <string_view>
#include <vector>
#include "Platform/Logger.h"

namespace utils {
	// Struct for operation result
	struct OpResult {
		bool ok = true;
		std::string message;

		static OpResult Success() { return { true, {} }; }
		static OpResult Failure(const std::string& msg) { return OpResult{ false, msg }; }
	};

	// Struct for axis mask
	struct AxisMask {
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

	// --- Colour Utilities ---
	mathlib::Vec3 hexToRGB(const std::string& hex, mathlib::Vec3& rgbOut);
	std::string rgbToHex(const mathlib::Vec3& rgb);
} // namespace commands