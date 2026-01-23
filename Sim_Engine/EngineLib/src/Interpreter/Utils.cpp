#include "pch.h"
#include "Scene/ObjectID.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace mathlib;
using namespace constants;

namespace utils {
	// --- Handlers and Utilities ---

	// Helper function to split a string_view by multiple delimiters
	std::vector<std::string> split(const std::string_view s, const std::string_view delims) {
		std::vector<std::string> out;

		size_t start = 0;

		auto push_token = [&](size_t a, size_t b) {
			if (b > a) { out.emplace_back(s.substr(a, b - a)); }
			};

		for (size_t i = 0; i < s.size(); ++i) {
			if (delims.find(s[i]) != std::string_view::npos) {
				push_token(start, i);
				start = i + 1;	
			}
		}
		push_token(start, s.size());
		return out;
	}

	// Helper function to trim whitespace from both ends of a string_view
	std::string_view trim(std::string_view str) {
		auto is_ws = [](unsigned char c) { return c == ' ' || c == '\t' || c == '\r'; }; // trim whitespace
		size_t a = 0;
		while (a < str.size() && is_ws(str[a])) { ++a; }
		size_t b = str.size();
		while (b > a && is_ws(str[b - 1])) { --b; }
		str = str.substr(a, b - a);
		return str;
	}

	// Helper function to convert a string to uppercase
	std::string toLower(std::string_view str) {
		std::string result;
		result.reserve(str.size());
		for (unsigned char c : str)
			result.push_back((char)std::tolower(c));
		return result;
	}

	// Helper function to convert a string to uppercase
	std::string toUpper(std::string_view str) {
		std::string result;
		result.reserve(str.size());
		for (unsigned char c : str)
			result.push_back((char)std::toupper(c));
		return result;
	}

	std::string stripBraces(std::string s) {
		if (!s.empty() && s.front() == '{' && s.back() == '}') return s.substr(1, s.size() - 2);
		return s;
	}


	// Helper function to convert a string to lowercase in place
	void ignoreCaseCompare(std::string& str) {
		for (char& c : str) {
			c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
		}
	}

	// Helper function to check if a string starts with a prefix
	bool startsWith(const std::string& str, const std::string& prefix) {
		return str.size() >= prefix.size() && str.substr(0, prefix.size()) == prefix;
	}

	// Helper function to check if a string ends with a suffix
	bool endsWith(const std::string& str, const std::string& suffix) {
		return str.size() >= suffix.size() && str.substr(str.size() - suffix.size()) == suffix;
	}

	// Helper function to check if a string contains a substring
	bool contains(const std::string& str, const std::string& substr) {
		return str.find(substr) != std::string::npos;
	}

	// Helper function to parse number from string_view
	template <typename T>
	std::optional<T> parseNumber(const std::string_view s) {
		T out = 0;
		auto first = s.data();
		auto last = s.data() + s.size();
		auto res = std::from_chars(first, last, out);
		if (res.ec != std::errc{} || res.ptr != last) { return std::nullopt; }
		return out;
	}

	bool isInteger(const std::string_view s) { return parseNumber<int>(s).has_value(); }
	bool isFloat(const std::string_view s) { return parseNumber<float>(s).has_value(); }
	bool isDouble(const std::string_view s) { return parseNumber<double>(s).has_value(); }
	bool isBoolean(const std::string_view s) {
		std::string lowerStr = toLower(s);
		return (lowerStr == "true" || lowerStr == "false" || lowerStr == "1" || lowerStr == "0");
	}

	// Helper function to convert string_view to integer
	std::optional<bool> toBoolean(const std::string s) {
		std::string_view lowerStr = std::string(toLower(s));
		if (lowerStr == "true" || lowerStr == "1") {
			return true;
		}
		else if (lowerStr == "false" || lowerStr == "0") {
			return false;
		}
		return std::nullopt;
	}

	// Helper function to convert hex string to RGB vector (wanted to make my own, so I did)
	mathlib::Vec3 hexToRGB(const std::string& hex) {
		if (hex.size() != 7 || hex[0] != '#') {
			D_ERROR("Invalid hex colour format: %s", hex.c_str());
			return mathlib::Vec3{ 0.f, 0.f, 0.f };
		}
		try {
			int r = std::stoi(hex.substr(1, 2), nullptr, 16);
			int g = std::stoi(hex.substr(3, 2), nullptr, 16);
			int b = std::stoi(hex.substr(5, 2), nullptr, 16);
			return mathlib::Vec3{ r / 255.0f, g / 255.0f, b / 255.0f };
		}
		catch (...) {
			D_ERROR("Failed to convert hex to RGB: %s", hex.c_str());
			return mathlib::Vec3{ 0.f, 0.f, 0.f };
		}
	}

	// Helper function to convert RGB vector to hex string
	std::string rgbToHex(const mathlib::Vec3& rgb) {
		char hex[8];
		std::snprintf(hex, sizeof(hex), "#%02X%02X%02X", static_cast<int>(rgb.x() * 255.0f), static_cast<int>(rgb.y() * 255.0f), static_cast<int>(rgb.z() * 255.0f));
		return std::string(hex);
	}

	// Helper function to parse double from string_view
	double utils::parseDouble(const std::string_view s) {
		double out = 0.0;
		auto first = s.data();
		auto last = s.data() + s.size();

		auto res = std::from_chars(first, last, out); // format: rotate(target, omega, startDeg, endDeg)
		if (res.ec != std::errc{} || res.ptr != last) { return 0.0; }
		return out;
	}

	float utils::parseFloat(const std::string s) {
		float out = 0.0f;
		auto first = s.data();
		auto last = s.data() + s.size();

		auto res = std::from_chars(first, last, out); // Format: COMMAND <identifier>/<axis> <first>, ...<args_n>..., <last> "# Description"
		if (res.ec != std::errc{} || res.ptr != last) { return 0.0f; }
		return out;
	}

	mathlib::Vec3 utils::parseVec3(const std::string& str) {
		std::string s = stripBraces(str);
		std::vector<std::string> vStr;
		std::string cur;
		cur.reserve(s.size());

		auto pushCurrent = [&]() {
			trim(cur);
			if (!cur.empty()) { vStr.push_back(cur); }
			cur.clear();
		};

		for (size_t i = 0; i < s.size(); ++i) {
			char c = s[i];
			if (c == ',') { pushCurrent(); continue; }
			cur.push_back(c);
		}
		pushCurrent();

		return mathlib::Vec3{ parseFloat(vStr[0]), parseFloat(vStr[1]), parseFloat(vStr[2]) };
	}


	AxisMask utils::parseAxisMask(const std::string& args) {
		std::string s = stripBraces(args);

		AxisMask mask{}; // <-- start empty (no recursion - trust me this was a pain)
		for (char c : s) {
			switch (c) {
			case 'X': case 'x': mask.x = true; break;
			case 'Y': case 'y': mask.y = true; break;
			case 'Z': case 'z': mask.z = true; break;
			default: break;
			}
		}

		if (!mask.any()) {
			D_WARN("No valid axes found in axis mask: %s. Defaulting to Z axis.", s.c_str());
			mask.z = true;
		}
		return mask;
	}

	bool utils::tryParseObjID(const std::string& s, scene::ObjectID& out) {
		std::string_view v = s;
		if (v.rfind("obj", 0) == 0) v.remove_prefix(3);

		unsigned id = 0;
		auto res = std::from_chars(v.data(), v.data() + v.size(), id);
		if (res.ec != std::errc{} || res.ptr != v.data() + v.size()) return false;
		out = (scene::ObjectID)id;
		return true;
	}

	// --- Unit Conversion Utilities ---
	double degToRad(double degrees) { return degrees * ( PI_d / 180.0); }
	mathlib::Vec3 degToRad(mathlib::Vec3& degrees) {
		return mathlib::Vec3{
			degrees.x() * (PI_d / 180.0),
			degrees.y() * (PI_d / 180.0),
			degrees.z() * (PI_d / 180.0)
		};
	}

	double radiansToDegrees(double radians) { return radians * (180.0 / PI_d); }
	mathlib::Vec3 radiansToDegrees(mathlib::Vec3& radians) {
		return mathlib::Vec3{
			radians.x() * (180.0 / PI_d),
			radians.y() * (180.0 / PI_d),
			radians.z() * (180.0 / PI_d)
		};
	}
}