#include "pch.h"
#include "Interpreter/Commands/SetCmd.h"
#include "Interpreter/IStoredProgram.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;
using namespace mathlib;

namespace commands {
	// Helper function to parse the integration method
	static IntegratorMethod parseMethod(const std::string& s) {
		if (s == "euler")    return IntegratorMethod::Euler;
		if (s == "midpoint") return IntegratorMethod::Midpoint;
		if (s == "heun")     return IntegratorMethod::Heun;
		if (s == "ralston")  return IntegratorMethod::Ralston;
		if (s == "rk4")      return IntegratorMethod::RK4;
		D_WARN("Integration Method not recognised -> %s ~ Defaulted to \"Euler Method\"", s);
		return IntegratorMethod::Euler;
	}

	// Helper function to parse the function definition
	static std::string parseFunc(const std::string& s) {
		return "Not implemented yet.";
	}

	// Helper function to parse the colour
	static Colour parseColourBlock(const std::string& s) {
		if (s == "red")			{ return Colour{ BlockColour::Red,		mathlib::Vec3{ 1.00f, 0.00f, 0.00f } }; }
		if (s == "green")		{ return Colour{ BlockColour::Green,	mathlib::Vec3{ 0.00f, 1.00f, 0.00f } }; }
		if (s == "blue")		{ return Colour{ BlockColour::Blue,		mathlib::Vec3{ 0.00f, 0.00f, 1.00f } }; }
		if (s == "yellow")		{ return Colour{ BlockColour::Yellow,	mathlib::Vec3{ 1.00f, 1.00f, 0.00f } }; }
		if (s == "cyan")		{ return Colour{ BlockColour::Cyan,		mathlib::Vec3{ 0.00f, 1.00f, 1.00f } }; }
		if (s == "magenta")		{ return Colour{ BlockColour::Magenta,	mathlib::Vec3{ 1.00f, 0.00f, 1.00f } }; }
		if (s == "white")		{ return Colour{ BlockColour::White,	mathlib::Vec3{ 1.00f, 1.00f, 1.00f } }; }
		if (s == "grey")		{ return Colour{ BlockColour::Grey,		mathlib::Vec3{ 0.50f, 0.50f, 0.50f } }; }
		if (s == "darkgrey")	{ return Colour{ BlockColour::DarkGrey,	mathlib::Vec3{ 0.25f, 0.25f, 0.25f } }; }
		if (s == "black")		{ return Colour{ BlockColour::Black,	mathlib::Vec3{ 0.00f, 0.00f, 0.00f } }; }
		return Colour{ BlockColour::Red, mathlib::Vec3{ 1.0f, 0.0f, 0.0f } };  // Default
	}

	static float parseFloat(const std::string s) {
		float out = 0.0f;
		auto first = s.data();
		auto last = s.data() + s.size();

		auto res = std::from_chars(first, last, out); // Format: COMMAND <identifier>/<axis> <first>, ...<args_n>..., <last> "# Description"
		if (res.ec != std::errc{} || res.ptr != last) { return 0.0f; }
		return out;
	}

	static Colour parseColourRGB(const std::string& str) {
		if (!str.empty() && str.front() == '{' && str.back() == '}') {
			std::string s = str.substr(1, str.size() - 2); // remove braces
			std::vector<std::string> rgbStr;
			std::string cur;
			cur.reserve(s.size());

			// trim whitespace!
			auto trim = [](std::string& str) {
				auto is_ws = [](unsigned char c) { return c == ' ' || c == '\t'; }; // trim whitespace
				size_t a = 0;
				while (a < str.size() && is_ws(str[a])) { ++a; }
				size_t b = str.size();
				while (b > a && is_ws(str[b - 1])) { --b; }
				str = str.substr(a, b - a);
			};

			auto pushCurrent = [&]() {
				trim(cur);
				if (!cur.empty()) { rgbStr.push_back(cur); }
				cur.clear();
			};

			for (size_t i = 0; i < s.size(); ++i) {
				char c = s[i];
				if (c == ',') { pushCurrent(); continue; }
				cur.push_back(c);
			}
			pushCurrent();

			Vec3 rgb = Vec3{ parseFloat(rgbStr[0]), parseFloat(rgbStr[1]), parseFloat(rgbStr[2]) };
			return Colour{ BlockColour::Custom, rgb };
		}
		return Colour{ BlockColour::Red, mathlib::Vec3{ 1.0f, 0.0f, 0.0f } };  // Default
	}

	static Colour parseColourHex(const std::string& str) {
		if (!str.empty() && str.starts_with('#')) {
			std::string s = str.substr(1);
			Vec3 rgb = utils::hexToRGB(s);
			return Colour{ BlockColour::Custom, rgb };
		}
		return Colour{ BlockColour::Red, mathlib::Vec3{ 1.0f, 0.0f, 0.0f } };  // Default
	}

	// Get colour from parameter
	void SetCmd::setColour(const mathlib::Vec3& rgb) {
		_colRGB = rgb;
		markCompleted();
	}
	void SetCmd::setColour(const std::string& hex) {
		Vec3 rgb = utils::hexToRGB(hex);
		setColour(rgb);
	}

	// Helper function to parse LoadTarget from string
	// Expected formats: "set(integrator,<method>)", "set(colour,<RGB>)", "set(colour,<hex>)"
	static std::optional<SetTarget> parseSetTarget(const std::string& id, const std::string& token) {
		if (startsWith(toLower(id), "integrator")) { std::string s = toLower(token); return SetTarget{ SetTargetType::IntegratorMethod, parseMethod(s) }; }
		if (startsWith(toLower(id), "colour")) { 
			std::string s = token; 
			Colour c;
			if (s.starts_with('#')) { c = parseColourHex(s); }
			else if (s.starts_with('{')) { c = parseColourRGB(s); }
			else { c = parseColourBlock(toLower(s)); }

			return SetTarget{ SetTargetType::Colour, {}, c};
		}
		return std::nullopt;
	}

	void SetCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void SetCmd::markCompleted() { setResult({ CmdState::Executed, {}, "set() ran successfully" }); }
	bool SetCmd::hasStarted() const { return getResult().state != CmdState::NotStarted; }

	// --- SetCmd Constructor ---
	SetCmd::SetCmd(const std::string& id, const std::string& tokens)
		: _id(id), _tokens(tokens) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	// --- SetCmd Method Implementations ---
	void SetCmd::execute() {
		if (!getProgram()) {
			std::string errMsg = "set() command has no program context.";
			markFailed(errMsg);
			D_FAIL("%s", errMsg.c_str());
			return;
		}
		if (_id == "integrator") {
			auto m = parseSetTarget(_id, _tokens);
			if (!m) {
				D_FAIL("Unknown integrator method : %s", _tokens);
				return;
			}

			getProgram()->setIntegratorMethod(m->method);
			markCompleted();
			D_SUCCESS("set() command executed: Integrator method set.");
			return;
		}
		if (_id == "colour") {
			auto t = parseSetTarget(_id, _tokens);
			if (!t) {
				markFailed("Invalid colour");
				return;
			}

			getProgram()->setColour(t->colour.rgb);
			markCompleted();
			return;
		}

		markFailed("Unknown set target: " + _id);
	}

	// --- Free Function to Create SetCmd ---
	std::unique_ptr<ICommand> CreateSetCmd(const std::string& id, const std::vector<std::string>& tokens) {
		if (tokens.size() != 1) { D_FAIL("set(integrator, <method>) expects exactly 1 argument.");}
		return std::make_unique<SetCmd>(id, tokens[0]);
	}
} // namespace commands