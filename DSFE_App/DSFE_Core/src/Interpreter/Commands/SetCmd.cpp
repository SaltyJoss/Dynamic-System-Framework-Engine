#include "pch.h"
// File:   SetCmd.cpp
// GitHub: SaltyJoss
#include "Interpreter/Commands/SetCmd.h"
#include "Interpreter/IStoredProgram.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;
using namespace mathlib;

namespace commands {
	// --- SetCmd Mark Methods ---
	void SetCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void SetCmd::markCompleted() { setResult({ CmdState::Executed, {}, "set() ran successfully" }); }
	bool SetCmd::hasStarted() const { return getResult().state != CmdState::NotStarted; }

	// Helper function to parse the integration method
	static IntegratorMethod parseMethod(const std::string& s) {
		if (s == "euler")			  return IntegratorMethod::Euler;
		if (s == "midpoint")		  return IntegratorMethod::Midpoint;
		if (s == "heun")			  return IntegratorMethod::Heun;
		if (s == "ralston")			  return IntegratorMethod::Ralston;
		if (s == "rk4")				  return IntegratorMethod::RK4;
		if (s == "rk45")			  return IntegratorMethod::RK45;
		if (s == "implicit_euler")	  return IntegratorMethod::ImplicitEuler;
		if (s == "implicit_midpoint") return IntegratorMethod::ImplicitMidpoint;
		if (s == "glrk2")			  return IntegratorMethod::GLRK2;
		if (s == "glrk3")			  return IntegratorMethod::GLRK3;
		D_WARN("Integration Method not recognised -> %s ~ Defaulted to \"Fourth-Order Runge Kutta\"", s.c_str());
		return IntegratorMethod::RK4;
	}

	// --- SetCmd Method Implementations ---

	// Helper function to parse LoadTarget from string
	// Expected formats: "set(integrator,<method>)", "set(colour,<RGB>)", "set(colour,<hex>)"
	static std::optional<SetTarget> parseSetTarget(const std::string& id, const std::string& token) {
		if (startsWith(toLower(id), "integrator")) { std::string s = toLower(token); return SetTarget{ SetTargetType::IntegratorMethod, parseMethod(s) }; }
		if (startsWith(toLower(id), "dt")) { std::string s = token; return SetTarget{ SetTargetType::FixedDt, {}, {}, utils::parseDouble(s) }; }
		if (startsWith(toLower(id), "gravity")) { std::string s = token; return SetTarget{ SetTargetType::Gravity, {}, {}, {}, utils::parseDouble(s)}; }
		if (startsWith(toLower(id), "omega")) { 
			std::string s = toLower(token); 

			AxisMask m = utils::parseAxisMask(s);
			if (!m.any()) { return std::nullopt; }

			Vec3 w = Vec3{ m.x ? utils::parseFloat(s) : 0.0f,
						   m.y ? utils::parseFloat(s) : 0.0f,
						   m.z ? utils::parseFloat(s) : 0.0f 
			};

			return SetTarget{ SetTargetType::Omega, {}, w }; 
		}
		return std::nullopt;
	}

	// Constructor
	SetCmd::SetCmd(const std::string& id, const std::string& tokens)
		: _id(id), _tokens(tokens) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	// Execute the command
	void SetCmd::execute() {
		if (!getProgram()) {
			std::string errMsg = "set() command has no program context.";
			markFailed(errMsg);
			D_FAIL("%s", errMsg.c_str());
			return;
		}

		if (_id == "integrator") {
			auto t = parseSetTarget(_id, _tokens);
			if (!t) { markFailed("Invalid integrator method"); return; }
			getProgram()->setIntegratorMethod(t->method);
			markCompleted();
			D_SUCCESS("set() command executed: Integrator method set.");
			return;
		}
		if (_id == "omega") {
			auto t = parseSetTarget(_id, _tokens);
			if (!t) { markFailed("Invalid omega"); return; }
			getProgram()->setOmega(t->omega, AngularUnits::DegPerSec);
			markCompleted();
			return;
		}
		if (_id == "dt") {
			auto t = parseSetTarget(_id, _tokens);
			if (!t) { markFailed("Invalid fixed_dt"); return; }
			getProgram()->setFixedDt(t->fixedDt);
			markCompleted();
			return;
		}
		if (_id == "gravity") {
			auto t = parseSetTarget(_id, _tokens);
			if (!t) { markFailed("Invalid gravity"); return; }
			getProgram()->setGravity(t->gravity);
			markCompleted();
			return;
		}

		markFailed("Unknown set target: " + _id);
	}

	// Factory function to create a SetCmd from arguments
	std::unique_ptr<ICommand> CreateSetCmd(const std::string& id, const std::vector<std::string>& tokens) {
		if (tokens.size() != 1) { D_FAIL("set(id, <val>) expects exactly 1 argument."); }
		return std::make_unique<SetCmd>(id, tokens[0]);
	}
} // namespace commands