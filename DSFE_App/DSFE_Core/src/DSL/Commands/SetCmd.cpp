/*
 * File: DSL/SetCmd.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "DSL/Commands/SetCmd.h"
#include "DSL/IStoredProgram.h"
#include "DSL/Utils.h"

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
		// Explicit variants
		if (s == "euler")			  return IntegratorMethod::Euler;
		if (s == "midpoint")		  return IntegratorMethod::Midpoint;
		if (s == "heun")			  return IntegratorMethod::Heun;
		if (s == "ralston")			  return IntegratorMethod::Ralston;
		if (s == "rk4")				  return IntegratorMethod::RK4;
		if (s == "rk45")			  return IntegratorMethod::RK45;
		// Implicit variants
		if (s == "implicit_euler")	  return IntegratorMethod::ImplicitEuler;
		if (s == "implicit_midpoint") return IntegratorMethod::ImplicitMidpoint;
		if (s == "glrk2")			  return IntegratorMethod::GLRK2;
		if (s == "glrk3")			  return IntegratorMethod::GLRK3;
		// Automatic Differentiation (AD) variants
		if (s == "ad_implicit_euler") 	 return IntegratorMethod::AD_ImplicitEuler;	
		if (s == "ad_implicit_midpoint") return IntegratorMethod::AD_ImplicitMidpoint;
		if (s == "ad_glrk2")			 return IntegratorMethod::AD_GLRK2;
		if (s == "ad_glrk3")			 return IntegratorMethod::AD_GLRK3;
		D_WARN("Integration Method not recognised -> %s ~ Defaulted to \"Fourth-Order Runge Kutta\"", s.c_str());
		return IntegratorMethod::RK4;
	}

	// --- SetCmd Method Implementations ---

	// Helper function to parse LoadTarget from string
	// Expected formats: "set(integrator,<method>)", "set(dt,<value>)", "set(gravity,<value>)", "set(vel,<wx>,<wy>,<wz>,<vx>,<vy>,<vz>)"
	static std::optional<SetTarget> parseSetTarget(const std::string& id, const std::string& token) {
		if (startsWith(toLower(id), "integrator")) { std::string s = toLower(token); return SetTarget{ SetTargetType::IntegratorMethod, parseMethod(s) }; }
		if (startsWith(toLower(id), "dt") || startsWith(toLower(id), "timestep")) { std::string s = token; return SetTarget{ SetTargetType::FixedDt, {}, {}, {}, utils::parseDouble(s) }; }
		if (startsWith(toLower(id), "gravity") || startsWith(toLower(id), "g") || startsWith(toLower(id), "grav")) {
			std::string s = toLower(token);
			if (s.empty()) { D_WARN("set(gravity, <value>/<x>, <y>, <z>) expects either 1 or 3 arguments."); return std::nullopt; }
			if (s.find(',') != std::string::npos) {
				mathlib::Vec3 g = utils::parseVec3(s);
				return SetTarget{ SetTargetType::Gravity, {}, {}, {}, {}, g };
			}
			mathlib::Vec3 g = mathlib::Vec3(0.0, 0.0, utils::parseDouble(s));
			return SetTarget{ SetTargetType::Gravity, {}, {}, {}, {}, g };
		}
		if (startsWith(toLower(id), "velocity") || startsWith(toLower(id), "vel")) { 
			std::string s = toLower(token); 
			if (s.empty()) { D_WARN("set(velocity/vel, <wx>, <wy>, <wz>, <vx>, <vy>, <vz>) expects 6 arguments."); return std::nullopt; }
			std::vector<AxisMask> m_vec = parseSpatialMask(s);
			if (m_vec.size() != 2) { D_WARN("set(velocity/vel, <wx>, <wy>, <wz>, <vx>, <vy>, <vz>) expects 6 arguments."); return std::nullopt; }
			mathlib::VecX sv = mathlib::VecX::Zero(6);
			sv << (m_vec[0].x ? utils::parseDouble(s) : 0.0),
				  (m_vec[0].y ? utils::parseDouble(s) : 0.0),
				  (m_vec[0].z ? utils::parseDouble(s) : 0.0),
				  (m_vec[1].x ? utils::parseDouble(s) : 0.0),
				  (m_vec[1].y ? utils::parseDouble(s) : 0.0),
				  (m_vec[1].z ? utils::parseDouble(s) : 0.0);
			return SetTarget{ SetTargetType::Velocity, {}, mathlib::Vec3(sv[0], sv[1], sv[2]), mathlib::Vec3(sv[3], sv[4], sv[5])};
		}
		return std::nullopt;
	}

	// Constructor
	SetCmd::SetCmd(const std::string& id, const std::string& tokens)
		: _id(toLower(id)), _tokens(tokens) 
	{
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
		if (_id == "velocity" || _id == "vel" || _id == "v") {
			auto t = parseSetTarget(_id, _tokens);
			if (!t) { markFailed("Invalid velocity"); return; }
			getProgram()->setVelocity(t->angular_vel, t->linear_vel); // Doesnt actually do anything right now
			markCompleted();
			D_SUCCESS("set() command executed: Velocity set.");
			return;
		}
		if (_id == "dt" || _id == "timestep" || _id == "fixed_dt") {
			auto t = parseSetTarget(_id, _tokens);
			if (!t) { markFailed("Invalid fixed_dt"); return; }
			getProgram()->setFixedDt(t->fixedDt);
			markCompleted();
			return;
		}
		if (_id == "gravity"  || _id == "g" || _id == "grav") {
			auto t = parseSetTarget(_id, _tokens);
			if (!t) { markFailed("Invalid gravity"); return; }
			getProgram()->setGravityVec(t->gravity);
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