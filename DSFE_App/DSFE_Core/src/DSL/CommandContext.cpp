/*
 * File: DSL/CommandContext.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "DSL/CommandContext.h"
#include "Scene/SimulationCore.h"
#include "Systems/RigidBodySystem.h"

#include "EngineLib/LogMacros.h"

using namespace mathlib;
using namespace constants;
using namespace utils;

namespace commands {
	/*
	 * HELPERS
	 */
	// Struct to hold parsed name components (base name and optional index)
	struct ParsedName {
		std::string base;
		int idx = 0;
		bool hasIdx = false;
	};
	// Helper Method to parse the indexed name from a string (e.g., "cube[2]" -> base="cube", idx=2)
	static ParsedName parsedIdxedName(const std::string& tok) {
		ParsedName p;
		auto lb = tok.find('[');
		if (lb == std::string::npos) { p.base = tok; return p; }
		auto rb = tok.find(']', lb);
		if (rb == std::string::npos) { p.base = tok; return p; }
		p.base = tok.substr(0, lb);
		std::string idxStr = tok.substr(lb + 1, rb - lb - 1);
		try { p.idx = std::stoi(idxStr); p.hasIdx = true; }
		catch (...) { p.hasIdx = false; }
		return p;
	}

	/*
	 * COMMAND CONTEXT IMPLEMENTATION
	 */
	// Constructor
	CommandContext::CommandContext(core::ISimulationCore* core)
		: _core(core), _angularUnits(AngularUnits::DegPerSec) {
	}

	// --- INITIALISATION METHODS ---

	// Starts the simulation if the simulation manager is available
	OpResult CommandContext::startSim() {
		if (!_core) {
			LOG_WARN("Simulation manager is null, cannot start simulation.");
			return OpResult::Failure("Simulation manager is null.");
		}
		_core->startSimulation();
		return OpResult::Success();
	}
	// Set the fixed delta time for the simulation
	OpResult CommandContext::setFixedDt(double dt) {
		if (!_core) { return OpResult::Failure("Simulation manager is null."); }
		if (dt <= 0.0) { return OpResult::Failure("Fixed dt must be positive."); }
		_core->setFixedDt(dt);
		return OpResult::Success(true);
	}
	// Loads a rigidBody by name and updates the context with the new rigidBody system
	OpResult CommandContext::loadRigidBody(const std::string& bodyName) {
		if (!_core) { return OpResult::Failure("SimulationCore is null."); }
		if (bodyName.empty()) return OpResult::Failure("RigidBody name is empty.");
		_core->loadRigidBody(bodyName);
		auto& rb = _core->activeBodyIdx() >= 0 ? _core->body(_core->activeBodyIdx()) : _core->rigidBodySystem();
		return OpResult::Success(true);
	}

	// --- GLOBAL STATE METHODS ---
	void CommandContext::setAngularUnits(AngularUnits units) { _angularUnits = units; }
	AngularUnits CommandContext::getAngularUnits() const { return _angularUnits; }
	// Sets the maximum absolute angular velocity (omega) clamp
	void CommandContext::setOmegaClamp(double maxAbsOmega) { _omegaClamp = maxAbsOmega; 	}
	double CommandContext::getOmegaClamp() const { return _omegaClamp; }
	// Stops all angular velocity for the body
	utils::OpResult CommandContext::setJointOmega(const std::string& childLink, double omegaDegPerSec) {
		double omegaRadPerSec = degToRad(omegaDegPerSec);
		auto& rb = _core->rigidBodySystem();
		rb.trySetJointOmegaRad(childLink, omegaRadPerSec);
		return OpResult::Success(true);
	}
	// Stops all angular velocity for the body
	utils::OpResult CommandContext::stopJointOmega(const std::string& childLink) { return setJointOmega(childLink, 0.0); }
	// Stops all angular velocity for the body
	core::ISimulationCore* CommandContext::Core() const { return _core; }
	systems::RigidBodySystem& CommandContext::RigidBody() const { return _core->rigidBodySystem(); }
	// --- HELPER METHODS ---
	// Returns the RigidBodySystem pointer for a given target string, which may include a body name prefix (e.g., "bodyName.memberName")
	systems::RigidBodySystem* CommandContext::resolveBody(const std::string& target) {
		if (!_core) { return nullptr; }
		const int nb = (int)_core->bodyCount();
		std::string bodyToken = target;
		auto dot = target.find('.');
		if (dot != std::string::npos) { bodyToken = target.substr(0, dot); }
		ParsedName pn = parsedIdxedName(bodyToken);
		if (dot != std::string::npos || pn.hasIdx) {
			std::string want = utils::toLower(pn.base);
			int seen = 0;
			for (int b = 0; b < nb; ++b) {
				auto& sys = _core->body(b);
				if (!sys.hasRigidBody()) { continue; }
				if (utils::toLower(sys.rigidBodyName()) != want) { continue; }
				if (pn.hasIdx) {
					if (seen == pn.idx) { return &sys; }
					++seen;
				} else { return &sys; }
			}
			return nullptr;
		}
		{
			auto& act = _core->rigidBodySystem();
			if (act.hasRigidBody() && bodyOwnsTarget(act, target)) { return &act; }
		}
		for (int b = 0; b < nb; ++b) {
			auto& sys = _core->body(b);
			if (sys.hasRigidBody() && bodyOwnsTarget(sys, target)) { return &sys; }
		}
		return nullptr;
	}
	// Returns the member name from a target string, which may include a body name prefix (e.g., "bodyName.memberName")
	bool CommandContext::bodyOwnsTarget(systems::RigidBodySystem& sys, const std::string& name) const {
		for (const auto& j : sys.joints()) {
			if (utils::toLower(j.child) == utils::toLower(name)) { return true; }
		}
		return false;
	}
	// Returns the member name from a target string, which may include a body name prefix (e.g., "bodyName.memberName")
	std::string CommandContext::memberName(const std::string& target) const {
		auto dot = target.find('.');
		if (dot == std::string::npos) {
			ParsedName pn = parsedIdxedName(target);
			return pn.base;
		}
		return target.substr(dot + 1);
	}

	Vec3 CommandContext::normaliseDirection(const Vec3& dir) const {
		const double x = dir.x();
		const double y = dir.y();
		const double z = dir.z();
		const double length = std::sqrt(x * x + y * y + z * z);
		if (length < 1e-6f) { return Vec3(0.0f, 0.0f, 0.0f); }
		const double invLen = 1.0f / length;
		return Vec3(x * invLen, y * invLen, z * invLen);
	}

	double CommandContext::getJointAngleRad(const std::string& link) const {
		auto& rb = _core->activeBodyIdx() >= 0 ? _core->body(_core->activeBodyIdx()) : _core->rigidBodySystem();
		double a = 0.0f;
		if (rb.tryGetJointAngleRad(link, a)) { return (double)a; }
		else { LOG_WARN("Failed to get joint angle for link '%s'", link.c_str()); }
		return 0.0;
	}

	// --- JOINT ANGLE METHODS ---

	utils::OpResult CommandContext::setJointTargetRad(const std::string& link, double thetaTargetRad) {
		auto& rb = _core->activeBodyIdx() >= 0 ? _core->body(_core->activeBodyIdx()) : _core->rigidBodySystem();
		if (!rb.trySetJointTargetRad(link, thetaTargetRad)) { 
			return OpResult::Failure("Failed to set joint target -> Joint not found or target rejected."); 
		}
		return OpResult::Success(true);
	}

	utils::OpResult CommandContext::setJointTargetDeltaRad(const std::string& link, double deltaRad) {
		auto& rb = _core->activeBodyIdx() >= 0 ? _core->body(_core->activeBodyIdx()) : _core->rigidBodySystem();
		double refRad = 0.0f;
		if (!rb.tryGetJointTargetRad(link, refRad)) { 
			return OpResult::Failure("Failed to get joint angle -> Joint not found."); 
		}
		const double targetRad = refRad + deltaRad;
		return setJointTargetRad(link, targetRad);
	}

	utils::OpResult CommandContext::setJointMaxOmegaRad(const std::string& link, double maxqd) {
		auto& rb = _core->activeBodyIdx() >= 0 ? _core->body(_core->activeBodyIdx()) : _core->rigidBodySystem();
		if (maxqd <= 0.0) { return OpResult::Failure("Max omega must be positive."); }
		if (!rb.trySetJointOmegaMaxRad(link, maxqd)) { 
			return OpResult::Failure("Failed to set joint max omega -> Joint not found or invalid value."); 
		}
		return OpResult::Success(true);
	}

	// Sets the reference angular velocity for a joint (rad/s)
	utils::OpResult CommandContext::setJointOmegaRefRad(const std::string& link, double qd_ref) {
		auto& rb = _core->activeBodyIdx() >= 0 ? _core->body(_core->activeBodyIdx()) : _core->rigidBodySystem();
		if (!rb.trySetJointOmegaRefRad(link, qd_ref)) { 
			return OpResult::Failure("Failed to set joint omega ref -> Joint not found or invalid value."); 
		}
		return OpResult::Success(true);
	}

	// Sets the reference angular acceleration for a joint (rad/s^2)
	utils::OpResult CommandContext::setJointAlphaRefRad(const std::string& link, double qdd_ref) {
		auto& rb = _core->activeBodyIdx() >= 0 ? _core->body(_core->activeBodyIdx()) : _core->rigidBodySystem();
		if (!rb.trySetJointAlphaRefRad(link, qdd_ref)) { 
			return OpResult::Failure("Failed to set joint alpha ref -> Joint not found or invalid value."); 
		}
		return OpResult::Success(true);
	}
	
	// --- READ-ONLY ACCESSORS ---
	// Checks if the current context has a valid rigidBody and if the specified link index is within bounds
	bool CommandContext::hasLink(std::size_t linkIndex) const {
		auto& rb = _core->rigidBodySystem();
		return linkIndex < rb.links().size();
	}
	// --- PRIVATE METHODS ---
	// Normalizes an angular velocity value based on the current omega clamp setting
	double CommandContext::NormaliseOmega(double omega) const {
		if (_omegaClamp > 0.0) {
			if (omega > _omegaClamp) { return _omegaClamp; }
			if (omega < -_omegaClamp) { return -_omegaClamp; }
		}
		return omega;
	}
	// Converts an angular velocity value from the current angular units to the internal representation (radians per second)
	double CommandContext::convertOmegaToInternal(double omega) const {
		if (_angularUnits == AngularUnits::DegPerSec) { return degToRad(omega); }
		return omega;
	}
} // namespace commands