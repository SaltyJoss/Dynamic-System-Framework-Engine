// DSFE_Core CommandContext.cpp
#include "pch.h"

#include "Interpreter/CommandContext.h"
#include "Scene/SimulationCore.h"
#include "Robots/RobotSystem.h"

#include "EngineLib/LogMacros.h"

using namespace mathlib;
using namespace constants;
using namespace utils;

namespace commands {
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

	OpResult CommandContext::loadSingleBody(const std::string& bodyName) {
		if (!_core) { return OpResult::Failure("Simulation manager is null."); }
		if (bodyName.empty()) return OpResult::Failure("Body name is empty.");
		/* Need to add logic here for single bodies since I removed physicsSystem. */
		return OpResult::Success(true);
	}

	// Loads a robot by name and updates the context with the new robot system
	OpResult CommandContext::loadMultibody(const std::string& bodyName) {
		if (!_core) { return OpResult::Failure("Simulation manager is null."); }
		if (bodyName.empty()) return OpResult::Failure("Robot name is empty.");
		_core->loadRobot(bodyName); // only load robot for now, as multibody is not finished
		auto& rs = _core->robotSystem();
		return OpResult::Success(true);
	}

	// --- GLOBAL STATE METHODS ---
	void CommandContext::setAngularUnits(AngularUnits units) { _angularUnits = units; }
	AngularUnits CommandContext::getAngularUnits() const { return _angularUnits; }
	
	void CommandContext::setOmegaClamp(double maxAbsOmega) { _omegaClamp = maxAbsOmega; 	}
	double CommandContext::getOmegaClamp() const { return _omegaClamp; }

	utils::OpResult CommandContext::setJointOmega(const std::string& childLink, double omegaDegPerSec) {
		double omegaRadPerSec = degToRad(omegaDegPerSec);
		auto& rs = _core->robotSystem();
		rs.trySetJointOmegaRad(childLink, omegaRadPerSec);
		return OpResult::Success(true);
	}
	 
	utils::OpResult CommandContext::stopJointOmega(const std::string& childLink) { return setJointOmega(childLink, 0.0); }

	core::ISimulationCore* CommandContext::Core() const { return _core; }
	robots::RobotSystem& CommandContext::Robot() const { return _core->robotSystem(); }

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
		auto& rs = _core->robotSystem();
		double a = 0.0f;
		if (rs.tryGetJointAngleRad(link, a)) { return (double)a; }
		else { LOG_WARN("Failed to get joint angle for link '%s'", link.c_str()); }
		return 0.0;
	}

	// --- JOINT ANGLE METHODS ---

	utils::OpResult CommandContext::setJointTargetRad(const std::string& link, double thetaTargetRad) {
		auto& rs = _core->robotSystem();
		if (!rs.trySetJointTargetRad(link, thetaTargetRad)) { 
			return OpResult::Failure("Failed to set joint target -> Joint not found or target rejected."); 
		}
		return OpResult::Success(true);
	}

	utils::OpResult CommandContext::setJointTargetDeltaRad(const std::string& link, double deltaRad) {
		auto& rs = _core->robotSystem();
		double refRad = 0.0f;
		if (!rs.tryGetJointTargetRad(link, refRad)) { 
			return OpResult::Failure("Failed to get joint angle -> Joint not found."); 
		}
		const double targetRad = refRad + deltaRad;
		return setJointTargetRad(link, targetRad);
	}

	utils::OpResult CommandContext::setJointMaxOmegaRad(const std::string& link, double maxqd) {
		auto& rs = _core->robotSystem();
		if (maxqd <= 0.0) { return OpResult::Failure("Max omega must be positive."); }
		if (!rs.trySetJointOmegaMaxRad(link, maxqd)) { 
			return OpResult::Failure("Failed to set joint max omega -> Joint not found or invalid value."); 
		}
		return OpResult::Success(true);
	}

	// Sets the reference angular velocity for a joint (rad/s)
	utils::OpResult CommandContext::setJointOmegaRefRad(const std::string& link, double qd_ref) {
		auto& rs = _core->robotSystem();
		if (!rs.trySetJointOmegaRefRad(link, qd_ref)) { 
			return OpResult::Failure("Failed to set joint omega ref -> Joint not found or invalid value."); 
		}
		return OpResult::Success(true);
	}

	// Sets the reference angular acceleration for a joint (rad/s^2)
	utils::OpResult CommandContext::setJointAlphaRefRad(const std::string& link, double qdd_ref) {
		auto& rs = _core->robotSystem();
		if (!rs.trySetJointAlphaRefRad(link, qdd_ref)) { 
			return OpResult::Failure("Failed to set joint alpha ref -> Joint not found or invalid value."); 
		}
		return OpResult::Success(true);
	}

	utils::OpResult CommandContext::updateJointRotateTo(double /*dt*/) {
		if (!_jnt.active) { return OpResult::Success(true); }

		auto& rs = _core->robotSystem();
		const bool done = rs.isJointAtTargetRad(_jnt.link, _jnt.epsAngle);
		if (done) { _jnt.active = false; return OpResult::Success(true); }

		SIM_ROTATE("Updating joint rotate to link='%s'", _jnt.link.c_str());

		return OpResult::Success(false);
	}

	utils::OpResult CommandContext::beginJointRotateTo(const std::string& link, double maxOmegaDegPerSec, double angleDeg) {
		auto& rs = _core->robotSystem();
		if (link.empty()) return OpResult::Failure("beginJointRotateTo -> empty link.");

		const double current = getJointAngleRad(link);
		const double target = degToRad(angleDeg);
		const double maxOmega = degToRad(maxOmegaDegPerSec);

		auto r1 = setJointMaxOmegaRad(link, maxOmega);
		if (!r1.ok) { return r1; }

		auto r2 = setJointTargetRad(link, target);
		if (!r2.ok) { return r2; }

		_jnt.link = link;
		_jnt.start = current;
		_jnt.target = target;
		_jnt.maxOmega = maxOmega;
		_jnt.active = true;
		_jnt.wrapShortest = true;
		_jnt.epsAngle = degToRad(0.5); // 0.5 degrees tolerance

		SIM_ROTATE("Begin joint rotate to link='%s' current=%.3f rad target=%.3f rad maxOmega=%.3f rad/s",
			link.c_str(), current, target, maxOmega);

		return OpResult::Success(false);
	}

	// --- RIGID MOTION METHODS ---

	//utils::OpResult CommandContext::updateRigidRotateTo(double dt) {
	//	if (!_rig.active || !_rig.obj) {
	//		D_INFO("No active rigid rotation.");
	//		return OpResult::Success();
	//	}

	//	auto& s = _rig.obj->state;

	//	Quat q = s.q;
	//	Quat q_err = _rig.qTarget * q.conjugate();
	//	q_err.normalize();

	//	if (q_err.w() < 0.0) { q_err.coeffs() *= -1.0; }

	//	double angle = 2.0 * std::acos(std::clamp(q_err.w(), -1.0, 1.0)); // [0,pi] clamp

	//	if (angle < _rig.epsAngle) {
	//		s.angularVelocity = Vec3::Zero();
	//		_rig.active = false;
	//		return OpResult::Success(true);
	//	}

	//	Vec3 axis;
	//	double sinHalf = std::sqrt(std::max(0.0, 1.0 - q_err.w() * q_err.w()));
	//	axis = (sinHalf < 1e-8) ? _rig.axisUnit : Vec3(q_err.x(), q_err.y(), q_err.z()) / sinHalf;

	//	double omega = std::min(_rig.maxOmega, angle / std::max(dt, 1e-6)); // simple “arrive in <= 1 step” clamp
	//	Vec3 w = omega * axis.normalized();

	//	s.angularVelocity = w;

	//	SIM_ROTATE("Updating rigid rotate to object id=%d angleErr=%.3f rad omega=%.3f rad/s axis=(%.3f, %.3f, %.3f)",
	//		(int)_rig.obj->id, angle, omega, axis.x(), axis.y(), axis.z());

	//	return OpResult::Success(false);
	//}

	// Starts a rigid rotation of the specified object around a given axis at a maximum angular velocity until it reaches the target angle
	//utils::OpResult CommandContext::beginRigidRotateTo(scene::Object* obj, Vec3 axisUnit, double maxOmegaDegPerSec, double angleDeg) {
	//	if (!obj) return OpResult::Failure("beginRigidRotateTo -> null object.");
	//	const double axisLen = axisUnit.norm();
	//	if (axisLen < 1e-8) return OpResult::Failure("beginRigidRotateTo => axis is zero.");

	//	axisUnit = axisUnit / axisLen;

	//	auto& s = obj->state;

	//	_rig.obj = obj;
	//	_rig.axisUnit = axisUnit;
	//	_rig.qStart = s.q;

	//	const double angRad = angleDeg * (PI_d / 180.0);
	//	Quat dq(std::cos(0.5 * angRad),
	//		axisUnit.x() * std::sin(0.5 * angRad),
	//		axisUnit.y() * std::sin(0.5 * angRad),
	//		axisUnit.z() * std::sin(0.5 * angRad));

	//	_rig.qTarget = (dq * _rig.qStart).normalized();
	//	_rig.maxOmega = degToRad(maxOmegaDegPerSec);
	//	_rig.active = true;

	//	SIM_ROTATE("Begin rigid rotate to object id=%d axis=(%.3f, %.3f, %.3f) angle=%.3f deg maxOmega=%.3f deg/s",
	//		(int)obj->id, axisUnit.x(), axisUnit.y(), axisUnit.z(), angleDeg, maxOmegaDegPerSec);

	//	return OpResult::Success(false);
	//}

	//// Rotates the specified object along given axes at a certain angular velocity for a time step dt
	//OpResult CommandContext::rotateObject(scene::Object* obj, AxisMask axes, double omega, double /*dt*/) {
	//	if (!obj || !obj->getMesh()) {
	//		SIM_FAIL("No object provided for rotation.");
	//		return OpResult::Failure("No object provided for rotation.");
	//	}
	//	auto& s = obj->state;
	//	// Normalize omega based on current angular units
	//	double internalOmega = NormaliseOmega(convertOmegaToInternal(omega));
	//	// Apply rotation to specified axes
	//	if (axes.x) { s.angularVelocity.x() = internalOmega; }
	//	if (axes.y) { s.angularVelocity.y() = internalOmega; }
	//	if (axes.z) { s.angularVelocity.z() = internalOmega; }

	//	SIM_ROTATE("omega(script)=%.3f units=%d -> internal(rad/s)=%.6f", omega, (int)_angularUnits, internalOmega);

	//	// return success
	//	return OpResult::Success(true);
	//}

	//// Rotates the current object along specified axes at a given angular velocity for a time step dt
	//OpResult CommandContext::rotateAxes(AxisMask axes, double omega, double /*dt*/) {
	//	scene::Object* obj = resolveCurrentObject();
	//	if (!obj || !obj->getMesh()) {
	//		SIM_FAIL("No object associated with this context.");
	//		return OpResult::Failure("No object associated with this context.");
	//	}
	//	auto& s = obj->state;

	//	// Normalize omega based on current angular units
	//	double internalOmega = NormaliseOmega(convertOmegaToInternal(omega));
	//	// Apply rotation to specified axes
	//	if (axes.x) { s.angularVelocity.x() = internalOmega; }
	//	if (axes.y) { s.angularVelocity.y() = internalOmega; }
	//	if (axes.z) { s.angularVelocity.z() = internalOmega; }

	//	return OpResult::Success(true);
	//}
	//	
	//// --- STOP MOTION METHODS ---

	//// Stops rotation of the specified object along the given axes
	//void CommandContext::stopRotation(scene::Object* obj, AxisMask axes) {
	//	if (!obj) return;
	//	auto& s = obj->state;

	//	angularVelocityPrev = s.angularVelocity; // store previous angular velocity

	//	if (axes.x) { s.angularVelocity.x() = 0.0; }
	//	if (axes.y) { s.angularVelocity.y() = 0.0; }
	//	if (axes.z) { s.angularVelocity.z() = 0.0; }
	//}

	//// Stops translation along specified axes
	//void CommandContext::stopTranslation(scene::Object* obj, AxisMask axes) {
	//	if (!obj) return;
	//	auto& s = obj->state;

	//	linearVelocityPrev = s.linearVelocity; // store previous linear velocity

	//	if (axes.x) { s.linearVelocity.x() = 0.0; }
	//	if (axes.y) { s.linearVelocity.y() = 0.0; }
	//	if (axes.z) { s.linearVelocity.z() = 0.0; }
	//}

	//// --- TRANSLATION COMMAND METHODS ---

	//// Translates the current object in world coordinates along a specified direction at a given velocity for a time step dt
	//OpResult CommandContext::translateWorld(const Vec3& direction, double distance, double /*vel*/) {
	//	scene::Object* obj = resolveCurrentObject();
	//	if (!obj) {
	//		SIM_FAIL("No object associated with this context.");
	//		return OpResult::Failure("No object associated with this context.");
	//	}

	//	const Vec3 translation = normaliseDirection(direction) * distance;
	//	obj->transform.position += translation;
	//	return OpResult::Success(true);
	//}

	//// Translates the current object along specified axes at a given velocity for a time step dt
	//const OpResult CommandContext::translateAxes(AxisMask axes, double vel, double dt) const {
	//	scene::Object* obj = resolveCurrentObject();
	//	if (!obj) {
	//		SIM_FAIL("No object associated with this context.");
	//		return OpResult::Failure("No object associated with this context.");
	//	}

	//	Vec3 translation = Vec3::Zero();
	//	if (axes.x) {
	//		obj->state.linearVelocity.x() = vel;
	//		translation.x() = vel * dt;
	//	}
	//	if (axes.y) {
	//		obj->state.linearVelocity.y() = vel;
	//		translation.y() = vel * dt;
	//	}
	//	if (axes.z) {
	//		obj->state.linearVelocity.z() = vel;
	//		translation.z() = vel * dt;
	//	}

	//	/*obj->transform.position += toGlm(translation);*/
	//	return OpResult::Success(true);
	//}

	// --- READ-ONLY ACCESSORS ---

	// Checks if the current context has a valid robot and if the specified link index is within bounds
	bool CommandContext::hasLink(std::size_t linkIndex) const {
		auto& rs = _core->robotSystem();
		return linkIndex < rs.links().size();
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