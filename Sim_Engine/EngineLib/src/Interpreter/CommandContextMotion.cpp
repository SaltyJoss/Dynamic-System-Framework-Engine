#include "pch.h"
#include "Interpreter/CommandContextMotion.h"
#include "Scene/SimulationManager.h"
#include "Scene/ObjectID.h"
#include "Robots/RobotSystem.h"

#include "EngineLib/LogMacros.h"

using namespace mathlib;
using namespace utils;

namespace commands {
	CommandContextMotion::CommandContextMotion(gui::simManager* sim, scene::ObjectID objID)
		: _sim(sim), _phys(sim ? &sim->getPhysicsSystem() : nullptr), _robot(sim ? sim->getRobotSystem() : nullptr),
		  _objID(objID), _defaultObjID(objID), _angularUnits(AngularUnits::DegPerSec) {
	}

	scene::ObjectID CommandContextMotion::getDefaultObjectID() const { return _defaultObjID; }
	scene::ObjectID CommandContextMotion::getObjectID() const { return _objID; }

	scene::Object* CommandContextMotion::resolveObject(scene::ObjectID id) const {
		if (!_sim) return nullptr;
		if (id == scene::ObjectID::INVALID_OBJECT_ID) return nullptr;
		return _sim->getObjectByID(id);
	}

	scene::Object* CommandContextMotion::resolveCurrentObject() const { return resolveObject(_objID); }
	scene::Object* CommandContextMotion::resolveDefaultObject() const { return resolveObject(_defaultObjID); }

	// --- GLOBAL STATE METHODS ---
	void CommandContextMotion::setAngularUnits(AngularUnits units) { _angularUnits = units; }
	AngularUnits CommandContextMotion::getAngularUnits() const { return _angularUnits; }
	
	void CommandContextMotion::setOmegaClamp(double maxAbsOmega) { _omegaClamp = maxAbsOmega; 	}
	double CommandContextMotion::getOmegaClamp() const { return _omegaClamp; }

	utils::OpResult CommandContextMotion::setOmega(const Vec3& omega) {
		scene::Object* obj = resolveCurrentObject();
		if (!obj) { return OpResult::Failure("No object selected."); }
		Vec3 w = omega;
		if (_angularUnits == AngularUnits::DegPerSec) { w *= (float)(PI / 180.0); }
		if (_omegaClamp > 0.0) {
			w.x() = (float)std::clamp((double)w.x(), -_omegaClamp, _omegaClamp);
			w.y() = (float)std::clamp((double)w.y(), -_omegaClamp, _omegaClamp);
			w.z() = (float)std::clamp((double)w.z(), -_omegaClamp, _omegaClamp);
		}
		obj->state.angularVelocity = w;
		return OpResult::Success(true);
	}

	utils::OpResult CommandContextMotion::setJointOmega(const std::string& childLink, double omegaDegPerSec) {
		if (!_robot) { return OpResult::Failure("No robot loaded."); }
		float omegaRadPerSec = (float)(omegaDegPerSec * (PI / 180.0));
		if (_angularUnits == AngularUnits::DegPerSec) { omegaRadPerSec = (float)(omegaDegPerSec * (PI / 180.0)); }
		_robot->trySetJointAngleRad(childLink, omegaRadPerSec);
		return OpResult::Success(true);
	}

	utils::OpResult CommandContextMotion::stopAllOmega() { return setOmega(mathlib::Vec3(0, 0, 0)); }

	// --- HELPER METHODS ---

	inline glm::vec3 toGlm(const mathlib::Vec3& v) { return glm::vec3(v.x(), v.y(), v.z()); }
	inline glm::quat toGlm(const mathlib::Quat& q) { return glm::quat(q.w(), q.x(), q.y(), q.z()); }

	Vec3 CommandContextMotion::normaliseDirection(const Vec3& dir) const {
		const float x = dir.x();
		const float y = dir.y();
		const float z = dir.z();

		const float length = std::sqrt(x * x + y * y + z * z);

		if (length < 1e-6f) { return Vec3(0.0f, 0.0f, 0.0f); }

		const float invLen = 1.0f / length;
		return Vec3(x * invLen, y * invLen, z * invLen);
	}

	double CommandContextMotion::getJointAngleRad(const std::string& link) const {
		if (!_robot) return 0.0;
		float a = 0.0f;
		if (_robot->tryGetJointAngleRad(link, a)) return (double)a;
		return 0.0;
	}

	void CommandContextMotion::setJointAngleRad(const std::string& link, double angleRad) {
		if (!_robot) return;
		_robot->trySetJointAngleRad(link, (float)angleRad);
	}

	// --- JOINT ANGLE METHODS ---

	utils::OpResult CommandContextMotion::updateRigidRotateTo(double dt) {
		if (!_rig.active || !_rig.obj) { 
			D_INFO("No active rigid rotation.");
			return OpResult::Success();
		}

		auto& s = _rig.obj->state;

		Quat q = s.q;
		Quat q_err = _rig.qTarget * q.conjugate();
		q_err.normalize();

		if (q_err.w() < 0.0) { q_err.coeffs() *= -1.0; }

		double angle = 2.0 * std::acos(glm::clamp((double)q_err.w(), -1.0, 1.0)); // [0,pi] clamp
		
		if (angle < _rig.epsAngle) {
			s.angularVelocity = Vec3::Zero();
			_rig.active = false;
			return OpResult::Success(true);
		}

		Vec3 axis;
		double sinHalf = std::sqrt(std::max(0.0, 1.0 - (double)q_err.w() * (double)q_err.w()));
		axis = (sinHalf < 1e-8) ? _rig.axisUnit : Vec3(q_err.x(), q_err.y(), q_err.z()) / (float)sinHalf;

		double omega = std::min(_rig.maxOmega, angle / std::max(dt, 1e-6)); // simple “arrive in <= 1 step” clamp
		Vec3 w = (float)omega * axis.normalized();

		s.angularVelocity = w;
		return OpResult::Success(false);
	}

	utils::OpResult CommandContextMotion::updateJointRotateTo(double dt) {
		if (!_jnt.active) {
			return OpResult::Success(true);
		}

		double current = getJointAngleRad(_jnt.link);
		double err = _jnt.target - current; // err = target - current

		if (_jnt.wrapShortest) { err = robots::RobotSystem::wrapToPi((float)err); }

		if (std::abs(err) < _jnt.epsAngle) {
			setJointAngleRad(_jnt.link, _jnt.target);
			_jnt.active = false;
			return OpResult::Success(true);
		}

		double step = std::clamp(err, -_jnt.maxOmega * dt, _jnt.maxOmega * dt);
		setJointAngleRad(_jnt.link, current + step);

		return OpResult::Success(false);
	}

	utils::OpResult CommandContextMotion::beginRigidRotateTo(scene::Object* obj, Vec3 axisUnit, double maxOmegaDegPerSec, double angleDeg) {
		if (!obj) return OpResult::Failure("beginRigidRotateTo -> null object.");
		const double axisLen = axisUnit.norm();
		if (axisLen < 1e-8) return OpResult::Failure("beginRigidRotateTo => axis is zero.");

		axisUnit = axisUnit / axisLen;

		auto& s = obj->state;

		_rig.obj = obj;
		_rig.axisUnit = axisUnit;
		_rig.qStart = s.q;

		const double angRad = angleDeg * (PI / 180.0);
		Quat dq(std::cos(0.5 * angRad),
			axisUnit.x() * std::sin(0.5 * angRad),
			axisUnit.y() * std::sin(0.5 * angRad),
			axisUnit.z() * std::sin(0.5 * angRad));

		_rig.qTarget = (dq * _rig.qStart).normalized();
		_rig.maxOmega = maxOmegaDegPerSec * (PI / 180.0); // rad/s
		_rig.active = true;
		return OpResult::Success(false);
	}

	utils::OpResult CommandContextMotion::beginJointRotateTo(const std::string& link, double maxOmegaDegPerSec, double angleDeg) {
		if (!_robot) return OpResult::Failure("beginJointRotateTo -> no robot.");
		if (link.empty()) return OpResult::Failure("beginJointRotateTo -> empty link.");

		const double current = getJointAngleRad(link);
		const double target = angleDeg * (PI / 180.0);

		_jnt.link = link;
		_jnt.start = current;
		_jnt.target = target;
		_jnt.maxOmega = maxOmegaDegPerSec * (PI / 180.0);
		_jnt.active = true;
		_jnt.wrapShortest = true;
		_jnt.epsAngle = 0.25 * (PI / 180.0);
		return OpResult::Success(false);
	}
		
	// --- STOP MOTION METHODS ---

	void CommandContextMotion::stopRotation(scene::Object* obj, AxisMask axes) {
		if (!obj) return;
		auto& s = obj->state;

		angularVelocityPrev = s.angularVelocity; // store previous angular velocity

		if (axes.x) s.angularVelocity.x() = 0.0;
		if (axes.y) s.angularVelocity.y() = 0.0;
		if (axes.z) s.angularVelocity.z() = 0.0;
	}

	void CommandContextMotion::stopTranslation(scene::Object* obj, AxisMask axes) {
		if (!obj) return;
		auto& s = obj->state;

		linearVelocityPrev = s.linearVelocity; // store previous linear velocity

		if (axes.x) s.linearVelocity.x() = 0.0;
		if (axes.y) s.linearVelocity.y() = 0.0;
		if (axes.z) s.linearVelocity.z() = 0.0;
	}

	// --- ROTATION COMMAND METHODS ---
	OpResult CommandContextMotion::rotateObject(scene::Object* obj, AxisMask axes, double omega, double dt) {
		if (!obj || !obj->getMesh()) {
			D_FAIL("No object provided for rotation.");
			return OpResult::Failure("No object provided for rotation.");
		}
		auto& s = obj->state;
		// Normalize omega based on current angular units
		double internalOmega = NormaliseOmega(convertOmegaToInternal(omega));
		// Apply rotation to specified axes
		if (axes.x) { s.angularVelocity.x() = internalOmega; }
		if (axes.y) { s.angularVelocity.y() = internalOmega; }
		if (axes.z) { s.angularVelocity.z() = internalOmega; }

		D_INFO("omega(script)=%.3f units=%d -> internal(rad/s)=%.6f",
			omega, (int)_angularUnits, internalOmega);

		// return success
		return OpResult::Success(true);
	}

	OpResult CommandContextMotion::rotateAxes(AxisMask axes, double omega, double dt) {
		scene::Object* obj = resolveCurrentObject();
		if (!obj || !obj->getMesh()) {
			D_FAIL("No object associated with this context.");
			return OpResult::Failure("No object associated with this context.");
		}
		auto& s = obj->state;

		// Normalize omega based on current angular units
		double internalOmega = NormaliseOmega(convertOmegaToInternal(omega));
		// Apply rotation to specified axes
		if (axes.x) { s.angularVelocity.x() = internalOmega; }
		if (axes.y) { s.angularVelocity.y() = internalOmega; }
		if (axes.z) { s.angularVelocity.z() = internalOmega; }

		return OpResult::Success(true);
	}
	
	OpResult CommandContextMotion::rotateJoint(std::string linkName, double angleDeg, double vel) {
		if (!_robot) return OpResult::Failure("No robot model");
		if (linkName.empty()) return OpResult::Failure("Empty link name.");

		// apply absolute
		_sim->setRobotLinkRotation(linkName, static_cast<float>(angleDeg)); // <-- absolute
		return OpResult::Success();
	}

	OpResult CommandContextMotion::rotationJointDelta(std::string linkName, double deltaDeg, double vel) {
		if (!_robot) return OpResult::Failure("No robot model.");
		if (linkName.empty()) return OpResult::Failure("Empty link name.");

		float cur = 0.0f;
		if (!_robot->tryGetJointAngleRad(linkName, cur))
			return OpResult::Failure("Joint not found for linkName.");

		cur = cur + glm::radians((float)deltaDeg);
		_robot->trySetJointAngleRad(linkName, cur);
		return OpResult::Success(true);
	}

	// --- TRANSLATION COMMAND METHODS ---

	OpResult CommandContextMotion::translateWorld(const Vec3& direction, double distance, double vel) {
		scene::Object* obj = resolveCurrentObject();
		if (!obj) {
			D_FAIL("No object associated with this context.");
			return OpResult::Failure("No object associated with this context.");
		}

		const Vec3 translation = normaliseDirection(direction) * static_cast<float>(distance);
		obj->transform.position += toGlm(translation);
		return OpResult::Success(true);
	}

	OpResult CommandContextMotion::translateAxes(AxisMask axes, double vel, double dt) {
		scene::Object* obj = resolveCurrentObject();
		if (!obj) {
			D_FAIL("No object associated with this context.");
			return OpResult::Failure("No object associated with this context.");
		}

		Vec3 translation = Vec3::Zero();
		if (axes.x) {
			obj->state.linearVelocity.x() = vel;
			translation.x() = static_cast<float>(vel * dt);
		}
		if (axes.y) {
			obj->state.linearVelocity.y() = vel;
			translation.y() = static_cast<float>(vel * dt);
		}
		if (axes.z) {
			obj->state.linearVelocity.z() = vel;
			translation.z() = static_cast<float>(vel * dt);
		}

		obj->transform.position += toGlm(translation);
		return OpResult::Success(true);
	}

	// --- READ-ONLY ACCESSORS ---

	bool CommandContextMotion::hasLink(std::size_t linkIndex) const {
		if (!_robot) return false;
		return _robot && linkIndex < _robot->links().size();
	}

	// --- PRIVATE METHODS ---

	double CommandContextMotion::NormaliseOmega(double omega) const {
		if (_omegaClamp > 0.0) {
			if (omega > _omegaClamp) { return _omegaClamp; }
			if (omega < -_omegaClamp) { return -_omegaClamp; }
		}
		return omega;
	}

	double CommandContextMotion::convertOmegaToInternal(double omega) const {
		if (_angularUnits == AngularUnits::DegPerSec) { return omega * (PI / 180.0); } // Convert degrees to radians
		return omega;
	}
} // namespace commands