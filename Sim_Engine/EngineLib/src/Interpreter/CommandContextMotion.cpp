#include "pch.h"
#include "Interpreter/CommandContextMotion.h"

#include "EngineLib/LogMacros.h"

namespace commands {
	CommandContextMotion::CommandContextMotion(gui::simManager* sim, scene::Object* obj)
		: _sim(sim), _phys(&sim->getPhysicsSystem()), _robot(&sim->getRobotModel()), 
		_obj(obj), _defaultObj(obj), _angularUnits(AngularUnits::DegPerSec) { }

	// --- GLOBAL STATE METHODS ---
	
	void CommandContextMotion::setAngularUnits(AngularUnits units) {
		_angularUnits = units;
	}
	
	AngularUnits CommandContextMotion::getAngularUnits() const {
		return _angularUnits;
	}
	
	void CommandContextMotion::setOmegaClamp(double maxAbsOmega) {
		_omegaClamp = maxAbsOmega;
	}
	
	double CommandContextMotion::getOmegaClamp() const {
		return _omegaClamp;
	}

	// --- HELPER METHODS ---

	inline glm::vec3 toGlm(const mathlib::Vec3& v) {
		return glm::vec3(v.x(), v.y(), v.z());
	}

	Vec3 CommandContextMotion::normaliseDirection(const Vec3& dir) const {
		const float x = dir.x();
		const float y = dir.y();
		const float z = dir.z();

		const float length = std::sqrt(x * x + y * y + z * z);

		if (length < 1e-6f) { return Vec3(0.0f, 0.0f, 0.0f); }

		const float invLen = 1.0f / length;
		return Vec3(x * invLen, y * invLen, z * invLen);
	}


	void CommandContextMotion::updateJointAngles(std::string linkName, double angleDeg, double vel) {
		// Sets min and max angle limits for the joint
		float minAngle = -360.0f; float maxAngle = 360.0f;

		// Find the joint and apply limits
		for (const auto& joint : _robot->joints) {
			if (joint.child == linkName) {
				if (joint.continuous) {
					minAngle = -std::numeric_limits<float>::infinity();
					maxAngle = std::numeric_limits<float>::infinity();
					D_INFO("Joint %s is continuous.", linkName.c_str());
				}
				else {
					minAngle = joint.minAngle;
					maxAngle = joint.maxAngle;
					D_INFO("Joint %s limits: [%.2f, %.2f]", linkName.c_str(), minAngle, maxAngle);
				}

				if (angleDeg < minAngle || angleDeg > maxAngle) {
					D_FAIL("Angle %.2f out of limits [%.2f, %.2f] for joint %s.", angleDeg, minAngle, maxAngle, linkName.c_str());
				}

				float& angle = _jointAngles[linkName];
				angle = static_cast<float>(angleDeg);
				D_INFO("Rotating joint %s from %.2f to %.2f at velocity %.2f deg/s.", linkName.c_str(), angle, static_cast<float>(angleDeg), static_cast<float>(vel));
			}


		}
	}

	void CommandContextMotion::applyJointAngles() {
		if (!_robot) return;
		for (auto& joint : _robot->joints) {
			auto it = _jointAngles.find(joint.child);
			if (it != _jointAngles.end()) {
				joint.angle = it->second;
				D_INFO("Applied angle %.2f to joint %s.", joint.angle, joint.child.c_str());
			}
		}
	}

	void CommandContextMotion::stopRotation(scene::Object* obj, AxisMask axes) {
		if (!obj) return;
		auto& s = obj->state;
		if (axes.x) s.angularVelocity.x() = 0.0;
		if (axes.y) s.angularVelocity.y() = 0.0;
		if (axes.z) s.angularVelocity.z() = 0.0;
	}

	void CommandContextMotion::stopTranslation(scene::Object* obj, AxisMask axes) {
		if (!obj) return;
		auto& s = obj->state;
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
		if (axes.x) {
			s.angularVelocity.x() = internalOmega;
		}
		if (axes.y) {
			s.angularVelocity.y() = internalOmega;
		}
		if (axes.z) {
			s.angularVelocity.z() = internalOmega;
		}

		D_INFO("omega(script)=%.3f units=%d -> internal(rad/s)=%.6f",
			omega, (int)_angularUnits, internalOmega);

		// return success
		return OpResult::Success();
	}

	OpResult CommandContextMotion::rotateAxes(AxisMask axes, double omega, double dt) {
		if (!_obj || !_obj->getMesh()) {
			D_FAIL("No object associated with this context.");
			return OpResult::Failure("No object associated with this context.");
		}

		auto& s = _obj->state;

		// Normalize omega based on current angular units
		double internalOmega = NormaliseOmega(convertOmegaToInternal(omega));

		// Apply rotation to specified axes
		if (axes.x) {
			s.angularVelocity.x() = internalOmega;
		}
		if (axes.y) {
			s.angularVelocity.y() = internalOmega;
		}
		if (axes.z) {
			s.angularVelocity.z() = internalOmega;
		}


		D_INFO("omega(script)=%.3f units=%d -> internal(rad/s)=%.6f",
			omega, (int)_angularUnits, internalOmega);

		// return success
		return OpResult::Success();
	}
	
	OpResult CommandContextMotion::rotateJoint(std::string linkName, double angleDeg, double vel) {
		if (!_robot) {
			D_FAIL("No robot model associated with this context.");
			return OpResult::Failure("No robot model associated with this context.");
		}
		if (linkName.empty()) {
			D_FAIL("Joint name cannot be empty.");
			return OpResult::Failure("Joint name cannot be empty.");
		}

		for (auto& joint : _robot->joints) {
			if (joint.child == linkName) {
				_currentAngle = joint.angle; // get current angle
				D_INFO("Current angle of joint %s: %.2f", linkName.c_str(), _currentAngle);
			}
		}
		float targetAngle = angleDeg;

		updateJointAngles(linkName, angleDeg, vel);
		applyJointAngles();

		return OpResult::Success();
	}

	OpResult CommandContextMotion::rotationJointDelta(std::string linkName, double deltaDeg, double vel) {
		if (!_robot) {
			D_FAIL("No robot model associated with this context.");
			return OpResult::Failure("No robot model associated with this context.");
		}
		if (linkName.empty()) {
			D_FAIL("Joint name cannot be empty.");
			return OpResult::Failure("Joint name cannot be empty.");
		}

		for (auto& joint : _robot->joints) {
			if (joint.child == linkName) {
				_currentAngle = joint.angle; // get current angle
				D_INFO("Current angle of joint %s: %.2f", linkName.c_str(), _currentAngle);
			}
		}

		float targetAngle = _currentAngle + static_cast<float>(deltaDeg);
		updateJointAngles(linkName, targetAngle, vel);
		applyJointAngles();

		return OpResult::Success();
	}

	// --- TRANSLATION COMMAND METHODS ---

	OpResult CommandContextMotion::translateWorld(const Vec3& direction, double distance, double vel) {
		if (!_obj) {
			D_FAIL("No object associated with this context.");
			return OpResult::Failure("No object associated with this context.");
		}

		const Vec3 translation = normaliseDirection(direction) * static_cast<float>(distance);
		_obj->transform.position += toGlm(translation);
		return OpResult::Success();
	}

	OpResult CommandContextMotion::translateAxes(AxisMask axes, double vel, double dt) {
		if (!_obj) {
			D_FAIL("No object associated with this context.");
			return OpResult::Failure("No object associated with this context.");
		}

		Vec3 translation = Vec3::Zero();
		if (axes.x) {
			_obj->state.linearVelocity.x() = vel;
			translation.x() = static_cast<float>(vel * dt);
		}
		if (axes.y) {
			_obj->state.linearVelocity.y() = vel;
			translation.y() = static_cast<float>(vel * dt);
		}
		if (axes.z) {
			_obj->state.linearVelocity.z() = vel;
			translation.z() = static_cast<float>(vel * dt);
		}

		_obj->transform.position += toGlm(translation);
		return OpResult::Success();
	}

	// --- READ-ONLY ACCESSORS ---

	bool CommandContextMotion::hasLink(std::size_t linkIndex) const {
		if (!_robot) return false;
		return linkIndex < _robot->links.size();
	}

	// --- PRIVATE METHODS ---
	double CommandContextMotion::NormaliseOmega(double omega) const {
		if (_omegaClamp > 0.0) {
			if (omega > _omegaClamp) return _omegaClamp;
			if (omega < -_omegaClamp) return -_omegaClamp;
		}
		return omega;
	}

	double CommandContextMotion::convertOmegaToInternal(double omega) const {
		if (_angularUnits == AngularUnits::DegPerSec) {
			return omega * (PI / 180.0); // Convert degrees to radians
		}
		return omega; // Already in radians
	}
} // namespace commands