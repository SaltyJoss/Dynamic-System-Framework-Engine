#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <cstddef>
#include <string>
#include <unordered_map>

#include "Physics/PhysicsSystem.h"
#include "Robots/RobotModel.h"
#include "Scene/Object.h"

#include "Platform/Logger.h"

using namespace mathlib;

namespace commands {
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

	// Class representing the command context
	class ENGINE_API CommandContextMotion {
	public:
		CommandContextMotion(physics::PhysicsSystem& phys, RobotModel& robot, scene::Object& obj);

		// --- GLOBAL STATE METHODS ---

		// Sets the angular units for rotation commands
		void setAngularUnits(AngularUnits units);
		// Gets the current angular units
		AngularUnits getAngularUnits() const;

		// Sets the maximum absolute angular velocity (omega) clamp
		void setOmegaClamp(double maxAbsOmega);
		// Gets the current omega clamp value
		double getOmegaClamp() const;

		RobotModel* getRobotModel() const { return _robot; }

		// --- ROTATION COMMAND METHODS ---

		// Rotates specified axes at a given angular velocity
		OpResult rotateAxes(AxisMask axes, double omega, double dt);
		// Rotates a joint by a specified angle at a given velocity
		OpResult rotateJoint(std::string linkName, double angleDeg, double vel);
		// Rotates a joint by a specified delta angle at a given velocity
		OpResult rotationJointDelta(std::string linkName, double deltaDeg, double vel);

		// --- TRANSLATION COMMAND METHODS ---

		// Translates in world coordinates along a specified direction
		OpResult translateWorld(const Vec3& direction, double distance, double vel);
		// Translates along specified axes at a given velocity
		OpResult translateAxes(AxisMask axes, double vel, double dt);

		// --- READ-ONLY ACCESSORS ---
		bool hasLink(std::size_t linkIndex) const;

	private:
		physics::PhysicsSystem* _phys = nullptr;
		RobotModel* _robot = nullptr;
		scene::Object* _obj = nullptr;

		AngularUnits _angularUnits = AngularUnits::RadPerSec;
		double _omegaClamp = 0.0; // Default: no clamp

		double NormaliseOmega(double omega) const;
		double convertOmegaToInternal(double omega) const;

		Vec3 normaliseDirection(const Vec3& dir) const;

		void updateJointAngles(std::string linkName, double angleDeg, double vel);
		void applyJointAngles();

		std::unordered_map<std::string, float> _jointAngles;

		float _currentAngle = 0.0; // current angle for rotation commands

		double _dtheta = 0.0; // angle displacement
		double _dt = 0.0; // time interval
		double _omega = 0.0; // angular velocity
	};
} // namespace commands