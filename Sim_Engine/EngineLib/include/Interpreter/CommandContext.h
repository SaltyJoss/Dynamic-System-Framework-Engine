#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>

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

	enum AngularUnits {
		DegPerSec,
		RadPerSec
	};

	class PhysicsSystem;
	class RobotModel;

	// Class representing the command context
	class ENGINE_API CommandContext {
	public:
		CommandContext(PhysicsSystem& physics, RobotModel& robot);

		// Get joint count
		size_t jointCount() const;
		// Get joint position
		double jointPos(size_t jointIndex) const;
		// Get joint velocity
		double jointVel(size_t jointIndex) const;

		// Set joint target position
		void setJointTargetPos(size_t jointIndex, double qTarget);
		// Set joint velocity
		void setJointVel(size_t jointIndex, double omega);

		// Check if a link exists
		bool hasLink(size_t linkIndex) const;

		// Rotates a joint by a specified angle at a given velocity
		void RotateJoint(size_t jointIndex, double angleDeg, double vel);
		// Rotates an object around a specified axis by a given angle at a certain velocity
		void RotateObject(size_t objectIndex, const Vec3& axis, double angleDeg, double vel);
	};
} // namespace commands