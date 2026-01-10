#include "pch.h"
#include "Interpreter/CommandContext.h"

namespace commands {

	size_t CommandContext::jointCount() const {
		return 0; // Placeholder
	}

	double CommandContext::jointPos(size_t jointIndex) const {
		return 0.0; // Placeholder
	}

	double CommandContext::jointVel(size_t jointIndex) const {
		return 0.0; // Placeholder
	}

	void CommandContext::setJointTargetPos(size_t jointIndex, double qTarget) {
		// Implementation to set the joint target position
	}

	void CommandContext::setJointVel(size_t jointIndex, double omega) {
		// Implementation to set the joint velocity
	}

	bool CommandContext::hasLink(size_t linkIndex) const {
		return false; // Placeholder
	}

	// Command Logic
	void CommandContext::RotateJoint(size_t jointIndex, double angleDeg, double vel) {
		// Implementation to rotate the joint
	}

	void CommandContext::RotateObject(size_t objectIndex, const Vec3& axis, double angleDeg, double vel) {
		// Implementation to rotate the object
	}

} // namespace commands