// DSFE_Core RobotSimSnapshot.cpp
#include "pch.h"

#include "Robots/RobotSimSnapshot.h"

namespace robots {
    bool RobotConstModel::jointAffectsLink(size_t jIdx, size_t lIdx) const {
		if (jIdx >= joints.size() || lIdx >= links.size()) { return false; }

		const std::string& targetJointChild = joints[jIdx].child;
		const std::string& targetLinkName = links[lIdx].name;

		// Check if the joint is an ancestor of the link in the kinematic tree
		std::string current = targetLinkName;

		while (true) {
			if (current == targetJointChild) { return true; } // joint affects this link
			bool movedUp = false;
			for (const auto& joint : joints) {
				if (joint.child == current) {
					current = joint.parent; // move up to the parent link
					movedUp = true;
					break;
				}
			}
			if (!movedUp) { break; } // reached the root link without finding the joint
		}

		return false; // joint does not affect this link
    }
}