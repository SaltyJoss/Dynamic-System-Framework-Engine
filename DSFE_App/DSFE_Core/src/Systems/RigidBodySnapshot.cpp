/*
 * File: Systems/RigidBodySnapshot.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "Systems/RigidBodySnapshot.h"

namespace systems {
	// Method to check if a joint affects a link
    bool RigidBodyConstModel::jointAffectsLink(size_t jIdx, size_t lIdx) const {
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

	// Method to get the index of a link by name, returns -1 if not found
	int RigidBodyConstModel::linkIndex(const std::string& linkName) const {
		auto it = linkNameToIndex.find(linkName);
		if (it != linkNameToIndex.end()) {
			return it->second;
		}
		else {
			return -1; // not found
		}
	}
}