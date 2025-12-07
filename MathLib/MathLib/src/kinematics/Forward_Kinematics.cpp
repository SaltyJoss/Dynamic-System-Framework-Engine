#include "pch.h"
#include "kinematics/DH_Params.h"
#include "kinematics/Forward_Kinematics.h"

namespace kinematics {
	/// <inheritdoc/>
	Pose FK::FK(const std::vector<DH_Params>& dh_p, const VecX& q) {
		Pose T = Pose::Identity(); // Initialize as identity matrix
		for (size_t i = 0; i < dh_p.size(); ++i) { // Loop through each joint
			double theta = dh_p[i].theta + (dh_p[i].isRevolute ? q(i) : 0.0);	// Adjust theta for revolute joints
			double d = dh_p[i].d + (dh_p[i].isPrismatic ? q(i) : 0.0);			// Adjust d for prismatic joints
			double a = dh_p[i].a			// Link length
			double alpha = dh_p[i].alpha;	// Link twist
			Pose A;	// Transformation matrix for current joint
			A << cos(theta), -sin(theta) * cos(alpha), sin(theta)* sin(alpha), a* cos(theta),	// First row
				sin(theta), cos(theta)* cos(alpha), -cos(theta) * sin(alpha), a* sin(theta),	// Second row
				0, sin(alpha), cos(alpha), d,	// Third row
				0, 0, 0, 1;	// Fourth row
			T = T * A;	// Update overall transformation
		}
		return T;	// Return end-effector pose
	}

	/// <inheritdoc/>
	std::vector<Pose> FK::linkTransformas(const std::vector<DH_Params>& dh_p, const VecX& q) {
		std::vector<Pose> transforms;	// Vector to hold transformation matrices for each link
		Pose T = Pose::Identity();
		for (size_t i = 0; i < dh_p.size(); ++i) {
			double theta = dh_p[i].theta + (dh_p[i].isRevolute ? q(i) : 0.0);
			double d = dh_p[i].d + (dh_p[i].isPrismatic ? q(i) : 0.0);
			double a = dh_p[i].a;
			double alpha = dh_p[i].alpha;
			Pose A;
			A << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
				sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
				0, sin(alpha), cos(alpha), d,
				0, 0, 0, 1;
			T = T * A;
			transforms.push_back(T);	// Store the transformation for the current link
		}
		return transforms;	// Return vector of link transformations
	}
}