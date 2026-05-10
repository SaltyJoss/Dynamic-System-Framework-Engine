#include "pch.h"
#include "kinematics/DH_Params.h"

namespace kinematics {
	/// <inheritdoc/>
	Mat4 dhTransform(const DH_Params& p, double joint_val) {
		double theta = p.theta + joint_val; // Update joint angle with provided joint value
		Mat4 transform = Mat4::Identity();
		transform(0, 0) = cos(theta);
		transform(0, 1) = -sin(theta) * cos(p.alpha);
		transform(0, 2) = sin(theta) * sin(p.alpha);
		transform(0, 3) = p.a * cos(theta);
		transform(1, 0) = sin(theta);
		transform(1, 1) = cos(theta) * cos(p.alpha);
		transform(1, 2) = -cos(theta) * sin(p.alpha);
		transform(1, 3) = p.a * sin(theta);
		transform(2, 0) = 0.0;
		transform(2, 1) = sin(p.alpha);
		transform(2, 2) = cos(p.alpha);
		transform(2, 3) = p.d;
		transform(3, 0) = 0.0;
		transform(3, 1) = 0.0;
		transform(3, 2) = 0.0;
		transform(3, 3) = 1.0;
		return transform;
	}
}