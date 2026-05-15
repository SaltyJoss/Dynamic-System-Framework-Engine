#include "pch.h"
#include "control/Error.h"

namespace control {
	/// <inheritdoc/>
	void jointErr(const VecX& q_desired, const VecX& q_current, VecX& out_error) {
		out_error = q_desired - q_current;
	}

	/// <inheritdoc/>
	PoseError poseErr(const Mat4& T_desired, const Mat4& T_current) {
		PoseError error;
		// Positional error
		error.position = T_desired.block<3, 1>(0, 3) - T_current.block<3, 1>(0, 3);
		// Orientational error (using rotation matrices)
		Mat3 R_desired = T_desired.block<3, 3>(0, 0);
		Mat3 R_current = T_current.block<3, 3>(0, 0);
		Mat3 R_error = R_desired * R_current.transpose();
		// Convert rotation matrix to roll-pitch-yaw angles
		double sy = sqrt(R_error(0, 0) * R_error(0, 0) + R_error(1, 0) * R_error(1, 0));
		bool singular = sy < 1e-6; // If
		double x, y, z;
		if (!singular) {
			x = atan2(R_error(2, 1), R_error(2, 2));
			y = atan2(-R_error(2, 0), sy);
			z = atan2(R_error(1, 0), R_error(0, 0));
		} else {
			x = atan2(-R_error(1, 2), R_error(1, 1));
			y = atan2(-R_error(2, 0), sy);
			z = 0;
		}
		error.orientation = Vec3(x, y, z);
		return error;
	}
}