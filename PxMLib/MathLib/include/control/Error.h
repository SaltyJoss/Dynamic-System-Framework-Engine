#pragma once

#include <core/MathLib.h>

using namespace mathlib;

namespace control {
	/// <summary>
	/// Structure to hold pose error information.
	/// </summary>
	template<typename Scalar>
	struct PoseError_T {
		Vec3_T<Scalar> position; // Positional error (x, y, z)
		Vec3_T<Scalar> orientation; // Orientational error (roll, pitch, yaw)
	};

	/// <summary>
	/// Computes the joint space error between the desired and current joint configurations.
	/// </summary>
	/// <param name="q_desired">The desired joint configuration.</param>
	/// <param name="q_current">The current joint configuration.</param>
	/// <param name="out_error">The output joint space error.</param>
	template<typename Scalar>
	void jointErr(const VecX_T<Scalar>& q_des, const VecX_T<Scalar>& q_cur, VecX_T<Scalar>& err) { err = q_des - q_cur; }

	/// <summary>
	/// Computes the pose error between the desired and current transformation matrices.
	/// </summary>
	/// <param name="T_desired">The desired transformation matrix.</param>
	/// <param name="T_current">The current transformation matrix.</param>
	/// <returns>A PoseError struct containing positional and orientational errors.</returns>
	template<typename Scalar>
	PoseError_T<Scalar> poseErr(const Mat4_T<Scalar>& T_des, const Mat4_T<Scalar>& T_cur) {
		PoseError error;
		// Positional error
		error.position = T_des.template block<3, 1>(0, 3) - T_cur.template block<3, 1>(0, 3);
		// Orientational error (using rotation matrices)
		Mat3_T<Scalar> R_des = T_des.template block<3, 3>(0, 0);
		Mat3_T<Scalar> R_cur = T_cur.template block<3, 3>(0, 0);
		Mat3_T<Scalar> R_err = R_des * R_cur.transpose();
		// Convert rotation matrix to roll-pitch-yaw angles
		Scalar sy = sqrt(R_err(0, 0) * R_err(0, 0) + R_err(1, 0) * R_err(1, 0));
		bool singular = sy < Scalar(1e-6);
		double x, y, z;
		if (!singular) {
			x = atan2(R_err(2, 1), R_err(2, 2));
			y = atan2(-R_err(2, 0), sy);
			z = atan2(R_err(1, 0), R_err(0, 0));
		}
		else {
			x = atan2(-R_err(1, 2), R_err(1, 1));
			y = atan2(-R_err(2, 0), sy);
			z = Scalar(0);
		}
		error.orientation = Vec3_T<Scalar>(x, y, z);
		return error;
	}
}