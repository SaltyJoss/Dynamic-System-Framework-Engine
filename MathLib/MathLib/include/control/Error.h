#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"

using namespace mathlib;

namespace control {
	/// <summary>
	/// Structure to hold pose error information.
	/// </summary>
	struct MATHLIB_API PoseError {
		Vec3 position; // Positional error (x, y, z)
		Vec3 orientation; // Orientational error (roll, pitch, yaw)
	};

	/// <summary>
	/// Computes the joint space error between the desired and current joint configurations.
	/// </summary>
	/// <param name="q_desired">The desired joint configuration.</param>
	/// <param name="q_current">The current joint configuration.</param>
	/// <param name="out_error">The output joint space error.</param>
	void jointErr(const VecX& q_desired, const VecX& q_current, VecX& out_error);

	/// <summary>
	/// Computes the pose error between the desired and current transformation matrices.
	/// </summary>
	/// <param name="T_desired">The desired transformation matrix.</param>
	/// <param name="T_current">The current transformation matrix.</param>
	/// <returns>A PoseError struct containing positional and orientational errors.</returns>
	PoseError poseErr(const Mat4& T_desired, const Mat4& T_current);
}