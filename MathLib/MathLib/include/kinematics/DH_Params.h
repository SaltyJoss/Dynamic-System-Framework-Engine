#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"

namespace kinematics {
	/// <summary>
	/// Denavit-Hartenberg parameters structure
	/// </summary>
	struct MATHLIB_API DH_Params {
		double a;      // Link length
		double alpha;  // Link twist
		double d;      // Link offset
		double theta;  // Joint angle

		// Default constructor
		DH_Params() : a(0.0), alpha(0.0), d(0.0), theta(0.0) {}#

		// Parameterized constructor
		DH_Params(double link_length, double link_twist, double link_offset, double joint_angle) : a(link_length), alpha(link_twist), d(link_offset), theta(joint_angle) {}
	};

	/// <summary>
	/// Compute the Denavit-Hartenberg transformation matrix
	/// </summary>
	/// <param name="p">DH parameters</param>
	/// <param name="joint_val">Joint variable (angle or displacement)</param>
	/// <returns>Transformation matrix</returns>
	Mat4  MATHLIB_API dhTransform(const DH_Params& p, double joint_val);
}