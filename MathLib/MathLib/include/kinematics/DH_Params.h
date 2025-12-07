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
	};

	class MATHLIB_API DH {
	public:
		/// <summary>
		/// Compute the Denavit-Hartenberg transformation matrix
		/// </summary>
		/// <param name="p">DH parameters</param>
		/// <param name="joint_val">Joint variable (angle or displacement)</param>
		/// <returns>Transformation matrix</returns>
		Mat4 dhTransform(const DH_Params& p, double joint_val);
	};

}