#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"

using namespace mathlib;

namespace kinematics {
	enum class JointType {
		Revolute,	// Revolute joints are represented by rotation about an axis
		Prismatic	// Prismatic joints are represented by translation along an axis
	};

	/// <summary>
	/// Denavit-Hartenberg parameters structure
	/// </summary>
	struct MATHLIB_API DH_Params {
		double a;      // Link length
		double alpha;  // Link twist
		double d;      // Link offset
		double theta;  // Joint angle
		JointType type; // Joint type (Revolute or Prismatic)
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
		
		/// <summary>
		/// Determines whether the specified joint p is revolute.
		/// </summary>
		/// <param name="p">The p.</param>
		/// <returns>
		///   <c>true</c> if [is joint revolute] [the specified p]; otherwise, <c>false</c>.
		/// </returns>
		bool isJointRevolute(const DH_Params& p) const {
			return p.type == JointType::Revolute;
		}
		
		/// <summary>
		/// Determines whether the specified joint p is prismatic.
		/// </summary>
		/// <param name="p">The joint.</param>
		/// <returns>
		///   <c>true</c> if [is joint prismatic] [the specified p]; otherwise, <c>false</c>.
		/// </returns>
		bool isJointPrismatic(const DH_Params& p) const {
			return p.type == JointType::Prismatic;
		}
	};
}