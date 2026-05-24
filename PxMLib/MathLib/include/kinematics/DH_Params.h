#pragma once

#include <core/MathLib.h>

using namespace mathlib;

namespace kinematics {
	enum class JointType_DH {
		Revolute,	// Revolute joints are represented by rotation about an axis
		Prismatic	// Prismatic joints are represented by translation along an axis
	};

	/// <summary>
	/// Denavit-Hartenberg parameters structure
	/// </summary>
	template<typename Scalar>
	struct DH_Params {
		Scalar a;      // Link length
		Scalar alpha;  // Link twist
		Scalar d;      // Link offset
		Scalar theta;  // Joint angle
		JointType_DH type; // Joint type (Revolute or Prismatic)
	};

	class DH {
	public:
		/// <summary>
		/// Compute the Denavit-Hartenberg transformation matrix
		/// </summary>
		/// <param name="p">DH parameters</param>
		/// <param name="joint_val">Joint variable (angle or displacement)</param>
		/// <returns>Transformation matrix</returns>
		template<typename Scalar>
		Mat4_T<Scalar> dhTransform(const DH_Params<Scalar>& p, Scalar joint_val) {
			Scalar theta = p.theta + joint_val; // Update joint angle with provided joint value
			Mat4_T<Scalar> transform = Mat4_T<Scalar>::Identity();
			transform(0, 0) = cos(theta);
			transform(0, 1) = -sin(theta) * cos(p.alpha);
			transform(0, 2) = sin(theta) * sin(p.alpha);
			transform(0, 3) = p.a * cos(theta);
			transform(1, 0) = sin(theta);
			transform(1, 1) = cos(theta) * cos(p.alpha);
			transform(1, 2) = -cos(theta) * sin(p.alpha);
			transform(1, 3) = p.a * sin(theta);
			transform(2, 0) = Scalar(0);
			transform(2, 1) = sin(p.alpha);
			transform(2, 2) = cos(p.alpha);
			transform(2, 3) = p.d;
			transform(3, 0) = Scalar(0);
			transform(3, 1) = Scalar(0);
			transform(3, 2) = Scalar(0);
			transform(3, 3) = Scalar(1);
			return transform;
		}

		/// <summary>
		/// Determines whether the specified joint p is revolute.
		/// </summary>
		/// <param name="p">The p.</param>
		/// <returns>
		///   <c>true</c> if [is joint revolute] [the specified p]; otherwise, <c>false</c>.
		/// </returns>
		template<typename Scalar>
		bool isJointRevolute(const DH_Params<Scalar>& p) const { return p.type == JointType_DH::Revolute; }

		/// <summary>
		/// Determines whether the specified joint p is prismatic.
		/// </summary>
		/// <param name="p">The joint.</param>
		/// <returns>
		///   <c>true</c> if [is joint prismatic] [the specified p]; otherwise, <c>false</c>.
		/// </returns>
		template<typename Scalar>
		bool isJointPrismatic(const DH_Params<Scalar>& p) const { return p.type == JointType_DH::Prismatic; }
	};
}