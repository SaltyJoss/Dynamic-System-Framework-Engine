#pragma once

#include <core/MathLib.h>
#include "kinematics/DH_Params.h"

using namespace mathlib;

namespace kinematics {
	class Transform_Utils {
	public:
		/// <summary>
		/// Create a transformation matrix from translation and rotation
		/// </summary>
		/// <param name="translation">Translation vector</param>
		/// <param name="rotation">Rotation quaternion</param>
		/// <returns>Transformation matrix</returns>
		Pose makeTransform(const Vec3& translation, const Eigen::Quaterniond& rotation) {
			Pose transform = Pose::Identity();
			transform.block<3, 1>(0, 3) = translation;					// Set translation vector
			transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();	// Set rotation matrix
			return transform;
		}
		
		/// <summary>
		/// Extract translation vector from transformation matrix
		/// </summary>
		/// <param name="transform">The transformation matrix</param>
		/// <returns>Translation vector</returns>
		Vec3 getTranslation(const Pose& transform) { return transform.block<3, 1>(0, 3); }

		/// <summary>
		/// Extract rotation quaternion from transformation matrix
		/// </summary>
		/// <param name="transform">The transformation matrix</param>
		/// <returns>Rotation quaternion</returns>
		Quat getRotation(const Pose& transform) { return (Quat)transform.block<3, 3>(0, 0); }

		/// <summary>
		/// Create transformation matrix from translation and rotation
		/// </summary>
		/// <param name="transform">The transformation matrix</param>
		/// <returns>Transformation matrix</returns>
		Pose fromTranslationRotation(const Vec3& translation, const Quat& rotation) {
			Pose transform = Pose::Identity();
			transform.block<3, 1>(0, 3) = translation;
			transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
			return transform;
		}

		/// <summary>
		/// Interpolate between two transformation matrices
		/// </summary>
		/// <param name="transform">The transformation matrix</param>
		/// <returns>Transformation matrix</returns>
		Pose interpolateTransforms(const Pose& t1, const Pose& t2, double t) { return t1 * (1.0 - t) + t2 * t; }

		/// <summary>
		/// Invert a transformation matrix
		/// </summary>
		/// <param name="transform">The transformation matrix</param>
		/// <returns>Transformation matrix</returns>
		Pose invertTransform(const Pose& transform) { return transform.inverse(); }
	};
}