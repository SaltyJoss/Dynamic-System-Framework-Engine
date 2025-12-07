#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"
#include "kinematics/DH_Param.h"

namespace kinematics {
	class MATHLIB_API Transform_Utils {
	public:
		/// <summary>
		/// Create a transformation matrix from translation and rotation
		/// </summary>
		/// <param name="translation">Translation vector</param>
		/// <param name="rotation">Rotation quaternion</param>
		/// <returns>Transformation matrix</returns>
		Pose makeTransform(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation);
		
		/// <summary>
		/// Extract translation vector from transformation matrix
		/// </summary>
		/// <param name="transform">The transformation matrix</param>
		/// <returns>Translation vector</returns>
		Vec3 getTranslation(const Pose& transform);

		/// <summary>
		/// Extract rotation quaternion from transformation matrix
		/// </summary>
		/// <param name="transform">The transformation matrix</param>
		/// <returns>Rotation quaternion</returns>
		Quat getRotation(const Pose& transform);

		/// <summary>
		/// Create transformation matrix from translation and rotation
		/// </summary>
		/// <param name="transform">The transformation matrix</param>
		/// <returns>Transformation matrix</returns>
		Pose fromTranslationRotation(const Vec3& translation, const Quat& rotation);

		/// <summary>
		/// Interpolate between two transformation matrices
		/// </summary>
		/// <param name="transform">The transformation matrix</param>
		/// <returns>Transformation matrix</returns>
		Pose interpolateTransforms(const Pose& t1, const Pose& t2, double t);

		/// <summary>
		/// Invert a transformation matrix
		/// </summary>
		/// <param name="transform">The transformation matrix</param>
		/// <returns>Transformation matrix</returns>
		Pose invertTransform(const Pose& transform);

	};
}