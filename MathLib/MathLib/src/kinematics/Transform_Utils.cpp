
#include "pch.h"
#include "kinematics/Transform_Utils.h"

namespace kinematics {
	/// <inheritdoc/>
	Pose Transform_Utils::makeTransform(const Vec3& translation, const Quat& rotation) {
		Pose transform = Pose::Identity();
		transform.block<3, 1>(0, 3) = translation;					// Set translation vector
		transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();	// Set rotation matrix
		return transform;
	}

	/// <inheritdoc/>
	Vec3 Transform_Utils::getTranslation(const Pose& transform) {
		return transform.block<3, 1>(0, 3); // Extract translation vector
	}

	/// <inheritdoc/>
	Quat Transform_Utils::getRotation(const Pose& transform) {
		return (Quat)transform.block<3, 3>(0, 0); // Convert rotation matrix to quaternion
	}

	/// <inheritdoc/>
	Pose Transform_Utils::fromTranslationRotation(const Vec3& translation, const Quat& rotation) {
		Pose transform = Pose::Identity();
		transform.block<3, 1>(0, 3) = translation;
		transform.block<3, 3>(0, 0) = rotation.toRotationMatrix();
		return transform;
	}

	/// <inheritdoc/>
	Pose Transform_Utils::interpolateTransforms(const Pose& t1, const Pose& t2, double t)  {
		// Interpolate translation
		return t1 * (1.0 - t) + t2 * t; // Simple linear interpolation for demonstration, can be improved!!!
	}

	/// <inheritdoc/>
	Pose Transform_Utils::invertTransform(const Pose& transform) {
		return transform.inverse();
	}
}