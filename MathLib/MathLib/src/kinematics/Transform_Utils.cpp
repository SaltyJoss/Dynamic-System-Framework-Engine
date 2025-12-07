
#include "pch.h"
#include "kinematics/Transform_Utils.h"

namespace kinematics {
	/// <inheritdoc/>
	Pose Transform_Utils::makeTransform(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation) {
		Pose transform = Pose::Identity();
		transform.translate(translation);
		transform.rotate(rotation);
		return transform;
	}

	/// <inheritdoc/>
	Vec3 Transform_Utils::getTranslation(const Pose& transform) {
		return transform.translation();
	}

	/// <inheritdoc/>
	Quat Transform_Utils::getRotation(const Pose& transform) {
		return Quat(transform.rotation());
	}

	/// <inheritdoc/>
	Pose Transform_Utils::fromTranslationRotation(const Vec3& translation, const Quat& rotation) {
		Pose transform = Pose::Identity();
		transform.translate(translation);
		transform.rotate(rotation);
		return transform;
	}

	/// <inheritdoc/>
	Pose Transform_Utils::interpolateTransforms(const Pose& t1, const Pose& t2, double t) {
		Vec3 trans1 = t1.translation();
		Vec3 trans2 = t2.translation();
		Quat rot1 = Quat(t1.rotation());
		Quat rot2 = Quat(t2.rotation());
		Vec
	}

	/// <inheritdoc/>
	Pose Transform_Utils::invertTransform(const Pose& transform) {
		return transform.inverse();
	}
}