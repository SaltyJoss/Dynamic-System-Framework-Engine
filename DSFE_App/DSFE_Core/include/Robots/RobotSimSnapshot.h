// DSFE_Core RobotSimSnapshot.h
#pragma once

#include "EngineCore.h"
#include "Robots/RobotModel.h"
#include <core/Types_tpl.h>

namespace robots {
	// Immutable robot data needed by solver threads
	struct DSFE_API RobotConstModel {
		std::string name;
		bool baseFrameIsAligned = false;
		double scale = 1.0;
		mathlib::Mat4 baseFrame = mathlib::Mat4::Identity();

		bool jointAffectsLink(size_t jIdx, size_t lIdx) const;

		std::vector<RobotLink> links;
		std::vector<RobotJoint> joints;

		std::unordered_map<std::string, int> linkNameToIndex;
		int linkIndex(const std::string& linkName) const;
	};

	// Runtime snapshot for one integration/derivative step
	template<typename Scalar>
	struct RobotSimSnapshot_T {

		const RobotConstModel* model = nullptr;

		mathlib::VecX_T<Scalar> q;   // joint angles
		mathlib::VecX_T<Scalar> qd;  // joint velocities

		mathlib::VecX_T<Scalar> q_ref;   // reference joint angles
		mathlib::VecX_T<Scalar> qd_ref;  // reference joint velocities
		mathlib::VecX_T<Scalar> qdd_ref; // reference joint accelerations

		mathlib::Mat4_T<Scalar> robotRootPose = mathlib::Mat4_T<Scalar>::Identity();

		bool baseIsFree = false;

		Scalar lastBaseForwardForce = Scalar(0);
		Scalar gravity = Scalar(0);

		eTorqueMode torqueMode = eTorqueMode::CONTROLLED;

		Scalar dt = Scalar(0);
		Scalar simTime = Scalar(0);
	};
	using RobotSimSnapshot = RobotSimSnapshot_T<double>;

	template<typename ToScalar, typename FromScalar>
	inline RobotSimSnapshot_T<ToScalar> castSnapshot(
		const RobotSimSnapshot_T<FromScalar>& src
	) {
		RobotSimSnapshot_T<ToScalar> dst;

		dst.model = src.model;

		dst.q = src.q.template cast<ToScalar>();
		dst.qd = src.qd.template cast<ToScalar>();

		dst.q_ref = src.q_ref.template cast<ToScalar>();
		dst.qd_ref = src.qd_ref.template cast<ToScalar>();
		dst.qdd_ref = src.qdd_ref.template cast<ToScalar>();

		dst.robotRootPose = src.robotRootPose.template cast<ToScalar>();

		dst.baseIsFree = src.baseIsFree;

		dst.lastBaseForwardForce = ToScalar(src.lastBaseForwardForce);
		dst.gravity = ToScalar(src.gravity);

		dst.torqueMode = src.torqueMode;

		dst.dt = ToScalar(src.dt);
		dst.simTime = ToScalar(src.simTime);

		return dst;
	}
} // namespace robots