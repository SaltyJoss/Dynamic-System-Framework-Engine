/*
 * File: Systems/RigidBodySnapshot.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include "Systems/RigidBodyModel.h"
#include <core/Types_tpl.h>

namespace systems {
	// Immutable system data needed by solver threads
	struct DSFE_API RigidBodyConstModel {
		std::string name;
		bool baseFrameIsAligned = false;
		double scale = 1.0;
		mathlib::Mat4 baseFrame = mathlib::Mat4::Identity();

		bool jointAffectsLink(size_t jIdx, size_t lIdx) const;

		std::vector<RigidBodyLink> links;
		std::vector<RigidBodyJoint> joints;

		std::unordered_map<std::string, int> linkNameToIndex;
		int linkIndex(const std::string& linkName) const;
	};

	// Runtime snapshot for one integration/derivative step
	template<typename Scalar>
	struct RigidBodySnapshot_T {

		const RigidBodyConstModel* model = nullptr;

		mathlib::VecX_T<Scalar> q;   // joint angles
		mathlib::VecX_T<Scalar> qd;  // joint velocities

		mathlib::VecX_T<Scalar> q_ref;   // reference joint angles
		mathlib::VecX_T<Scalar> qd_ref;  // reference joint velocities
		mathlib::VecX_T<Scalar> qdd_ref; // reference joint accelerations

		mathlib::Mat4_T<Scalar> root_pose = mathlib::Mat4_T<Scalar>::Identity();

		bool baseIsFree = false;

		Scalar lastBaseForwardForce = Scalar(0);
		Scalar gravity = Scalar(0);

		eTorqueMode torqueMode = eTorqueMode::CONTROLLED;

		Scalar dt = Scalar(0);
		Scalar simTime = Scalar(0);
	};
	using RigidBodySnapshot = RigidBodySnapshot_T<double>;

	template<typename ToScalar, typename FromScalar>
	inline RigidBdoySnapshot_T<ToScalar> castSnapshot(
		const RigidBodySnapshot_T<FromScalar>& src
	) {
		RigidBodySnapshot_T<ToScalar> dst;

		dst.model = src.model;

		dst.q = src.q.template cast<ToScalar>();
		dst.qd = src.qd.template cast<ToScalar>();

		dst.q_ref = src.q_ref.template cast<ToScalar>();
		dst.qd_ref = src.qd_ref.template cast<ToScalar>();
		dst.qdd_ref = src.qdd_ref.template cast<ToScalar>();

		dst.root_pose = src.root_pose.template cast<ToScalar>();

		dst.baseIsFree = src.baseIsFree;

		dst.lastBaseForwardForce = ToScalar(src.lastBaseForwardForce);
		dst.gravity = ToScalar(src.gravity);

		dst.torqueMode = src.torqueMode;

		dst.dt = ToScalar(src.dt);
		dst.simTime = ToScalar(src.simTime);

		return dst;
	}
} // namespace rigidbodys