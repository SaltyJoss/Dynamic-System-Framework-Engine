// DSFE_Core SpatialDynamics.h
#pragma once

#include "EngineCore.h"
#include "Robots/SpatialModel.h"
#include "Robots/DynamicsTypes.h"
#include <core/SpatialMath.h>

namespace robots {

	class DSFE_API SpatialDynamics {
	public:
		static void computeSpatialKinematicsAndBias(
			const SpatialModel& model,
			const mathlib::VecX& q, const mathlib::VecX& qd,
			std::vector<mathlib::SpatialMat>& Xup_out,
			std::vector<mathlib::SpatialVec>& v_out,
			std::vector<mathlib::SpatialVec>& c_out
		);

		static void computeAccelerations_RNEA(
			const SpatialModel& model,
			const mathlib::VecX& qdd,
			const std::vector<mathlib::SpatialMat>& Xup,
			const std::vector<mathlib::SpatialVec>& c,
			const mathlib::VecX& g,
			std::vector<mathlib::SpatialVec>& a_out
		);

		static void computeBackwardForces_RNEA(
			const SpatialModel& model,
			const std::vector<mathlib::SpatialMat>& Xup,
			const std::vector<mathlib::SpatialVec>& v, 
			const std::vector<mathlib::SpatialVec>& a,
			mathlib::VecX& tau_out
		);

		static mathlib::VecX RNEA(
			const SpatialModel& model,
			const mathlib::VecX& q,
			const mathlib::VecX& qd,
			const mathlib::VecX& qdd,
			DynamicsScratch& scratch
		);

		static mathlib::MatX CRBA(
			const SpatialModel& model,
			const std::vector<mathlib::SpatialMat>& Xup,
			DynamicsScratch& scratch
		);

		static void computeArticulatedBodies_ABA(
			const SpatialModel& model,
			const std::vector<SpatialMat>& Xup,
			const std::vector<SpatialVec>& v, 
			const std::vector<SpatialVec>& c,
			const mathlib::VecX& tau,
			std::vector<SpatialMat>& IA_out,
			std::vector<SpatialVec>& pA_out,
			std::vector<SpatialMat>& Ia_out,
			mathlib::VecX& u_out,
			mathlib::VecX& d_out,
			std::vector<SpatialVec>& U_out
		);

		static void computeAccelerations_ABA(
			const SpatialModel& model,
			const std::vector<SpatialMat>& Xup,
			const std::vector<SpatialVec>& c,
			const mathlib::VecX& u_out,
			const mathlib::VecX& d_out,
			const std::vector<SpatialVec>& U,
			const SpatialVec& a0,
			std::vector<SpatialVec>& a_out,
			mathlib::VecX& qdd_out
		);

		static mathlib::VecX ABA(
			const SpatialModel& model,
			const mathlib::VecX& q,
			const mathlib::VecX& qd,
			const mathlib::VecX& tau,
			DynamicsScratch& scratch
		);
	};
}