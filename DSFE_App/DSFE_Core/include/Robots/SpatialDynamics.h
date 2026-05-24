// DSFE_Core SpatialDynamics.h
#pragma once

#include "EngineCore.h"
#include "Robots/SpatialModel.h"
#include "Robots/DynamicsTypes.h"

namespace robots {
	class DSFE_API SpatialDynamics {
	public:
		template<typename Scalar>
		static void computeSpatialKinematicsAndBias(
			const SpatialModel<Scalar>& model,
			const mathlib::VecX_T<Scalar>& q,
			const mathlib::VecX_T<Scalar>& qd,
			std::vector<mathlib::SpatialMat_T<Scalar>>& Xup_out,
			std::vector<mathlib::SpatialVec_T<Scalar>>& v_out,
			std::vector<mathlib::SpatialVec_T<Scalar>>& c_out
		);
		
		template<typename Scalar>
		static void computeAccelerations_RNEA(
			const SpatialModel<Scalar>& model,
			const mathlib::VecX_T<Scalar>& qdd,
			const std::vector<mathlib::SpatialMat_T<Scalar>>& Xup,
			const std::vector<mathlib::SpatialVec_T<Scalar>>& c,
			const mathlib::VecX_T<Scalar>& g,
			std::vector<mathlib::SpatialVec_T<Scalar>>& a_out
		);

		template<typename Scalar>
		static void computeBackwardForces_RNEA(
			const SpatialModel<Scalar>& model,
			const std::vector<mathlib::SpatialMat_T<Scalar>>& Xup,
			const std::vector<mathlib::SpatialVec_T<Scalar>>& v,
			const std::vector<mathlib::SpatialVec_T<Scalar>>& a,
			mathlib::VecX_T<Scalar>& tau_out
		);

		template<typename Scalar>
		static mathlib::VecX_T<Scalar> RNEA(
			const SpatialModel<Scalar>& model,
			const mathlib::VecX_T<Scalar>& q,
			const mathlib::VecX_T<Scalar>& qd,
			const mathlib::VecX_T<Scalar>& qdd,
			DynamicsScratch<Scalar>& scratch
		);

		template<typename Scalar>
		static mathlib::MatX_T<Scalar> CRBA(
			const SpatialModel<Scalar>& model,
			const std::vector<mathlib::SpatialMat_T<Scalar>>& Xup,
			DynamicsScratch<Scalar>& scratch
		);

		template<typename Scalar>
		static void computeArticulatedBodies_ABA(
			const SpatialModel<Scalar>& model,
			const std::vector<SpatialMat_T<Scalar>>& Xup,
			const std::vector<SpatialVec_T<Scalar>>& v,
			const std::vector<SpatialVec_T<Scalar>>& c,
			const mathlib::VecX_T<Scalar>& tau,
			std::vector<SpatialMat_T<Scalar>>& IA_out,
			std::vector<SpatialVec_T<Scalar>>& pA_out,
			std::vector<SpatialMat_T<Scalar>>& Ia_out,
			mathlib::VecX_T<Scalar>& u_out,
			mathlib::VecX_T<Scalar>& d_out,
			std::vector<SpatialVec_T<Scalar>>& U_out
		);

		template<typename Scalar>
		static void computeAccelerations_ABA(
			const SpatialModel<Scalar>& model,
			const std::vector<SpatialMat_T<Scalar>>& Xup,
			const std::vector<SpatialVec_T<Scalar>>& c,
			const mathlib::VecX_T<Scalar>& u_out,
			const mathlib::VecX_T<Scalar>& d_out,
			const std::vector<SpatialVec_T<Scalar>>& U,
			const SpatialVec_T<Scalar>& a0,
			std::vector<SpatialVec_T<Scalar>>& a_out,
			mathlib::VecX_T<Scalar>& qdd_out
		);

		template<typename Scalar>
		static mathlib::VecX_T<Scalar> ABA(
			const SpatialModel<Scalar>& model,
			const mathlib::VecX_T<Scalar>& q,
			const mathlib::VecX_T<Scalar>& qd,
			const mathlib::VecX_T<Scalar>& tau,
			DynamicsScratch<Scalar>& scratch
		);
	};
}

#include "Robots/SpatialDynamics.inl"