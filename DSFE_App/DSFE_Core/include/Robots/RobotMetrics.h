// DSFE_Core RobotMetrics.h
#pragma once

#include "EngineCore.h"


namespace robots {
	// Per-joint metrics
	template<typename Scalar>
	struct RobotMetrics {
		// State
		mathlib::VecX_T<Scalar> q;
		mathlib::VecX_T<Scalar> qd;
		mathlib::VecX_T<Scalar> qdd;

		mathlib::VecX_T<Scalar> err;
		mathlib::VecX_T<Scalar> errd;

		// Dynamics
		mathlib::VecX_T<Scalar> I_eff;
		mathlib::VecX_T<Scalar> tau;

		// Constraints / realism
		mathlib::VecX_T<Scalar> tau_barrier;
		mathlib::VecX_T<Scalar> tau_sat;

		// Energy, Work, & Power
		mathlib::VecX_T<Scalar> KE;
		mathlib::VecX_T<Scalar> PE;
		mathlib::VecX_T<Scalar> E_total;
		mathlib::VecX_T<Scalar> W_actuator;

		// Stability flags
		std::vector<uint8_t> sat_flag;

		void resize(size_t n) {
			q.resize(n); qd.resize(n); qdd.resize(n);
			err.resize(n); errd.resize(n);
			I_eff.resize(n); tau.resize(n);
			tau_barrier.resize(n); tau_sat.resize(n);
			KE.resize(n); PE.resize(n); E_total.resize(n); W_actuator.resize(n);
			sat_flag.resize(n, 0);
		}
	};
}