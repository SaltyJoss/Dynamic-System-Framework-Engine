// DSFE_Core RobotMetrics.h
#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>

namespace robots {
	// Per-joint metrics
	struct RobotMetrics {
		// State
		mathlib::VecX q;
		mathlib::VecX qd;
		mathlib::VecX qdd;

		mathlib::VecX err;
		mathlib::VecX errd;

		// Dynamics
		mathlib::VecX I_eff;
		mathlib::VecX tau;

		// Constraints / realism
		mathlib::VecX tau_barrier;
		mathlib::VecX tau_sat;

		// Energy, Work, & Power
		mathlib::VecX KE;
		mathlib::VecX PE;
		mathlib::VecX E_total;
		mathlib::VecX W_actuator;

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