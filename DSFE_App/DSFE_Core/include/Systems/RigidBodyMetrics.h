// DSFE_Core RobotMetrics.h
#pragma once

#include "EngineCore.h"


namespace systems {
	// Per-joint metrics
	template<typename Scalar>
	struct RigidBodyMetrics {
		// State
		mathlib::VecX_T<Scalar> q;
		mathlib::VecX_T<Scalar> qd;
		mathlib::VecX_T<Scalar> qdd;

		mathlib::VecX_T<Scalar> err;
		mathlib::VecX_T<Scalar> errd;

		// Dynamics
		mathlib::VecX_T<Scalar> I_eff;
		mathlib::VecX_T<Scalar> tau;
		mathlib::VecX_T<Scalar> tau_g;

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
			I_eff.resize(n); tau.resize(n); tau_g.resize(n);
			tau_barrier.resize(n); tau_sat.resize(n);
			KE.resize(n); PE.resize(n); E_total.resize(n); W_actuator.resize(n);
			sat_flag.resize(n, 0);
		}
	};

	// Free-body metrics
	template<typename Scalar>
	struct FreeBodyMetrics {
		//State
		mathlib::Vec3_T<Scalar> pos;
		mathlib::Quat_T<Scalar> orient;
		mathlib::Vec3_T<Scalar> linVel;
		mathlib::Vec3_T<Scalar> angVel;
		// Time Derivates
		mathlib::Vec3_T<Scalar> linAcc;
		mathlib::Vec3_T<Scalar> angAcc;
		mathlib::Vec3_T<Scalar> F_net;
		mathlib::Vec3_T<Scalar> tau_net;
		// Energy & Performance
		Scalar KE;
		Scalar PE;
		mathlib::Vec3_T<Scalar> linMomentum;
		mathlib::Vec3_T<Scalar> angMomentum;
		// Sleep State
		bool sleepState = false;
		// Mass & Inertia
		Scalar mass;
		Scalar inverse_mass;
		mathlib::Mat3_T<Scalar> inertia;
		mathlib::Mat3_T<Scalar> inverse_inertia;

		void resize(size_t n) {
			pos.resize(n); orient.resize(n); linVel.resize(n); angVel.resize(n);
			linAcc.resize(n); angAcc.resize(n); F_net.resize(n); tau_net.resize(n);
			KE.resize(n); PE.resize(n); linMomentum.resize(n); angMomentum.resize(n);
			mass.resize(n); inverse_mass.resize(n);
			inertia.resize(3, 3 * n); inverse_inertia.resize(3, 3 * n);
		}
	};
} // namespace systems