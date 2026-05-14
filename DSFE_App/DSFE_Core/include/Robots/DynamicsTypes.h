// DSFE_Core DynamicsTypes.h
#pragma once

#include "EngineCore.h"
#include "MathLibAPI.h"
#include <core/Types.h>
#include "Robots/RobotMetrics.h"

namespace robots {
	struct DynamicsScratch {
		mathlib::MatX M; // mass matrix
		mathlib::VecX rhs; // right-hand side vector for dynamics equations (Coriolis, gravity, control torques)
		mathlib::VecX h; // Coriolis and centrifugal bias vector
		mathlib::VecX g; // gravity torque vector
		mathlib::VecX tau; // control torque vector

		std::vector<Pose> T_world;
		std::vector<Pose> jointWorldPoses;

		size_t jointCap = 0;
		size_t linkCap = 0;

		// Resizes the scratch buffers
		void resize(size_t nJoints, size_t nLinks) {
			// Do not resize if the current capacities are sufficient
			if (jointCap == nJoints
				&& linkCap && nLinks) {
				return;
			}

			M.resize(nJoints, nJoints);
			rhs.resize(nJoints);
			h.resize(nJoints);
			g.resize(nJoints);
			tau.resize(nJoints);
			T_world.resize(nLinks);
			jointWorldPoses.resize(nJoints);
		}

		void zero() {
			M.setZero();
			rhs.setZero();
			h.setZero();
			g.setZero();
			tau.setZero();
			for (auto& T : T_world) T.setIdentity();
			for (auto& T : jointWorldPoses) T.setIdentity();
		}
	};

	struct DynamicsResult {
		mathlib::VecX dxdt;
		mathlib::VecX qdd;

		RobotMetrics metrics;

		void resize(size_t n) {
			dxdt.resize(2 * n);
			qdd.resize(n);
			metrics.resize(n);
		}
	};
}