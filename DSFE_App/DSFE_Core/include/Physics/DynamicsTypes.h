/*
 * File: Physics/DynamicsTypes.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include <core/Types.h>
#include <core/SpatialMath>

#include "Systems/RigidBodyMetrics.h"

namespace physics {
	// Scratch buffers for dense dynamics
	template<typename Scalar>
	struct DenseDynamicsScratch {
		mathlib::MatX_T<Scalar> M;   // mass matrix
		mathlib::VecX_T<Scalar> rhs; // right-hand side vector for dynamics equations (Coriolis, gravity, control torques)
		mathlib::VecX_T<Scalar> h;   // Coriolis and centrifugal bias vector
		mathlib::VecX_T<Scalar> tau; // control torque vector
		mathlib::VecX_T<Scalar> tau_g; // gravity torque vector
		mathlib::VecX_T<Scalar> I_eff_controller; // effective inertia vector for controller design (e.g., for inverse dynamics control)

		std::vector<mathlib::Pose_T<Scalar>> T_world;
		std::vector<mathlib::Pose_T<Scalar>> jointWorldPoses;

		size_t jointCap = 0;
		size_t linkCap = 0;

		// Resizes the scratch buffers
		void resize(size_t nJoints, size_t nLinks) {
			// Do not resize if the current capacities are sufficients
			if (jointCap == nJoints
				&& linkCap == nLinks) {
				return;
			}

			// Dense buffers
			M.resize(nJoints, nJoints);
			rhs.resize(nJoints);
			h.resize(nJoints);
			tau.resize(nJoints);
			tau_g.resize(nJoints);
			I_eff_controller.resize(nJoints);
			T_world.resize(nLinks);
			jointWorldPoses.resize(nJoints);

			jointCap = nJoints;
			linkCap = nLinks;
		}

		// Sets all buffers to zero or identity
		//  * (ONLY FOR DEBUGGING PURPOSES, CALL clear() FOR PRODUCTION USE)
		void zero() {
			// Dense buffers
			M.setZero();
			rhs.setZero();
			h.setZero();
			tau.setZero();
			tau_g.setZero();
			I_eff_controller.setZero();
			for (auto& T : T_world) T.setIdentity();
			for (auto& T : jointWorldPoses) T.setIdentity();
		}

		void clear() {
			// Dense buffers
			M.resize(0, 0);
			rhs.resize(0);
			h.resize(0);
			tau.resize(0);
			tau_g.resize(0);
			I_eff_controller.resize(0);
			T_world.clear();
			jointWorldPoses.clear();
			jointCap = 0;
			linkCap = 0;
		}
	};

	// Scratch buffers for spatial dynamics computations
	template<typename Scalar>
	struct SpatialDynamicsScratch {
		std::vector<mathlib::SpatialMat_T<Scalar>> Xup; // spatial transformation from parent to current link
		std::vector<mathlib::SpatialMat_T<Scalar>> IA;  // articulated body inertia
		std::vector<mathlib::SpatialMat_T<Scalar>> Ia;  // articulated body inertia in the link frame
		
		std::vector<mathlib::SpatialVec_T<Scalar>> v;  // spatial velocity
		std::vector<mathlib::SpatialVec_T<Scalar>> c;  // spatial bias acceleration
		std::vector<mathlib::SpatialVec_T<Scalar>> a;  // spatial acceleration
		std::vector<mathlib::SpatialVec_T<Scalar>> pA; // articulated bias force
		std::vector<mathlib::SpatialVec_T<Scalar>> U;  // articulated body force
		std::vector<mathlib::SpatialVec_T<Scalar>> f_ext;  // spatial force

		std::vector<mathlib::MatX_T<Scalar>> dblk;   // 6x6 per joint (free joints use it)
		std::vector<mathlib::VecX_T<Scalar>> ublk;   // 6 per joint

		mathlib::VecX_T<Scalar> g; // gravity vector in spatial coordinates (6D)
		mathlib::VecX_T<Scalar> u; // joint force contribution
		mathlib::VecX_T<Scalar> d; // joint inertia contribution

		std::vector<std::vector<mathlib::SpatialMat_T<Scalar>>> dXup_dq; // derivative of spatial transformation w.r.t. joint angles

		std::vector<std::vector<mathlib::SpatialVec_T<Scalar>>> dv_dq;  // derivative of spatial velocity w.r.t. joint angles
		std::vector<std::vector<mathlib::SpatialVec_T<Scalar>>> dv_dqd; // derivative of spatial velocity w.r.t. joint velocities
		std::vector<std::vector<mathlib::SpatialVec_T<Scalar>>> dc_dq;  // derivative of spatial bias acceleration w.r.t. joint angles
		std::vector<std::vector<mathlib::SpatialVec_T<Scalar>>> dc_dqd; // derivative of spatial bias acceleration w.r.t. joint velocities

		size_t jointCap = 0;

		// Resizes the scratch buffers
		void resize(size_t nJoints) {
			// Do not resize if the current capacities are sufficient
			if (jointCap == nJoints) { return; }

			// Spatial buffers
			Xup.resize(nJoints);
			IA.resize(nJoints);
			Ia.resize(nJoints);
			v.resize(nJoints);
			c.resize(nJoints);
			a.resize(nJoints);
			pA.resize(nJoints);
			U.resize(nJoints);
			f_ext.resize(nJoints);
			dblk.resize(nJoints);
			ublk.resize(nJoints);
			
			g.resize(6); // gravity vector is always 6D
			u.resize(nJoints);
			d.resize(nJoints);

			dXup_dq.resize(nJoints);
			dv_dq.resize(nJoints);
			dv_dqd.resize(nJoints);
			dc_dq.resize(nJoints);
			dc_dqd.resize(nJoints);

			jointCap = nJoints;
		}

		void clear() {
			Xup.clear();
			IA.clear();
			Ia.clear();
			v.clear();
			c.clear();
			a.clear();
			pA.clear();
			U.clear();
			f_ext.clear();
			dblk.clear();
			ublk.clear();

			g.resize(0);
			u.resize(0);
			d.resize(0);
			
			dXup_dq.clear();
			dv_dq.clear();
			dv_dqd.clear();
			dc_dq.clear();
			dc_dqd.clear();

			jointCap = 0;
		}
	};

	// Output structure for dynamics computations
	template<typename Scalar>
	struct DynamicsResult {
		mathlib::VecX_T<Scalar> dxdt;
		mathlib::VecX_T<Scalar> qdd;

		systems::RigidBodyMetrics<Scalar> metrics;

		void resize(size_t n) {
			dxdt.resize(2 * n);
			qdd.resize(n);
			metrics.resize(n);
		}
	};

	// Central scratch structure that contains all buffers needed for dynamics computations, both dense and spatial
	template<typename Scalar>
	struct DynamicsScratch {
		DenseDynamicsScratch<Scalar> dense;
		SpatialDynamicsScratch<Scalar> spatial;
		// TODO add kinematics scratch

		// Gravity scratch buffer
		mathlib::VecX_T<Scalar> g;

		// Resizes all scratch buffers using the given number of joints and links
		void resize(size_t nJoints, size_t nLinks) {
			dense.resize(nJoints, nLinks);
			spatial.resize(nJoints);
			g.resize(6);
		}

		// Clears all scratch buffers
		void clear() {
			dense.clear();
			spatial.clear();
			g.resize(0);
		}
	};
} // namespace physics