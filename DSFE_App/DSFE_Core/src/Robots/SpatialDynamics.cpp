// DSFE_Core SpatialDynamics.cpp
#include "pch.h"

#include "Robots/SpatialDynamics.h"

namespace robots {
	// Recursive function to compute spatial velocities using the articulated body algorithm
	void SpatialDynamics::computeSpatialKinematicsAndBias(
		const SpatialModel& model,
		const mathlib::VecX& q,
		const mathlib::VecX& qd,
		std::vector<mathlib::SpatialMat>& Xup_out,
		std::vector<mathlib::SpatialVec>& v_out,
		std::vector<mathlib::SpatialVec>& c_out
	) {
		const size_t n = model.joints.size();
		v_out.resize(n);
		Xup_out.resize(n);
		c_out.resize(n);

		for (size_t i = 0; i < n; ++i) {
			const SpatialJoint& j = model.joints[i];

			// Joint Transform XJ
			mathlib::SpatialMat XJ = mathlib::SpatialMat::Identity();
			
			if (j.type == eJointType::REVOLUTE) {
				Eigen::AngleAxisd aa(q[i], j.S.angular().normalized());
				mathlib::Mat3 R = aa.toRotationMatrix();
				XJ = mathlib::spatialTransform(R, mathlib::Vec3::Zero());
			}
			else if (j.type == eJointType::PRISMATIC) {
				mathlib::Vec3 r = q[i] * j.S.linear().normalized();
				XJ = mathlib::spatialTransform(mathlib::Mat3::Identity(), r);
			}

			Xup_out[i] = XJ * j.Xtree; // Combined Transform
			mathlib::SpatialVec vJ = j.S * qd[i]; // Joint Velocity

			// Root Link
			if (j.parent < 0) { v_out[i] = vJ; }
			else { v_out[i] = Xup_out[i] * v_out[j.parent] + vJ; }

			c_out[i] = crossMotion(v_out[i], vJ); // Coriolis Term
		}
	}

	// Recursive function to compute spatial accelerations using the articulated body algorithm
	void SpatialDynamics::computeAccelerations_RNEA(
		const SpatialModel& model,
		const mathlib::VecX& qdd, 
		const std::vector<mathlib::SpatialMat>& Xup,
		const std::vector<mathlib::SpatialVec>& c,
		const mathlib::VecX& g,
		std::vector<mathlib::SpatialVec>& a_out
	) {
		const size_t n = model.joints.size();
		a_out.resize(n);

		SpatialVec a0;
		a0.v << g.segment<3>(0), g.segment<3>(3); // Base acceleration (gravity)

		for (size_t i = 0; i < n; ++i) {
			const SpatialJoint& j = model.joints[i];
			mathlib::SpatialVec aJ = j.S * qdd[i]; // Joint Acceleration

			// Root Link
			if (j.parent < 0) {
				a_out[i] = Xup[i] * a0 + aJ + c[i];
				continue;
			}
			
			a_out[i] = Xup[i] * a_out[j.parent] + aJ + c[i];
		}
	}

	// Recursive function to compute inverse dynamics (joint torques) using the articulated body algorithm
	void SpatialDynamics::computeBackwardForces_RNEA(
		const SpatialModel& model,
		const std::vector<mathlib::SpatialMat>& Xup,
		const std::vector<mathlib::SpatialVec>& v,
		const std::vector<mathlib::SpatialVec>& a,
		mathlib::VecX& tau_out
	) {
		const size_t n = model.joints.size();
		tau_out.resize(n);

		std::vector<mathlib::SpatialVec> f(n);

		// Forward Force Computation
		for (size_t i = 0; i < n; ++i) {
			const SpatialJoint& j = model.joints[i];
			mathlib::SpatialVec I_v = j.inertia * v[i];
			mathlib::SpatialVec coriolis = crossForce(v[i], I_v);
			f[i].v = j.inertia * a[i].v + coriolis.v;
		}

		// Backward Recursion Computation
		for (int i = (int)n - 1; i >= 0; --i) {
			const SpatialJoint& j = model.joints[i];
			tau_out[i] = j.S.dot(f[i]);
			if (j.parent >= 0) { f[j.parent] += Xup[i].transpose() * f[i]; }
		}
	}

	// Main function to compute inverse dynamics (joint torques) given joint states and accelerations using the Recursive Newton-Euler Algorithm (RNEA)
	mathlib::VecX SpatialDynamics::RNEA(
		const SpatialModel& model,
		const mathlib::VecX& q,
		const mathlib::VecX& qd,
		const mathlib::VecX& qdd,
		DynamicsScratch& scratch
	) {
		const size_t n = model.joints.size();

		// TODO Remove these temp scratches AFTER debugging
		std::vector<mathlib::SpatialVec> v(n);
		std::vector<mathlib::SpatialMat> Xup(n);
		std::vector<mathlib::SpatialVec> c(n);
		std::vector<mathlib::SpatialVec> a(n);
		mathlib::VecX tau;

		// Compute spatial velocities and transforms
		computeSpatialKinematicsAndBias(model, q, qd, Xup, v, c);
		// Compute spatial accelerations
		computeAccelerations_RNEA(model, qdd, Xup, c, scratch.g, a);
		// Compute inverse dynamics (joint torques)
		computeBackwardForces_RNEA(model, Xup, v, a, tau);

		return tau;
	}

	// Main function to compute the mass matrix of the robot at a given configuration using the Composite Rigid Body Algorithm (CRBA)
	mathlib::MatX SpatialDynamics::CRBA(
		const SpatialModel& model,
		const std::vector<mathlib::SpatialMat>& Xup,
		DynamicsScratch& scratch
	) {
		const size_t n = model.joints.size();
		scratch.dense.M.setZero(n, n);
		std::vector<SpatialMat> Ic(n); // spatial inertia for each link

		// Initialise spatial inertia for each link based on the robot model
		for (size_t i = 0; i < n; ++i) { Ic[i] = model.joints[i].inertia; }

		// Upward pass: propagate spatial inertia from child links to parent joints
		for (int i = (int)n - 1; i >= 0; --i) {
			const SpatialJoint& j = model.joints[i];
			if (j.type == eJointType::FIXED) { continue; }
			int p = j.parent;
			if (p >= 0) {
				Ic[p] += Xup[i].transpose() * Ic[i] * Xup[i];
			}
		}

		// Downward pass: compute mass matrix contributions for each joint
		for (size_t i = 0; i < n; ++i) {
			const SpatialJoint& j = model.joints[i];
			if (j.type == eJointType::FIXED) { continue; }
			SpatialVec F = Ic[i] * j.S;
			scratch.dense.M(i, i) = j.S.dot(F);

			int jIdx = (int)i;
			while (model.joints[jIdx].parent >= 0) {
				int p = model.joints[jIdx].parent;
				F = Xup[jIdx].transpose() * F;
				scratch.dense.M(i, p) = model.joints[p].S.dot(F);
				scratch.dense.M(p, i) = scratch.dense.M(i, p);
				jIdx = p;
			}
		}
		return scratch.dense.M; // [kg*m^2], mass matrix computed using the Composite Rigid Body Algorithm (CRBA)
	}

	// Recursive function to compute articulated body inertias and bias forces using the Articulated Body Algorithm (ABA)
	void SpatialDynamics::computeArticulatedBodies_ABA(
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
	) {
		const size_t n = model.joints.size();

		// Resize scratch buffers
		IA_out.resize(n);
		pA_out.resize(n);
		Ia_out.resize(n);
		U_out.resize(n);
		u_out.resize(n);
		d_out.resize(n);

		// Upward pass: compute articulated body inertias and bias forces
		for (int i = (int)n - 1; i >= 0; --i) {
			const SpatialJoint& j = model.joints[i];
			
			if (j.type == eJointType::FIXED) {
				Ia_out[i] = IA_out[i];
				if (j.parent >= 0) {
					IA_out[j.parent] += Xup[i].transpose() * Ia_out[i] * Xup[i];
					pA_out[j.parent] += Xup[i].transpose() * pA_out[i];
				}
				continue;
			}

			U_out[i] = IA_out[i] * j.S;
			d_out[i] = dot(j.S, U_out[i]);
			if (std::abs(d_out[i]) < 1e-12) { d_out[i] = 1e-12; } // Regularisation to avoid singularities

			u_out[i] = tau[i] - dot(j.S, pA_out[i]);
			Ia_out[i] = IA_out[i] - outer(U_out[i]) / d_out[i];

			// pA = pA + Ia * c + U * (u/d)
			pA_out[i] += Ia_out[i] * c[i] + U_out[i] * (u_out[i] / d_out[i]);

			if (j.parent >= 0) {
				IA_out[j.parent] += Xup[i].transpose() * Ia_out[i] * Xup[i];
				pA_out[j.parent] += Xup[i].transpose() * pA_out[i];
			}
		}
	}

	// Recursive function to compute joint accelerations using the Articulated Body Algorithm (ABA)
	void SpatialDynamics::computeAccelerations_ABA(
		const SpatialModel& model,
		const std::vector<SpatialMat>& Xup,
		const std::vector<SpatialVec>& c,
		const mathlib::VecX& u_out,
		const mathlib::VecX& d_out,
		const std::vector<SpatialVec>& U,
		const SpatialVec& a0,
		std::vector<SpatialVec>& a_out,
		mathlib::VecX& qdd_out
	) {
		const size_t n = model.joints.size();
		a_out.resize(n);
		qdd_out.resize(n);

		for (size_t i = 0; i < n; ++i) {
			const SpatialJoint& j = model.joints[i];

			if (j.parent < 0) { a_out[i] = Xup[i] * a0 + c[i]; }
			else { a_out[i] = Xup[i] * a_out[j.parent] + c[i]; }

			if (j.type == eJointType::FIXED) {
				qdd_out[i] = 0.0;
				continue;
			}

			qdd_out[i] = (u_out[i] - U[i].dot(a_out[i])) / d_out[i];
			a_out[i] += j.S * qdd_out[i];
		}
	}

	// Main function to compute joint accelerations given joint states and torques using the Articulated Body Algorithm (ABA)
	mathlib::VecX SpatialDynamics::ABA(
		const SpatialModel& model,
		const mathlib::VecX& q,
		const mathlib::VecX& qd,
		const mathlib::VecX& tau,
		DynamicsScratch& scratch
	) {
		const size_t n = model.joints.size();
		VecX qdd = VecX::Zero(n);

		SpatialVec a0; // base acceleration (gravity)
		a0.v << scratch.g.segment<3>(0), scratch.g.segment<3>(3);

		computeSpatialKinematicsAndBias(
			model, q, qd,
			scratch.spatial.Xup,
			scratch.spatial.v,
			scratch.spatial.c
		);

		for (size_t i = 0; i < n; ++i) {
			scratch.spatial.IA[i] = model.joints[i].inertia; // Articulated Body Inertia
			scratch.spatial.pA[i] = crossForce(scratch.spatial.v[i], (scratch.spatial.IA[i] * scratch.spatial.v[i]));
		}

		// Compute articulated body inertias and bias forces
		computeArticulatedBodies_ABA(
			model, scratch.spatial.Xup,
			scratch.spatial.v, scratch.spatial.c, tau,
			scratch.spatial.IA, scratch.spatial.pA, scratch.spatial.Ia,
			scratch.spatial.u, scratch.spatial.d, scratch.spatial.U
		);

		// Compute joint accelerations using the articulated body algorithm
		computeAccelerations_ABA(
			model, scratch.spatial.Xup, scratch.spatial.c,
			scratch.spatial.u, scratch.spatial.d, scratch.spatial.U,
			a0, scratch.spatial.a,qdd
		);

		return qdd; // [rad/s^2], joint accelerations computed using the Articulated Body Algorithm (ABA)
	}
}