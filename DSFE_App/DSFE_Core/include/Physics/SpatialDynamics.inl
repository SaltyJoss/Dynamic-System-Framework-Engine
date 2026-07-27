/*
 * File: Physics/SpatialDynamics.inl
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

namespace physics {
	template<typename Scalar>
	void SpatialDynamics::computeSpatialKinematicsAndBias(
		const systems::SpatialModel<Scalar>& model,
		const mathlib::VecX_T<Scalar>& q,
		const mathlib::VecX_T<Scalar>& qd,
		std::vector<mathlib::SpatialMat_T<Scalar>>& Xup_out,
		std::vector<mathlib::SpatialVec_T<Scalar>>& v_out,
		std::vector<mathlib::SpatialVec_T<Scalar>>& c_out
	) {
		const size_t n = model.joints.size();
		v_out.resize(n);
		Xup_out.resize(n);
		c_out.resize(n);

		for (size_t i = 0; i < n; ++i) {
			const systems::SpatialJoint<Scalar>& j = model.joints[i];

			// Joint Transform XJ
			mathlib::SpatialMat_T<Scalar> XJ = mathlib::SpatialMat_T<Scalar>::Identity();

			if (j.type == systems::eJointType::REVOLUTE) {
				mathlib::Vec3_T<Scalar> axis = mathlib::safeNormalised(j.S.angular());
				mathlib::Mat3_T<Scalar> R = mathlib::AngleAxis(q[i], axis);
				mathlib::Vec3_T<Scalar> r = mathlib::Vec3_T<Scalar>::Zero();
				XJ = mathlib::spatialTransform(R, r);
			}
			else if (j.type == systems::eJointType::PRISMATIC) {
				mathlib::Vec3_T<Scalar> axis = mathlib::safeNormalised(j.S.linear());
				mathlib::Vec3_T<Scalar> r = q[i] * axis;
				mathlib::Mat3_T<Scalar> R = mathlib::Mat3_T<Scalar>::Identity();
				XJ = mathlib::spatialTransform(R, r);
			}

			Xup_out[i] = XJ * j.Xtree; // Combined Transform
			mathlib::SpatialVec_T<Scalar> vJ = j.S * qd[i]; // Joint Velocity

			// Root Link
			if (j.parent < 0) { v_out[i] = vJ; }
			else { v_out[i] = Xup_out[i] * v_out[j.parent] + vJ; }

			c_out[i] = crossMotion(v_out[i], vJ); // Coriolis Term

			using corScalar = typename std::decay_t<decltype(c_out[i].v(0))>;
			static_assert(std::is_same_v<corScalar, Scalar>, "c_out scalar type does not match model scalar type");
		}
	}

	template<typename Scalar>
	void SpatialDynamics::computeAccelerations_RNEA(
		const systems::SpatialModel<Scalar>& model,
		const mathlib::VecX_T<Scalar>& qdd,
		const std::vector<mathlib::SpatialMat_T<Scalar>>& Xup,
		const std::vector<mathlib::SpatialVec_T<Scalar>>& c,
		const mathlib::VecX_T<Scalar>& g,
		std::vector<mathlib::SpatialVec_T<Scalar>>& a_out
	) {
		const size_t n = model.joints.size();
		a_out.resize(n);

		mathlib::SpatialVec_T<Scalar> a0; // base acceleration (gravity)
		a0.v <<
			g.template segment<3>(0),
			g.template segment<3>(3);

		for (size_t i = 0; i < n; ++i) {
			const systems::SpatialJoint<Scalar>& j = model.joints[i];
			mathlib::SpatialVec_T<Scalar> aJ = j.S * qdd[i]; // Joint Acceleration

			// Root Link
			if (j.parent < 0) {
				a_out[i] = Xup[i] * a0 + aJ + c[i];
				continue;
			}

			a_out[i] = Xup[i] * a_out[j.parent] + aJ + c[i];
		}
	}

	template<typename Scalar>
	void SpatialDynamics::computeBackwardForces_RNEA(
		const systems::SpatialModel<Scalar>& model,
		const std::vector<mathlib::SpatialMat_T<Scalar>>& Xup,
		const std::vector<mathlib::SpatialVec_T<Scalar>>& v,
		const std::vector<mathlib::SpatialVec_T<Scalar>>& a,
		mathlib::VecX_T<Scalar>& tau_out
	) {
		const size_t n = model.joints.size();
		tau_out.resize(n);

		std::vector<mathlib::SpatialVec_T<Scalar>> f(n);

		// Forward Force Computation
		for (size_t i = 0; i < n; ++i) {
			const systems::SpatialJoint<Scalar>& j = model.joints[i];
			mathlib::SpatialVec_T<Scalar> I_v = j.inertia * v[i];
			mathlib::SpatialVec_T<Scalar> coriolis = crossForce(v[i], I_v);
			f[i].v = j.inertia * a[i].v + coriolis.v;
		}

		// Backward Recursion Computation
		for (int i = (int)n - 1; i >= 0; --i) {
			const systems::SpatialJoint<Scalar>& j = model.joints[i];
			tau_out[i] = j.S.dot(f[i]);
			mathlib::SpatialMat_T<Scalar> XupT = Xup[i].transpose();
			if (j.parent >= 0) { f[j.parent] += XupT * f[i]; }
		}
	}

	template<typename Scalar>
	mathlib::VecX_T<Scalar> SpatialDynamics::RNEA(
		const systems::SpatialModel<Scalar>& model,
		const mathlib::VecX_T<Scalar>& q,
		const mathlib::VecX_T<Scalar>& qd,
		const mathlib::VecX_T<Scalar>& qdd,
		DynamicsScratch<Scalar>& scratch
	) {
		const size_t n = model.joints.size();

		// TODO Remove these temp scratches AFTER debugging
		std::vector<mathlib::SpatialVec_T<Scalar>> v(n);
		std::vector<mathlib::SpatialMat_T<Scalar>> Xup(n);
		std::vector<mathlib::SpatialVec_T<Scalar>> c(n);
		std::vector<mathlib::SpatialVec_T<Scalar>> a(n);
		mathlib::VecX_T<Scalar> tau;

		// Compute spatial velocities and transforms
		computeSpatialKinematicsAndBias(model, q, qd, Xup, v, c);
		// Compute spatial accelerations
		computeAccelerations_RNEA(model, qdd, Xup, c, scratch.spatial.g, a);
		// Compute inverse dynamics (joint torques)
		computeBackwardForces_RNEA(model, Xup, v, a, tau);

		return tau;
	}

	template<typename Scalar>
	mathlib::MatX_T<Scalar> SpatialDynamics::CRBA(
		const systems::SpatialModel<Scalar>& model,
		const std::vector<mathlib::SpatialMat_T<Scalar>>& Xup,
		DynamicsScratch<Scalar>& scratch
	) {
		const size_t n = model.joints.size();
		scratch.dense.M.setZero(n, n);
		std::vector<mathlib::SpatialMat_T<Scalar>> Ic(n); // spatial inertia for each link

		// Initialise spatial inertia for each link based on the robot model
		for (size_t i = 0; i < n; ++i) { Ic[i] = model.joints[i].inertia; }

		// Upward pass: propagate spatial inertia from child links to parent joints
		for (int i = (int)n - 1; i >= 0; --i) {
			const systems::SpatialJoint<Scalar>& j = model.joints[i];
			if (j.type == systems::eJointType::FIXED) { continue; }
			int p = j.parent;
			if (p >= 0) {
				mathlib::MatX_T<Scalar> XupT = Xup[i].transpose();
				Ic[p] += XupT * Ic[i] * Xup[i];
			}
		}

		// Downward pass: compute mass matrix contributions for each joint
		for (size_t i = 0; i < n; ++i) {
			const systems::SpatialJoint<Scalar>& j = model.joints[i];
			if (j.type == systems::eJointType::FIXED) { continue; }
			mathlib::SpatialVec_T<Scalar> F = Ic[i] * j.S;
			scratch.dense.M(i, i) = j.S.dot(F);

			int jIdx = (int)i;
			while (model.joints[jIdx].parent >= 0) {
				int p = model.joints[jIdx].parent;
				mathlib::SpatialMat_T<Scalar> XupT = Xup[jIdx].transpose(); // TODO Make Eigen-copatible operator overloads for spatial transforms to avoid the errors from this transpose operation in a matrix multiplication context
				F = XupT * F;
				scratch.dense.M(i, p) = model.joints[p].S.dot(F);
				scratch.dense.M(p, i) = scratch.dense.M(i, p);
				jIdx = p;
			}
		}
		return scratch.dense.M; // [kg*m^2], mass matrix computed using the Composite Rigid Body Algorithm (CRBA)
	}

	template<typename Scalar>
	void SpatialDynamics::computeArticulatedBodies_ABA(
		const systems::SpatialModel<Scalar>& model,
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
			const systems::SpatialJoint<Scalar>& j = model.joints[i];

			if (j.type == systems::eJointType::FIXED) {
				Ia_out[i] = IA_out[i];
				if (j.parent >= 0) {
					mathlib::SpatialMat_T<Scalar> XupT = Xup[i].transpose();
					IA_out[j.parent] += XupT * Ia_out[i] * Xup[i];
					pA_out[j.parent] += XupT * pA_out[i];
				}
				continue;
			}

			U_out[i] = IA_out[i] * j.S;
			d_out[i] = dot(j.S, U_out[i]);
			if (d_out[i] < Scalar(1e-12)) {
				d_out[i] = Scalar(1e-12);
			}

			u_out[i] = tau[i] - dot(j.S, pA_out[i]);
			Ia_out[i] = IA_out[i] - outer(U_out[i]) / d_out[i];

			// pA = pA + Ia * c + U * (u/d)
			pA_out[i] += Ia_out[i] * c[i] + U_out[i] * (u_out[i] / d_out[i]);

			if (j.parent >= 0) {
				mathlib::SpatialMat_T<Scalar> XupT = Xup[i].transpose();
				IA_out[j.parent] += XupT * Ia_out[i] * Xup[i];
				pA_out[j.parent] += XupT * pA_out[i];
			}
		}
	}

	template<typename Scalar>
	void SpatialDynamics::computeAccelerations_ABA(
		const systems::SpatialModel<Scalar>& model,
		const std::vector<SpatialMat_T<Scalar>>& Xup,
		const std::vector<SpatialVec_T<Scalar>>& c,
		const mathlib::VecX_T<Scalar>& u_out,
		const mathlib::VecX_T<Scalar>& d_out,
		const std::vector<SpatialVec_T<Scalar>>& U,
		const SpatialVec_T<Scalar>& a0,
		std::vector<SpatialVec_T<Scalar>>& a_out,
		mathlib::VecX_T<Scalar>& qdd_out
	) {
		const size_t n = model.joints.size();
		a_out.resize(n);
		qdd_out.resize(n);

		for (size_t i = 0; i < n; ++i) {
			const systems::SpatialJoint<Scalar>& j = model.joints[i];

			if (j.parent < 0) { a_out[i] = Xup[i] * a0 + c[i]; }
			else { a_out[i] = Xup[i] * a_out[j.parent] + c[i]; }

			if (j.type == systems::eJointType::FIXED) {
				qdd_out[i] = Scalar(0);
				continue;
			}

			qdd_out[i] = (u_out[i] - U[i].dot(a_out[i])) / d_out[i];
			a_out[i] += j.S * qdd_out[i];
		}
	}

	template<typename Scalar>
	mathlib::VecX_T<Scalar> SpatialDynamics::ABA(
		const systems::SpatialModel<Scalar>& model,
		const mathlib::VecX_T<Scalar>& q,
		const mathlib::VecX_T<Scalar>& qd,
		const mathlib::VecX_T<Scalar>& tau,
		DynamicsScratch<Scalar>& scratch
	) {
		const size_t n = model.joints.size();
		mathlib::VecX_T<Scalar> qdd = mathlib::VecX_T<Scalar>::Zero(n);

		mathlib::SpatialVec_T<Scalar> a0; // base acceleration (gravity)
		a0.v <<
			scratch.spatial.g.template segment<3>(0),
			scratch.spatial.g.template segment<3>(3);

		computeSpatialKinematicsAndBias(
			model, q, qd,
			scratch.spatial.Xup,
			scratch.spatial.v,
			scratch.spatial.c
		);

		const bool hasExt = (scratch.spatial.f_ext.size() == n);
		for (size_t i = 0; i < n; ++i) {
			scratch.spatial.IA[i] = model.joints[i].inertia; // Articulated Body Inertia
			scratch.spatial.pA[i] = crossForce(scratch.spatial.v[i], (scratch.spatial.IA[i] * scratch.spatial.v[i]));
			if (hasExt) { scratch.spatial.pA[i] += scratch.spatial.f_ext[i]; }
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
			a0, scratch.spatial.a, qdd
		);

		return qdd; // [rad/s^2], joint accelerations computed using the Articulated Body Algorithm (ABA)
	}
} // namespace physics