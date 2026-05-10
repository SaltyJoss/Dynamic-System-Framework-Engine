// DSFE_Core SpatialDynamics.cpp
#include "pch.h"

#include "Robots/SpatialDynamics.h"

namespace robots {
	// Recursive function to compute spatial velocities using the articulated body algorithm
	void SpatialDynamics::computeSpatialVelocities(
		const SpatialModel& model,
		const mathlib::VecX& q,
		const mathlib::VecX& qd,
		std::vector<mathlib::SpatialVec>& v_out,
		std::vector<mathlib::SpatialMat>& Xup_out

	) {
		const size_t n = model.joints.size();
		v_out.resize(n);
		Xup_out.resize(n);

		for (size_t i = 0; i < n; ++i) {
			const SpatialJoint& j = model.joints[i];

			// Joint Transform XJ
			mathlib::SpatialMat XJ = mathlib::SpatialMat::Identity();
			
			if (j.type == eJointType::REVOLUTE) {
				Eigen::AngleAxisd aa(
					q[i],
					j.S.angular().normalized()
				);

				mathlib::Mat3 R = aa.toRotationMatrix();
				XJ = mathlib::spatialTransform(R, mathlib::Vec3::Zero());
			}
			else if (j.type == eJointType::PRISMATIC) {
				mathlib::Vec3 r = q[i] * j.S.linear().normalized();
				XJ = mathlib::spatialTransform(mathlib::Mat3::Identity(), r);
			}

			// Combined Transform
			Xup_out[i] = XJ * j.Xtree;

			// Joint Velocity
			mathlib::SpatialVec vJ = j.S * qd[i];

			// Root Link
			if (j.parent < 0) {
				v_out[i] = vJ;
				continue;
			}

			// Propagate Velocity
			mathlib::SpatialVec v_parent;

			v_parent.v = Xup_out[i] * v_out[j.parent].v;
			v_out[i] = v_parent + vJ;

			if (j.name == "joint03") {
				LOG_INFO_ONCE("joint=%s, parent=%d | w=(%f, %f, %f), v=(%f, %f, %f)"
					, j.name.c_str(), j.parent,
					v_out[i].angular().x(), v_out[i].angular().y(), v_out[i].angular().z(),
					v_out[i].linear().x(), v_out[i].linear().y(), v_out[i].linear().z()
				);
			}
		}
	}

	// Recursive function to compute spatial accelerations using the articulated body algorithm
	void SpatialDynamics::computeSpatialAccelerations(
		const SpatialModel& model,
		const mathlib::VecX& q,
		const mathlib::VecX& qd,
		const mathlib::VecX& qdd,
		const std::vector<mathlib::SpatialVec>& v,
		const std::vector<mathlib::SpatialMat>& Xup,
		std::vector<mathlib::SpatialVec>& a_out
	) {
		const size_t n = model.joints.size();
		a_out.resize(n);
		
		mathlib::SpatialVec gravity;
		gravity.v << 0, 0, 0, 0, 0, 9.81;

		for (size_t i = 0; i < n; ++i) {
			const SpatialJoint& j = model.joints[i];

			// Joint Acceleration
			mathlib::SpatialVec aJ = j.S * qdd[i];

			// Velocity Product Term
			mathlib::SpatialVec crossTerm;
			crossTerm.v = mathlib::motionCrossMatrix(v[i]) * (j.S * qd[i]).v;

			// Root Link
			if (j.parent < 0) {
				a_out[i].v = Xup[i] * gravity.v + aJ.v + crossTerm.v;
				continue;
			}
			
			a_out[i].v = Xup[i] * a_out[j.parent].v + aJ.v + crossTerm.v;
		}
	}

	// Recursive function to compute inverse dynamics (joint torques) using the articulated body algorithm
	void SpatialDynamics::computeInverseDynamics(
		const SpatialModel& model,
		const std::vector<mathlib::SpatialVec>& v,
		const std::vector<mathlib::SpatialVec>& a,
		const std::vector<mathlib::SpatialMat>& Xup,
		mathlib::VecX& tau_out
	) {
		const size_t n = model.joints.size();
		tau_out.resize(n);

		std::vector<mathlib::SpatialVec> f(n);

		// Forward Force Computation
		for (size_t i = 0; i < n; ++i) {
			const SpatialJoint& j = model.joints[i];
			mathlib::SpatialVec I_v;
			
			mathlib::SpatialVec coriolis;
			coriolis.v = mathlib::forceCrossMatrix(v[i]) * I_v.v;

			f[i].v = j.inertia * a[i].v + coriolis.v;
		}

		// Backward Recursion Computation
		for (int i = (int)n - 1; i >= 0; --i) {
			const SpatialJoint& j = model.joints[i];

			tau_out[i] = j.S.v.transpose() * f[i].v;

			if (j.parent >= 0) {
				f[j.parent].v += Xup[i].transpose() * f[i].v;
			}
		}
	}


	mathlib::VecX SpatialDynamics::inverseDynamics(
		const SpatialModel& model,
		const mathlib::VecX& q,
		const mathlib::VecX& qd,
		const mathlib::VecX& qdd
	) {
		const size_t n = model.joints.size();

		std::vector<mathlib::SpatialVec> v(n);
		std::vector<mathlib::SpatialMat> Xup(n);
		std::vector<mathlib::SpatialVec> a(n);
		mathlib::VecX tau;

		// Compute spatial velocities and transforms
		computeSpatialVelocities(model, q, qd, v, Xup);
		// Compute spatial accelerations
		computeSpatialAccelerations(model, q, qd, qdd, v, Xup, a);
		// Compute inverse dynamics (joint torques)
		computeInverseDynamics(model, v, a, Xup, tau);

		return tau;
	}
}