// DSFE_Core SpatialDynamics.cpp
#include "pch.h"

#include "Robots/SpatialDynamics.h"

namespace robots {
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
}