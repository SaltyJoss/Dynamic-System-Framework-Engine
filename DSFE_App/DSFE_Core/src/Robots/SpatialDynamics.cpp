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
		}
	}
}