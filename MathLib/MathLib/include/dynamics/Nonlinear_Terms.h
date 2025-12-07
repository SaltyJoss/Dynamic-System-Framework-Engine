#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"
#include "kinematics/DH_Param.h"
#include "dynamics/RigidBody.h"

namespace dynamics {
	class MATHLIB_API Nonlinear_Terms {
	public:
		/// <summary>
		/// Computes the nonlinear terms (Coriolis and centrifugal forces) of a robotic manipulator.
		/// </summary>
		/// <param name="dh_p">A vector of DH_Param structures representing the Denavit-Hartenberg parameters of each link.</param>
		/// <param name="inertias">A vector of LinkInertia structures representing the inertia properties of each link.</param>
		/// <param name="q">A vector of joint angles.</param>
		/// <param name="q_dot">A vector of joint velocities.</param>
		/// <returns>A VecX representing the nonlinear terms of the manipulator.</returns>
		VecX coriolisCentrifugal(const std::vector<DH_Param>& dh_p, const std::vector<LinkInertia>& inertias, const VecX& q, const VecX& q_dot);

		/// <summary>
		/// Computes the gravity torque vector of a robotic manipulator.
		/// </summary>
		/// <param name="dh_p">A vector of DH_Param structures representing the Denavit-Hartenberg parameters of each link.</param>
		/// <param name="inertias">A vector of LinkInertia structures representing the inertia properties of each link.</param>
		/// <param name="q">A vector of joint angles.</param>
		/// <param name="gravity">A Vec3 representing the gravity vector.</param>
		/// <returns>A VecX representing the gravity torque vector of the manipulator.</returns>
		VecX gravityTorque(const std::vector<DH_Param>& dh_p, const std::vector<LinkInertia>& inertias, const VecX& q, const Vec3& gravity = Vec3{0,0,0});
	};
}