#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"
#include "kinematics/DH_Param.h"
#include "dynamics/RigidBody.h"

namespace dynamics {
	class MATHLIB_API StateSpace {
	public:
		/// <summary>
		/// Computes the dynamic right-hand side of the robotic manipulator's equations of motion.
		/// </summary>
		/// <param name="dh_p">A vector of DH_Param structures representing the Denavit-Hartenberg parameters of each link.</param>
		/// <param name="inertias">A vector of LinkInertia structures representing the inertia properties of each link.</param>
		/// <param name="q">A vector of joint angles.</param>
		/// <param name="q_dot">A vector of joint velocities.</param>
		/// <param name="tau">A vector of joint torques.</param>
		/// <param name="gravity">A Vec3 representing the gravity vector.</param>
		/// <returns>A VecX representing the dynamic right-hand side of the equations of motion.</returns>
		VecX dynamicRHS(const std::vector<DH_Param>& dh_p, const std::vector<LinkInertia>& inertias, const VecX& q, const VecX& q_dot, const VecX& tau, const Vec3& gravity = Vec3{ 0,0,0 });
	};
}