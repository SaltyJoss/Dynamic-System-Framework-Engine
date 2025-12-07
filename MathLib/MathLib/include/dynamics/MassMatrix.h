#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"
#include "kinematics/DH_Param.h"
#include "dynamics/RigidBody.h"

namespace dynamics {
	class MATHLIB_API MassMatrix {
	public:
		/// <summary>
		/// Computes the mass matrix of a robotic manipulator using the provided DH parameters, link inertias, and joint angles.
		/// </summary>
		/// <param name="dh_p">A vector of DH_Param structures representing the Denavit-Hartenberg parameters of each link.</param>
		/// <param name="inertias">A vector of LinkInertia structures representing the inertia properties of each link.</param>
		/// <param name="q">A vector of joint angles.</param>
		/// <returns>A MatX representing the mass matrix of the manipulator.</returns>
		MatX massMatrix(const std::vector<DH_Param>& dh_p, const std::vector<LinkInertia>& inertias, const VecX& q);
	};
}