#pragma once

#include "MathLibAPI.h"
#include "kinematics/DH_Params.h"
#include "core/Types.h"

namespace kinematics {
	class MATHAPI_LIB Jacobian {
	public:
		/// <summary>
		/// Compute the geometric Jacobian matrix for a robotic manipulator
		/// </summary>
		/// <param name="dh_p">Denavit-Hartenberg parameters for each joint</param>
		/// <param name="q">Joint variables (angles for revolute joints, displacements for prismatic joints)</param>
		/// <returns>Geometric Jacobian matrix as a 6xN matrix, where N is the number of joints</returns>
		MatX jacobianGeometric(const std::vector<DH_Params>& dh_p, const VecX& q);

		/// <summary>
		/// Compute the numerical Jacobian matrix using finite differences
		/// </summary>
		/// <param name="f">Function that maps joint variables to end-effector pose</param>
		/// <param name="q">Joint variables at which to compute the Jacobian</param>
		/// <param name="h">Small perturbation value for finite differences</param>
		/// <returns>Numerical Jacobian matrix as a 6xN matrix, where N is the number of joints</returns>
		MatX numericalJacobian(const std::function<Pose(const VecX&)>& f, const VecX& q, double h = 1e-6);
	};
}