#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"
#include "kinematics/DH_Param.h"

namespace kinematics {
	class MATHLIB_API FK {
	public:
		/// <summary>
		/// Forward Kinematics using Denavit-Hartenberg parameters
		/// </summary
		/// <param name="dh_p">Denavit-Hartenberg parameters for each joint</param>
		/// <param name="q">Joint variables (angles for revolute joints, displacements for prismatic joints)</param>
		/// <returns>End-effector pose as a 4x4 transformation matrix</returns>
		Pose FK(const std::vector<DH_Params>& dh_p, const VecX& q);

		/// <summary>
		/// Compute the transformation matrices for each link in the kinematic chain
		/// </summary>
		/// <param name="dh_p">Denavit-Hartenberg parameters for each joint</param>
		/// <param name="q">Joint variables (angles for revolute joints, displacements for prismatic joints)</param>
		/// <returns>Vector of transformation matrices for each link</returns>
		std::vector<Pose> linkTransformas(const std::vector<DH_Params>& dh_p, const VecX& q);
}