#pragma once

#include <core/MathLib.h>
#include "kinematics/DH_Params.h"
#include "kinematics/Forward_Kinematics.h"

using namespace mathlib;

namespace kinematics {
	class Jacobian {
	public:
		/// <summary>
		/// Compute the geometric Jacobian matrix for a robotic manipulator
		/// </summary>
		/// <param name="dh_p">Denavit-Hartenberg parameters for each joint</param>
		/// <param name="q">Joint variables (angles for revolute joints, displacements for prismatic joints)</param>
		/// <returns>Geometric Jacobian matrix as a 6xN matrix, where N is the number of joints</returns>
		MatX jacobianGeometric(const std::vector<DH_Params<double>>& dh_p, const VecX& q) {
			size_t n = dh_p.size();	// Number of joints
			MatX J = MatX::Zero(6, n);	// Initialize Jacobian matrix

			auto T_list = Forward_Kinematics().linkTransforms_DH(dh_p, q);	// Get link transformations

			Pose T_end = T_list.back();			// End-effector transformation
			Vec3 p_end = T_end.block<3, 1>(0, 3);	// End-effector position

			Vec3 z_base = Vec3::UnitZ();	// Initial z-axis
			Vec3 p_base = Vec3::Zero();		// Initial position

			for (size_t i = 0; i < n; ++i) {
				Vec3 z_i, p_i;

				if (i == 0) {
					z_i = z_base;
					p_i = p_base;
				}
				else {
					z_i = T_list[i - 1].block<3, 1>(0, 2);  // z_{i-1}
					p_i = T_list[i - 1].block<3, 1>(0, 3);  // p_{i-1}
				}
				if (dh_p[i].type == JointType_DH::Revolute) {
					J.block<3, 1>(0, i) = z_i.cross(p_end - p_i);	// Linear velocity part
					J.block<3, 1>(3, i) = z_i;						// Angular velocity part
				}
				else if (dh_p[i].type == JointType_DH::Prismatic) {
					J.block<3, 1>(0, i) = z_i;						// Linear velocity part
					J.block<3, 1>(3, i) = Vec3::Zero();			// Angular velocity part
				}
			}
			return J;	// Return Geometric Jacobian matrix
		}

		/// <summary>
		/// Compute the numerical Jacobian matrix using finite differences
		/// </summary>
		/// <param name="f">Function that maps joint variables to end-effector pose</param>
		/// <param name="q">Joint variables at which to compute the Jacobian</param>
		/// <param name="h">Small perturbation value for finite differences</param>
		/// <returns>Numerical Jacobian matrix as a 6xN matrix, where N is the number of joints</returns>
		MatX numericalJacobian(const std::function<Pose(const VecX&)>& f, const VecX& q, double h = 1e-6) {
			size_t n = q.size();
			MatX J = MatX::Zero(6, n);

			Pose T0 = f(q);	// Compute pose at original joint variables
			Vec3 p0 = T0.block<3, 1>(0, 3);	// Position part
			Eigen::Matrix3d R0 = T0.block<3, 3>(0, 0);	// Rotation part

			for (size_t i = 0; i < n; ++i) {
				VecX q_perturbed = q;

				q_perturbed(i) += h;											// Perturb joint variable
				Pose T_perturbed = f(q_perturbed);								// Compute perturbed pose
				Vec3 p_perturbed = T_perturbed.block<3, 1>(0, 3);				// Perturbed position
				Eigen::Matrix3d R_perturbed = T_perturbed.block<3, 3>(0, 0);	// Perturbed rotation

				J.block<3, 1>(0, i) = (p_perturbed - p0) / h;	// Numerical derivative for position
				Eigen::Matrix3d R_diff = R0.transpose() * R_perturbed;	// Rotation difference
				Eigen::AngleAxisd angle_axis(R_diff);	// Convert to angle-axis representation

				J.block<3, 1>(3, i) = (angle_axis.axis() * angle_axis.angle()) / h;	// Numerical derivative for orientation
			}
			return J;	// Return numerical Jacobian matrix
		}
	};
}