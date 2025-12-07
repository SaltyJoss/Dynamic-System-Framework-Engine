#include "pch.h"
#include "kinematics/Jacobian.h"

namespace kinematics {
	/// <inheritdoc/>
	MatX Jacobian::jacobianGeometric(const std::vector<DH_Params>& dh_p, const VecX& q) {
		size_t n = dh_p.size();	// Number of joints
		MatX J = MatX::Zero(6, n);	// Initialize Jacobian matrix

		std::vector<Pose> transforms = FK().linkTransformas(dh_p, q);	// Get link transformations

		Pose T_end = transforms.back();	// End-effector transformation
		Vec3 p_end = T_end.block<3, 1>(0, 3);	// End-effector position
		Vec3 z_prev = Vec3::UnitZ();	// Initial z-axis
		Vec3 p_prev = Vec3::Zero();	// Initial position

		for (size_t i = 0; i < n; ++i) {
			Vec3 z_i = transforms[i].block<3, 1>(0, 2);	// z-axis of current joint
			Vec3 p_i = transforms[i].block<3, 1>(0, 3);	// position of current joint
			if (dh_p[i].isRevolute) {
				J.block<3, 1>(0, i) = z_i.cross(p_end - p_i);	// Linear velocity part
				J.block<3, 1>(3, i) = z_i;						// Angular velocity part
			}
			else if (dh_p[i].isPrismatic) {
				J.block<3, 1>(0, i) = z_i;						// Linear velocity part
				J.block<3, 1>(3, i) = Vec3::Zero();			// Angular velocity part
			}
		}
		return J;	// Return Geometric Jacobian matrix
	}

	/// <inheritdoc/>
	MatX Jacobian::numericalJacobian(const std::function<Pose(const VecX&)>& f, const VecX& q, double h) {
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
}