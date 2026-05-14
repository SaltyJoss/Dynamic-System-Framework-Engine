#include "pch.h"
// File:   RobotDynamics.cpp
// GitHub: SaltyJoss
#include "Robots/RobotDynamics.h"
#include "Robots/RobotKinematics.h"
#include "Robots/RobotSimSnapshot.h"

#include "Robots/SpatialModel.h"
#include "Robots/SpatialDynamics.h"

#include "Robots/TrajectoryManager.h"

#include <cmath>

#include <Core/Utils.h>
#include <kinematics/Forward_Kinematics.h>

#include "EngineLib/LogMacros.h"

namespace robots {
	// Constructor
	RobotDynamics::RobotDynamics() 
		: _kinematics(std::make_unique<RobotKinematics>()) {
	}

	// Computes the inertia tensor of a robot link
	mathlib::Mat3 RobotDynamics::computeLinkInertiaTensor(const RobotLink& link) const {
		const robots::Inertia& I = link.inertial.inertia;

		// Construct the inertia tensor matrix
		Mat3 M = Mat3::Zero();
		M << I.ixx, I.ixy, I.ixz,
			 I.ixy, I.iyy, I.iyz,
			 I.ixz, I.iyz, I.izz;

		return M; // [kg*m^2], (3x3) inertia tensor in link frame
	}

	// Computes the contribution of a single joint and its child link to the effective inertia I_eff of the joint
	double RobotDynamics::computeJointInertiaContribution(
		const RobotJoint& joint,
		const RobotLink& link,
		const mathlib::Pose& jointWorldPose,   // pose of joint frame in world
		const mathlib::Pose& linkWorldPose     // pose of the link in world
	) const {
		const double mass = link.inertial.mass;

		// Rotation from link frame to world frame
		Mat3 R_joint = jointWorldPose.block<3, 3>(0, 0);
		// Joint axis in world frame
		Vec3 axis_world = (R_joint * joint.axis).normalized();

		// Position of joint in world frame
		Vec3 joint_pos_world = jointWorldPose.block<3, 1>(0, 3);

		// Rotation from link frame to world frame
		Mat3 R_link = linkWorldPose.block<3, 3>(0, 0);
		Vec3 link_pos_world = linkWorldPose.block<3, 1>(0, 3);
		Vec3 com_world = R_link * link.inertial.com_xyz + link_pos_world;

		// Translational contribution (parallel axis theorem)
		Vec3 r = com_world - joint_pos_world;

		// Translational contribution to inertia about the joint axis
		double I_trans = mass * (axis_world.cross(r)).squaredNorm();

		// Rotational contribution
		Mat3 I_local = computeLinkInertiaTensor(link);
		Mat3 I_world = R_link * I_local * R_link.transpose();

		// Rotational contribution to inertia about the joint axis
		double I_rot = axis_world.transpose() * I_world * axis_world;
		double I_eff_i = I_trans + I_rot;

		return std::max(I_eff_i, 1e-6); // [kg*m^2], I_eff contribution of this joint + floor to avoid singularities
	}

	// Computes the full mass matrix M(q) based on the current state and robot configuration
	void RobotDynamics::computeMassMatrix(
		const RobotConstModel& robot,
		const std::vector<mathlib::Pose>& T_world,
		const std::vector<mathlib::Pose>& jointWorldPoses,
		mathlib::MatX& M_out
	) const {
		const size_t n = robot.joints.size();
		M_out.resize(n, n);
		M_out.setZero();

		// Compute its contribution to the mass matrix for each link
		for (size_t k = 0; k < robot.links.size(); ++k) {
			const RobotLink& link = robot.links[k];
			const double m = link.inertial.mass;

			if (m <= 0.0) { continue; }

			const Mat3 R = T_world[k].block<3, 3>(0, 0);  // Rotation from link frame to world frame
			const Vec3 p = T_world[k].block<3, 1>(0, 3);  // Center of mass of the link in world frame
			const Vec3 com = R * link.inertial.com_xyz + p; // Center of mass in world frame

			Mat3 I_local = computeLinkInertiaTensor(link); // inertia tensor in link frame
			Mat3 I_world = R * I_local * R.transpose();	   // inertia tensor in world frame

			// Compute Jacobian columns for each joint and accumulate mass matrix contributions
			for (size_t i = 0; i < n; ++i) {
				const RobotJoint& j_i = robot.joints[i];
				if (j_i.type == eJointType::FIXED) { continue; }

				if (!robot.jointAffectsLink(i, k)) { continue; } // skip if joint i does not affect link k

				const Pose& T_joint_i = jointWorldPoses[i]; // pose of joint i in world frame

				// Rotation from joint i frame to world frame
				const Mat3 R_i = T_joint_i.block<3, 3>(0, 0); // rotation from joint i frame to world frame
				const Vec3 p_i = T_joint_i.block<3, 1>(0, 3); // joint position in world frame
				const Vec3 z_i = (R_i * j_i.axis).normalized(); // joint axis in world frame

				Vec3 J_vi = z_i.cross(com - p_i); // linear velocity Jacobian column for joint i
				Vec3 J_wi = z_i;				  // angular velocity Jacobian column for joint i

				// Computes the contribution to the mass matrix from this link for joints i and j
				for (size_t j = 0; j < n; ++j) {
					const RobotJoint& j_j = robot.joints[j];
					if (j_j.type == eJointType::FIXED) { continue; }

					if (!robot.jointAffectsLink(j, k)) { continue; } // skip if joint j does not affect link k

					const Pose& T_joint_j = jointWorldPoses[j]; // pose of joint i in world frame

					const Mat3 R_j = T_joint_j.block<3, 3>(0, 0);
					const Vec3 p_j = T_joint_j.block<3, 1>(0, 3);
					const Vec3 z_j = (R_j * j_j.axis).normalized();

					Vec3 J_vj = z_j.cross(com - p_j); // linear velocity Jacobian column for joint j
					Vec3 J_wj = z_j;				  // angular velocity Jacobian column for joint j

					M_out(i, j) += m * J_vi.dot(J_vj) + J_wi.transpose() * I_world * J_wj; // [kg*m^2]
				}
			}
		}
	}

	// Computes the Coriolis and centrifugal bias vector h(q, qd) based on the current state and robot configuration
	mathlib::VecX RobotDynamics::computeCoriolisVector(
		const RobotConstModel& robot,
		const mathlib::VecX& q,
		const mathlib::VecX& qd,
		const std::vector<mathlib::Pose>& T_world,
		const mathlib::MatX& M
	) const {
		const size_t n = robot.joints.size();
		const double eps = 1e-6; // small value to prevent division by zero
		mathlib::VecX q_eps = q;

		std::vector<MatX> dM_dq(n, MatX::Zero(n, n)); // partial derivatives of M with respect to each joint angle
		VecX x_eps(2 * n); // state vector for kinematics

		std::vector<Pose> T_world_eps; // forward kinematics for perturbed configurations
		T_world_eps.resize(robot.links.size());

		std::vector<Pose> jointWorldPoses_eps;
		jointWorldPoses_eps.resize(n);

		MatX M_plus(n, n);

		// Finite difference approximation of dM/dq for each joint
		for (size_t k = 0; k < n; ++k) {
			q_eps = q; // reset to original configuration for each joint perturbation
			q_eps[k] += eps; // perturb joint k by a small amount

			// Construct the state vector for the perturbed configuration
			for (size_t i = 0; i < n; ++i) {
				x_eps[i] = q_eps[i];
				x_eps[n + i] = qd[i];
			}

			// Compute forward kinematics for the perturbed state
			_kinematics->computeForwardKinematics_fromState(robot, x_eps, T_world_eps);

			jointWorldPoses_eps = _kinematics->calcJointWorldPoses(T_world_eps, robot);

			computeMassMatrix(robot, T_world_eps, jointWorldPoses_eps, M_plus); // mass matrix for the perturbed configuration
			
			dM_dq[k] = (M_plus - M) / eps; // [kg*m^2/rad], partial derivative of mass matrix with
		}

		// Compute Coriolis and centrifugal bias vector h using Christoffel symbols of the first kind
		VecX h = VecX::Zero(n);
		for (size_t i = 0; i < n; ++i) {
			for (size_t j = 0; j < n; ++j) {
				for (size_t k = 0; k < n; ++k) {
					double C_ijk = 0.5 * (dM_dq[k](i, j) + dM_dq[j](i, k) - dM_dq[i](j, k)) * qd[k]; // Christoffel symbol of the first kind for indices (i, j, k)
					h(i) += C_ijk * qd[j] * qd[k]; // contribution to Coriolis and centrifugal bias for joint i from joints j and k
				}
			}
		}
		return h; // [Nm], Coriolis and centrifugal bias vector for the robot at configuration q and velocity qd
	}

	// Computes the gravity torque for a joint based on the current state and robot configuration
	mathlib::VecX RobotDynamics::computeGravityTorque(
		const RobotConstModel& robot,
		const std::vector<mathlib::Pose>& T_world,
		const std::vector<mathlib::Pose>& jointWorldPoses
	) const {
		const size_t n = robot.joints.size();
		mathlib::VecX tau_G = VecX::Zero(n);
		double g{ _gravity }; // [m/s^2], gravity acceleration magnitude

		// For each joint, sum the gravity contributions from all links
		for (size_t i = 0; i < n; ++i) {
			const RobotJoint& j = robot.joints[i];
			if (j.type == eJointType::FIXED) { continue; }

			double tau_g_i = 0.0; // [Nm], gravity torque contribution for joint i

			const Pose& T_joint = jointWorldPoses[i]; // pose of joint i in world frame

			const Mat3 R_i = T_joint.block<3, 3>(0, 0);
			const Vec3 p_i = T_joint.block<3, 1>(0, 3);
			const Vec3 axis_world = (R_i * robot.joints[i].axis).normalized();

			// For each link, compute the gravitational force and its torque contribution about joint i
			for (size_t k = 0; k < robot.links.size(); ++k) {
				const RobotLink& link = robot.links[k];
				const double m = link.inertial.mass;
				if (m <= 0.0) { continue; }

				if (!robot.jointAffectsLink(i, k)) { continue; }

				// Link's center of mass in world frame
				const Mat3 R_k = T_world[k].block<3, 3>(0, 0);
				const Vec3 com_world = R_k * link.inertial.com_xyz + T_world[k].block<3, 1>(0, 3);

				// Gravitational force on the link
				Vec3 g_world = Vec3(0.0, 0.0, -g); // [m/s^2], gravity vector in world frame
				const Vec3 F_g = m * g_world; // [N], gravitational force on the link in world frame

				// Torque contribution from this link's weight about joint i
				const Vec3 r = com_world - p_i; // [m]

				// Torque = r × F_g projected onto joint axis
				tau_g_i += axis_world.dot(r.cross(F_g));
			}
			tau_G[i] = tau_g_i;
		}
		return tau_G; // [Nm], gravity torques for each joint
	}

	void RobotDynamics::analyticalJacobian(
		const RobotConstModel& robot,
		const mathlib::VecX& x,
		mathlib::MatX& J_out,
		DenseDynamicsScratch& scratch
	) {
		const size_t n = robot.joints.size();

		Eigen::Map<const VecX> q_local(x.data(), n);
		Eigen::Map<const VecX> qd_local(x.data() + n, n);

		_kinematics->computeForwardKinematics_fromState(robot, x, scratch.T_world);
		scratch.jointWorldPoses = _kinematics->calcJointWorldPoses(scratch.T_world, robot);
		computeMassMatrix(robot, scratch.T_world, scratch.jointWorldPoses, scratch.M);

		J_out.setZero(2 * n, 2 * n); // [rad/rad] position part, [rad/s / rad/s] velocity part
		J_out.block(0, n, n, n).setIdentity();

		MatX dTau_dq = MatX::Zero(n, n);
		MatX dTau_dv = MatX::Zero(n, n);

		for (size_t i = 0; i < n; ++i) {
			const RobotJoint& joint = robot.joints[i];
			if (joint.type == eJointType::FIXED) { continue; }

			const double wn = joint.wn_target;
			const double z = joint.zeta_target;
			const double I_eff = std::max(scratch.M(i, i), 1e-6);

			const double k_p = I_eff * wn * wn;
			const double k_d = 2.0 * z * I_eff * wn;

			dTau_dq(i, i) = -k_p;

			double qd_i = qd_local[i];
			double tanh_term = std::tanh(qd_i / 1e-2);
			double stiff_friction_slope = -0.05 * (1.0 - tanh_term * tanh_term) / 1e-2;
			dTau_dv(i, i) = -k_d - 0.2 + stiff_friction_slope;
		}

		auto solver = scratch.M.ldlt();
		MatX da_dq = solver.solve(dTau_dq);
		MatX da_dv = solver.solve(dTau_dv);

		J_out.block(n, 0, n, n) = da_dq;
		J_out.block(n, n, n, n) = da_dv;
	}

	// Computes the derivative of the state vector (q, qd) based on the current state and robot configurations
	mathlib::VecX RobotDynamics::derivative_dense(
		double /*t*/,
		const mathlib::VecX& x,
		const RobotSimSnapshot& snap,
		DynamicsScratch& scratch,
		DynamicsResult& out
	) {
		const size_t n = snap.model->joints.size();
		mathlib::VecX dx(2 * n);

		// Map the input state vector to joint angles and velocities
		Eigen::Map<const VecX> q(x.data(), n);
		Eigen::Map<const VecX> qd(x.data() + n, n);

		_kinematics->computeForwardKinematics_fromState(*snap.model, x, scratch.dense.T_world);

		scratch.dense.jointWorldPoses = _kinematics->calcJointWorldPoses(scratch.dense.T_world, *snap.model);

		computeMassMatrix(*snap.model, scratch.dense.T_world, scratch.dense.jointWorldPoses, scratch.dense.M); // [kg*m^2], full mass matrix for the robot at configuration q
		scratch.dense.h = computeCoriolisVector(*snap.model, q, qd, scratch.dense.T_world, scratch.dense.M); // [Nm], full Coriolis and centrifugal torque vector

		scratch.g.setZero();
		if (snap.torqueMode != eTorqueMode::NONE) {
			scratch.g = computeGravityTorque(*snap.model, scratch.dense.T_world, scratch.dense.jointWorldPoses);
		}

		scratch.dense.tau.setZero();
		for (size_t i = 0; i < n; ++i) {
			const RobotJoint& joint = snap.model->joints[i];

			// Fixed joints
			if (joint.type == eJointType::FIXED) {
				out.metrics.q[i] = q[i];
				out.metrics.qd[i] = qd[i];
				out.metrics.qdd[i] = 0.0;
				continue;
			}

			const double wn = joint.wn_target;	 // [rad/s], natural frequency
			const double z = joint.zeta_target;  // damping ratio

			const double err = snap.q_ref[i] - q[i];	   // [rad], position error
			const double err_d = snap.qd_ref[i] - qd[i]; // [rad/s], velocity error

			const double I_eff = std::max(scratch.dense.M(i, i), 1e-6); // [kg*m^2], effective inertia for joint i with floor to prevent singularities
			const double k_p = I_eff * wn * wn;		 // [Nm/rad], proportional gain
			const double k_d = 2.0 * z * I_eff * wn; // [Nm/(rad/s)], derivative gain

			double tau_i = k_p * err + k_d * err_d + I_eff * snap.qdd_ref[i]; // [Nm], control torque for joint i
			tau_i += scratch.g[i]; // Gravity compensation
			tau_i += scratch.dense.h[i]; // add Coriolis and centrifugal bias
			tau_i -= /*joint.dynamics.damping*/ 0.2 * qd[i]; // subtract viscous damping
			tau_i -= /*joint.dynamics.friction*/ 0.05 * std::tanh(qd[i] / 1e-2); // subtract Coulomb friction

			scratch.dense.tau[i] = tau_i;

			// metrics
			out.metrics.q[i] = q[i];
			out.metrics.qd[i] = qd[i];

			out.metrics.err[i] = err;
			out.metrics.errd[i] = err_d;

			out.metrics.I_eff[i] = I_eff;
			out.metrics.tau[i] = tau_i;

			//_metrics.tau_sat[i] = tau_sat;
			//_metrics.sat_flag[i] = saturated;
		}

		// Solve Forward Dynamics: M(q) qdd = tau - h(q, qd) - g(q)
		scratch.dense.rhs.noalias() = scratch.dense.tau - scratch.dense.h - scratch.g; // [Nm], right-hand side of the dynamics equation M*qdd = tau - h - g

		// Solve for Accelerations
		out.qdd = scratch.dense.M.ldlt().solve(scratch.dense.rhs); // [rad/s^2], joint accelerations computed from dynamics
		out.metrics.qdd = out.qdd;

		// Fill derivatives
		dx.head(n) = qd;
		dx.tail(n) = out.qdd;

		return dx;
	}

	mathlib::VecX RobotDynamics::derivative_spatial(
		const robots::SpatialModel& model,
		double /*t*/,
		const mathlib::VecX& x,
		const RobotSimSnapshot& snap,
		DynamicsScratch& scratch,
		DynamicsResult& out
	) {
		const size_t n = snap.model->joints.size();
		VecX dx(2 * n);

		Eigen::Map<const VecX> q(x.data(), n);
		Eigen::Map<const VecX> qd(x.data() + n, n);

		scratch.dense.tau.setZero();

		for (size_t i = 0; i < n; ++i) {
			const SpatialJoint& joint = model.joints[i];
			if (joint.type == eJointType::FIXED) { continue; }

			const double wn = 5.0;
			const double z = 0.7;

			const double err = snap.q_ref[i] - q[i];
			const double err_d = snap.qd_ref[i] - qd[i];

			const double I_eff = std::max(scratch.dense.M(i, i), 1e-6);
			const double k_p = I_eff * wn * wn;
			const double k_d = 2.0 * z * I_eff * wn;

			double tau_i = k_p * err + k_d * err_d + I_eff * snap.qdd_ref[i];
			tau_i += scratch.g[i];
			tau_i += scratch.dense.h[i];
			tau_i -= 0.2 * qd[i];
			tau_i -= 0.05 * std::tanh(qd[i] / 1e-2);

			scratch.dense.tau[i] = tau_i;

			out.metrics.q[i] = q[i];
			out.metrics.qd[i] = qd[i];

			out.metrics.err[i] = err;
			out.metrics.errd[i] = err_d;

			out.metrics.I_eff[i] = I_eff;
			out.metrics.tau[i] = tau_i;
		}

		out.qdd = SpatialDynamics::ABA(model, q, qd, scratch.dense.tau, scratch);
		out.metrics.qdd = out.qdd;
		dx.head(n) = qd;
		dx.tail(n) = out.qdd;
		
		return dx;
	}

	mathlib::VecX RobotDynamics::derivative_with_gains(
		double t, const mathlib::VecX& x, const RobotSimSnapshot& snap,
		const mathlib::VecX& kp, const mathlib::VecX& kd,
		DynamicsScratch& scratch,
		DynamicsResult& out
	) {
		const size_t n = snap.model->joints.size();
		mathlib::VecX dx(2 * n);

		Eigen::Map<const VecX> q(x.data(), n);
		Eigen::Map<const VecX> qd(x.data() + n, n);

		_kinematics->computeForwardKinematics_fromState(*snap.model, x, scratch.dense.T_world);
		scratch.dense.jointWorldPoses = _kinematics->calcJointWorldPoses(scratch.dense.T_world, *snap.model);

		computeMassMatrix(*snap.model, scratch.dense.T_world, scratch.dense.jointWorldPoses, scratch.dense.M);
		scratch.dense.h = computeCoriolisVector(*snap.model, q, qd, scratch.dense.T_world, scratch.dense.M);

		scratch.g.setZero();
		if (snap.torqueMode != eTorqueMode::NONE) {
			scratch.g = computeGravityTorque(*snap.model, scratch.dense.T_world, scratch.dense.jointWorldPoses);
		}

		scratch.dense.tau.setZero();
		for (size_t i = 0; i < n; ++i) {
			if (snap.model->joints[i].type == eJointType::FIXED) continue;

			// FIXED: Use the constant, frozen step-start values passed down
			double tau_i = kp[i] * (snap.q_ref[i] - q[i]) + kd[i] * (snap.qd_ref[i] - qd[i]) + std::max(scratch.dense.M(i, i), 1e-6) * snap.qdd_ref[i];
			tau_i += scratch.g[i] + scratch.dense.h[i];
			tau_i -= 0.2 * qd[i];
			tau_i -= 0.05 * std::tanh(qd[i] / 1e-2);

			scratch.dense.tau[i] = tau_i;
		}

		scratch.dense.rhs.noalias() = scratch.dense.tau - scratch.dense.h - scratch.g;
		out.qdd = scratch.dense.M.ldlt().solve(scratch.dense.rhs);
		out.metrics.qdd = out.qdd;

		dx.head(n) = qd;
		dx.tail(n) = out.qdd;
		return dx;
	}

	void RobotDynamics::jacobian_with_gains(
		const mathlib::VecX& x, const RobotSimSnapshot& snap,
		const mathlib::VecX& kp, const mathlib::VecX& kd, mathlib::MatX& F_out,
		DenseDynamicsScratch& scratch
	) {
		const size_t n = snap.model->joints.size();

		F_out.setZero(2 * n, 2 * n);
		F_out.block(0, n, n, n).setIdentity();

		Eigen::Map<const VecX> q(x.data(), n);
		Eigen::Map<const VecX> qd(x.data() + n, n);

		// Compute a local mass matrix for this exact stage evaluation frame
		_kinematics->computeForwardKinematics_fromState(*snap.model, x, scratch.T_world);
		scratch.jointWorldPoses = _kinematics->calcJointWorldPoses(scratch.T_world, *snap.model);
		computeMassMatrix(*snap.model, scratch.T_world, scratch.jointWorldPoses, scratch.M);

		mathlib::MatX dTau_dq = mathlib::MatX::Zero(n, n);
		mathlib::MatX dTau_dv = mathlib::MatX::Zero(n, n);

		for (size_t i = 0; i < n; ++i) {
			if (snap.model->joints[i].type == eJointType::FIXED) continue;

			dTau_dq(i, i) = -kp[i];

			double tanh_term = std::tanh(qd[i] / 1e-2);
			double stiff_friction = -0.05 * (1.0 - tanh_term * tanh_term) / 1e-2;
			dTau_dv(i, i) = -kd[i] - 0.2 + stiff_friction;
		}

		auto solver = scratch.M.ldlt();
		F_out.block(n, 0, n, n) = solver.solve(dTau_dq);
		F_out.block(n, n, n, n) = solver.solve(dTau_dv);
	}
}