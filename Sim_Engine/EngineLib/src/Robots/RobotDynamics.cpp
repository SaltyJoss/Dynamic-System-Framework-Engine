#include "pch.h"
// File:   RobotDynamics.cpp
// GitHub: SaltyJoss
#include "Robots/RobotDynamics.h"
#include "Robots/RobotKinematics.h"
#include "Robots/RobotModel.h"
#include "Robots/TrajectoryManager.h"
#include <Core/Utils.h>
#include <kinematics/Forward_Kinematics.h>

#include "EngineLib/LogMacros.h"

namespace robots {
	// Constructor
	RobotDynamics::RobotDynamics(RobotModel& robot, eTorqueMode mode)
		: _robot(robot), _kinematics(std::make_unique<RobotKinematics>(robot)) {
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
	mathlib::MatX RobotDynamics::computeMassMatrix(
		const std::vector<double>& /*q*/,
		const std::vector<mathlib::Pose>& T_world
	) const {
		const size_t n = _robot.joints.size();
		MatX M = MatX::Zero(n, n); // mass matrix to be computed

		// Compute its contribution to the mass matrix for each link - based on its mass, inertia, and Jacobian columns for each joint
		for (size_t k = 0; k < _robot.links.size(); ++k) {
			const RobotLink& link = _robot.links[k];
			const double m = link.inertial.mass;

			// Skip massless links
			if (m <= 0.0) { continue; }

			// Rotation and position of the link in world frame
			const Mat3 R = T_world[k].block<3, 3>(0, 0);  // Rotation from link frame to world frame
			const Vec3 p = T_world[k].block<3, 1>(0, 3);  // Center of mass of the link in world frame
			const Vec3 com = R * link.inertial.com_xyz + p; // Center of mass in world frame

			// Inertia tensor of the link in world frame
			Mat3 I_local = computeLinkInertiaTensor(link); // inertia tensor in link frame
			Mat3 I_world = R * I_local * R.transpose();	   // inertia tensor in world frame

			// Compute Jacobian columns for each joint and accumulate mass matrix contributions
			for (size_t i = 0; i < n; ++i) {
				const RobotJoint& j_i = _robot.joints[i];
				// Skip fixed joints since they don't contribute to the mass matrix
				if (j_i.type == eJointType::FIXED) { continue; }

				// Rotation from joint i frame to world frame
				const Mat3 R_i = T_world[i + 1].block<3, 3>(0, 0); // rotation from joint i frame to world frame
				const Vec3 z_i = R_i * j_i.axis;			   // joint axis in world frame
				const Vec3 p_i = T_world[i + 1].block<3, 1>(0, 3); // joint position in world frame

				// Only include contribution if joint i affects link k
				if (k <= i) continue;

				// Jacobian columns for joint i
				Vec3 J_vi = z_i.cross(com - p_i); // linear velocity Jacobian column for joint i
				Vec3 J_wi = z_i;				  // angular velocity Jacobian column for joint i

				// Computes the contribution to the mass matrix from this link for joints i and j
				for (size_t j = 0; j < n; ++j) {
					const RobotJoint& j_j = _robot.joints[j];
					// Skip fixed joints since they don't contribute to the mass matrix
					if (j_j.type == eJointType::FIXED) { continue; }

					// Rotation from joint j frame to world frame
					const Mat3 R_j = T_world[j + 1].block<3, 3>(0, 0); // rotation from joint j frame to world frame
					const Vec3 z_j = R_j * j_j.axis;			   // joint axis in world frame
					const Vec3 p_j = T_world[j + 1].block<3, 1>(0, 3); // joint position in world frame

					// Only include contribution if joint i affects link k
					if (k <= j) continue;

					// Jacobian columns for joints i and j
					Vec3 J_vj = z_j.cross(com - p_j); // linear velocity Jacobian column for joint j
					Vec3 J_wj = z_j;				  // angular velocity Jacobian column for joint j
					// Mass matrix contribution from this link for joints i and j

					// Mass matrix contribution from this link for joints i and j
					M(i, j) += m * J_vi.dot(J_vj) + J_wi.transpose() * I_world * J_wj;
				}
			}
		}
		return M; // [kg*m^2], mass matrix for the robot at configuration q
	}

	// Computes the gravity torque for a joint based on the current state and robot configuration
	std::vector<double> RobotDynamics::computeGravityTorque(
		const std::vector<double>& q,
		const std::vector<mathlib::Pose>& T_world,
		mathlib::VecX x
	) const {
		const size_t n = _robot.joints.size();
		std::vector<double> tau_G(n, 0.0); // [Nm], gravity torque for each joint
		double g{ _gravity }; // [m/s^2], gravity acceleration magnitude

		// Create state vector with current joint angles
		for (size_t k = 0; k < q.size(); ++k) {
			x[k] = q[k]; // [rad]
		}

		// For each joint, sum the gravity contributions from all links
		for (size_t i = 0; i < n; ++i) {
			const RobotJoint& j = _robot.joints[i];
			if (j.type == eJointType::FIXED) { continue; }

			double tau_g_i = 0.0; // [Nm], gravity torque contribution for joint i

			const Vec3 p_i = T_world[i + 1].block<3, 1>(0, 3);
			const Mat3 R_i = T_world[i + 1].block<3, 3>(0, 0);
			const Vec3 axis_world = (R_i * _robot.joints[i].axis).normalized();

			// For each link, compute the gravitational force and its torque contribution about joint i
			for (size_t k = 0; k < _robot.links.size(); ++k) {
				const RobotLink& link = _robot.links[k];
				const double m = link.inertial.mass;
				if (m <= 0.0) { continue; }

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

	// Computes the control torque for a joint based on the current state, reference, and robot configuration
	mathlib::VecX RobotDynamics::computeAppliedTorques(
		const std::vector<double>& q,
		const std::vector<double>& qd,
		const std::vector<double>& eta,
		const std::vector<mathlib::Pose>& /*T_world*/,
		std::vector<double> I_eff,
		std::vector<double> tau_g
	) const {
		const size_t n = _robot.joints.size();
		VecX tau = VecX::Zero(n); // [Nm], torque for each joint

		switch (_robot.torqueMode) {
		case eTorqueMode::PASSIVE:
			// Compute passive damping and friction torques
			for (size_t i = 0; i < n; ++i) {
				const RobotJoint& j = _robot.joints[i];
				if (j.type == eJointType::FIXED) { continue; }

				const double c = j.dynamics.damping;
				const double mu = j.dynamics.friction;
				const double v_eps = 1e-2; // small velocity threshold for friction model

				tau[i] -= c * qd[i]; // viscous damping
				tau[i] -= mu * std::tanh(qd[i] / v_eps); // Coulomb friction with a small velocity threshold
			}
			break;
		case eTorqueMode::CONTROLLED:
			// State-Consistent effective inertia
			for (size_t i = 0; i < n; ++i) {
				const RobotJoint& j = _robot.joints[i];
				if (j.type == eJointType::FIXED) { continue; }

				// Compute control torque using the computed metrics for this joint
				RobotMetrics m = computeJointMetrics(
					j, I_eff[i],
					q[i], qd[i], eta[i],
					j.q_ref, j.qd_ref, j.qdd_ref,
					0.0, tau_g[i]
				);
				tau[i] = m.tau;
			}
			break;
		}
		return tau; // [Nm], applied torques for each joint
	}

	// Computes control and dynamics metrics for a specific joint based on the current state and reference
	RobotMetrics RobotDynamics::computeJointMetrics(
		const RobotJoint& joint, double I_eff,
		double q, double qd, double eta,
		double q_ref, double qd_ref, double qdd_ref,
		double tau_c, double tau_g
	) const {
		RobotMetrics m{};
		if (_robot.torqueMode == eTorqueMode::NONE) {
			// Current states
			m.theta = q;	   // [rad]
			m.omega = qd;	   // [rad/s]
			// Effective inertia
			m.I_eff = I_eff; // [kg*m^2]
			// Control parameters
			m.tau = 0.0;
			m.tau_fb = 0.0;
			m.tau_coriolis = 0.0;
			m.tau_gravity = 0.0;
			m.tau_damping = 0.0;
			m.tau_friction = 0.0;
			m.tau_sat = 0.0;
			m.tau_barrier = 0.0;

			return m;
		}

		// Current states
		m.theta = q;  // [rad]
		m.omega = qd; // [rad/s]
		// Errors
		m.err = q_ref - q;	   // [rad]
		m.err_d = qd_ref - qd; // [rad/s]

		// Effective inertia
		m.I_eff = I_eff; // [kg*m^2]

		// Energy metrics
		m.KE = 0.5 * I_eff * qd * qd; // [J], kinetic energy of the joint
		double P_grav = tau_g * qd; // [W], power due to gravity torque
		m.PE += -P_grav * dt(); // [J], potential energy proxy based on gravity power (scaled down for interpretability)
		m.E_total = m.KE + m.PE;	  // [J], total mechanical energy of the joint

		// Control parameters
		const double wn = joint.wn_target;	 // [rad/s], natural frequency
		const double z  = joint.zeta_target; // damping ratio
		const double b  = joint.beta_target; // overshoot ratio

		// Compute PID gains
		double k_p = m.I_eff * wn * wn;		 // [Nm/rad],     proportional gain
		double k_i = b * k_p * wn;			 // [Nm/(rad*s)], integral gain
		double k_d = 2.0 * z * m.I_eff * wn; // [Nm/(rad/s)], derivative gain

		// Integral term with anti-windup
		double tau_i = k_i * eta;
		if (joint.limits.maxEffort > 0.0f) {
			const double rho = 0.3; // fraction of max effort allocated to I-term
			const double tau_i_max = rho * joint.limits.maxEffort;
			tau_i = std::clamp(tau_i, -tau_i_max, tau_i_max);
		}

		// Inverse dynamics control law (PD + feedforward)
		double tau_fb = k_p * m.err + tau_i + k_d * m.err_d;

		// Feedforward term based on reference acceleration and passive dynamics compensation
		double tau_ff = m.I_eff * qdd_ref + tau_c;
		if (_robot.torqueMode == eTorqueMode::CONTROLLED) { tau_ff += tau_g; }

		// Passive dynamics
		const double c = joint.dynamics.damping;
		const double mu = joint.dynamics.friction;
		const double v_eps = 1e-2; // small velocity threshold

		// Friction model (viscous + Coulomb/Stribeck)
		double tau_damping{ 0.0 }, tau_friction{ 0.0 };
		tau_damping = c * qd;
		tau_friction = mu * std::tanh(qd / v_eps);

		// Net torque
		m.tau = tau_fb + tau_ff - (tau_damping + tau_friction); // [Nm], net torque applied to the joint after passive dynamics

		// Cache torques in metrics
		m.tau_fb = tau_fb;			   // [Nm], feedback control torque
		m.tau_coriolis = tau_c;		   // [Nm], Coriolis and centrifugal torque
		m.tau_gravity = tau_g;		   // [Nm], gravity torque
		m.tau_damping = tau_damping;   // [Nm], viscous damping torque
		m.tau_friction = tau_friction; // [Nm], coulomb friction torque

		// Cache Work and Power metrics
		m.W_actuator = m.tau * qd;		  // [W], actuator power (positive for power generation, negative for power consumption)
		m.P_damping  = tau_damping * qd;  // [W], power dissipated by damping
		m.P_friction = tau_friction * qd; // [W], power dissipated by friction

		// Hip reaction compensation
		if (_baseIsFree && joint.name.find("hip_pitch") != std::string::npos) {
			const double hipReactionGain = 0.7;
			// If base is free-floating, apply a fraction of the last measured base forward force as a counter-torque to the hip pitch joint to help stabilise the base
			m.tau -= hipReactionGain * _lastBaseForwardForce;
		}

		// Torque saturation and velocity soft limits only in CONTROLLED mode
		if (_robot.torqueMode == eTorqueMode::CONTROLLED) {
			double tau_preSat = m.tau;

			// Effort clamp
			if (joint.limits.maxEffort > 0.0f) {
				const double E_max = joint.limits.maxEffort;
				/*m.tau = std::clamp(m.tau, -E_max, E_max);*/
			}

			m.tau_sat = tau_preSat - m.tau;
			m.sat_flag = (m.tau_sat != 0.0);

			// Velocity soft limit
			const double wMax_hw = std::abs(joint.limits.maxqd);
			const double wMax_traj = std::abs(joint.limits.omegaRefMaxRad_s); // or derived from trajectory manager

			double tau_preBarrier = m.tau;

			// Apply soft velocity barrier
			/*applyOmegaBarrier(m.tau, omega, wMax_hw, m.I_eff);*/

			// Cache barrier torque and overspeed metrics
			m.tau_barrier = tau_preBarrier - m.tau;
			m.wMax_hw = wMax_hw;
			m.wMax_traj = wMax_traj;
			m.traj_overspeed = std::max(0.0, std::abs(qd) - wMax_traj);
			m.traj_overspeed_flag = (m.traj_overspeed > 0.05); // 0.05 rad/s threshold
		}

		return m;
	}

	// Computes the Coriolis and centrifugal torque for a joint based on the current state and robot configuration
	mathlib::VecX RobotDynamics::derivative(
		double /*t*/,
		const mathlib::VecX& x
	) const {
		const size_t n = static_cast<int>(_robot.joints.size());
		mathlib::VecX dx(3 * n);

		// Extract state
		std::vector<double> q(n), qd(n), eta(n);
		for (size_t i = 0; i < n; ++i) {
			q[i] = x[i];
			qd[i] = x[i + n];
			eta[i] = x[i + 2 * n];
		}

		// Compute forward kinematics to get the pose of each link in the world frame
		std::vector<Pose> T_world = _kinematics->computeForwardKinematics_fromState(x);
		// Compute world poses of each joint for inertia calculations
		std::vector<Pose> jointWorldPose = _kinematics->calcJointWorldPoses(T_world, _robot.joints);

		// Compute effective inertia for each joint based on current configuration
		std::vector<double> I_eff(n, 0.0);
		for (size_t i = 0; i < n; ++i) {
			for (size_t k = i + 1; k < _robot.links.size(); ++k) {
				I_eff[i] += computeJointInertiaContribution(
					_robot.joints[i],
					_robot.links[k],
					jointWorldPose[i], // pose of joint i in world frame
					T_world[k]
				);
			}
			I_eff[i] = std::max(I_eff[i], 1e-6);
		}

		// Compute mass matrix M(q)
		MatX M_full = computeMassMatrix(q, T_world); // [kg*m^2], full mass matrix for the robot at configuration q

		// Compute gravity torques for each joint
		std::vector<double> tau_gravity(n, 0.0);
		// Compute applied torques based on control mode and current state
		VecX tau = VecX::Zero(n);

		// Compute gravity torques if in a torque mode that requires it
		if (_robot.torqueMode != eTorqueMode::NONE) {
			// Compute gravity torque
			tau_gravity = computeGravityTorque(q, T_world, x);
			// Compute applied torques based on control mode
			tau = computeAppliedTorques(q, qd, eta, T_world, I_eff, tau_gravity);
		}

		// Build list of active (non-fixed) joints
		std::vector<size_t> active;
		for (size_t i = 0; i < n; ++i) {
			if (_robot.joints[i].type != eJointType::FIXED) {
				active.push_back(i);
			}
		}
		const size_t m = active.size();

		// Build reduced system
		MatX M(m, m);
		VecX tau_r = VecX::Zero(m);
		VecX G_r = VecX::Zero(m);

		// Fill reduced mass matrix and torque vectors for active joints
		for (size_t r = 0; r < m; ++r) {
			size_t i = active[r];

			tau_r[r] = tau[i];		 // applied torque for active joint i
			G_r[r] = tau_gravity[i]; // gravity torque for active joint i

			// Fill the reduced mass matrix row for active joint i
			for (size_t c = 0; c < m; ++c) {
				int j = active[c];
				M(r, c) = M_full(i, j);
			}
		}

		// Debugging info about the mass matrix
		for (int i = 0; i < M.rows(); ++i) {
			double rowNorm = M.row(i).norm();
			LOG_INFO_ONCE("Row %d norm = %.6e", i, rowNorm);
		}

		// Debugging info about the reduced system
		LOG_INFO_ONCE("Reduced system size = %zu", m);
		double rcond = M.fullPivLu().rcond();
		LOG_INFO_ONCE("Reduced M rcond: %.6e", rcond);
		Eigen::JacobiSVD<MatX> svd(M);
		LOG_INFO_ONCE("Reduced min singular value: %.6e", svd.singularValues().minCoeff());

		// Solved for qdd
		Eigen::CompleteOrthogonalDecomposition<MatX> cod(M);
		VecX qdd_r;
		if (_robot.torqueMode == eTorqueMode::NONE) {
			qdd_r = cod.solve(tau_r); // tau_r = 0, so checks for consistency of M
		}
		else {
			qdd_r = cod.solve(tau_r - G_r); // M qdd = tau - G -> qdd = M^-1 (tau - G)
		}

		// Expand qdd back to full size, filling zeros for fixed joints
		Eigen::VectorXd qdd = Eigen::VectorXd::Zero(n);
		for (size_t r = 0; r < m; ++r) {
			qdd[active[r]] = qdd_r[r];
		}

		LOG_INFO_ONCE("Rank(M) = %d", (int)cod.rank());
		LOG_INFO_ONCE("||tau|| = %.6e", tau.norm());
		LOG_INFO_ONCE("||G|| = %.6e", G_r.norm());
		LOG_INFO_ONCE("||qdd|| = %.6e", qdd.norm());

		// Fill in derivatives for all joints
		for (size_t i = 0; i < n; ++i) {
			const RobotJoint& joint = _robot.joints[i];

			// For fixed joints, the derivative of angle and velocity is zero
			if (joint.type == eJointType::FIXED) {
				dx[i] = 0.0;
				dx[i + n] = 0.0;
				dx[i + 2 * n] = 0.0;
				continue;
			}

			// Compute error for integral term
			double err_i = _robot.joints[i].q_ref - q[i];

			// For revolute and prismatic joints, fill in the derivatives
			dx[i] = qd[i];
			dx[i + n] = qdd[i];
			dx[i + 2 * n] = err_i; // integrate error
		}

		return dx;
	}

	// Set the robot model reference for dynamics calculations
	void RobotDynamics::setRobot(RobotModel& robot) {
		_robot = robot;
		_kinematics->setRobot(robot);
	}
}