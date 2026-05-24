// DSFE_Core RobotDynamics.inl
#pragma once

namespace robots {
	// Computes the inertia tensor of a robot link
	template<typename Scalar>
	mathlib::Mat3_T<Scalar> RobotDynamics::computeLinkInertiaTensor(const RobotLink& link) const {
		const robots::Inertia& I = link.inertial.inertia;

		// Construct the inertia tensor matrix
		mathlib::Mat3_T<Scalar> M = mathlib::Mat3_T<Scalar>::Zero();
		M <<
			I.ixx, I.ixy, I.ixz,
			I.ixy, I.iyy, I.iyz,
			I.ixz, I.iyz, I.izz;

		return M; // [kg*m^2], (3x3) inertia tensor in link frame
	}

	// Computes the contribution of a single joint and its child link to the effective inertia I_eff of the joint
	template<typename Scalar>
	Scalar RobotDynamics::computeJointInertiaContribution(
		const RobotJoint& joint,
		const RobotLink& link,
		const mathlib::Pose_T<Scalar>& jointWorldPose,
		const mathlib::Pose_T<Scalar>& linkWorldPose
	) const {
		const Scalar m = (Scalar)link.inertial.mass;

		// Rotation from link frame to world frame
		mathlib::Mat3_T<Scalar> R_joint = jointWorldPose.template block<3, 3>(0, 0);
		// Joint axis in world frame
		mathlib::Vec3_T<Scalar> axis_world = (R_joint * joint.axis).normalized();

		// Position of joint in world frame
		mathlib::Vec3_T<Scalar> joint_pos_world = jointWorldPose.template block<3, 1>(0, 3);

		// Rotation from link frame to world frame
		mathlib::Mat3_T<Scalar> R_link = linkWorldPose.template block<3, 3>(0, 0);
		mathlib::Vec3_T<Scalar> link_pos_world = linkWorldPose.template block<3, 1>(0, 3);
		mathlib::Vec3_T<Scalar> com_world = R_link * Vec3_T<Scalar>(link.inertial.com_xyz) + link_pos_world;

		// Translational contribution (parallel axis theorem)
		mathlib::Vec3_T<Scalar> r = com_world - joint_pos_world;

		// Translational contribution to inertia about the joint axis
		Scalar I_trans = m * (axis_world.cross(r)).squaredNorm();

		// Rotational contribution
		mathlib::Mat3_T<Scalar> I_local = computeLinkInertiaTensor(link);
		mathlib::Mat3_T<Scalar> I_world = R_link * I_local * R_link.transpose();

		// Rotational contribution to inertia about the joint axis
		Scalar I_rot = axis_world.transpose() * I_world * axis_world;
		Scalar I_eff_i = I_trans + I_rot;

		return Eigen::numext::maxi(I_eff_i, Scalar(1e-6)); // [kg*m^2], I_eff contribution of this joint + floor to avoid singularities
	}

	// Computes the full mass matrix M(q) based on the current state and robot configuration
	template<typename Scalar>
	void RobotDynamics::computeMassMatrix(
		const RobotConstModel& robot,
		const std::vector<mathlib::Pose_T<Scalar>>& T_world,
		const std::vector<mathlib::Pose_T<Scalar>>& jointWorldPoses,
		mathlib::MatX_T<Scalar>& M_out
	) const {
		const size_t n = robot.joints.size();
		M_out.resize(n, n);
		M_out.setZero();

		// Compute its contribution to the mass matrix for each link
		for (size_t k = 0; k < robot.links.size(); ++k) {
			const RobotLink& link = robot.links[k];
			const Scalar m = link.inertial.mass;

			if (m <= Scalar(0)) { continue; }

			const mathlib::Mat3_T<Scalar> R = T_world[k].template block<3, 3>(0, 0);  // Rotation from link frame to world frame
			const mathlib::Vec3_T<Scalar> p = T_world[k].template block<3, 1>(0, 3);  // Center of mass of the link in world frame
			const mathlib::Vec3_T<Scalar> com = R * link.inertial.com_xyz + p; // Center of mass in world frame

			mathlib::Mat3_T<Scalar> I_local = computeLinkInertiaTensor(link); // inertia tensor in link frame
			mathlib::Mat3_T<Scalar> I_world = R * I_local * R.transpose();	   // inertia tensor in world frame

			// Compute Jacobian columns for each joint and accumulate mass matrix contributions
			for (size_t i = 0; i < n; ++i) {
				const RobotJoint& j_i = robot.joints[i];
				if (j_i.type == eJointType::FIXED) { continue; }
				if (!robot.jointAffectsLink(i, k)) { continue; } // skip if joint i does not affect link k

				const mathlib::Pose_T<Scalar>& T_joint_i = jointWorldPoses[i]; // pose of joint i in world frame

				// Rotation from joint i frame to world frame
				const mathlib::Mat3_T<Scalar> R_i = T_joint_i.template block<3, 3>(0, 0); // rotation from joint i frame to world frame
				const mathlib::Vec3_T<Scalar> p_i = T_joint_i.template block<3, 1>(0, 3); // joint position in world frame
				const mathlib::Vec3_T<Scalar> z_i = (R_i * j_i.axis).normalized(); // joint axis in world frame

				mathlib::Vec3_T<Scalar> J_vi = z_i.cross(com - p_i); // linear velocity Jacobian column for joint i
				mathlib::Vec3_T<Scalar> J_wi = z_i;				  // angular velocity Jacobian column for joint i

				// Computes the contribution to the mass matrix from this link for joints i and j
				for (size_t j = 0; j < n; ++j) {
					const RobotJoint& j_j = robot.joints[j];
					if (j_j.type == eJointType::FIXED) { continue; }

					if (!robot.jointAffectsLink(j, k)) { continue; } // skip if joint j does not affect link k

					const mathlib::Pose_T<Scalar>& T_joint_j = jointWorldPoses[j]; // pose of joint i in world frame

					const mathlib::Mat3_T<Scalar> R_j = T_joint_j.template block<3, 3>(0, 0);
					const mathlib::Vec3_T<Scalar> p_j = T_joint_j.template block<3, 1>(0, 3);
					const mathlib::Vec3_T<Scalar> z_j = mathlib::safeNormalised(R_j * j_j.axis);

					mathlib::Vec3_T<Scalar> J_vj = z_j.cross(com - p_j); // linear velocity Jacobian column for joint j
					mathlib::Vec3_T<Scalar> J_wj = z_j;				  // angular velocity Jacobian column for joint j

					M_out(i, j) += m * J_vi.dot(J_vj) + J_wi.transpose() * I_world * J_wj; // [kg*m^2]
				}
			}
		}
	}

	// Computes the Coriolis and centrifugal bias vector h(q, qd) based on the current state and robot configuration
	template<typename Scalar>
	mathlib::VecX_T<Scalar> RobotDynamics::computeCoriolisVector(
		const RobotConstModel& robot,
		const mathlib::VecX_T<Scalar>& q,
		const mathlib::VecX_T<Scalar>& qd,
		const std::vector<mathlib::Pose_T<Scalar>>& T_world,
		const mathlib::MatX_T<Scalar>& M
	) const {
		const size_t n = robot.joints.size();
		const Scalar eps = Scalar(1e-6); // small value to prevent division by zero
		mathlib::VecX_T<Scalar> q_eps = q;

		std::vector<mathlib::MatX_T<Scalar>> dM_dq(n, mathlib::MatX_T<Scalar>::Zero(n, n)); // partial derivatives of M with respect to each joint angle
		mathlib::VecX_T<Scalar> x_eps(2 * n); // state vector for kinematics

		std::vector<mathlib::Pose_T<Scalar>> T_world_eps; // forward kinematics for perturbed configurations
		T_world_eps.resize(robot.links.size());

		std::vector<mathlib::Pose_T<Scalar>> jointWorldPoses_eps;
		jointWorldPoses_eps.resize(n);

		mathlib::MatX_T<Scalar> M_plus(n, n);

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
			_kinematics->computeForwardKinematics_fromState<Scalar>(robot, x_eps, T_world_eps);
			jointWorldPoses_eps = _kinematics->calcJointWorldPoses<Scalar>(T_world_eps, robot);
			computeMassMatrix(robot, T_world_eps, jointWorldPoses_eps, M_plus); // mass matrix for the perturbed configuration

			dM_dq[k] = (M_plus - M) / eps; // [kg*m^2/rad], partial derivative of mass matrix with
		}

		// Compute Coriolis and centrifugal bias vector h using Christoffel symbols of the first kind
		VecX_T<Scalar> h = VecX_T<Scalar>::Zero(n);
		for (size_t i = 0; i < n; ++i) {
			for (size_t j = 0; j < n; ++j) {
				for (size_t k = 0; k < n; ++k) {
					Scalar C_ijk = 0.5 * (dM_dq[k](i, j) + dM_dq[j](i, k) - dM_dq[i](j, k)) * qd[k]; // Christoffel symbol of the first kind for indices (i, j, k)
					h(i) += C_ijk * qd[j] * qd[k]; // contribution to Coriolis and centrifugal bias for joint i from joints j and k
				}
			}
		}
		return h; // [Nm], Coriolis and centrifugal bias vector for the robot at configuration q and velocity qd
	}

	// Computes the gravity torque for a joint based on the current state and robot configuration
	template<typename Scalar>
	mathlib::VecX_T<Scalar> RobotDynamics::computeGravityTorque(
		const RobotConstModel& robot,
		const std::vector<mathlib::Pose_T<Scalar>>& T_world,
		const std::vector<mathlib::Pose_T<Scalar>>& jointWorldPoses
	) const {
		const size_t n = robot.joints.size();
		mathlib::VecX_T<Scalar> tau_G = mathlib::VecX_T<Scalar>::Zero(n);
		Scalar g{ _gravity }; // [m/s^2], gravity acceleration magnitude

		// For each joint, sum the gravity contributions from all links
		for (size_t i = 0; i < n; ++i) {
			const RobotJoint& j = robot.joints[i];
			if (j.type == eJointType::FIXED) { continue; }

			Scalar tau_g_i = Scalar(0); // [Nm], gravity torque contribution for joint i

			const mathlib::Pose_T<Scalar>& T_joint = jointWorldPoses[i]; // pose of joint i in world frame

			const mathlib::Mat3_T<Scalar> R_i = T_joint.block<3, 3>(0, 0);
			const mathlib::Vec3_T<Scalar> p_i = T_joint.block<3, 1>(0, 3);
			const mathlib::Vec3_T<Scalar> axis_world = (R_i * robot.joints[i].axis).normalized();

			// For each link, compute the gravitational force and its torque contribution about joint i
			for (size_t k = 0; k < robot.links.size(); ++k) {
				const RobotLink& link = robot.links[k];
				const Scalar m = (Scalar)link.inertial.mass;
				if (m <= Scalar(0)) { continue; }

				if (!robot.jointAffectsLink(i, k)) { continue; }

				// Link's center of mass in world frame
				const mathlib::Mat3_T<Scalar> R_k = T_world[k].template block<3, 3>(0, 0);
				const mathlib::Vec3_T<Scalar> com_world = R_k * link.inertial.com_xyz + T_world[k].template block<3, 1>(0, 3);

				// Gravitational force on the link
				mathlib::Vec3_T<Scalar> g_world;
				g_world = mathlib::Vec3_T<Scalar>(0.0, 0.0, -g); // [m/s^2], gravity vector in world frame
				const mathlib::Vec3_T<Scalar> F_g = m * g_world; // [N], gravitational force on the link in world frame
				const mathlib::Vec3_T<Scalar> r = com_world - p_i; // [m]

				// Torque = r × F_g projected onto joint axis
				tau_g_i += axis_world.dot(r.cross(F_g));
			}
			tau_G[i] = tau_g_i;
		}
		return tau_G; // [Nm], gravity torques for each joint
	}

	// Computes the analytical Jacobian matrix J(q) for the robot based on the current state and robot configuration
	template<typename Scalar>
	void RobotDynamics::analyticalJacobian(
		const RobotConstModel& robot,
		const mathlib::VecX_T<Scalar>& x,
		mathlib::MatX_T<Scalar>& J_out,
		DenseDynamicsScratch<Scalar>& scratch
	) {
		const size_t n = robot.joints.size();

		Eigen::Map<const mathlib::VecX_T<Scalar>> q_local(x.data(), n);
		Eigen::Map<const mathlib::VecX_T<Scalar>> qd_local(x.data() + n, n);

		_kinematics->computeForwardKinematics_fromState<Scalar>(robot, x, scratch.T_world);
		scratch.jointWorldPoses = _kinematics->calcJointWorldPoses<Scalar>(scratch.T_world, robot);
		computeMassMatrix<Scalar>(robot, scratch.T_world, scratch.jointWorldPoses, scratch.M);

		J_out.setZero(2 * n, 2 * n); // [rad/rad] position part, [rad/s / rad/s] velocity part
		J_out.block(0, n, n, n).setIdentity();

		mathlib::MatX_T<Scalar> dTau_dq = mathlib::MatX::Zero(n, n);
		mathlib::MatX_T<Scalar> dTau_dv = mathlib::MatX::Zero(n, n);

		for (size_t i = 0; i < n; ++i) {
			const RobotJoint& joint = robot.joints[i];
			if (joint.type == eJointType::FIXED) { continue; }

			const Scalar wn = static_cast<Scalar>(joint.wn_target);	 // [rad/s], natural frequency
			const Scalar z = static_cast<Scalar>(joint.zeta_target);  // damping ratio
			const Scalar eps = static_cast<Scalar>(1e-6);

			const Scalar I_eff = mathlib::LSE_smoothMax(scratch.M(i, i), eps);

			const Scalar k_p = I_eff * wn * wn;
			const Scalar k_d = Scalar(2.0) * z * I_eff * wn;

			const Scalar b = static_cast<Scalar>(0.2); // viscous damping coefficient
			const Scalar c = static_cast<Scalar>(0.05); // Coulomb friction coefficient
			const Scalar eps_f = static_cast<Scalar>(1e-2);

			dTau_dq(i, i) = -k_p;

			Scalar qd_i = qd_local[i];
			Scalar tanh_term = mathlib::tanh(qd_i / eps_f);
			Scalar stiff_friction_slope = c * (Scalar(1) - tanh_term * tanh_term) / eps_f;
			dTau_dv(i, i) = -k_d - b + stiff_friction_slope;
		}

		auto solver = scratch.M.ldlt();
		mathlib::MatX_T<Scalar> da_dq = solver.solve(dTau_dq);
		mathlib::MatX_T<Scalar> da_dv = solver.solve(dTau_dv);

		J_out.block(n, 0, n, n) = da_dq;
		J_out.block(n, n, n, n) = da_dv;
	}

	// Computes the Coriolis and centrifugal torque for a joint based on the current state and robot configuration
	template<typename Scalar>
	mathlib::VecX_T<Scalar> RobotDynamics::derivative_dense(
		Scalar t,
		const mathlib::VecX_T<Scalar>& x,
		const RobotSimSnapshot_T<Scalar>& snap,
		DynamicsScratch<Scalar>& scratch,
		DynamicsResult<Scalar>& out
	) {
		const size_t n = snap.model->joints.size();
		mathlib::VecX_T<Scalar> dx(2 * n);

		// Map the input state vector to joint angles and velocities
		Eigen::Map<const mathlib::VecX_T<Scalar>> q(x.data(), n);
		Eigen::Map<const mathlib::VecX_T<Scalar>> qd(x.data() + n, n);

		_kinematics->computeForwardKinematics_fromState<Scalar>(*snap.model, x, scratch.dense.T_world);

		scratch.dense.jointWorldPoses = _kinematics->calcJointWorldPoses<Scalar>(scratch.dense.T_world, *snap.model);

		computeMassMatrix<Scalar>(*snap.model, scratch.dense.T_world, scratch.dense.jointWorldPoses, scratch.dense.M); // [kg*m^2], full mass matrix for the robot at configuration q
		scratch.dense.h = computeCoriolisVector<Scalar>(*snap.model, q, qd, scratch.dense.T_world, scratch.dense.M); // [Nm], full Coriolis and centrifugal torque vector

		scratch.g.setZero();
		if (snap.torqueMode != eTorqueMode::NONE) {
			scratch.g = computeGravityTorque<Scalar>(*snap.model, scratch.dense.T_world, scratch.dense.jointWorldPoses);
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

			const Scalar wn = static_cast<Scalar>(joint.wn_target);	 // [rad/s], natural frequency
			const Scalar z = static_cast<Scalar>(joint.zeta_target);  // damping ratio

			const Scalar err = snap.q_ref[i] - q[i];	   // [rad], position error
			const Scalar err_d = snap.qd_ref[i] - qd[i]; // [rad/s], velocity error

			const Scalar eps = static_cast<Scalar>(1e-6);

			const Scalar I_eff = mathlib::LSE_smoothMax(scratch.dense.M(i, i), eps); // [kg*m^2], effective inertia for joint i with floor to prevent singularities
			const Scalar k_p = I_eff * wn * wn;		 // [Nm/rad], proportional gain
			const Scalar k_d = Scalar(2.0) * z * I_eff * wn; // [Nm/(rad/s)], derivative gain

			const Scalar b = static_cast<Scalar>(0.2); // viscous damping coefficient
			const Scalar c = static_cast<Scalar>(0.05); // Coulomb friction coefficient
			const Scalar eps_f = static_cast<Scalar>(1e-2);

			Scalar tau_i = k_p * err + k_d * err_d + I_eff * snap.qdd_ref[i]; // [Nm], control torque for joint i
			tau_i += scratch.g[i]; // Gravity compensation
			tau_i += scratch.dense.h[i]; // add Coriolis and centrifugal bias
			tau_i -= b * qd[i]; // subtract viscous damping
			tau_i -= c * mathlib::tanh(qd[i] / eps_f); // subtract Coulomb friction


			scratch.dense.tau[i] = tau_i;

			out.metrics.q[i] = mathlib::real(q[i]);
			out.metrics.qd[i] = mathlib::real(qd[i]);

			out.metrics.err[i] = mathlib::real(err);
			out.metrics.errd[i] = mathlib::real(err_d);

			out.metrics.I_eff[i] = mathlib::real(I_eff);
			out.metrics.tau[i] = mathlib::real(tau_i);

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

	template<typename Scalar>
	mathlib::VecX_T<Scalar> RobotDynamics::derivative_spatial(
		const robots::SpatialModel<Scalar>& model,
		Scalar t,
		const mathlib::VecX_T<Scalar>& x,
		const RobotSimSnapshot_T<Scalar>& snap,
		DynamicsScratch<Scalar>& scratch,
		DynamicsResult<Scalar>& out
	) {
		const size_t n = model.joints.size();
		mathlib::VecX_T<Scalar> dx(2 * n);

		Eigen::Map<const mathlib::VecX_T<Scalar>> q(x.data(), n);
		Eigen::Map<const mathlib::VecX_T<Scalar>> qd(x.data() + n, n);

		SpatialDynamics::computeSpatialKinematicsAndBias<Scalar>(
			model, q, qd,
			scratch.spatial.Xup,
			scratch.spatial.v,
			scratch.spatial.c
		);

		mathlib::MatX_T<Scalar> M = SpatialDynamics::CRBA<Scalar>(model, scratch.spatial.Xup, scratch);
		// RNEA to compute gravity compensation (q, 0, 0) for gravity, (q, qd, 0) for Coriolis
		mathlib::VecX_T<Scalar> qd_zero = mathlib::VecX_T<Scalar>::Zero(n);
		mathlib::VecX_T<Scalar> qdd_zero = mathlib::VecX_T<Scalar>::Zero(n);
		mathlib::VecX_T<Scalar> tau_g = SpatialDynamics::RNEA<Scalar>(model, q, qd_zero, qdd_zero, scratch);

		scratch.dense.tau.setZero();
		for (size_t i = 0; i < n; ++i) {
			const SpatialJoint<Scalar>& joint = model.joints[i];
			if (!isControlledJoint(joint.type)) {
				scratch.dense.tau[i] = Scalar(0);
				continue;
			}

			const Scalar wn = static_cast<Scalar>(5);
			const Scalar z = static_cast<Scalar>(0.7);

			const Scalar err = snap.q_ref[i] - q[i];
			const Scalar err_d = snap.qd_ref[i] - qd[i];

			const Scalar eps = static_cast<Scalar>(1e-6);

			const Scalar I_eff = mathlib::LSE_smoothMax(M(i, i), eps);
			const Scalar k_p = I_eff * wn * wn;
			const Scalar k_d = Scalar(2) * z * I_eff * wn;

			const Scalar b = static_cast<Scalar>(0.2); // viscous damping coefficient
			const Scalar c = static_cast<Scalar>(0.05); // Coulomb friction coefficient
			const Scalar eps_f = static_cast<Scalar>(1e-2);

			Scalar tau_i = k_p * err + k_d * err_d + I_eff * snap.qdd_ref[i];
			tau_i -= b * qd[i];
			//tau_i -= b * mathlib::tanh(qd[i] / eps_f);

			scratch.dense.tau[i] = tau_i;

			out.metrics.q[i] = mathlib::real(q[i]);
			out.metrics.qd[i] = mathlib::real(qd[i]);

			out.metrics.err[i] = mathlib::real(err);
			out.metrics.errd[i] = mathlib::real(err_d);

			out.metrics.I_eff[i] = mathlib::real(I_eff);
			out.metrics.tau[i] = mathlib::real(tau_i);
		}

		out.qdd = SpatialDynamics::ABA<Scalar>(model, q, qd, scratch.dense.tau, scratch);
		out.metrics.qdd = out.qdd;
		dx.head(n) = qd;
		dx.tail(n) = out.qdd;
		return dx;
	}

	template<typename Scalar>
	void RobotDynamics::jacobian_spatial(
		const robots::SpatialModel<Scalar>& model,
		const mathlib::VecX_T<Scalar>& x,
		const RobotSimSnapshot_T<Scalar>& snap,
		const mathlib::VecX_T<Scalar>& kp,
		const mathlib::VecX_T<Scalar>& kd,
		mathlib::MatX_T<Scalar>& F_out,
		DynamicsScratch<Scalar>& scratch
	) {
		const size_t n = model.joints.size();

		F_out.setZero(2 * n, 2 * n);
		F_out.block(0, n, n, n).setIdentity();

		Eigen::Map<const mathlib::VecX_T<Scalar>> q(x.data(), n);
		Eigen::Map<const mathlib::VecX_T<Scalar>> qd(x.data() + n, n);

		SpatialDynamics::computeSpatialKinematicsAndBias<Scalar>(
			model, q, qd,
			scratch.spatial.Xup,
			scratch.spatial.v,
			scratch.spatial.c
		);

		scratch.dense.M = SpatialDynamics::CRBA(model, scratch.spatial.Xup, scratch);

		mathlib::MatX_T<Scalar> dTau_dq = mathlib::MatX_T<Scalar>::Zero(n, n);
		mathlib::MatX_T<Scalar> dTau_dv = mathlib::MatX_T<Scalar>::Zero(n, n);

		for (size_t i = 0; i < n; ++i) {
			const SpatialJoint<Scalar>& joint = model.joints[i];
			if (!isControlledJoint(joint.type)) { continue; }
			dTau_dq(i, i) = -kp[i];
			const Scalar b = static_cast<Scalar>(0.2); // viscous damping coefficient
			const Scalar c = static_cast<Scalar>(0.05); // Coulomb friction coefficient
			const Scalar eps_f = static_cast<Scalar>(1e-2);

			Scalar tanh_term = mathlib::tanh(qd[i] / eps_f);
			Scalar stiff_friction_slope = -c * (Scalar(1.0) - tanh_term * tanh_term) / eps_f;
			dTau_dv(i, i) = -kd[i] - b;
		}

		Eigen::LDLT<mathlib::MatX_T<Scalar>> solver(scratch.dense.M); // compute the Cholesky decomposition of the mass matrix for efficient solving
		mathlib::MatX_T<Scalar> dqdd_dtau_q = solver.solve(dTau_dq); // compute the partial derivative of qdd with respect to q
		mathlib::MatX_T<Scalar> dqdd_dtau_v = solver.solve(dTau_dv); // compute the partial derivative of qdd with respect to qd

		F_out.block(n, 0, n, n) = dqdd_dtau_q; // fill the Jacobian block for qdd with respect to q
		F_out.block(n, n, n, n) = dqdd_dtau_v; // fill the Jacobian block for qdd with respect to qd)
	}

	// Computes the derivative of the state vector with control gains based on the current state and robot configurations
	template<typename Scalar>
	mathlib::VecX_T<Scalar> RobotDynamics::derivative_with_gains(
		Scalar t,
		const mathlib::VecX_T<Scalar>& x,
		const RobotSimSnapshot_T<Scalar>& snap,
		const mathlib::VecX_T<Scalar>& kp,
		const mathlib::VecX_T<Scalar>& kd,
		DynamicsScratch<Scalar>& scratch,
		DynamicsResult<Scalar>& out
	) {
		const size_t n = snap.model->joints.size();
		mathlib::VecX_T<Scalar> dx(2 * n);

		Eigen::Map<const mathlib::VecX_T<Scalar>> q(x.data(), n);
		Eigen::Map<const mathlib::VecX_T<Scalar>> qd(x.data() + n, n);

		_kinematics->computeForwardKinematics_fromState<Scalar>(*snap.model, x, scratch.dense.T_world);
		scratch.dense.jointWorldPoses = _kinematics->calcJointWorldPoses<Scalar>(scratch.dense.T_world, *snap.model);

		computeMassMatrix(*snap.model, scratch.dense.T_world, scratch.dense.jointWorldPoses, scratch.dense.M);
		scratch.dense.h = computeCoriolisVector<Scalar>(*snap.model, q, qd, scratch.dense.T_world, scratch.dense.M);

		scratch.g.setZero();
		if (snap.torqueMode != eTorqueMode::NONE) {
			scratch.g = computeGravityTorque<Scalar>(*snap.model, scratch.dense.T_world, scratch.dense.jointWorldPoses);
		}

		scratch.dense.tau.setZero();
		for (size_t i = 0; i < n; ++i) {
			if (snap.model->joints[i].type == eJointType::FIXED) continue;

			const Scalar eps = static_cast < Scalar>(1e-6);
			const Scalar b = static_cast<Scalar>(0.2); // viscous damping coefficient
			const Scalar c = static_cast<Scalar>(0.05); // Coulomb friction coefficient
			const Scalar eps_f = static_cast<Scalar>(1e-2);

			Scalar tau_i = kp[i] * (snap.q_ref[i] - q[i]) + kd[i] * (snap.qd_ref[i] - qd[i]) + mathlib::LSE_smoothMax(scratch.dense.M(i, i), eps) * snap.qdd_ref[i];
			tau_i += scratch.g[i] + scratch.dense.h[i];
			tau_i -= b * qd[i];
			tau_i -= c * mathlib::tanh(qd[i] / eps_f);

			scratch.dense.tau[i] = tau_i;
		}

		scratch.dense.rhs.noalias() = scratch.dense.tau - scratch.dense.h - scratch.g;
		out.qdd = scratch.dense.M.ldlt().solve(scratch.dense.rhs);
		out.metrics.qdd = out.qdd;

		dx.head(n) = qd;
		dx.tail(n) = out.qdd;
		return dx;
	}

	// Computes the Jacobian matrix with control gains based on the current state and robot configuration
	template<typename Scalar>
	void RobotDynamics::jacobian_with_gains(
		const mathlib::VecX_T<Scalar>& x,
		const RobotSimSnapshot_T<Scalar>& snap,
		const mathlib::VecX_T<Scalar>& kp,
		const mathlib::VecX_T<Scalar>& kd,
		mathlib::MatX_T<Scalar>& F_out,
		DenseDynamicsScratch<Scalar>& scratch
	) {
		const size_t n = snap.model->joints.size();

		F_out.setZero(2 * n, 2 * n);
		F_out.block(0, n, n, n).setIdentity();

		Eigen::Map<const mathlib::VecX_T<Scalar>> q(x.data(), n);
		Eigen::Map<const mathlib::VecX_T<Scalar>> qd(x.data() + n, n);

		// Compute a local mass matrix for this exact stage evaluation frame
		_kinematics->computeForwardKinematics_fromState<Scalar>(*snap.model, x, scratch.T_world);
		scratch.jointWorldPoses = _kinematics->calcJointWorldPoses<Scalar>(scratch.T_world, *snap.model);
		computeMassMatrix<Scalar>(*snap.model, scratch.T_world, scratch.jointWorldPoses, scratch.M);

		mathlib::MatX_T<Scalar> dTau_dq = mathlib::MatX_T<Scalar>::Zero(n, n);
		mathlib::MatX_T<Scalar> dTau_dv = mathlib::MatX_T<Scalar>::Zero(n, n);

		for (size_t i = 0; i < n; ++i) {
			if (snap.model->joints[i].type == eJointType::FIXED) continue;

			dTau_dq(i, i) = -kp[i];

			const Scalar b = static_cast<Scalar>(0.2); // viscous damping coefficient
			const Scalar c = static_cast<Scalar>(0.05); // Coulomb friction coefficient
			const Scalar eps_f = static_cast<Scalar>(1e-2);

			Scalar tanh_term = mathlib::tanh(qd[i] / eps_f);
			Scalar stiff_friction = -c * (Scalar(1.0) - tanh_term * tanh_term) / eps_f;
			dTau_dv(i, i) = -kd[i] - b + stiff_friction;
		}

		auto solver = scratch.M.ldlt();
		F_out.block(n, 0, n, n) = solver.solve(dTau_dq);
		F_out.block(n, n, n, n) = solver.solve(dTau_dv);
	}
} // namespace robots