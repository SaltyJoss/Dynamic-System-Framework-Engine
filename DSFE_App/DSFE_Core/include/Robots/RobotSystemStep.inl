// DSFE_Core RobotSystemStep.inl
#pragma once

namespace robots {
	// Method to take a snapshot of the current robot state
	template<typename Scalar>
	RobotSimSnapshot_T<Scalar> RobotSystem::takeSnapshot(Scalar simTime) const {
		RobotSimSnapshot_T<Scalar> snap;
		snap.model = &_constModel;
		const size_t n = (size_t)_robot.joints.size();

		snap.q.resize(n);
		snap.qd.resize(n);

		snap.q_ref.resize(n);
		snap.qd_ref.resize(n);
		snap.qdd_ref.resize(n);

		for (size_t i = 0; i < n; ++i) {
			const auto& j = _robot.joints[i];

			snap.q[i] = j.q;
			snap.qd[i] = j.qd;

			snap.q_ref[i] = j.q_ref;
			snap.qd_ref[i] = j.qd_ref;
			snap.qdd_ref[i] = j.qdd_ref;
		}

		snap.robotRootPose = _robotRootPose.template cast<Scalar>();
		snap.baseIsFree = _baseIsFree;
		snap.lastBaseForwardForce = Scalar(_lastBaseForwardForce);
		snap.gravity = Scalar(_gravity);

		snap.torqueMode = _robot.torqueMode;

		snap.dt = Scalar(_dynamics->dt());
		snap.simTime = simTime;

		return snap;
	}

	template<typename Scalar, typename IntegratorT>
	RobotStepResult_T<Scalar> RobotSystem::step_impl(const mathlib::VecX_T<Scalar>& x, Scalar dt, Scalar t, IntegratorT& integrator) {
		RobotStepResult_T<Scalar> result;
		result.snap = takeSnapshot<Scalar>(t);
		auto& snap = result.snap;
		const size_t n = snap.model->joints.size();

		Eigen::Map<const mathlib::VecX_T<Scalar>> q(x.data(), n);
		Eigen::Map<const mathlib::VecX_T<Scalar>> qd(x.data() + n, n);

		mathlib::VecX_T<Scalar> qdd(n);
		for (size_t i = 0; i < n; ++i) { qdd[i] = _robot.joints[i].qdd_ref; }

		std::vector<Pose_T<Scalar>> T_start(snap.model->links.size());
		_kinematics->computeForwardKinematics_fromState(*snap.model, x, T_start);
		std::vector<Pose_T<Scalar>> jointWorldPoses_start = _kinematics->calcJointWorldPoses(T_start, *snap.model);

		SpatialModel<Scalar> spatialModel = _spatialModel.template cast<Scalar>();
		DynamicsScratch<Scalar> dynScratch;
		DynamicsResult<Scalar> dynResult;
		dynScratch.resize(n, snap.model->links.size());
		dynResult.resize(n);

		SpatialDynamics::computeSpatialKinematicsAndBias<Scalar>(
			spatialModel,
			q, qd,
			dynScratch.spatial.Xup,
			dynScratch.spatial.v, dynScratch.spatial.c
		);

		// CRBA only for controller inertia scaling
		//LOG_INFO("Computing M_start via CRBA");
		mathlib::MatX_T<Scalar> M_start = SpatialDynamics::CRBA<Scalar>(
			spatialModel,
			dynScratch.spatial.Xup,
			dynScratch
		);
		//LOG_INFO("M_start dims = %d x %d", (int)M_start.rows(), (int)M_start.cols());

		// Cache frozen joint gains for this step
		mathlib::VecX_T<Scalar> kp_frozen(n), kd_frozen(n);
		for (size_t i = 0; i < n; ++i) {
			const auto& joint = snap.model->joints[i];
			//LOG_INFO("Snap model joint name = %s", joint.name.c_str());
			if (joint.type == eJointType::FIXED) { continue; }

			//LOG_INFO("Max of M_start and 1e-6: %.3f", mathlib::max(M_start(i, i), Scalar(1e-6)));
			//LOG_INFO("I_eff_controller size = %d", (int)dynScratch.dense.I_eff_controller.size());
			dynScratch.dense.I_eff_controller[i] = mathlib::max(M_start(i, i), Scalar(1e-6));
			const Scalar I_eff = dynScratch.dense.I_eff_controller[i];

			kp_frozen[i] = I_eff * joint.wn_target * joint.wn_target;
			kd_frozen[i] = Scalar(2) * joint.zeta_target * I_eff * joint.wn_target;
		}

		// Compute RNEA torques for feedforward control
		mathlib::VecX_T<Scalar> tau_rnea = SpatialDynamics::RNEA<Scalar>(
			spatialModel,
			q, qd, qdd,
			dynScratch
		); // [Nm]
		result.tau_rnea = tau_rnea;
		//LOG_INFO_ONCE("tau_rnea size = %d", (double)tau_rnea.size());

		// Define the derivative function for integration, capturing necessary variables by reference
		auto f_deriv = [&, kp_frozen, kd_frozen](auto t, const auto& xIn) {
			return _dynamics->derivative_spatial<Scalar>(
				spatialModel,
				t, xIn,
				snap,
				dynScratch, dynResult
			);
		};
		// Define the Jacobian function for integration, capturing necessary variables by reference
		auto f_J = [&, kp_frozen, kd_frozen](const mathlib::VecX_T<Scalar>& xIn, mathlib::MatX_T<Scalar>& J_out) {
			_dynamics->jacobian_spatial<Scalar>(
				spatialModel,
				xIn, snap,
				kp_frozen, kd_frozen,
				J_out, dynScratch
			);
		};

		if constexpr (std::is_same_v<std::remove_cvref_t<IntegratorT>, integration::IntegrationService>) {
			mathlib::VecX x_real = x.template cast<double>();
			auto step = integrator.step(_curIntMethod, x_real, static_cast<double>(t), static_cast<double>(dt), f_deriv, f_J);
			result.stepOut.x_next = step.x_next.template cast<Scalar>();
			result.stepOut.dt_taken = step.dt_taken;
			result.stepOut.dt_sug = step.dt_sug;
		}
		else if constexpr (std::is_same_v<std::remove_cvref_t<IntegratorT>, integration::DifferentiableIntegrator>) {
			result.stepOut = integrator.step(_curIntMethod_AD, x, t, dt, f_deriv);
		}

		for (int i = 0; i < result.stepOut.x_next.size(); ++i) {
			const auto v = mathlib::real(result.stepOut.x_next[i]);
			if (std::isnan(v) || std::isinf(v)) { LOG_ERROR("Non-finite x_next[%d] = %f", i, (double)v); } // TODO add Scalar isnan and isinf checks to mathlib and use those instead (need to handle both float and double cases)
		}
		result.dynamics = dynResult;
		return result;
	}

	template<typename Scalar>
	void RobotSystem::postStepUpdate(const RobotStepResult_T<Scalar>& result, const mathlib::VecX& x) {
		const size_t n = result.snap.model->joints.size();

		Eigen::Map<const mathlib::VecX> q_next(x.data(), n);
		Eigen::Map<const mathlib::VecX> qd_next(x.data() + n, n);

		// Enforce joint limits
		for (auto& j : _robot.joints) { enforceJointLimits(j); }

		// Recompute kinematics and dynamics at the new state for logging and control purposes
		std::vector<Pose> T_world(result.snap.model->links.size());
		_kinematics->computeForwardKinematics_fromState<double>(*result.snap.model, x, T_world);
		std::vector<Pose> jointWorldPoses = _kinematics->calcJointWorldPoses<double>(T_world, *result.snap.model);

		DynamicsScratch<double> dynScratch;

		// Compute spatial kinematics and bias terms for the new state
		SpatialDynamics::computeSpatialKinematicsAndBias<double>(
			_spatialModel,
			q_next, qd_next,
			dynScratch.spatial.Xup,
			dynScratch.spatial.v, dynScratch.spatial.c
		);
		// Compute mass matrix at the new state
		mathlib::MatX M = SpatialDynamics::CRBA<double>(_spatialModel, dynScratch.spatial.Xup, dynScratch);
		mathlib::VecX tau_g = _dynamics->computeGravityTorque<double>(*result.snap.model, T_world, jointWorldPoses);

		// Extract real parts of relevant variables for logging and control
		mathlib::VecX q_real = result.snap.q.unaryExpr([](const auto& v) { return mathlib::real(v); });
		mathlib::VecX qd_real = result.snap.qd.unaryExpr([](const auto& v) { return mathlib::real(v); });
		mathlib::VecX q_ref_real = result.snap.q_ref.unaryExpr([](const auto& v) { return mathlib::real(v); });
		mathlib::VecX qd_ref_real = result.snap.qd_ref.unaryExpr([](const auto& v) { return mathlib::real(v); });
		mathlib::VecX tau_rnea_real = result.tau_rnea.unaryExpr([](const auto& v) { return mathlib::real(v); });

		// Compute system kinetic energy: E_kin = 0.5 * qd^T * M(q) * qd
		double sys_KE = 0.5 * qd_real.transpose() * M * qd_real; // [J], kinetic energy of the robot at configuration q and velocity qd

		// Compute system potential energy at configuration q (relative to gravity)
		double sys_PE = 0.0;
		double g = _dynamics->getGravity();

		for (size_t k = 0; k < _robot.links.size(); ++k) {
			const RobotLink& link = _robot.links[k];
			const double m = link.inertial.mass;
			if (m <= 0.0) { continue; }
			Vec3 com_world = (T_world[k].block<3, 3>(0, 0) * link.inertial.com_xyz) + T_world[k].block<3, 1>(0, 3);
			sys_PE += m * g * com_world.z();
		}

		const double sys_E = sys_KE + sys_PE; // total mechanical energy of the system

		// Log metrics to buffer if logging is enabled
		robots::JointLogBuffer* buf = nullptr;
		if (_useInternalLogging) { int idx = _activeLogBufIdx.load(std::memory_order_acquire); buf = &_logBuffers[idx]; }
		else { buf = _logBuffer; }

		if (buf) {
			auto dynResult = result.dynamics;
			for (size_t i = 0; i < n; ++i) {
				const RobotJoint& j = _robot.joints[i];

				const double I_eff = (j.type == eJointType::FIXED) ? 1.0 : mathlib::real(dynResult.metrics.I_eff[i]);
				const double err = q_ref_real[i] - q_real[i];
				const double err_d = qd_ref_real[i] - qd_real[i];

				JointLogBuffer::JointLogEntry e{};

				e.sim_time = _simTime;
				e.dt_taken = mathlib::real(result.stepOut.dt_taken);
				e.dt_sug = mathlib::real(result.stepOut.dt_sug);
				e.theta = q_real[i]; e.omega = qd_real[i]; e.alpha = mathlib::real(dynResult.metrics.qdd[i]);
				e.err = err; e.err_d = err_d;
				e.I_eff = I_eff;
				e.tau = mathlib::real(dynResult.metrics.tau[i]); e.tau_ff = tau_rnea_real[i];  e.tau_gravity = tau_g[i];
				e.tau_sat = mathlib::real(dynResult.metrics.tau_sat[i]);
				e.KE = sys_KE; e.PE = sys_PE; e.E_total = sys_E;
				e.clamp_theta = mathlib::real(_clampTheta[i]); e.clamp_omega = mathlib::real(_clampOmega[i]);
				e.sat_flag = mathlib::real(dynResult.metrics.sat_flag[i]); e.joint_index = (int)i;
				buf->push_entry(e);
			}
		}
	}

	template<size_t NVar>
	void RobotSystem::step_AD(double dt, double simTime) {
		if (!hasRobot()) { return; }
		using Dual = mathlib::DualNumber_T<double, NVar>;
		_simTime = simTime;
		const size_t n = _robot.joints.size();
		mathlib::VecX_T<Dual> x = packState().template cast<Dual>();

		assert((size_t)x.size() <= NVar && "State size exceeds the number of dual variables."); // Checks state vector size is within the dual variable limit
		for (size_t i = 0; i < (size_t)x.size(); ++i) { x[i].dual[i] = 1.0; }

		auto result = step_impl<Dual>(x, Dual(dt), Dual(simTime), *_AD_integrator);
		mathlib::VecX x_real = result.stepOut.x_next.unaryExpr([](const auto& v) { return mathlib::real(v); });
		unpackState(x_real);
		_dynamics->setDt(result.stepOut.dt_taken);

		postStepUpdate(result, x_real);

		// Update base pose if free-floating
		if (_baseIsFree) {
			integrateBaseTranslation(dt);
			updateBaseRootPose();
		}
		// Update kinematics
		computeRobotKinematics(_worldTransforms);
	}
}