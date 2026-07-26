/*
 * File: Systems/RigidBodySystemStep.inl
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

namespace systems {
	template<typename T>
	T RigidBodySystem::clampJointAngle_T(const RigidBodyJoint& joint, T angleRad) {
		if (joint.limits.continuous) { return mathlib::wrapRad<T>(angleRad); }
		else { return std::clamp(angleRad, T(joint.limits.minAngle), T(joint.limits.maxAngle)); }
	}

	// Method to take a snapshot of the current rigidbody state
	template<typename T>
	RigidBodySnapshot_T<T> RigidBodySystem::takeSnapshot(T simTime) const {
		RigidBodySnapshot_T<T> snap;
		snap.model = &_constModel;
		const size_t n = (size_t)_body.joints.size();

		snap.q.resize(n);
		snap.qd.resize(n);

		snap.q_ref.resize(n);
		snap.qd_ref.resize(n);
		snap.qdd_ref.resize(n);

		for (size_t i = 0; i < n; ++i) {
			const auto& j = _body.joints[i];

			snap.q[i] = j.q;
			snap.qd[i] = j.qd;

			snap.q_ref[i] = j.q_ref;
			snap.qd_ref[i] = j.qd_ref;
			snap.qdd_ref[i] = j.qdd_ref;
		}

		snap.root_pose = _root_pose.template cast<T>();
		snap.baseIsFree = _baseIsFree;
		snap.lastBaseForwardForce = T(_lastBaseForwardForce);
		snap.gravity = T(_gravity);

		snap.torqueMode = _body.torqueMode;

		snap.dt = T(_dynamics->dt());
		snap.simTime = simTime;

		return snap;
	}

	template<typename Scalar, typename IntegratorT>
	RigidBodyStepResult_T<Scalar> RigidBodySystem::step_impl(
		const mathlib::VecX_T<Scalar>& x,
		Scalar dt, Scalar t, IntegratorT& integrator,
		physics::DynamicsScratch<Scalar>& dynamicScratch, physics::DynamicsResult<Scalar>& dynamicResult
	) {
		RigidBodyStepResult_T<Scalar> result;
		result.snap = takeSnapshot<Scalar>(t);
		auto& snap = result.snap;
		const size_t n = snap.model->joints.size();

		Eigen::Map<const mathlib::VecX_T<Scalar>> q(x.data(), n);
		Eigen::Map<const mathlib::VecX_T<Scalar>> qd(x.data() + n, n);

		mathlib::VecX_T<Scalar> qdd(n);
		for (size_t i = 0; i < n; ++i) { qdd[i] = _body.joints[i].qdd_ref; }

		std::vector<Pose_T<Scalar>> T_start(snap.model->links.size());
		_kinematics->computeForwardKinematics_fromState(*snap.model, x, T_start);
		std::vector<Pose_T<Scalar>> jointWorldPoses_start = _kinematics->calcJointWorldPoses(T_start, *snap.model);

		SpatialModel<Scalar> spatialModel = _spatialModel.template cast<Scalar>();
		auto& dynScratch = dynamicScratch;
		auto& dynResult = dynamicResult;

		physics::SpatialDynamics::computeSpatialKinematicsAndBias<Scalar>(
			spatialModel,
			q, qd,
			dynScratch.spatial.Xup,
			dynScratch.spatial.v, dynScratch.spatial.c
		);

		// CRBA only for controller inertia scaling
		mathlib::MatX_T<Scalar> M_start = physics::SpatialDynamics::CRBA<Scalar>(
			spatialModel,
			dynScratch.spatial.Xup,
			dynScratch
		);

		// Cache frozen joint gains for this step
		mathlib::VecX_T<Scalar> kp_frozen(n), kd_frozen(n);
		for (size_t i = 0; i < n; ++i) {
			const auto& joint = snap.model->joints[i];
			//LOG_INFO("Snap model joint name = %s", joint.name.c_str());
			if (joint.type == eJointType::FIXED) { continue; }

			dynScratch.dense.I_eff_controller[i] = mathlib::max(M_start(i, i), Scalar(1e-6));
			const Scalar I_eff = dynScratch.dense.I_eff_controller[i];

			kp_frozen[i] = I_eff * joint.wn_target * joint.wn_target;
			kd_frozen[i] = Scalar(2) * joint.zeta_target * I_eff * joint.wn_target;
		}

		// Compute RNEA torques for feedforward control
		mathlib::VecX_T<Scalar> tau_rnea = physics::SpatialDynamics::RNEA<Scalar>(
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

		if (!x.allFinite()) { LOG_ERROR("[step_impl] input state already non-finite"); }

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

	template<typename T>
	void RigidBodySystem::postStepUpdate(const mathlib::VecX& x, const physics::DynamicsScratch<T>& dynScratch, const RigidBodyStepResult_T<T>& result) {
		const size_t n = result.snap.model->joints.size();

		Eigen::Map<const mathlib::VecX> q_next(x.data(), n);
		Eigen::Map<const mathlib::VecX> qd_next(x.data() + n, n);

		// Enforce joint limits
		/*for (auto& j : _body.joints) { enforceJointLimits(j); }*/

		// Recompute kinematics and dynamics at the new state for logging and control purposes
		std::vector<Pose> T_world(result.snap.model->links.size());
		_kinematics->computeForwardKinematics_fromState<double>(*result.snap.model, x, T_world);

		// Alternative would be just 

		// Compute mass matrix at the new state
		mathlib::MatX M = dynScratch.dense.M.unaryExpr([](const auto& v) { return mathlib::real(v); });

		// Extract real parts of relevant variables for logging and control
		mathlib::VecX q_real = result.snap.q.unaryExpr([](const auto& v) { return mathlib::real(v); });
		mathlib::VecX qd_real = result.snap.qd.unaryExpr([](const auto& v) { return mathlib::real(v); });
		mathlib::VecX q_ref_real = result.snap.q_ref.unaryExpr([](const auto& v) { return mathlib::real(v); });
		mathlib::VecX qd_ref_real = result.snap.qd_ref.unaryExpr([](const auto& v) { return mathlib::real(v); });
		mathlib::VecX tau_rnea_real = result.tau_rnea.unaryExpr([](const auto& v) { return mathlib::real(v); });

		// Compute system kinetic energy: E_kin = 0.5 * qd^T * M(q) * qd
		double sys_KE = 0.5 * qd_real.transpose() * M * qd_real; // [J], kinetic energy of the rigidbody at configuration q and velocity qd

		// Compute system potential energy at configuration q (relative to gravity)
		double sys_PE = 0.0;
		double g = _dynamics->getGravity();

		for (size_t k = 0; k < _body.links.size(); ++k) {
			const RigidBodyLink& link = _body.links[k];
			const double m = link.inertial.mass;
			if (m <= 0.0) { continue; }
			mathlib::Vec3 com_world = (T_world[k].block<3, 3>(0, 0) * link.inertial.com_xyz) + T_world[k].block<3, 1>(0, 3);
			sys_PE += m * g * com_world.z();
		}

		const double sys_E = sys_KE + sys_PE; // total mechanical energy of the system

		// Log metrics to buffer if logging is enabled
		systems::JointLogBuffer* buf = nullptr;
		if (_useInternalLogging) { int idx = _activeLogBufIdx.load(std::memory_order_acquire); buf = &_logBuffers[idx]; }
		else { buf = _logBuffer; }

		if (buf) {
			auto dynResult = result.dynamics;
			for (size_t i = 0; i < n; ++i) {
				const RigidBodyJoint& j = _body.joints[i];

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
				e.tau = mathlib::real(dynResult.metrics.tau[i]); e.tau_ff = tau_rnea_real[i];  e.tau_gravity = 0.0;
				e.tau_sat = mathlib::real(dynResult.metrics.tau_sat[i]);
				e.KE = sys_KE; e.PE = sys_PE; e.E_total = sys_E;
				e.clamp_theta = mathlib::real(_clampTheta[i]); e.clamp_omega = mathlib::real(_clampOmega[i]);
				e.sat_flag = mathlib::real(dynResult.metrics.sat_flag[i]); e.joint_index = (int)i;
				buf->push_entry(e);
			}
		}
	}

	template<typename Scalar>
	void RigidBodySystem::assembleExtForces(physics::DynamicsScratch<Scalar>& dynScratch) {
		const size_t n = _body.joints.size();
		scratch.spatial.f_ext.assign(n, mathlib::SpatialVec_T<Scalar>()); // reset external forces
		if (_pendingExtForces.empty()) { return; }
		for (const auto& [jointIdx, linkIdx, worldPoint, worldForce] : _pendingExtForces) {
			// Link world pose from last kinematics update
			const Mat4& T = _worldTransforms[linkIdx];
			const Mat3 R = T.block<3, 3>(0, 0);
			const Vec3 o = T.block<3, 1>(0, 3);
			// Transform world force to link frame
			const Vec3 F_link = R.transpose() * worldForce.template cast<double>();
			const Vec3 r_world = worldPoint.template cast<double>() - o;
			const Vec3 moment_world = r_world.cross(worldForce.template cast<double>());
			const Vec3 M_link = R.transpose() * moment_world;
			// Compute spatial force in link frame
			mathlib::SpatialVec_T<Scalar> fs(
				M_link.template cast<Scalar>(), // angular slot (moment)
				F_link.template cast<Scalar>()  // linear slot  (force)
			);
			scratch.spatial.f_ext[jointIdx] += fs; // accumulate external force for this joint
		}
	}

	template<size_t NVar>
	void RigidBodySystem::step_AD(double dt, double simTime) {
		if (!hasRigidBody()) { return; }
		using Dual = mathlib::DualNumber_T<double, NVar>;
		_simTime = simTime;
		const size_t n = _body.joints.size();
		mathlib::VecX_T<Dual> x = packState_AD();

		assert((size_t)x.size() <= NVar && "State size exceeds the number of dual variables."); // Checks state vector size is within the dual variable limit
		for (size_t i = 0; i < (size_t)x.size(); ++i) { x[i].dual[i] = 1.0; }
		
		auto result = step_impl<Dual>(x, Dual(dt), Dual(simTime), *_AD_integrator, _dynScratch_AD, _dynResult_AD);
		unpackState_AD(result.stepOut.x_next);
		_dynamics->setDt(result.stepOut.dt_taken);

		auto x_real = result.stepOut.x_next.unaryExpr([](const auto& v) { return mathlib::real(v); });
		postStepUpdate(x_real, _dynScratch_AD, result);

		// Update base pose if free-floating
		if (_baseIsFree) {
			integrateBaseTranslation(dt);
			updateBaseRootPose();
		}
		// Update kinematics
		computeRigidBodyKinematics(_worldTransforms);
	}
}