/*
 * File: Systems/RigidBodySystemStep.inl
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

namespace systems {
	/*
	 * Clamp a joint angle to its limits, taking into account whether the joint is continuous or not.
	 */
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
			if (jointDOF(j.type) == 1) {
				snap.q[i] = j.q;
				snap.qd[i] = j.qd;
				snap.q_ref[i] = j.q_ref;
				snap.qd_ref[i] = j.qd_ref;
				snap.qdd_ref[i] = j.qdd_ref;
			} else {
				snap.q[i] = T(0);
				snap.qd[i] = T(0);
				snap.q_ref[i] = T(0);
				snap.qd_ref[i] = T(0);
				snap.qdd_ref[i] = T(0);
			}
			
		}

		snap.root_pose = _root_pose.template cast<T>();
		snap.baseIsFree = _baseIsFree;
		snap.lastBaseForwardForce = T(_lastBaseForwardForce);
		snap.gravity = _gravity.template cast<T>();

		snap.dt = T(_dynamics->dt());
		snap.simTime = simTime;

		return snap;
	}
	/*
	 * Step the rigid body system using a specified integrator and dynamics scratch space.
	 */
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
		int nv = 0;
		for (const auto& j : snap.model->joints) { nv += jointDOF(j.type); }

		Eigen::Map<const mathlib::VecX_T<Scalar>> q(x.data(), nv);
		Eigen::Map<const mathlib::VecX_T<Scalar>> qd(x.data() + nv, nv);

		mathlib::VecX_T<Scalar> qdd = mathlib::VecX_T<Scalar>::Zero(nv);
		int off = 0;
		for (const auto& j : _body.joints) {
			const int dof = jointDOF(j.type);
			if (dof == 1) { qdd[off] = j.qdd_ref; }
			off += dof;
		}

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
	/*
	 * Post-step update to log metrics and update internal state after a simulation step.
	 */
	template<typename T>
	void RigidBodySystem::postStepUpdate(const mathlib::VecX& x, const physics::DynamicsScratch<T>& dynScratch, const RigidBodyStepResult_T<T>& result) {
		const size_t n = result.snap.model->joints.size();
		int nv = 0;
		for (const auto& j : _body.joints) { nv += jointDOF(j.type); }
		Eigen::Map<const mathlib::VecX> q_next(x.data(), nv);
		Eigen::Map<const mathlib::VecX> qd_next(x.data() + nv, nv);

		// Recompute kinematics and dynamics at the new state for logging and control purposes
		std::vector<Pose> T_world(result.snap.model->links.size());
		_kinematics->computeForwardKinematics_fromState<double>(*result.snap.model, x, T_world);
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
		auto g = _dynamics->getGravityVec();

		for (size_t k = 0; k < _body.links.size(); ++k) {
			const RigidBodyLink& link = _body.links[k];
			const double m = link.inertial.mass;
			if (m <= 0.0) { continue; }
			mathlib::Vec3 com_world = (T_world[k].block<3, 3>(0, 0) * link.inertial.com_xyz) + T_world[k].block<3, 1>(0, 3);
			sys_PE += -m * g.dot(com_world);
		}

		const double sys_E = sys_KE + sys_PE; // total mechanical energy of the system

		if (hasFreeJoint()) {
            logFreeBodyMetrics(T_world, result, sys_PE);   // free bodies use their own log path
        } else {
            logJointMetrics(
				n, x, result,
				q_real, qd_real,
				q_ref_real, qd_ref_real,
				tau_rnea_real,
				sys_KE, sys_PE, sys_E
			);
        }
	}
	/*
	 * Log metrics for each joint to the joint log buffer
	 */
	template<typename T>
	void RigidBodySystem::logJointMetrics(
		const size_t n,
		const mathlib::VecX& x, const RigidBodyStepResult_T<T>& result,
		const mathlib::VecX& q, const mathlib::VecX& qd, 
		const mathlib::VecX& q_ref, const mathlib::VecX& qd_ref,
		const mathlib::VecX& tau_rnea,
		double sys_KE, double sys_PE, double sys_E
	) {
		// Log metrics to buffer if logging is enabled
		systems::JointLogBuffer* buf = nullptr;
		if (_useInternalLogging) { int idx = _activeLogBufIdx.load(std::memory_order_acquire); buf = &_logBuffers[idx]; }
		else { buf = _logBuffer; }

		if (buf) {
			auto dynResult = result.dynamics;
			int off = 0;
			for (size_t i = 0; i < n; ++i) {
				const RigidBodyJoint& j = _body.joints[i];
				const int dof = jointDOF(j.type);
				if (dof == 0) { continue; } // skip fixed joints
				const double I_eff = (j.type == eJointType::FIXED) ? 1.0 : mathlib::real(dynResult.metrics.I_eff[i]);
				const double err = q_ref[i] - q[off];
				const double err_d = qd_ref[i] - qd[off];
				// Log the joint metrics to the buffer
				JointLogBuffer::JointLogEntry e{};
				e.joint_name = j.name;
				e.sim_time = _simTime;
				e.dt_taken = mathlib::real(result.stepOut.dt_taken);
				e.dt_sug = mathlib::real(result.stepOut.dt_sug);
				e.theta = q[off]; e.omega = qd[off]; e.alpha = mathlib::real(dynResult.metrics.qdd[off]);
				e.err = err; e.err_d = err_d;
				e.I_eff = I_eff;
				e.tau = mathlib::real(dynResult.metrics.tau[i]); e.tau_ff = tau_rnea[i];  e.tau_gravity = mathlib::real(dynResult.metrics.tau_g[i]);
				e.tau_sat = mathlib::real(dynResult.metrics.tau_sat[i]);
				e.KE = sys_KE; e.PE = sys_PE; e.E_total = sys_E;
				e.clamp_theta = mathlib::real(_clampTheta[i]); e.clamp_omega = mathlib::real(_clampOmega[i]);
				e.sat_flag = mathlib::real(dynResult.metrics.sat_flag[i]); e.joint_index = (int)i;
				buf->push_entry(e);
			}
		}
	}
	/*
	 * Log metrics for free bodies (6-DOF joints) to the free body log buffer
	 */
	template<typename T>
	void RigidBodySystem::logFreeBodyMetrics(
		const std::vector<Pose>& T_world,
		const RigidBodyStepResult_T<T>& result,
		double sys_PE
	) {
		// pick the buffer (mirrors the joint path)
		systems::FreeBodyLogBuffer* buf = nullptr;
		if (_useInternalLogging_fb) { int idx = _activeLogBufIdx_fb.load(std::memory_order_acquire); buf = &_logBuffers_fb[idx]; }
		else { buf = _freeBodyLogBuffer; }
		if (!buf) { return; }

		auto dynResult = result.dynamics;
		const auto g = _dynamics->getGravityVec();

		// per-DOF offset table so we can pull this free joint's qdd segment
		int off = 0, bodyIdx = 0;
		for (size_t i = 0; i < _body.joints.size(); ++i) {
			const RigidBodyJoint& j = _body.joints[i];
			const int dof = jointDOF(j.type);
			if (dof != 6) { off += dof; continue; }   // only free joints logged here

			const RigidBodyLink& link = _body.links[/* child link index of j */ i];  // adjust to your link lookup
			const double m = link.inertial.mass;
			mathlib::Mat3 I;
            I << link.inertial.inertia.ixx, link.inertial.inertia.ixy, link.inertial.inertia.ixz,
                 link.inertial.inertia.ixy, link.inertial.inertia.iyy, link.inertial.inertia.iyz,
                 link.inertial.inertia.ixz, link.inertial.inertia.iyz, link.inertial.inertia.izz;

			// velocity (free_vel is [angular; linear])
			const mathlib::Vec3 w = j.free_vel.head<3>();
			const mathlib::Vec3 v = j.free_vel.tail<3>();
			// acceleration from ABA qdd (same [angular; linear] split)
			const mathlib::Vec3 aAng = mathlib::real(dynResult.metrics.qdd.segment(off, 6)).template head<3>();
			const mathlib::Vec3 aLin = mathlib::real(dynResult.metrics.qdd.segment(off, 6)).template tail<3>();

			// energies
			const double KE = 0.5 * m * v.squaredNorm() + 0.5 * w.dot(I * w);
			// PE for a single free body: -m g . com_world
			mathlib::Vec3 com_world = (T_world[i].block<3,3>(0,0) * link.inertial.com_xyz) + T_world[i].block<3,1>(0,3);
			const double PE = -m * g.dot(com_world);

			// momenta
			const mathlib::Vec3 p = m * v;
			const mathlib::Vec3 L = I * w;
			// net wrench (Newton-Euler): F = m a,  tau = I aAng + w x (I w)
			const mathlib::Vec3 F_net   = m * aLin;
			const mathlib::Vec3 tau_net = I * aAng + w.cross(I * w);

			systems::FreeBodyLogBuffer::FreeBodyLogEntry e{};
			e.body_name = j.name;
			e.sim_time = _simTime;
			e.dt_taken = mathlib::real(result.stepOut.dt_taken);
			e.dt_sug   = mathlib::real(result.stepOut.dt_sug);
			e.pos_x = j.free_pos.x(); e.pos_y = j.free_pos.y(); e.pos_z = j.free_pos.z();
			e.quat_w = j.free_qref.w(); e.quat_x = j.free_qref.x(); e.quat_y = j.free_qref.y(); e.quat_z = j.free_qref.z();
			e.linVel_x = v.x(); e.linVel_y = v.y(); e.linVel_z = v.z();
			e.angVel_x = w.x(); e.angVel_y = w.y(); e.angVel_z = w.z();
			e.linAcc_x = aLin.x(); e.linAcc_y = aLin.y(); e.linAcc_z = aLin.z();
			e.angAcc_x = aAng.x(); e.angAcc_y = aAng.y(); e.angAcc_z = aAng.z();
			e.F_net_x = F_net.x(); e.F_net_y = F_net.y(); e.F_net_z = F_net.z();
			e.tau_net_x = tau_net.x(); e.tau_net_y = tau_net.y(); e.tau_net_z = tau_net.z();
			e.KE = KE; e.PE = PE; e.E_total = KE + PE;
			e.linMom_x = p.x(); e.linMom_y = p.y(); e.linMom_z = p.z();
			e.angMom_x = L.x(); e.angMom_y = L.y(); e.angMom_z = L.z();
			e.mass = m; e.Ixx = I(0,0); e.Iyy = I(1,1); e.Izz = I(2,2);
			e.sleep_state = 0.0;
			e.body_index = bodyIdx++;
			buf->push_entry(e);

			off += dof;
		}
	}
	/*
	 * Assemble external forces into the dynamics scratch space for the current step.
	 */
	template<typename Scalar>
	void RigidBodySystem::assembleExtForces(physics::DynamicsScratch<Scalar>& scratch) const {
		// Setup external forces for each joint based on the pending external forces list
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
			mathlib::SpatialVec_T<Scalar> fs(M_link.template cast<Scalar>(), F_link.template cast<Scalar>());
			scratch.spatial.f_ext[jointIdx] += fs; // accumulate external force for this joint
		}
	}
	/*
	 * Step the rigid body system using automatic differentiation with NVar dual variables.
	 * This method is templated on the number of dual variables (NVar) to allow for compile-time optimisation and flexibility in the number of variables being differentiated.
	 */
	template<size_t NVar>
	void RigidBodySystem::step_AD(double dt, double simTime) {
		if (!hasRigidBody()) { return; }
		using Dual = mathlib::DualNumber_T<double, NVar>;
		_simTime = simTime;
		const size_t n = _body.joints.size();
		mathlib::VecX_T<Dual> x = packState_AD();
		// Apply floor contact forces if enabled (Bit crude but yeah)
		{
			constexpr double k_floor = 400000.0;
			constexpr double c_floor = 2000.0;
			const size_t nl = _body.links.size();
			if (_prevLinkY.size() != nl) { _prevLinkY.assign(nl, 0.0); }
			for (size_t i = 0; i < nl; ++i) {
				const Mat4& T = _worldTransforms[i];
				const double lowY = linkWorldMinY(i, T); 
				const double vy = (lowY - _prevLinkY[i]) / (_dynamics->dt() > 0 ? _dynamics->dt() : (1.0/180.0));
				_prevLinkY[i] = lowY;
				if (lowY < 0.0) {
					double Fy = -k_floor * lowY - c_floor * vy;
					if (Fy < 0.0) { Fy = 0.0; }                 // floor only pushes, never pulls
					setLinkExtForce(
						_body.links[i].name,
					    Vec3(T(0,3), lowY, T(2,3)),
					    Vec3(0.0, Fy, 0.0)
					);
				}
			}
		}

		assert((size_t)x.size() <= NVar && "State size exceeds the number of dual variables."); // Checks state vector size is within the dual variable limit
		for (size_t i = 0; i < (size_t)x.size(); ++i) { x[i].dual[i] = 1.0; }
		
		auto result = step_impl<Dual>(x, Dual(dt), Dual(simTime), *_AD_integrator, _dynScratch_AD, _dynResult_AD);
		unpackState_AD(result.stepOut.x_next);
		_dynamics->setDt(result.stepOut.dt_taken);

		auto x_real = result.stepOut.x_next.unaryExpr([](const auto& v) { return mathlib::real(v); });
		//postStepUpdate(x_real, _dynScratch_AD, result);

		// Update base pose if free-floating
		// if (_baseIsFree) {
		// 	integrateBaseTranslation(dt);
		// 	updateBaseRootPose();
		// }
		// Update kinematics
		computeRigidBodyKinematics(_worldTransforms);
	}
}