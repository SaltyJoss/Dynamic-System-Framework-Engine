/*
 * File: Systems/RigidBodySystem.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "Systems/RigidBodySystem.h"
#include "Systems/RigidBodyLoader.h"

#include <stack>
#include <unordered_set>
#include <algorithm>

#include <core/Utils.h>
#include "Systems/TrajectoryManager.h"
#include "Platform/Paths.h"

#include "EngineLib/LogMacros.h"
#include "Platform/DataManager.h"

using namespace mathlib;
using namespace constants;
using namespace physics;

namespace systems {
	// Helper function to convert std::vector<double> to Eigen::VectorXd
	static VecX toVecX(const std::vector<double>& a) {
		VecX v(a.size());
		for (size_t i = 0; i < a.size(); ++i) { v(i) = a[i]; }
		return v;
	}

	// Constructor
	RigidBodySystem::RigidBodySystem()
		: _integrator(std::make_unique<integration::IntegrationService>()), _curIntMethod(integration::eIntegrationMethod::RK4), 
		_AD_integrator(std::make_unique<integration::DifferentiableIntegrator>()), _curIntMethod_AD(integration::eAutoDiffIntegrationMethod::AD_ImplicitEuler),
		_kinematics(std::make_unique<physics::RigidBodyKinematics>()), _dynamics(std::make_unique<physics::RigidBodyDynamics>()),
		_torqueMode(eTorqueMode::CONTROLLED) {
		if (!_integrator ) { LOG_WARN("RigidBodySystem got null IntegrationService*"); }
	}
	// Destructor
	RigidBodySystem::~RigidBodySystem() = default;

	const systems::RigidBodyModel& RigidBodySystem::model() const { return _body; }
	std::vector<std::string> RigidBodySystem::linkNames() const {
		std::vector<std::string> names;
		for (const auto& link : _body.links) { names.push_back(link.name); }
		return names;
	}

	/*
	 * Method to compute the offset of a joint's state in the packed state vector based on its index
	 */
	int RigidBodySystem::jointStateOffset(size_t joint_idx) const {
		int off = 0;
		for (size_t i = 0; i < joint_idx; ++i) { off += jointDOF(_body.joints[i].type); }
		return off;
	}
	/*
	 * Method to compute the total degrees of freedom (DOF) of the rigidBody system based on its joints
	 */
	int RigidBodySystem::totalDOF() const {
		int nv = 0;
		for (const auto& j : _body.joints) { nv += jointDOF(j.type); }
		return nv;
	}
	/*
	 * Method to check if the rigidBody system has any free-floating joints
	 */
	bool RigidBodySystem::hasFreeJoint() const {
		for (const auto& j : _body.joints) { if (j.type == eJointType::FREE) { return true; } }
		return false;
	}
	
	// --- HELPER METHODS ---

	// Method to clamp a joint angle to its limits
	double RigidBodySystem::clampJointAngle(const RigidBodyJoint& joint, double angleRad) {
		if (joint.limits.continuous) { return wrapRad(angleRad); }
		else { return std::clamp(angleRad, joint.limits.minAngle, joint.limits.maxAngle); }
	}

	// Method to apply a soft velocity barrier to joint torque using a quadratic "wall" function (basically a softer version of a hard velocity limit)
	static void applyOmegaBarrier(double& tau, double omega, double wMax, double I_eff) {
		if (wMax <= 0.0) return;

		// Check if we're in the "soft zone" near the velocity limit
		const double absw = std::abs(omega); // [rad/s]
		const double wSoft = 0.90 * wMax;	 // [rad/s]

		if (absw <= wSoft) { return; }

		// Check if we're above the hard limit (with some tolerance)
		const double t = (absw - wSoft) / (wMax - wSoft); // [0, 1] as we go from wSoft to wMax
		const double T = 0.2; // [s], time constant for how quickly the wall ramps up
		const double wall = (I_eff / T) * (t * t) * (absw - wSoft); // [Nm]

		// Apply opposing torque to reduce |omega|
		tau -= wall * (omega >= 0.0 ? 1.0 : -1.0);
	}

	// --- ROBOT STATE INTEGRATION METHODS ---

	// Method to build a name-to-index map for rigidBody links
	void RigidBodySystem::buildLinkIndex() {
		_link_idx.clear();
		for (size_t i = 0; i < _body.links.size(); i++) {
			_link_idx[_body.links[i].name] = (int)i;
			LOG_INFO("Link %zu: %s -> index %d", i, _body.links[i].name.c_str(), (int)i);
		}
	}

	// Method to build the spatial model (kinematic tree) from the rigidBody model
	void RigidBodySystem::buildSpatialModel() {
		_spatialModel.joints.clear();
		const size_t n = _body.joints.size();
		_spatialModel.joints.resize(n);
		// Build the spatial model from the rigidBody joints
		for (size_t i = 0; i < n; ++i) {
			const RigidBodyJoint& j = _body.joints[i];
			auto& sj = _spatialModel.joints[i];
			sj.name = j.name;
			sj.type = j.type;
			// Find parent joint
			sj.parent = -1;
			for (size_t p = 0; p < n; ++p) {
				if (_body.joints[p].child == j.parent) {
					sj.parent = (int)p; break;
				}
			}
			// Build XTree
			mathlib::Mat3 R = j.origin_q.toRotationMatrix();
			mathlib::Vec3 r = j.origin_xyz;			
			sj.Xtree = mathlib::spatialTransform(R, r);
			// Build Spatial Inertia
			int childLinkIdx = -1;
			for (size_t l = 0; l < _body.links.size(); ++l) {
				if (_body.links[l].name == j.child) {
					childLinkIdx = (int)l; break;
				}
			}
			if (childLinkIdx >= 0) {
				const RigidBodyLink& link = _body.links[childLinkIdx];
				mathlib::Mat3 I_com;
				const auto& I = link.inertial.inertia;
				I_com <<
					I.ixx, I.ixy, I.ixz,
					I.ixy, I.iyy, I.iyz,
					I.ixz, I.iyz, I.izz;
				sj.inertia = mathlib::spatialInertia(
					link.inertial.mass,
					link.inertial.com_xyz,
					I_com
				);
			}
			// Build S vector (motion subspace)
			switch (j.type) {
				case eJointType::REVOLUTE:
					sj.S = mathlib::SpatialVec(j.axis.normalized(), mathlib::Vec3::Zero());
					sj.nfDOF = 1;
					break;
				case eJointType::PRISMATIC:
					sj.S = mathlib::SpatialVec(mathlib::Vec3::Zero(), j.axis.normalized());
					sj.nfDOF = 1;
					break;
				case eJointType::FREE:
					sj.S = mathlib::SpatialVec();
					sj.nfDOF = 6;
					break;
				default:
					sj.S = mathlib::SpatialVec();
					sj.nfDOF = 0;
					break;
			}
			sj.free_qref = j.free_qref; // Store the free joint reference orientation
		}
		LOG_INFO("SpatialModel built: joints=%d", (long long)_spatialModel.joints.size());
	}

	// Method to pack rigidBody joint states into a state vector
	mathlib::VecX RigidBodySystem::packState() const {
		int nv = 0;
		// Count total DOF
		for (const auto& j : _body.joints) { nv += jointDOF(j.type); }
		mathlib::VecX x(2 * nv);
		int off = 0;
		// Pack angles and velocities
		for (const auto& j : _body.joints) {
			const int dof = jointDOF(j.type);
			if (dof == 0) { continue; } // Skip fixed joints
			if (dof == 1) { // Revolute or Prismatic joint
				x[off] = j.q;
				x[off + nv] = j.qd;
			}
			else {
				x.segment(off, 3) = j.free_rot_v;	 // Euler angles
				x.segment(off + 3, 3) = j.free_pos;	 // Position
				x.segment(nv + off, 6) = j.free_vel; // linear + angular velocity
			}
			off += dof; // increment offset by the DOF of the joint
		}
		return x;
	}
	
	// Method to unpack state vector into rigidBody joints
	void RigidBodySystem::unpackState(const mathlib::VecX& x) {
		const size_t n = static_cast<int>(_body.joints.size());
		int nv = 0;
		for (const auto& j : _body.joints) { nv += jointDOF(j.type); }

		// Resize clamping vectors if necessary
		if (_clampTheta.size() != n) { _clampTheta.assign(n, 0); }
		if (_clampOmega.size() != n) { _clampOmega.assign(n, 0); }

		int off = 0;
		for (size_t i = 0; i < n; ++i) {
			auto& j = _body.joints[i];
			const int dof = jointDOF(j.type);
			if (dof == 0) { _clampTheta[i] = 0; _clampOmega[i] = 0; continue; } // Skip fixed joints

			if (dof == 1) { // Revolute or Prismatic joint
				// Current states
				double theta_in = x[off];		  // [rad]
				double omega_in = x[off + nv];	  // [rad/s]
				double theta_out = clampJointAngle(j, theta_in); // [rad], clamped to joint limits
				double wMax_hw = std::abs(j.limits.maxqd); // [rad/s], max |omega| for this joint
				double omega_out = omega_in; // [rad/s], will be clamped if necessary

				// Velocity limit clamping
				if (wMax_hw > 0.0f) {
					const double eps = 0.05f;
					if (std::abs(omega_in) > (1.0f + eps) * wMax_hw) {
						omega_out = std::clamp(omega_in, -wMax_hw, wMax_hw);
					}
				}

				// Velocity limit enforcement
				if (theta_out != theta_in) {
					const double upperLimit = j.limits.maxAngle;
					const double lowerLimit = j.limits.minAngle;
					if (theta_out >= upperLimit && omega_in > 0.0f) { omega_out = 0.0f; }
					if (theta_out <= lowerLimit && omega_in < 0.0f) { omega_out = 0.0f; }
				}

				// Record clamping
				_clampTheta[i] = (theta_in != theta_out) ? 1 : 0;
				_clampOmega[i] = (omega_in != omega_out) ? 1 : 0;
				// Update joint states
				j.q = theta_out;
				j.qd = omega_out;
			}
			else {
				mathlib::Vec3 rot_v = x.segment(off, 3); // Euler angles
				j.free_pos = x.segment(off + 3, 3);		 // Position
				j.free_vel = x.segment(nv + off, 6);	 // linear + angular velocity
				j.free_qref = (j.free_qref * expToQuat(rot_v)).normalized(); // Update quaternion based on Euler angles
				j.free_rot_v = mathlib::Vec3::Zero(); // Reset Euler angles to zero after conversion
				_clampTheta[i] = 0;
				_clampOmega[i] = 0; // No clamping for free joints
			}
			off += dof; // increment offset by the DOF of the joint
		}
	}

	// Method to pack rigidBody joint states into a state vector
	mathlib::VecX_T<DualNumber_T<double, 14>> RigidBodySystem::packState_AD() const {
		using Dual = DualNumber_T<double, 14>; // only hardcoded since I am testing the same arm, TODO provide a better final way to derive the NVar val.
		const size_t n = static_cast<int>(_body.joints.size());
		mathlib::VecX_T<Dual> x(2 * n);

		// Pack angles and velocities
		for (size_t i = 0; i < n; ++i) {
			auto& j = _body.joints[i];

			// Current states
			x[i] = Dual(j.q, { 0.0 });
			x[i + n] = Dual(j.qd, { 0.0 });
		}
		return x; // state vector
	}

	// Method to unpack state vector into rigidBody joints
	void RigidBodySystem::unpackState_AD(const mathlib::VecX_T<DualNumber_T<double, 14>>& x) {
		using Dual = DualNumber_T<double, 14>;
		const size_t n = static_cast<int>(_body.joints.size());

		// Resize clamping vectors if necessary
		if (_clampTheta.size() != n) { _clampTheta.assign(n, 0); }
		if (_clampOmega.size() != n) { _clampOmega.assign(n, 0); }

		// For each joint
		for (size_t i = 0; i < n; ++i) {
			auto& j = _body.joints[i];

			// Current states
			Dual theta_in = x[i];		  // [rad]
			Dual omega_in = x[i + n];	  // [rad/s]

			// Clamp joint angle
			Dual theta_out = clampJointAngle_T<Dual>(j, theta_in);

			// max |omega|
			const double wMax_hw = mathlib::abs(j.limits.maxqd);
			Dual omega_out = omega_in;

			// Velocity limit clamping
			if (wMax_hw > 0.0) {
				omega_out = wMax_hw * mathlib::tanh(omega_in / wMax_hw); // smoothly clamp omega to wMax_hw using a tanh function
			}

			// Velocity limit enforcement
			if (theta_out != theta_in) {
				const double upperLimit = j.limits.maxAngle;
				const double lowerLimit = j.limits.minAngle;
				if (theta_out >= upperLimit && omega_in > Dual(0)) { omega_out = Dual(0); }
				if (theta_out <= lowerLimit && omega_in < Dual(0)) { omega_out = Dual(0); }
			}

			// Record clamping
			_clampTheta[i] = (theta_in != theta_out) ? 1 : 0;
			_clampOmega[i] = (omega_in != omega_out) ? 1 : 0;
			// Update joint states
			j.q = mathlib::real(theta_out);
			j.qd = mathlib::real(omega_out);
		}
	}

	// Method to pack reference state vector (target angles and velocities) for control
	mathlib::VecX RigidBodySystem::packRefState() const {
		const size_t n = (int)_body.joints.size();
		mathlib::VecX x(2 * n);
		for (size_t i = 0; i < n; ++i) {
			auto& j = _body.joints[i];

			// Pack reference angles and velocities
			x[i] = j.q_ref;
			x[i + n] = j.qd_ref;
		}
		return x; // reference state vector
	}

	// Method to unpack reference state vector into rigidBody joints
	void RigidBodySystem::unpackRefState(const mathlib::VecX& x) {
		const size_t n = (int)_body.joints.size();
		for (size_t i = 0; i < n; ++i) {
			auto& j = _body.joints[i];
			j.q_ref = x[i];					   // [rad]
			j.qd_ref = x[i + n];				   // [rad/s]
			j.q_ref = clampJointAngle(j, j.q_ref); // [rad]
		}
	}

	// Method to enforce joint limits after integration
	void RigidBodySystem::enforceJointLimits(RigidBodyJoint& j) {
		if (j.limits.continuous) { return; }

		const double lo = j.limits.minAngle;
		const double hi = j.limits.maxAngle;

		if (j.q < lo) { j.q = lo; if (j.qd < 0.0f) { j.qd = 0.0f; }}
		if (j.q > hi) { j.q = hi; if (j.qd > 0.0f) { j.qd = 0.0f; }}
	}

	double RigidBodySystem::linkWorldMinY(size_t linkIdx, const Mat4& T) const {
		const RigidBodyLink& L = _body.links[linkIdx];
		if (!L.hasBounds) { return T(1,3); }   // no geometry -> fall back to origin
		double minY = 1e30;
		for (int c = 0; c < 8; ++c) {
			const double x = (c & 1) ? L.aabbMax.x() : L.aabbMin.x();
			const double y = (c & 2) ? L.aabbMax.y() : L.aabbMin.y();
			const double z = (c & 4) ? L.aabbMax.z() : L.aabbMin.z();
			const double wy = T(1,0)*x + T(1,1)*y + T(1,2)*z + T(1,3);
			if (wy < minY) { minY = wy; }
		}
		return minY;
	}

	// Method to advance the rigidBody state by dt using the selected integrator
	void RigidBodySystem::step(double dt, double simTime) {
		if (!_hasBody) { return; }

		bool hasFree = false;
		for (const auto& j : _body.joints) {
			if (j.type == eJointType::FREE) {
				hasFree = true; break;
			}
		}

		if (_useAutoDiff && !hasFree) {
			step_AD<AD_VARS>(dt, simTime);
			return;
		}

		_simTime = simTime;
		const size_t n = _body.joints.size();
		mathlib::VecX x = packState();
		
		assembleExtForces(_dynScratch);
		auto result = step_impl<double>(x, dt, simTime, *_integrator, _dynScratch, _dynResult);
		clearExtForces();

		unpackState(result.stepOut.x_next);
		_dynamics->setDt(result.stepOut.dt_taken);

		const auto scratchCopy = _dynScratch;
		const auto resultCopy = result;

		postStepUpdate(resultCopy.stepOut.x_next, scratchCopy, resultCopy);

		// Update base pose if free-floating
		// if (_baseIsFree) {
		// 	integrateBaseTranslation(dt);
		// 	updateBaseRootPose();
		// }
		// Update kinematics
		computeRigidBodyKinematics(_worldTransforms);
	}

	// Method to step the reference trajectory and update joint reference states
	void RigidBodySystem::updateTrajectoryInputs(control::TrajectoryManager& traj, double t) {
		if (!_hasBody) { return; }
		
		const size_t n = _body.joints.size();
		if (n <= 0) { return; }

		// Sample trajectories ("ground truth" inputs)
		for (size_t i = 0; i < n; ++i) {
			RigidBodyJoint& j = _body.joints[i];
			control::TrajState<double> s{};
			// Try to evaluate trajectory
			if (traj.tryEval(std::string(j.child), t, s)) {
				j.q_ref    = clampJointAngle(j, s.q); // set ref angle
				j.qd_ref  = s.qd;
				j.qdd_ref = s.qdd;
			}
			// Store inputs
			else {
				j.qdd_ref = 0.0f;
				j.qd_ref = 0.0f;
			}

			auto* buf = _refBuffer;
			if (buf) {
				// Sim Metadata
				buf->sim_time.push_back(t);
				// Reference states
				buf->theta_ref.push_back(j.q_ref);
				buf->omega_ref.push_back(j.qd_ref);
				buf->alpha_ref.push_back(j.qdd_ref);
				// Joint Index
				buf->joint_index.push_back((int)i);
			}
		}
	}

	// --- ROBOT LOADING AND RESET METHODS ---

	// Method to load a rigidBody model by name
	void RigidBodySystem::loadRigidBody(const std::string& name) {
		if (name == _loadedName) {
			LOG_INFO("RigidBody '%s' is already loaded, skipping load.", name.c_str());
			D_INFO("RigidBody '%s' is already loaded, skipping load.", name.c_str());
			return;
		}

		// Reset control parameters to target values so that if the new rigidBody has different defaults, we start with those
		resetNaturalFrequencyToTarget();
		resetDampingRatioToTarget();

		// Construct path to rigidBody JSON file
		const std::filesystem::path bodyPath = paths::assets() / name;
		if (!std::filesystem::exists(bodyPath)) {
			LOG_ERROR("RigidBody file not found -> %s", bodyPath.string().c_str());
			D_ERROR("RigidBody file not found -> %s", bodyPath.string().c_str());
			return;
		}

		// Load rigidBody model from JSON
		_body = systems::RigidBodyLoader::load(bodyPath.string());
		const size_t n = _body.joints.size();
		const size_t m = _body.links.size();

		_constModel.name = _body.name;
		_constModel.scale = _body.scale;

		_constModel.baseFrame = _body.baseFrame;
		_constModel.baseFrameIsAligned = _body.baseFrameIsEngineAligned;

		_constModel.links = _body.links;
		_constModel.joints = _body.joints;

		_constModel.linkNameToIndex.clear();
		for (size_t i = 0; i < _constModel.links.size(); ++i) {
			_constModel.linkNameToIndex[_constModel.links[i].name] = (int)i;
		}

		LOG_INFO_ONCE(
			"CONST MODEL: links=%lld joints=%lld",
			(long long)_constModel.links.size(),
			(long long)_constModel.joints.size()
		);

		_loadedName = name;
		_baseIsFree = false;

		// Check if any joint is free-floating to determine if the base is free
		for (const auto& joint : _body.joints) {
			if (joint.type == eJointType::FREE) {
				_baseIsFree = true;
				break;
			}
		}

		_root_home = _body.baseFrame;
		_root_pose = _root_home;

		_q_home = _body.makeJointVector();
		_home_valid = true;

		buildLinkIndex();
		buildSpatialModel();
		_hasBody = true;

		resetRigidBody();

		LOG_INFO("Loaded RigidBody model -> %s", name.c_str());
		D_SUCCESS("Loaded RigidBody model -> %s", name.c_str());
	}

	// Method to reset the rigidBody to its home position
	void RigidBodySystem::resetRigidBody() {
		if (!_hasBody || !_home_valid) { LOG_ERROR("Reset aborterd."); return; }
		_root_pose = _root_home;
		_body.setJointVector(_q_home);

		for (auto& joint : _body.joints) {
			joint.qd = 0.0;
			joint.q_ref = joint.q;
			joint.qd_ref = 0.0;
			joint.qdd_ref = 0.0;
		}

		// Reset base state if free-floating
		_basePos = Vec3(0, 0, 0);
		_baseVel = Vec3(0, 0, 0);
		_baseAcc = Vec3(0, 0, 0);

		// Assuming base orientation is represented as a yaw angle for simplicity
		_baseYaw = 0.0;
		_baseYawRate = 0.0;
		_baseYawAcc = 0.0;

		_dynScratch.clear();
		_dynScratch_AD.clear();

		_dynResult.resize(0);
		_dynResult_AD.resize(0);

		_dynScratch.resize(_body.joints.size(), _body.links.size());
		_dynScratch_AD.resize(_body.joints.size(), _body.links.size());
		_dynResult.resize(_body.joints.size());
		_dynResult_AD.resize(_body.joints.size());

		_dynScratch.spatial.g = mathlib::VecX::Zero(6);
		_dynScratch.spatial.g.segment<3>(3) = _gravity;

		_dynScratch_AD.spatial.g = mathlib::VecX_T<DualNumber_T<double, 14>>::Zero(6);
		_dynScratch_AD.spatial.g.segment<3>(3) = _gravity.template cast<DualNumber_T<double, 14>>();

		// Reset adaptive integrator so it doesn't carry a stale step size
		_integrator->resetAdaptiveState();

		computeRigidBodyKinematics(_worldTransforms);
		D_INFO("RigidBody reset to home position.");
		D_SUCCESS("RigidBody reset to home position.");
	}

	void RigidBodySystem::stopAll() {
		if (!_hasBody) return;
		for (auto& joint : _body.joints) {
			joint.qd = 0.0f;
			joint.q_ref = joint.q;
		}
	}

	// RigidBodySystem accesor for to get the latest free body log entry for a specific body index
	bool RigidBodySystem::latestFreeBodyEntry(FreeBodyLogBuffer::FreeBodyLogEntry& out, int bodyIdx) const {
		const FreeBodyLogBuffer* buf = nullptr;
		if (_useInternalLogging_fb) { int idx = _activeLogBufIdx.load(std::memory_order_acquire); buf = &_logBuffers_fb[idx]; }
		else { buf = _freeBodyLogBuffer; }
		if (!buf || buf->size() == 0) { return false; }
		for (size_t k = buf->size(); k-- > 0; ) {
			if (buf->body_index[k] == bodyIdx) {
				out.sim_time = buf->sim_time[k]; out.dt_taken = buf->dt_taken[k]; out.dt_sug = buf->dt_sug[k];
				out.pos_x = buf->pos_x[k]; out.pos_y = buf->pos_y[k]; out.pos_z = buf->pos_z[k];
				out.quat_w = buf->quat_w[k]; out.quat_x = buf->quat_x[k]; out.quat_y = buf->quat_y[k]; out.quat_z = buf->quat_z[k];
				out.linVel_x = buf->linVel_x[k]; out.linVel_y = buf->linVel_y[k]; out.linVel_z = buf->linVel_z[k];
				out.angVel_x = buf->angVel_x[k]; out.angVel_y = buf->angVel_y[k]; out.angVel_z = buf->angVel_z[k];
				out.linAcc_x = buf->linAcc_x[k]; out.linAcc_y = buf->linAcc_y[k]; out.linAcc_z = buf->linAcc_z[k];
				out.angAcc_x = buf->angAcc_x[k]; out.angAcc_y = buf->angAcc_y[k]; out.angAcc_z = buf->angAcc_z[k];
				out.F_net_x = buf->F_net_x[k]; out.F_net_y = buf->F_net_y[k]; out.F_net_z = buf->F_net_z[k];
				out.tau_net_x = buf->tau_net_x[k]; out.tau_net_y = buf->tau_net_y[k]; out.tau_net_z = buf->tau_net_z[k];
				out.KE = buf->KE[k]; out.PE = buf->PE[k]; out.E_total = buf->E_total[k];
				out.linMom_x = buf->linMom_x[k]; out.linMom_y = buf->linMom_y[k]; out.linMom_z = buf->linMom_z[k];
				out.angMom_x = buf->angMom_x[k]; out.angMom_y = buf->angMom_y[k]; out.angMom_z = buf->angMom_z[k];
				out.mass = buf->mass[k]; out.Ixx = buf->Ixx[k]; out.Iyy = buf->Iyy[k]; out.Izz = buf->Izz[k];
				out.sleep_state = buf->sleep_state[k]; out.body_index = buf->body_index[k];
				return true;
			}
		}
		return false;
	}

	integration::IntegrationService* RigidBodySystem::getIntegrator() { return _integrator.get(); }
	const integration::IntegrationService* RigidBodySystem::getIntegrator() const { return _integrator.get(); }
	void RigidBodySystem::setStandardIntegrator(integration::eIntegrationMethod m) {
		_curIntMethod = m; _integrator->setIntegrationMethod(m);
	}

	integration::DifferentiableIntegrator* RigidBodySystem::getADIntegrator() { return _AD_integrator.get(); }
	const integration::DifferentiableIntegrator* RigidBodySystem::getADIntegrator() const { return _AD_integrator.get(); }
	void RigidBodySystem::setADIntegrator(integration::eAutoDiffIntegrationMethod m) {
		_curIntMethod_AD = m; _AD_integrator->setIntegrationMethod(m);
	}

	std::shared_ptr<integration::IntegratorState> RigidBodySystem::runtimeIntegratorState() {
		return _useAutoDiff ? _AD_integrator->runtimeState() : _integrator->runtimeState();
	}

	std::shared_ptr<const integration::IntegratorState> RigidBodySystem::runtimeIntegratorState() const {
		return _useAutoDiff ? _AD_integrator->runtimeState() : _integrator->runtimeState();
	}

	// --- ROBOT KINEMATICS AND JOINT STATE METHODS ---

	std::string RigidBodySystem::findRootLink() const {
		std::unordered_set<std::string> children;
		for (const auto& joint : _body.joints) { children.insert(joint.child); }
		for (const auto& link : _body.links) {
			if (children.find(link.name) == children.end()) {
				return link.name;
			}
		}
		return _body.links.empty() ? "" : _body.links.front().name; // fallback
	}

	// Method to get the world origin of a specific rigidBody link by name
	bool RigidBodySystem::linkWorldOrigin(const std::string& linkName, mathlib::Vec3& outOrigin) const {
		auto it = _link_idx.find(linkName);
		if (it == _link_idx.end()) { return false; }
		outOrigin = _worldTransforms[it->second].block<3,1>(0,3);
		return true;
	}
	// Method to apply an external force to a specific rigidBody link at a given world point
	bool RigidBodySystem::setLinkExtForce(const std::string& linkName, const mathlib::Vec3& worldPoint, const mathlib::Vec3& worldForce) {
		if (!_hasBody) { return false; }
		auto it = _link_idx.find(linkName);
		if (it == _link_idx.end()) { return false; }
		const int linkIdx = it->second;

		// Find the joint whose child is this link — that's the body ABA indexes.
		int jointIdx = -1;
		for (size_t j = 0; j < _body.joints.size(); ++j) {
			auto cit = _link_idx.find(_body.joints[j].child);
			if (cit != _link_idx.end() && cit->second == linkIdx) { jointIdx = (int)j; break; }
		}
		if (jointIdx < 0) { return false; }   // root/base link: no governing joint (see note)

		_pendingExtForces.emplace_back(jointIdx, linkIdx, worldPoint, worldForce);
		return true;
	}
	// Overload to apply an external force to a specific rigidBody link at its world origin
	bool RigidBodySystem::setLinkExtForce(const std::string& linkName, const mathlib::Vec3& worldForce) {
		mathlib::Vec3 o;
		if (!linkWorldOrigin(linkName, o)) { return false; }
		return setLinkExtForce(linkName, o, worldForce);
	}
	// Method to clear all pending external forces applied to rigidBody links
	void RigidBodySystem::clearExtForces() { _pendingExtForces.clear(); }

	// Method to update the pose of each rigidBody link based on current joint angles using forward kinematics
	void RigidBodySystem::computeRigidBodyKinematics(std::vector<Mat4>& world) {
		if (!_hasBody) {
			world.clear();
			return;
		};

		world.resize(_body.links.size());
		for (auto& T : world) { T = Mat4::Identity(); }

		// Find root link
		const std::string rootName = findRootLink();
		auto itRoot = _link_idx.find(rootName);
		if (itRoot == _link_idx.end()) {
			LOG_WARN_ONCE("RigidBodySystem::updateRigidBodyKinematics: root link '%s' not found in link index", rootName.c_str());
			return;
		}

		// Set root link pose
		int rootIdx = itRoot->second;
		world[rootIdx] = _root_pose;

		for (const auto& j : _body.joints) {
			if (j.type == eJointType::FREE) {
				mathlib::Quat q_full = (j.free_qref * expToQuat(j.free_rot_v)).normalized();
				Mat4 T = Mat4::Identity();
				T.block<3, 3>(0, 0) = q_full.toRotationMatrix(); // set rotation to free_qref * exp(free_rot_v)
				T.block<3, 1>(0, 3) = j.free_pos; // set translation to free_pos
				auto itC = _link_idx.find(j.child); // find child link index
				if (itC != _link_idx.end()) { world[itC->second] = T; }
			}
		}

		// parent -> children joints
		std::unordered_map<std::string, std::vector<const RigidBodyJoint*>> children;
		children.reserve(_body.joints.size());
		for (const auto& j : _body.joints) children[j.parent].push_back(&j);

		std::stack<std::string> st;
		st.push(rootName);

		// Traverse the kinematic tree using DFS
		while (!st.empty()) {
			std::string parentName = st.top(); 
			st.pop();

			// Skip if parent link not found
			auto itP = _link_idx.find(parentName);
			if (itP == _link_idx.end()) { continue; }
			int pIdx = itP->second;

			const Mat4& T_parent = world[pIdx];
			// Find children joints
			auto it = children.find(parentName);
			if (it == children.end()) { continue; }

			// For each child joint
			for (const RigidBodyJoint* jp : it->second) {
				const RigidBodyJoint& j = *jp;
				auto itC = _link_idx.find(j.child);
				if (itC == _link_idx.end()) { continue; }
				int cIdx = itC->second;

				// Joint origin transform
				Mat4 T_joint = Mat4::Identity();
				T_joint.block<3, 1>(0, 3) = j.origin_xyz;

				// Joint origin rotation
				Mat4 R_joint = Mat4::Identity();
				R_joint.block<3, 3>(0, 0) = j.origin_q.toRotationMatrix();

				// Compute child link pose in world frame
				Mat4 T_child = T_parent * T_joint * R_joint;

				Vec3 axis = j.axis.norm() > 1e-8 ? j.axis.normalized() : Vec3(0, 0, 1); // default axis if zero

				// Apply joint rotation for revolute joints
				if (j.type == eJointType::REVOLUTE) {
					Mat4 R_q = Mat4::Identity();
					R_q.block<3, 3>(0, 0) = Eigen::AngleAxisd(j.q, axis).toRotationMatrix();
					T_child = T_child * R_q;
				}
				else if (j.type == eJointType::PRISMATIC) {
					Mat4 T_q = Mat4::Identity();
					T_q.block<3, 1>(0, 3) = axis * j.q; // translate along joint axis by q
					T_child = T_child * T_q;
				}

				// FIXED joints: no motion
				world[cIdx] = T_child;
				st.push(j.child);
			}
		}
	}

	// --- JOINT STATE GETTERS AND SETTERS ---

	// Method to get the angle of a specific rigidBody joint
	bool RigidBodySystem::tryGetJointAngleRad(const std::string& childLink, double& outAngle) const {
		if (!_hasBody) { return false; }
		if (hasFreeJoint()) { return false; }
		// Find joint child matching childLink
		for (const auto& joint : _body.joints) {
			if (joint.child == childLink) {
				outAngle = joint.q;
				return true;
			}
		}
		return false;
	}

	// Method to set the angle of a specific rigidBody joint
	bool RigidBodySystem::trySetJointAngleRad(const std::string& childLink, double angleRad) {
		if (!_hasBody) { return false; }
		if (hasFreeJoint()) { return false; }
		// Find joint child matching childLink
		for (auto& joint : _body.joints) {
			if (joint.child == childLink) {
				joint.q = clampJointAngle(joint, angleRad); // clamp to joint limits
				return true;
			}
		}
		return false;
	}

	/*
	 * Method to get the free-floating joint velocity of a specific rigidBody joint
	 * @param linkName: The name of the link associated with the free joint
	 */
	bool RigidBodySystem::tryGetFreeVelocity(const std::string& linkName, mathlib::VecX& outVel) const {
		if (!_hasBody) { return false; }
		for (const auto& joint : _body.joints) {
			if (joint.type == eJointType::FREE && joint.child == linkName) {
				outVel = joint.free_vel;
				return true;
			}
		}
		return false;
	}

	/*
	 * Method to set the free-floating joint velocity of a specific rigidBody joint
	 * @param linkName: The name of the link associated with the free joint
	 */
	bool RigidBodySystem::trySetFreeVelocity(const std::string& linkName, const mathlib::VecX& vel) {
		if (!_hasBody) { return false; }
		for (auto& j : _body.joints) {
			if (j.type == eJointType::FREE && j.child == linkName) {
				j.free_vel = vel.head<6>();
				LOG_INFO("Set '%s' free_vel=[%.3f %.3f %.3f %.3f %.3f %.3f]", linkName.c_str(), vel(0),vel(1),vel(2),vel(3),vel(4),vel(5));
				return true;
			}
			LOG_INFO("trySetFreeVelocity: joint '%s' is not free or does not match linkName='%s'", j.child.c_str(), linkName.c_str());
		}
		LOG_WARN("No free joint for link '%s'", linkName.c_str());
		return false;
	}

	// Method to get the angular velocity of a specific rigidBody joint
	bool RigidBodySystem::tryGetJointOmegaRad(const std::string& childLink, double& outOmega) const {
		if (!_hasBody) { return false; }
		// Find joint child matching childLink
		for (const auto& joint : _body.joints) {
			if (joint.child == childLink) {
				outOmega = joint.qd;
				return true;
			}
		}
		return false;
	}

	// Method to set the angular velocity of a specific rigidBody joint
	bool RigidBodySystem::trySetJointOmegaRad(const std::string& childLink, double omegaRad) {
		if (!_hasBody) { return false; }
		// Find joint child matching childLink
		for (auto& joint : _body.joints) {
			if (joint.child == childLink) {
				joint.qd = omegaRad;
				return true;
			}
		}
		return false;
	}

	// Method to directly inject an angular velocity into the state vector for a specific rigidBody joint (bypassing any clamping or limits)
	bool RigidBodySystem::injectJointOmegaRad(const std::string& childLink, double omega) {
		if (!_hasBody) return false;
		const size_t n = _body.joints.size();
		// Find joint child matching childLink
		for (size_t i = 0; i < n; ++i) {
			if (_body.joints[i].child == childLink) {
				// Modify actual state vector
				mathlib::VecX x = packState();
				x[i + n] = omega;  // velocity slot
				unpackState(x);
				return true;
			}
		}
		return false;
	}

	// Method to get the target angle (reference) of a specific rigidBody joint in radians
	bool RigidBodySystem::tryGetJointTargetRad(const std::string& childLink, double& outTargetRad) const {
		if (!_hasBody) { return false; }
		for (const auto& joint : _body.joints) {
			if (joint.child == childLink) {
				outTargetRad = joint.q_ref;
				return true;
			}
		}
		return false;
	}

	// Method to set the target angle (reference) of a specific rigidBody joint in radians
	bool RigidBodySystem::trySetJointTargetRad(const std::string& childLink, double targetRad) {
		if (!_hasBody) { return false; }
		for (auto& joint : _body.joints) {
			if (joint.child == childLink) {
				if (joint.limits.continuous) { joint.q_ref = wrapRad(targetRad); }
				else { joint.q_ref = clampJointAngle(joint, targetRad); } // clamp to joint 
				return true;
			}
		}
		return false;
	}

	//	Method to get the maximum angular velocity of a specific rigidBody joint in radians
	bool RigidBodySystem::tryGetJointOmegaMaxRad(const std::string& childLink, double& maxOmegaRad) const {
		if (!_hasBody) { return false; }
		for (const auto& joint : _body.joints) {
			if (joint.child == childLink) {
				maxOmegaRad = joint.limits.maxqd;
				return true;
			}
		}
		return false;
	}

	//	Method to set the maximum angular velocity of a specific rigidBody joint in radians
	bool RigidBodySystem::trySetJointOmegaMaxRad(const std::string& childLink, double maxOmegaRad) {
		if (!_hasBody) { return false; }
		if (maxOmegaRad <= 0.0f) { return false; }
		for (auto& joint : _body.joints) {
			if (joint.child == childLink) {
				joint.limits.maxqd = maxOmegaRad;
				return true;
			}
		}
		return false;
	}

	// Method to increment the target angle (reference) of a specific rigidBody joint in radians
	bool RigidBodySystem::tryAddJointTargetRad(const std::string& childLink, double deltaRad) {
		if (!_hasBody) { return false; }
		for (auto& joint : _body.joints) {
			if (joint.child == childLink) {
				double t = joint.q_ref + deltaRad;
				if (joint.limits.continuous) { t = wrapRad(t); }
				else { t = std::clamp(t, joint.limits.minAngle, joint.limits.maxAngle); }
				joint.q_ref = t; // clamp to joint limits
				return true;
			}
		}
		return false;
	}

	// Method to set the reference angular velocity of a specific rigidBody joint in radians
	bool RigidBodySystem::trySetJointOmegaRefRad(const std::string& childLink, double omegaRefRad) {
		if (!_hasBody) { return false; }
		for (auto& joint : _body.joints) {
			if (joint.child == childLink) {
				joint.qd_ref = omegaRefRad;
				return true;
			}
		}
		return false;
	}

	// Method to set the reference angular acceleration of a specific rigidBody joint in radians
	bool RigidBodySystem::trySetJointAlphaRefRad(const std::string& childLink, double alphaRefRad) {
		if (!_hasBody) { return false; }
		for (auto& joint : _body.joints) {
			if (joint.child == childLink) {
				joint.qdd_ref = alphaRefRad;
				return true;
			}
		}
		return false;
	}

	// Method to set the max Omega reference of a specific rigidBody joint in radians
	bool RigidBodySystem::trySetJointOmegaRefMaxRad(const std::string& childLink, double maxOmegaRad) {
		if (!_hasBody) { return false; }
		if (maxOmegaRad <= 0.0f) { return false; }
		for (auto& joint : _body.joints) {
			if (joint.child == childLink) {
				joint.limits.omegaRefMaxRad_s = maxOmegaRad;
				return true;
			}
		}
		return false;
	}

	// Method to zero the reference derivatives (velocity and acceleration) of a specific rigidBody joint
	bool RigidBodySystem::tryZeroJointRefDerivatives() {
		if (!_hasBody) { return false; }
		for (auto& joint : _body.joints) {
			joint.qd_ref = 0.0f;
			joint.qdd_ref = 0.0f;
		}
		return true;
	}

	// Method to check if a specific rigidBody joint is at its target angle within a tolerance (radians)
	bool RigidBodySystem::isJointAtTargetRad(const std::string& childLink, double tolRad) const {
		if (!_hasBody) { return false; }
		if (tolRad < 0.0f) { tolRad = -tolRad; }

		for (const auto& joint : _body.joints) {
			if (joint.child == childLink) {
				double err = joint.q_ref - joint.q;
				if (joint.limits.continuous) { err = wrapToPi(err); }
				err = std::abs(err);
				return err <= tolRad;
			}
		}
		return false;
	}

	// Method to check if a specific rigidBody joint is at its target angle within a tolerance (degrees)
	bool RigidBodySystem::isJointAtTargetDeg(const std::string& childLink, double tolDeg) const { 
		return isJointAtTargetRad(childLink, radians(tolDeg)); 
	}

	// Method to check if a specific rigidBody joint is near a target angle within a tolerance (radians)
	bool RigidBodySystem::isJointNearAngleRad(const std::string& childLink, double targetRad, double tolRad) const {
		if (!_hasBody) { return false; }
		tolRad = std::abs(tolRad);

		for (const auto& joint : _body.joints) {
			if (joint.child == childLink) {
				double err = targetRad - joint.q;
				if (joint.limits.continuous) { err = wrapToPi(err); }
				return std::abs(err) <= tolRad;
			}
		}
		return false;
	}

	// Method to check if a specific rigidBody joint is near a target angle within a tolerance (degrees)
	bool RigidBodySystem::isJointNearAngleDeg(const std::string& childLink, double targetDeg, double tolDeg) const { 
		return isJointNearAngleRad(childLink, radians(targetDeg), radians(tolDeg));
	}

	// --- ROBOT LINK AND ROOT POSE METHODS ---

	// Method to set the rotation angle of a specific rigidBody link angle in degrees
	bool RigidBodySystem::setRigidBodyLinkRotation(const std::string& childLinkName, double angleDeg) {
		for (auto& j : _body.joints) {
			if (j.child == childLinkName) {
				j.q = radians(angleDeg);
				computeRigidBodyKinematics(_worldTransforms);
				return true;
			}
		}
		return false;
	}

	Mat4 RigidBodySystem::setRigidBodyRoot(const Vec3& pos, const Quat& rot) {
		Mat4 T = Mat4::Identity();
		T.block<3, 1>(0, 3) = pos;
		Mat4 R = Mat4::Identity();
		R.block<3, 3>(0, 0) = rot.toRotationMatrix();
		return T * R;
	}

	// Method to set the rigidBody root pose in world coordinates
	void RigidBodySystem::setRigidBodyRootPose(const Vec3& pos, const Quat& rot) {
		_root_pose = setRigidBodyRoot(pos, rot);
	}

	// Method to set the rigidBody root home pose in world coordinates
	void RigidBodySystem::setRigidBodyRootHome(const Vec3& pos, const Quat& rot) {
		_root_home = setRigidBodyRoot(pos, rot);;
		_root_pose = _root_home;
	}

	// Method to set the default pose of the rigidBody using joint angles in degrees
	bool RigidBodySystem::setDefaultPoseDeg() {
		if (!_hasBody) { return false; }
		_q_home = _body.makeJointVector();
		_home_valid = true;
		return true;
	}

	// --- ROBOT BASE INTEGRATION METHODS ---

	// Method to set the default pose of the rigidBody using joint angles in radians
	double RigidBodySystem::computeForwardDrive() const {
		double drive = 0.0;
		for (const auto& j : _body.joints) {
			if (j.name.find("hip_pitch") != std::string::npos) { drive += -j.qd; }
		}
		return drive;
	}

	// Method to integrate the base translation of the rigidBody based on leg joint angles (for legged systems)
	void RigidBodySystem::integrateBaseTranslation(double dt) {
		double hipL = 0.0;
		double hipR = 0.0;

		tryGetJointAngleRad("left_hip_pitch_link", hipL);
		tryGetJointAngleRad("right_hip_pitch_link", hipR);

		// Positive when left leg is in stance
		const double gaitPhase = hipR - hipL;

		// Tunable gain: rad -> N
		const double driveGain = 180.0;
		double F_forward = -driveGain * gaitPhase;
		_lastBaseForwardForce = F_forward;

		Vec3 dampingForce = -_baseLinearDamping * _baseVel;
		Vec3 F_world(F_forward, 0.0, 0.0);
		F_world += dampingForce;
		_baseAcc = F_world / _baseMass;

		_baseVel += _baseAcc * dt;
		_basePos += _baseVel * dt;

		LOG_INFO_ONCE("hipL=%.3f hipR=%.3f gaitPhase=%.3f", hipL, hipR, hipR - hipL);
		LOG_INFO_ONCE("baseVel = (%.3f, %.3f, %.3f)", _baseVel.x(), _baseVel.y(), _baseVel.z());
	}

	// Method to update the rigidBody root pose based on the integrated base translation (for legged systems)
	void RigidBodySystem::updateBaseRootPose() {
		Mat4 T = Mat4::Identity();
		T.block<3, 1>(0, 3) = Vec3(_basePos.x(), _basePos.y(), _basePos.z());

		Mat4 R = Mat4::Identity();
		R.block<3, 3>(0, 0) = Eigen::AngleAxisd(_baseYaw, Vec3(0, 1, 0)).toRotationMatrix();

		_root_pose = T * R * _root_home;
	}

	// --- ROBOT SYSTEM CONFIGURATION METHODS ---

	// Method to set the gravity strength for the rigidBody system
	void RigidBodySystem::setGravity(double g) {
		_gravity = mathlib::Vec3(0.0, 0.0, g);
		_dynamics->setGravity(g);
	}
	//
	void RigidBodySystem::setGravityVec(const mathlib::Vec3& g) {
		_gravity = g;
		_dynamics->setGravityVec(g);
	}

	// Set the torque mode for the rigidBody system
	void RigidBodySystem::setTorqueMode(eTorqueMode mode) { _body.torqueMode = mode; }

	// Method to claim the current active log buffer for exporting logged data (returns pointer to buffer active before swap)
	std::unique_ptr<systems::JointLogBuffer> RigidBodySystem::claimExportLogBuffer() {
		// swap active buffer index
		std::lock_guard<std::mutex> lk(_logSwapMutex);				 // ensure thread safety during swap
		int prev = _activeLogBufIdx.load(std::memory_order_acquire); // get current active buffer index
		int next = 1 - prev;										 // compute next buffer index (toggle between 0 and 1)
		_activeLogBufIdx.store(next, std::memory_order_release);	 // set next buffer as active for logging

		auto out = std::make_unique<systems::JointLogBuffer>(); // create a new buffer to return to caller
		out->swap(_logBuffers[prev]); // swap contents of previous active buffer with new buffer

		return out;
	}
	// Method to claim the current active log buffer for exporting logged data (returns pointer to buffer active before swap)
	std::unique_ptr<systems::FreeBodyLogBuffer> RigidBodySystem::claimExportLogBuffer_fb() {
		// swap active buffer index
		std::lock_guard<std::mutex> lk(_logSwapMutex_fb);				 // ensure thread safety during swap
		int prev = _activeLogBufIdx_fb.load(std::memory_order_acquire); // get current active buffer index
		int next = 1 - prev;											 // compute next buffer index (toggle between 0 and 1)
		_activeLogBufIdx_fb.store(next, std::memory_order_release);	 // set next buffer as active for logging

		auto out = std::make_unique<systems::FreeBodyLogBuffer>(); // create a new buffer to return to caller
		out->swap(_logBuffers_fb[prev]); // swap contents of previous active buffer with new buffer

		return out;
	}

	// Method to enable or disable the use of internal log buffers for recording joint metrics during simulation
	void RigidBodySystem::useInternalLogBuffer(bool enable) {
		_useInternalLogging = enable;
		if (enable) {
			_logBuffers[0].clear();	   // clear both buffers to start fresh
			_logBuffers[1].clear();	   // clear both buffers to start fresh
			_activeLogBufIdx.store(0); // reset active buffer index to 0
		}
	}
	// Method to enable or disabled the use of internal log buffers for recording free body metrics during simulation
	void RigidBodySystem::useInternalLogBuffer_fb(bool enable) {
		_useInternalLogging_fb = enable;
		if (enable) {
			_logBuffers_fb[0].clear();	   // clear both buffers to start fresh
			_logBuffers_fb[1].clear();	   // clear both buffers to start fresh
			_activeLogBufIdx_fb.store(0); // reset active buffer index to 0
		}
	}

	// Method to reserve capacity in the internal log buffers to optimize performance by avoiding reallocations during logging
	void RigidBodySystem::reserveInternalLogBuffers(size_t expected) {
		_logBuffers[0].reserve(expected); // reserve both buffers to avoid reallocations during logging
		_logBuffers[1].reserve(expected); // reserve both buffers to avoid reallocations during logging
	}
	// Method to reserve capacity in the internal log buffers for free body metrics to optimize performance by avoiding reallocations during logging
	void RigidBodySystem::reserveInternalLogBuffers_fb(size_t expected) {
		_logBuffers_fb[0].reserve(expected); // reserve both buffers to avoid reallocations during logging
		_logBuffers_fb[1].reserve(expected); // reserve both buffers to avoid reallocations during logging
	}

} // namespace systems