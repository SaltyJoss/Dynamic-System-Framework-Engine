// DSFE_Core RobotSystem.cpp
#include "pch.h"

#include "Robots/RobotSystem.h"

#include <kinematics/Forward_Kinematics.h>
#include "Robots/RobotKinematics.h"
#include "Robots/RobotDynamics.h"
#include "Robots/RobotLoader.h"

#include <Robots/SpatialDynamics.h>

#include <stack>
#include <unordered_set>
#include <algorithm>

#include <Core/Utils.h>
#include "Robots/TrajectoryManager.h"
#include "Platform/Paths.h"

#include "EngineLib/LogMacros.h"
#include "Platform/DataManager.h"

using namespace mathlib;
using namespace constants;

namespace robots {
	// Constructor
	RobotSystem::RobotSystem()
		: _integrator(std::make_unique<integration::IntegrationService>()), _curIntMethod(integration::eIntegrationMethod::RK4), 
		_kinematics(std::make_unique<RobotKinematics>()), _dynamics(std::make_unique<RobotDynamics>()),
		_torqueMode(eTorqueMode::CONTROLLED) {
		if (!_integrator) { LOG_WARN("RobotSystem got null IntegrationService*"); }
	}
	// Destructor
	RobotSystem::~RobotSystem() = default;

	const robots::RobotModel& RobotSystem::model() const {
		return _robot;
	}

	// Helper function to convert std::vector<double> to Eigen::VectorXd
	static VecX toVecX(const std::vector<double>& a) {
		VecX v(a.size());
		for (size_t i = 0; i < a.size(); ++i) { v(i) = a[i]; }
		return v;
	}

	// --- HELPER METHODS ---

	// Method to clamp a joint angle to its limits
	double RobotSystem::clampJointAngle(const RobotJoint& joint, double angleRad) {
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

	// Method to build a name-to-index map for robot links
	void RobotSystem::buildLinkIndex() {
		_linkIndex.clear();
		for (size_t i = 0; i < _robot.links.size(); i++) {
			_linkIndex[_robot.links[i].name] = (int)i;
			LOG_INFO("Link %zu: %s -> index %d", i, _robot.links[i].name.c_str(), (int)i);
		}
	}

	// Method to build the spatial model (kinematic tree) from the robot model
	void RobotSystem::buildSpatialModel() {
		_spatialModel.joints.clear();
		const size_t n = _robot.joints.size();
		_spatialModel.joints.resize(n);

		for (size_t i = 0; i < n; ++i) {
			const RobotJoint& j = _robot.joints[i];
			auto& sj = _spatialModel.joints[i];

			sj.name = j.name;
			sj.type = j.type;

			// Find parent joint
			sj.parent = -1;
			for (size_t p = 0; p < n; ++p) {
				if (_robot.joints[p].child == j.parent) {
					sj.parent = (int)p;
					break;
				}
			}

			// Build XTree
			mathlib::Mat3 R = j.origin_q.toRotationMatrix();
			mathlib::Vec3 r = j.origin_xyz;			
			sj.Xtree = mathlib::spatialTransform(R, r);

			// Build Spatial Inertia
			int childLinkIdx = -1;
			for (size_t l = 0; l < _robot.links.size(); ++l) {
				if (_robot.links[l].name == j.child) {
					childLinkIdx = (int)l;
					break;
				}
			}

			if (childLinkIdx >= 0) {
				const RobotLink& link = _robot.links[childLinkIdx];
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
					break;
				case eJointType::PRISMATIC:
					sj.S = mathlib::SpatialVec(mathlib::Vec3::Zero(), j.axis.normalized());
					break;
				default:
					sj.S = mathlib::SpatialVec();
					break;
			}
		}
		LOG_INFO("SpatialModel built: joints=%d", (long long)_spatialModel.joints.size());
	}

	// Method to pack robot joint states into a state vector
	mathlib::VecX RobotSystem::packState() const {
		const size_t n = static_cast<int>(_robot.joints.size());
		mathlib::VecX x(2 * n);

		// Pack angles and velocities
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];

			// Current states
			x[i] = j.q;
			x[i + n] = j.qd;
		}
		return x; // state vector
	}
	
	// Method to unpack state vector into robot joints
	void RobotSystem::unpackState(const mathlib::VecX& x) {
		const size_t n = static_cast<int>(_robot.joints.size());

		// Resize clamping vectors if necessary
		if (_clampTheta.size() != n) { _clampTheta.assign(n, 0); }
		if (_clampOmega.size() != n) { _clampOmega.assign(n, 0); }

		// For each joint
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];

			// Current states
			double theta_in = x[i];		  // [rad]
			double omega_in = x[i + n];	  // [rad/s]

			// Clamp joint angle
			double theta_out = clampJointAngle(j, theta_in);

			// max |omega|
			double wMax_hw = std::abs(j.limits.maxqd);
			double omega_out = omega_in;

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
	}

	// Method to pack reference state vector (target angles and velocities) for control
	mathlib::VecX RobotSystem::packRefState() const {
		const size_t n = (int)_robot.joints.size();
		mathlib::VecX x(2 * n);
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];

			// Pack reference angles and velocities
			x[i] = j.q_ref;
			x[i + n] = j.qd_ref;
		}
		return x; // reference state vector
	}

	// Method to unpack reference state vector into robot joints
	void RobotSystem::unpackRefState(const mathlib::VecX& x) {
		const size_t n = (int)_robot.joints.size();
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];
			j.q_ref = x[i];					   // [rad]
			j.qd_ref = x[i + n];				   // [rad/s]
			j.q_ref = clampJointAngle(j, j.q_ref); // [rad]
		}
	}

	// Method to enforce joint limits after integration
	void RobotSystem::enforceJointLimits(RobotJoint& j) {
		if (j.limits.continuous) { return; }

		const double lo = j.limits.minAngle;
		const double hi = j.limits.maxAngle;

		if (j.q < lo) { j.q = lo; if (j.qd < 0.0f) { j.qd = 0.0f; }}
		if (j.q > hi) { j.q = hi; if (j.qd > 0.0f) { j.qd = 0.0f; }}
	}

	// Method to take a snapshot of the current robot state
	RobotSimSnapshot RobotSystem::takeSnapshot(double simTime) const {
		RobotSimSnapshot snap;
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

		snap.robotRootPose = _robotRootPose;
		snap.baseIsFree = _baseIsFree;
		snap.lastBaseForwardForce = _lastBaseForwardForce;
		snap.gravity = _gravity;

		snap.torqueMode = _robot.torqueMode;

		snap.dt = _dynamics->dt();
		snap.simTime = simTime;

		return snap;
	}

	// Method to advance the robot state by dt using the selected integrator
	void RobotSystem::step(double dt, double simTime) {
		if (!_hasRobot) { return; }
		_simTime = simTime;
		mathlib::VecX x = packState();

		RobotSimSnapshot_T<double> snap = takeSnapshot(simTime);
		const size_t n = snap.model->joints.size();

		Eigen::Map<const mathlib::VecX_T<double>> q(x.data(), n);
		Eigen::Map<const mathlib::VecX_T<double>> qd(x.data() + n, n);

		mathlib::VecX_T<double> qdd(n);
		for (size_t i = 0; i < n; ++i) { qdd[i] = _robot.joints[i].qdd_ref; }

		std::vector<Pose> T_start(snap.model->links.size());
		_kinematics->computeForwardKinematics_fromState(*snap.model, x, T_start);
		std::vector<Pose> jointWorldPoses_start = _kinematics->calcJointWorldPoses(T_start, *snap.model);

		SpatialDynamics::computeSpatialKinematicsAndBias<double>(
			_spatialModel,
			q, qd,
			_dynScratch.spatial.Xup,
			_dynScratch.spatial.v, _dynScratch.spatial.c
		);

		// CRBA only for controller inertia scaling
		mathlib::MatX_T<double> M_start = SpatialDynamics::CRBA<double>(
			_spatialModel,
			_dynScratch.spatial.Xup,
			_dynScratch
		);

		// Cache frozen joint gains for this step
		mathlib::VecX_T<double> kp_frozen(n), kd_frozen(n);
		for (size_t i = 0; i < n; ++i) {
			const auto& joint = snap.model->joints[i];
			if (joint.type == eJointType::FIXED) { continue; }

			_dynScratch.dense.I_eff_controller[i] = std::max(M_start(i, i), 1e-6);
			const double I_eff = _dynScratch.dense.I_eff_controller[i];

			kp_frozen[i] = I_eff * joint.wn_target * joint.wn_target;
			kd_frozen[i] = 2.0 * joint.zeta_target * I_eff * joint.wn_target;
		}

		// Compute RNEA torques for feedforward control
		mathlib::VecX_T<double> tau_rnea = SpatialDynamics::RNEA<double>(
			_spatialModel,
			q, qd, qdd,
			_dynScratch
		); // [Nm]
		LOG_INFO_ONCE("tau_rnea size = %lld", (long long)tau_rnea.size());

		// Define the derivative function for integration, capturing necessary variables by reference
		auto f_deriv = [&, kp_frozen, kd_frozen](auto t, const auto& xIn) {
			return _dynamics->derivative_spatial<double>(
				_spatialModel,
				t, xIn,
				snap,
				_dynScratch, _dynResult
			);
		};
		// Define the Jacobian function for integration, capturing necessary variables by reference
		auto f_J = [&, kp_frozen, kd_frozen](const mathlib::VecX_T<double>& xIn, mathlib::MatX_T<double>& J_out) {
			_dynamics->jacobian_spatial<double>(
				_spatialModel,
				xIn, snap,
				kp_frozen, kd_frozen,
				J_out, _dynScratch
			);
		};
		
		auto step = _integrator->step(_curIntMethod, x, simTime, dt, f_deriv, f_J);

		unpackState(step.x_next);
		_dynamics->setDt(step.dt_taken);

		Eigen::Map<const mathlib::VecX_T<double>> q_next(step.x_next.data(), n);
		Eigen::Map<const mathlib::VecX_T<double>> qd_next(step.x_next.data() + n, n);

		// Enforce joint limits
		for (auto& j : _robot.joints) { enforceJointLimits(j); }

		// Recompute kinematics and dynamics at the new state for logging and control purposes
		std::vector<Pose_T<double>> T_world(snap.model->links.size());
		_kinematics->computeForwardKinematics_fromState<double>(*snap.model, step.x_next, T_world);
		std::vector<Pose_T<double>> jointWorldPoses = _kinematics->calcJointWorldPoses<double>(T_world, *snap.model);

		// Compute spatial kinematics and bias terms for the new state
		SpatialDynamics::computeSpatialKinematicsAndBias<double>(
			_spatialModel,
			q_next, qd_next,
			_dynScratch.spatial.Xup,
			_dynScratch.spatial.v, _dynScratch.spatial.c
		);
		// Compute mass matrix at the new state
		mathlib::MatX_T<double> M_full = SpatialDynamics::CRBA<double>(
			_spatialModel,
			_dynScratch.spatial.Xup,
			_dynScratch
		);

		mathlib::VecX_T<double> tau_g = _dynamics->computeGravityTorque<double>(*snap.model, T_world, jointWorldPoses);

		// Compute system kinetic energy: E_kin = 0.5 * qd^T * M(q) * qd
		double sys_KE = 0.5 * qd_next.transpose() * M_full * qd_next; // [J], kinetic energy of the robot at configuration q and velocity qd

		// Compute system potential energy at configuration q (relative to gravity)
		double sys_PE = 0.0;
		double g = _dynamics->getGravity();

		for (size_t k = 0; k < _robot.links.size(); ++k) {
			const RobotLink& link = _robot.links[k];
			const double m = link.inertial.mass;

			if (m <= 0.0) { continue; }

			Vec3 com_world = 
				(T_world[k].block<3, 3>(0, 0) * link.inertial.com_xyz) +
				T_world[k].block<3, 1>(0, 3);

			sys_PE += m * g * com_world.z();
		}

		const double sys_E = sys_KE + sys_PE; // total mechanical energy of the system

		// Log metrics to buffer if logging is enabled
		robots::JointLogBuffer* buf = nullptr;

		// If using internal logging, get the active buffer
		if (_useInternalLogging) {
			int idx = _activeLogBufIdx.load(std::memory_order_acquire);
			buf = &_logBuffers[idx];
		} else {
			buf = _logBuffer;
		}

		if (buf) {
			for (size_t i = 0; i < n; ++i) {
				const RobotJoint& j = snap.model->joints[i];

				const double I_eff = (j.type == eJointType::FIXED) ? 1.0 : _dynResult.metrics.I_eff[i];
				const double err = snap.q_ref[i] - q_next[i];
				const double err_d = snap.qd_ref[i] - qd_next[i];

				JointLogBuffer::JointLogEntry e{};

				e.sim_time = simTime;
				e.dt_taken = step.dt_taken;
				e.dt_sug = step.dt_sug;
				e.theta = q_next[i]; e.omega = qd_next[i]; e.alpha = _dynResult.metrics.qdd[i];
				e.err = err; e.err_d = err_d;
				e.I_eff = I_eff;
				e.tau = _dynResult.metrics.tau[i]; e.tau_ff = tau_rnea[i];  e.tau_gravity = tau_g[i];
				e.tau_sat = _dynResult.metrics.tau_sat[i]; 
				e.KE = sys_KE; e.PE = sys_PE; e.E_total = sys_E;
				e.clamp_theta = (double)_clampTheta[i]; e.clamp_omega = (double)_clampOmega[i];
				e.sat_flag = _dynResult.metrics.sat_flag[i]; e.joint_index = (int)i;
				buf->push_entry(e);
			}
		}

		// Update base pose if free-floating
		if (_baseIsFree) {
			integrateBaseTranslation(dt);
			updateBaseRootPose();
		}

		// Update kinematics
		computeRobotKinematics(_worldTransforms);
	}

	// Method to step the reference trajectory and update joint reference states
	void RobotSystem::updateTrajectoryInputs(control::TrajectoryManager& traj, double t) {
		if (!_hasRobot) { return; }
		
		const size_t n = _robot.joints.size();
		if (n <= 0) { return; }

		// Sample trajectories ("ground truth" inputs)
		for (size_t i = 0; i < n; ++i) {
			RobotJoint& j = _robot.joints[i];
			control::TrajState s{};
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

	// Method to load a robot model by name
	void RobotSystem::loadRobot(const std::string& name) {
		if (name == _loadedName) {
			LOG_INFO("Robot '%s' is already loaded, skipping load.", name.c_str());
			D_INFO("Robot '%s' is already loaded, skipping load.", name.c_str());
			return;
		}

		// Reset control parameters to target values so that if the new robot has different defaults, we start with those
		resetNaturalFrequencyToTarget();
		resetDampingRatioToTarget();

		// Construct path to robot JSON file
		const std::filesystem::path jsonPath = paths::assets() / "objects" / "Robotic_Arm_Models" / name / (name + ".json");
		if (!std::filesystem::exists(jsonPath)) {
			LOG_ERROR("Robot JSON file not found -> %s", jsonPath.string().c_str());
			D_ERROR("Robot JSON file not found -> %s", jsonPath.string().c_str());
			return;
		}

		// Load robot model from JSON
		_robot = robots::RobotLoader::loadFromJSON(jsonPath.string());
		const size_t n = _robot.joints.size();
		const size_t m = _robot.links.size();

		_constModel.name = _robot.name;
		_constModel.scale = _robot.scale;

		_constModel.baseFrame = _robot.baseFrame;
		_constModel.baseFrameIsAligned = _robot.baseFrameIsEngineAligned;

		_constModel.links = _robot.links;
		_constModel.joints = _robot.joints;

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
		for (const auto& joint : _robot.joints) {
			if (joint.type == eJointType::FREE) {
				_baseIsFree = true;
				break;
			}
		}

		_robotRootHome = _robot.baseFrame;
		_robotRootPose = _robotRootHome;

		_robotQHome = _robot.makeJointVector();
		_robotHomeValid = true;

		buildLinkIndex();
		buildSpatialModel();
		_hasRobot = true;

		_dynScratch.resize(n, m);
		_dynResult.resize(n);

		resetRobot();

		LOG_INFO("Loaded robot model -> %s", name.c_str());
		D_SUCCESS("Loaded robot model -> %s", name.c_str());
	}

	// Method to reset the robot to its home position
	void RobotSystem::resetRobot() {
		if (!_hasRobot || !_robotHomeValid) { return; }
		_robotRootPose = _robotRootHome;
		_robot.setJointVector(_robotQHome);

		for (auto& joint : _robot.joints) {
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

		_dynScratch.resize(_robot.joints.size(), _robot.links.size());
		_dynResult.resize(_robot.joints.size());

		// Reset adaptive integrator so it doesn't carry a stale step size
		_integrator->resetAdaptiveState();

		computeRobotKinematics(_worldTransforms);
		D_INFO("Robot reset to home position.");
		D_SUCCESS("Robot reset to home position.");
	}

	void RobotSystem::stopAll() {
		if (!_hasRobot) return;
		for (auto& joint : _robot.joints) {
			joint.qd = 0.0f;
			joint.q_ref = joint.q;
		}
	}

	integration::IntegrationService* RobotSystem::getIntegrator() { return _integrator.get(); }
	const integration::IntegrationService* RobotSystem::getIntegrator() const { return _integrator.get(); }

	// --- ROBOT KINEMATICS AND JOINT STATE METHODS ---

	std::string RobotSystem::findRootLink() const {
		std::unordered_set<std::string> children;
		for (const auto& joint : _robot.joints) { children.insert(joint.child); }
		for (const auto& link : _robot.links) {
			if (children.find(link.name) == children.end()) {
				return link.name;
			}
		}
		return _robot.links.empty() ? "" : _robot.links.front().name; // fallback
	}

	// Method to update the pose of each robot link based on current joint angles using forward kinematics
	void RobotSystem::computeRobotKinematics(std::vector<Mat4>& world) {
		if (!_hasRobot) {
			world.clear();
			return;
		};

		world.resize(_robot.links.size());
		for (auto& T : world) { T = Mat4::Identity(); }

		// Find root link
		const std::string rootName = findRootLink();
		auto itRoot = _linkIndex.find(rootName);
		if (itRoot == _linkIndex.end()) {
			LOG_WARN_ONCE("RobotSystem::updateRobotKinematics: root link '%s' not found in link index", rootName.c_str());
			return;
		}

		// Set root link pose
		int rootIdx = itRoot->second;
		world[rootIdx] = _robotRootPose;

		// parent -> children joints
		std::unordered_map<std::string, std::vector<const RobotJoint*>> children;
		children.reserve(_robot.joints.size());
		for (const auto& j : _robot.joints) children[j.parent].push_back(&j);

		std::stack<std::string> st;
		st.push(rootName);

		// Traverse the kinematic tree using DFS
		while (!st.empty()) {
			std::string parentName = st.top(); 
			st.pop();

			// Skip if parent link not found
			auto itP = _linkIndex.find(parentName);
			if (itP == _linkIndex.end()) { continue; }
			int pIdx = itP->second;

			const Mat4& T_parent = world[pIdx];


			// Find children joints
			auto it = children.find(parentName);
			if (it == children.end()) continue;

			// For each child joint
			for (const RobotJoint* jp : it->second) {
				const RobotJoint& j = *jp;
				auto itC = _linkIndex.find(j.child);
				if (itC == _linkIndex.end()) { continue; }
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

	// Method to get the angle of a specific robot joint
	bool RobotSystem::tryGetJointAngleRad(const std::string& childLink, double& outAngle) const {
		if (!_hasRobot) { return false; }
		// Find joint child matching childLink
		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				outAngle = joint.q;
				return true;
			}
		}
		return false;
	}

	// Method to set the angle of a specific robot joint
	bool RobotSystem::trySetJointAngleRad(const std::string& childLink, double angleRad) {
		if (!_hasRobot) { return false; }
		// Find joint child matching childLink
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				joint.q = clampJointAngle(joint, angleRad); // clamp to joint limits
				return true;
			}
		}
		return false;
	}

	// Method to get the angular velocity of a specific robot joint
	bool RobotSystem::tryGetJointOmegaRad(const std::string& childLink, double& outOmega) const {
		if (!_hasRobot) { return false; }
		// Find joint child matching childLink
		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				outOmega = joint.qd;
				return true;
			}
		}
		return false;
	}

	// Method to set the angular velocity of a specific robot joint
	bool RobotSystem::trySetJointOmegaRad(const std::string& childLink, double omegaRad) {
		if (!_hasRobot) { return false; }
		// Find joint child matching childLink
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				joint.qd = omegaRad;
				return true;
			}
		}
		return false;
	}

	// Method to directly inject an angular velocity into the state vector for a specific robot joint (bypassing any clamping or limits)
	bool RobotSystem::injectJointOmegaRad(const std::string& childLink, double omega) {
		if (!_hasRobot) return false;
		const size_t n = _robot.joints.size();
		// Find joint child matching childLink
		for (size_t i = 0; i < n; ++i) {
			if (_robot.joints[i].child == childLink) {
				// Modify actual state vector
				mathlib::VecX x = packState();
				x[i + n] = omega;  // velocity slot
				unpackState(x);
				return true;
			}
		}
		return false;
	}

	// Method to get the target angle (reference) of a specific robot joint in radians
	bool RobotSystem::tryGetJointTargetRad(const std::string& childLink, double& outTargetRad) const {
		if (!_hasRobot) { return false; }
		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				outTargetRad = joint.q_ref;
				return true;
			}
		}
		return false;
	}

	// Method to set the target angle (reference) of a specific robot joint in radians
	bool RobotSystem::trySetJointTargetRad(const std::string& childLink, double targetRad) {
		if (!_hasRobot) { return false; }
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				if (joint.limits.continuous) { joint.q_ref = wrapRad(targetRad); }
				else { joint.q_ref = clampJointAngle(joint, targetRad); } // clamp to joint 
				return true;
			}
		}
		return false;
	}

	//	Method to get the maximum angular velocity of a specific robot joint in radians
	bool RobotSystem::tryGetJointOmegaMaxRad(const std::string& childLink, double& maxOmegaRad) const {
		if (!_hasRobot) { return false; }
		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				maxOmegaRad = joint.limits.maxqd;
				return true;
			}
		}
		return false;
	}

	//	Method to set the maximum angular velocity of a specific robot joint in radians
	bool RobotSystem::trySetJointOmegaMaxRad(const std::string& childLink, double maxOmegaRad) {
		if (!_hasRobot) { return false; }
		if (maxOmegaRad <= 0.0f) { return false; }
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				joint.limits.maxqd = maxOmegaRad;
				return true;
			}
		}
		return false;
	}

	// Method to increment the target angle (reference) of a specific robot joint in radians
	bool RobotSystem::tryAddJointTargetRad(const std::string& childLink, double deltaRad) {
		if (!_hasRobot) { return false; }
		for (auto& joint : _robot.joints) {
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

	// Method to set the reference angular velocity of a specific robot joint in radians
	bool RobotSystem::trySetJointOmegaRefRad(const std::string& childLink, double omegaRefRad) {
		if (!_hasRobot) { return false; }
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				joint.qd_ref = omegaRefRad;
				return true;
			}
		}
		return false;
	}

	// Method to set the reference angular acceleration of a specific robot joint in radians
	bool RobotSystem::trySetJointAlphaRefRad(const std::string& childLink, double alphaRefRad) {
		if (!_hasRobot) { return false; }
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				joint.qdd_ref = alphaRefRad;
				return true;
			}
		}
		return false;
	}

	// Method to set the max Omega reference of a specific robot joint in radians
	bool RobotSystem::trySetJointOmegaRefMaxRad(const std::string& childLink, double maxOmegaRad) {
		if (!_hasRobot) { return false; }
		if (maxOmegaRad <= 0.0f) { return false; }
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				joint.limits.omegaRefMaxRad_s = maxOmegaRad;
				return true;
			}
		}
		return false;
	}

	// Method to zero the reference derivatives (velocity and acceleration) of a specific robot joint
	bool RobotSystem::tryZeroJointRefDerivatives() {
		if (!_hasRobot) { return false; }
		for (auto& joint : _robot.joints) {
			joint.qd_ref = 0.0f;
			joint.qdd_ref = 0.0f;
		}
		return true;
	}

	// Method to check if a specific robot joint is at its target angle within a tolerance (radians)
	bool RobotSystem::isJointAtTargetRad(const std::string& childLink, double tolRad) const {
		if (!_hasRobot) { return false; }
		if (tolRad < 0.0f) { tolRad = -tolRad; }

		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				double err = joint.q_ref - joint.q;
				if (joint.limits.continuous) { err = wrapToPi(err); }
				err = std::abs(err);
				return err <= tolRad;
			}
		}
		return false;
	}

	// Method to check if a specific robot joint is at its target angle within a tolerance (degrees)
	bool RobotSystem::isJointAtTargetDeg(const std::string& childLink, double tolDeg) const { 
		return isJointAtTargetRad(childLink, radians(tolDeg)); 
	}

	// Method to check if a specific robot joint is near a target angle within a tolerance (radians)
	bool RobotSystem::isJointNearAngleRad(const std::string& childLink, double targetRad, double tolRad) const {
		if (!_hasRobot) { return false; }
		tolRad = std::abs(tolRad);

		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				double err = targetRad - joint.q;
				if (joint.limits.continuous) { err = wrapToPi(err); }
				return std::abs(err) <= tolRad;
			}
		}
		return false;
	}

	// Method to check if a specific robot joint is near a target angle within a tolerance (degrees)
	bool RobotSystem::isJointNearAngleDeg(const std::string& childLink, double targetDeg, double tolDeg) const { 
		return isJointNearAngleRad(childLink, radians(targetDeg), radians(tolDeg));
	}

	// --- ROBOT LINK AND ROOT POSE METHODS ---

	// Method to set the rotation angle of a specific robot link angle in degrees
	bool RobotSystem::setRobotLinkRotation(const std::string& childLinkName, double angleDeg) {
		for (auto& j : _robot.joints) {
			if (j.child == childLinkName) {
				j.q = radians(angleDeg);
				computeRobotKinematics(_worldTransforms);
				return true;
			}
		}
		return false;
	}

	Mat4 RobotSystem::setRobotRoot(const Vec3& pos, const Quat& rot) {
		Mat4 T = Mat4::Identity();
		T.block<3, 1>(0, 3) = pos;
		Mat4 R = Mat4::Identity();
		R.block<3, 3>(0, 0) = rot.toRotationMatrix();
		return T * R;
	}

	// Method to set the robot root pose in world coordinates
	void RobotSystem::setRobotRootPose(const Vec3& pos, const Quat& rot) {
		_robotRootPose = setRobotRoot(pos, rot);
	}

	// Method to set the robot root home pose in world coordinates
	void RobotSystem::setRobotRootHome(const Vec3& pos, const Quat& rot) {
		_robotRootHome = setRobotRoot(pos, rot);;
		_robotRootPose = _robotRootHome;
	}

	// Method to set the default pose of the robot using joint angles in degrees
	bool RobotSystem::setDefaultPoseDeg() {
		if (!_hasRobot) { return false; }
		_robotQHome = _robot.makeJointVector();
		_robotHomeValid = true;
		return true;
	}

	// --- ROBOT BASE INTEGRATION METHODS ---

	// Method to set the default pose of the robot using joint angles in radians
	double RobotSystem::computeForwardDrive() const {
		double drive = 0.0;
		for (const auto& j : _robot.joints) {
			if (j.name.find("hip_pitch") != std::string::npos) {
				drive += -j.qd;
			}
		}
		return drive;
	}

	// Method to integrate the base translation of the robot based on leg joint angles (for legged robots)
	void RobotSystem::integrateBaseTranslation(double dt) {
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

		LOG_INFO_ONCE("hipL=%.3f hipR=%.3f gaitPhase=%.3f",
			hipL, hipR, hipR - hipL
		);
		LOG_INFO_ONCE("baseVel = (%.3f, %.3f, %.3f)",
			_baseVel.x(), _baseVel.y(), _baseVel.z()
		);
	}

	// Method to update the robot root pose based on the integrated base translation (for legged robots)
	void RobotSystem::updateBaseRootPose() {
		Mat4 T = Mat4::Identity();
		T.block<3, 1>(0, 3) = Vec3(_basePos.x(), _basePos.y(), _basePos.z());

		Mat4 R = Mat4::Identity();
		R.block<3, 3>(0, 0) = Eigen::AngleAxisd(_baseYaw, Vec3(0, 1, 0)).toRotationMatrix();

		_robotRootPose = T * R * _robotRootHome;
	}

	// --- ROBOT SYSTEM CONFIGURATION METHODS ---

	// Method to set the gravity strength for the robot system
	void RobotSystem::setGravity(double g) {
		_gravity = g;
		_dynamics->setGravity(g);
	}

	// Set the torque mode for the robot system
	void RobotSystem::setTorqueMode(eTorqueMode mode) { _robot.torqueMode = mode; }

	// Method to claim the current active log buffer for exporting logged data (returns pointer to buffer active before swap)
	std::unique_ptr<robots::JointLogBuffer> RobotSystem::claimExportLogBuffer() {
		// swap active buffer index
		std::lock_guard<std::mutex> lk(_logSwapMutex);				 // ensure thread safety during swap
		int prev = _activeLogBufIdx.load(std::memory_order_acquire); // get current active buffer index
		int next = 1 - prev;										 // compute next buffer index (toggle between 0 and 1)
		_activeLogBufIdx.store(next, std::memory_order_release);	 // set next buffer as active for logging

		auto out = std::make_unique<robots::JointLogBuffer>(); // create a new buffer to return to caller
		out->swap(_logBuffers[prev]); // swap contents of previous active buffer with new buffer

		return out;
	}

	// Method to enable or disable the use of internal log buffers for recording joint metrics during simulation
	void RobotSystem::useInternalLogBuffer(bool enable) {
		_useInternalLogging = enable;
		if (enable) {
			_logBuffers[0].clear();	   // clear both buffers to start fresh
			_logBuffers[1].clear();	   // clear both buffers to start fresh
			_activeLogBufIdx.store(0); // reset active buffer index to 0
		}
	}

	// Method to reserve capacity in the internal log buffers to optimize performance by avoiding reallocations during logging
	void RobotSystem::reserveInternalLogBuffers(size_t expected) {
		_logBuffers[0].reserve(expected); // reserve both buffers to avoid reallocations during logging
		_logBuffers[1].reserve(expected); // reserve both buffers to avoid reallocations during logging
	}

} // namespace robots