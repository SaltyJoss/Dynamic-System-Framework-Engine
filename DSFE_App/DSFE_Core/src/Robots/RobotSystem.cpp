// DSFE_Core RobotSystem.cpp
#include "pch.h"

#include "Robots/RobotSystem.h"
#include "Robots/RobotLoader.h"

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
		_AD_integrator(std::make_unique<integration::DifferentiableIntegrator>()), _curIntMethod_AD(integration::eAutoDiffIntegrationMethod::AD_ImplicitEuler),
		_kinematics(std::make_unique<RobotKinematics>()), _dynamics(std::make_unique<RobotDynamics>()),
		_torqueMode(eTorqueMode::CONTROLLED) {
		if (!_integrator ) { LOG_WARN("RobotSystem got null IntegrationService*"); }
	}
	// Destructor
	RobotSystem::~RobotSystem() = default;

	const robots::RobotModel& RobotSystem::model() const { return _robot; }

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

	// Method to pack robot joint states into a state vector
	mathlib::VecX_T<DualNumber_T<double, 14>> RobotSystem::packState_AD() const {
		using Dual = DualNumber_T<double, 14>; // only hardcoded since I am testing the same arm, TODO provide a better final way to derive the NVar val.
		const size_t n = static_cast<int>(_robot.joints.size());
		mathlib::VecX_T<Dual> x(2 * n);

		// Pack angles and velocities
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];

			// Current states
			x[i] = Dual(j.q, { 0.0 });
			x[i + n] = Dual(j.qd, { 0.0 });
		}
		return x; // state vector
	}

	// Method to unpack state vector into robot joints
	void RobotSystem::unpackState_AD(const mathlib::VecX_T<DualNumber_T<double, 14>>& x) {
		using Dual = DualNumber_T<double, 14>;
		const size_t n = static_cast<int>(_robot.joints.size());

		// Resize clamping vectors if necessary
		if (_clampTheta.size() != n) { _clampTheta.assign(n, 0); }
		if (_clampOmega.size() != n) { _clampOmega.assign(n, 0); }

		// For each joint
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];

			// Current states
			Dual theta_in = x[i];		  // [rad]
			Dual omega_in = x[i + n];	  // [rad/s]

			// Clamp joint angle
			Dual theta_out = clampJointAngle_T<Dual>(j, theta_in);

			// max |omega|
			Dual wMax_hw = mathlib::abs(j.limits.maxqd);
			Dual omega_out = omega_in;

			// Velocity limit clamping
			if (wMax_hw > Dual(0)) {
				const Dual eps = Dual(5e-2);
				if (mathlib::abs(omega_in) > (Dual(1) + eps) * wMax_hw) {
					omega_out = std::clamp(omega_in, -wMax_hw, wMax_hw);
				}
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

	// Method to advance the robot state by dt using the selected integrator
	void RobotSystem::step(double dt, double simTime) {
		if (!_hasRobot) { return; }

		if (_useAutoDiff) {
			step_AD<AD_VARS>(dt, simTime);
			return;
		}

		_simTime = simTime;
		const size_t n = _robot.joints.size();
		mathlib::VecX x = packState();

		auto result = step_impl<double>(x, dt, simTime, *_integrator, _dynScratch, _dynResult);

		unpackState(result.stepOut.x_next);
		_dynamics->setDt(result.stepOut.dt_taken);

		const auto scratchCopy = _dynScratch;
		const auto resultCopy = result;

		postStepUpdate(resultCopy.stepOut.x_next, scratchCopy, resultCopy);

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

		resetRobot();

		LOG_INFO("Loaded robot model -> %s", name.c_str());
		D_SUCCESS("Loaded robot model -> %s", name.c_str());
	}

	// Method to reset the robot to its home position
	void RobotSystem::resetRobot() {
		if (!_hasRobot || !_robotHomeValid) { LOG_ERROR("Reset aborterd."); return; }
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

		_dynScratch.clear();
		_dynScratch_AD.clear();

		_dynResult.resize(0);
		_dynResult_AD.resize(0);

		_dynScratch.resize(_robot.joints.size(), _robot.links.size());
		_dynScratch_AD.resize(_robot.joints.size(), _robot.links.size());
		_dynResult.resize(_robot.joints.size());
		_dynResult_AD.resize(_robot.joints.size());

		_dynScratch.g.setConstant(_gravity);

		LOG_INFO("dynScratch=%p", &_dynScratch);
		LOG_INFO("dynScratchAD=%p", &_dynScratch_AD);
		LOG_INFO("M rows=%d cols=%d", (int)_dynScratch.dense.M.rows(), (int)_dynScratch.dense.M.cols());
		LOG_INFO("Xup size=%d", (int)_dynScratch.spatial.Xup.size());

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

	integration::DifferentiableIntegrator* RobotSystem::getADIntegrator() { return _AD_integrator.get(); }
	const integration::DifferentiableIntegrator* RobotSystem::getADIntegrator() const { return _AD_integrator.get(); }

	std::shared_ptr<integration::IntegratorState> RobotSystem::runtimeIntegratorState() {
		return _useAutoDiff ? _AD_integrator->runtimeState() : _integrator->runtimeState();
	}

	std::shared_ptr<const integration::IntegratorState> RobotSystem::runtimeIntegratorState() const {
		return _useAutoDiff ? _AD_integrator->runtimeState() : _integrator->runtimeState();
	}

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
			if (j.name.find("hip_pitch") != std::string::npos) { drive += -j.qd; }
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

		LOG_INFO_ONCE("hipL=%.3f hipR=%.3f gaitPhase=%.3f", hipL, hipR, hipR - hipL);
		LOG_INFO_ONCE("baseVel = (%.3f, %.3f, %.3f)", _baseVel.x(), _baseVel.y(), _baseVel.z());
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