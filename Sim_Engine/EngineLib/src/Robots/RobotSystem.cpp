#include "pch.h"
// File:   RobotSystem.cpp
// GitHub: SaltyJoss
#include "Robots/RobotSystem.h"
#include "Scene/Object.h"
#include "Scene/Mesh.h"

#include <kinematics/Forward_Kinematics.h>
#include "Robots/RobotKinematics.h"
#include "Robots/RobotDynamics.h"
#include "Robots/RobotLoader.h"
#include "Robots/RobotCommon.h"

#include <stack>
#include <unordered_set>

#include <glm/gtc/matrix_transform.hpp>
#include <Core/Utils.h>
#include "Robots/TrajectoryManager.h"
#include "Platform/Paths.h"

#include "EngineLib/LogMacros.h"
#include "Platform/DataManager.h"

using namespace mathlib;
using namespace constants;

namespace robots {
	// Constructor
	RobotSystem::RobotSystem(std::vector<std::unique_ptr<scene::Object>>& objects, spawnFn meshLoader)
		: _objects(objects), _loadMeshReturn(std::move(meshLoader)), _torqueMode(eTorqueMode::CONTROLLED),
		_integrator(std::make_unique<integration::IntegrationService>()), _curIntMethod(integration::eIntegrationMethod::RK4), 
		_kinematics(std::make_unique<RobotKinematics>(_robot)), _dynamics(std::make_unique<RobotDynamics>(_robot, _torqueMode)) {
		if (!_integrator) { LOG_WARN("RobotSystem got null IntegrationService*"); }
	}
	// Destructor
	RobotSystem::~RobotSystem() = default;

	// --- 'toGlm' OVERLOADS ---

	// Converts an Eigen 3D vector to a glm::vec3
	static glm::vec3 toGlm(const Vec3& v) { 
		return glm::vec3(
			static_cast<float>(v.x()),
			static_cast<float>(v.y()),
			static_cast<float>(v.z())
			);
	}
	// Converts an Eigen quaternion to a glm::quat, taking into account the different ordering of components (w, x, y, z) vs (x, y, z, w)
	static glm::quat toGlm(const Quat& q) {
		return glm::quat(
			static_cast<float>(q.w()),
			static_cast<float>(q.x()),
			static_cast<float>(q.y()),
			static_cast<float>(q.z())
		); // (w, x, y, z)
	}
	
	// Converts a 3x3 Eigen matrix to a glm::mat3, taking into account the row-major to column-major conversion
	static glm::mat3 toGlm(const Mat3& m) {
		glm::mat3 g(1.0f);
		for (int c = 0; c < 3; ++c)
			for (int r = 0; r < 3; ++r)
				g[c][r] = static_cast<float>(m(r, c));
		return g; // (3x3)
	}

	// Converts a 4x4 Eigen matrix to a glm::mat4, taking into account the row-major to column-major conversion
	static glm::mat4 toGlm(const Mat4& m) {
		glm::mat4 g(1.0f);
		for (int c = 0; c < 4; ++c)
			for (int r = 0; r < 4; ++r)
				g[c][r] = static_cast<float>(m(r, c));
		return g; // (4x4)
	}

	// --- HELPER METHODS ---

	// Method to clamp a joint angle to its limits
	double RobotSystem::clampJointAngle(const RobotJoint& joint, double angleRad) {
		if (joint.limits.continuous) { return wrapRad(angleRad); }
		else { return glm::clamp(angleRad, joint.limits.minAngle, joint.limits.maxAngle); }
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

	// Method to create Object instances for each robot link
	void RobotSystem::instantiateRobotLinks() {
		// For each link, load its visual mesh(es), apply the visual origin transform, and create a scene::Object
		for (auto& link : _robot.links) {
			link.attachedObjects.clear();

			// Per-mesh material entries (new format with meshEntries)
			if (!link.visual.meshEntries.empty()) {
				for (const auto& entry : link.visual.meshEntries) {
					const auto fullPath = (paths::assets() / "objects" / "Robotic_Arm_Models" / entry.meshFile).string();
					auto partObjs = _loadMeshReturn(fullPath);

					// If no meshes were loaded for this entry, skip it
					for (auto* obj : partObjs) {
						scene::Mesh* mesh = obj->getMesh();
						if (mesh) {
							// Apply per-mesh material if specified, otherwise use link-level material
							if (entry.hasMaterial) {
								const Vec4& rgba = entry.material;
								mesh->setAlbedo(glm::vec3(rgba.x(), rgba.y(), rgba.z()));
								mesh->setMetallic(entry.metallic);
								mesh->setRoughness(entry.roughness);
							}
							// Fallback to link-level material
							else {
								const Vec4& rgba = link.visual.material;
								mesh->setAlbedo(glm::vec3(rgba.x(), rgba.y(), rgba.z()));
								mesh->setMetallic(link.visual.metallic);
								mesh->setRoughness(link.visual.roughness);
							}
							mesh->rebuildGPU();
						}

						// Set object properties
						obj->name = link.name;
						obj->category = scene::ObjectCategory::RobotLink;
						obj->transform.scale = glm::vec3(_robot.scale);
						link.attachedObjects.push_back(obj);
					}
				}

				if (!link.attachedObjects.empty()) {
					link.attachedObject = link.attachedObjects[0];
				}
				continue;
			}

			// If no mesh entries, fall back to legacy single mesh or multiple mesh files
			std::vector<scene::Object*> objs;

			// Explicit multiple meshes
			if (!link.visual.meshFiles.empty()) {
				for (const auto& meshRelPath : link.visual.meshFiles) {
					const auto fullPath = (paths::assets() / "objects" / "Robotic_Arm_Models" / meshRelPath).string();
					auto partObjs = _loadMeshReturn(fullPath);
					objs.insert(objs.end(), partObjs.begin(), partObjs.end());
				}
			}
			// Single mesh file
			else if (!link.visual.meshFile.empty()) {
				const auto fullPath = (paths::assets() / "objects" / "Robotic_Arm_Models" / link.visual.meshFile).string();
				objs = _loadMeshReturn(fullPath);
			}
			else {
				LOG_WARN("Link %s has no visual meshes defined", link.name.c_str());
				continue;
			}

			// If no meshes were loaded, skip this link
			if (objs.empty()) {
				LOG_WARN("No meshes found for link %s", link.name.c_str());
				continue;
			}

			// Merge multiple meshes into one Object (if necessary)
			scene::Object* obj = objs[0];
			scene::Mesh* baseMesh = obj->getMesh();

			// If there are multiple meshes (e.g., from a multi-part OBJ), merge them into the first one
			for (size_t i = 1; i < objs.size(); ++i) {
				scene::Mesh* extraMesh = objs[i]->getMesh();
				if (extraMesh && baseMesh) {
					baseMesh->appendGeometry(*extraMesh);
				}

				objs[i]->name.clear();
				objs[i]->category = scene::ObjectCategory::General;
			}

			// Apply visual origin transform to the merged mesh
			if(baseMesh) {
				const Vec4& rgba = link.visual.material;

				// Apply visual material properties
				baseMesh->setAlbedo(glm::vec3(rgba.x(), rgba.y(), rgba.z()));
				baseMesh->setMetallic(link.visual.metallic);
				baseMesh->setRoughness(link.visual.roughness);

				baseMesh->rebuildGPU();
			}

			obj->name = link.name;
			obj->category = scene::ObjectCategory::RobotLink;
			obj->transform.scale = glm::vec3(_robot.scale);
			link.attachedObject = obj;
			link.attachedObjects.push_back(obj);
		}

		LOG_INFO_ONCE("Instantiated %zu robot links", _robot.links.size());
		D_INFO_ONCE("Instantiated %zu robot links", _robot.links.size());
	}

	// Method to build a name-to-index map for robot links
	void RobotSystem::buildLinkIndex() {
		_linkIndex.clear();
		for (size_t i = 0; i < _robot.links.size(); i++) {
			_linkIndex[_robot.links[i].name] = (int)i;
			LOG_INFO("Link %zu: %s -> index %d", i, _robot.links[i].name.c_str(), (int)i);
		}
	}

	// Method to pack robot joint states into a state vector
	mathlib::VecX RobotSystem::packState() const {
		const size_t n = static_cast<int>(_robot.joints.size());
		mathlib::VecX x(3 * n);

		// Pack angles and velocities
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];

			// Current states
			x[i] = j.thetaRad;
			x[i + n] = j.omegaRad_s;
			x[i + 2 * n] = j.eta; // integral state
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
			double eta_in   = x[i + 2 * n]; // integral state

			// Clamp joint angle
			double theta_out = clampJointAngle(j, theta_in);

			// max |omega|
			double wMax_hw = std::abs(j.limits.maxOmegaRad_s);
			double omega_out = omega_in;

			// Velocity limit clamping
			if (wMax_hw > 0.0f) {
				const double eps = 0.05f;
				if (std::abs(omega_in) > (1.0f + eps) * wMax_hw) {
					omega_out = glm::clamp(omega_in, -wMax_hw, wMax_hw);
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
			j.thetaRad = theta_out;
			j.omegaRad_s = omega_out;
			j.eta = eta_in;
		}
	}

	// Method to pack reference state vector (target angles and velocities) for control
	mathlib::VecX RobotSystem::packRefState() const {
		const size_t n = (int)_robot.joints.size();
		mathlib::VecX x(2 * n);
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];

			// Pack reference angles and velocities
			x[i] = (double)j.thetaRefRad;
			x[i + n] = (double)j.omegaRefRad_s;
		}
		return x; // reference state vector
	}

	// Method to unpack reference state vector into robot joints
	void RobotSystem::unpackRefState(const mathlib::VecX& x) {
		const size_t n = (int)_robot.joints.size();
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];
			j.thetaRefRad = x[i];					   // [rad]
			j.omegaRefRad_s = x[i + n];				   // [rad/s]
			// Clamp reference angle to joint limits
			j.thetaRefRad = clampJointAngle(j, j.thetaRefRad); // [rad]
		}
	}

	// Method to enforce joint limits after integration
	void RobotSystem::enforceJointLimits(RobotJoint& j) {
		if (j.limits.continuous) { return; }

		const double lo = j.limits.minAngle;
		const double hi = j.limits.maxAngle;

		if (j.thetaRad < lo) { j.thetaRad = lo; if (j.omegaRad_s < 0.0f) { j.omegaRad_s = 0.0f; }}
		if (j.thetaRad > hi) { j.thetaRad = hi; if (j.omegaRad_s > 0.0f) { j.omegaRad_s = 0.0f; }}
	}

	// Method to advance the robot state by dt using the selected integrator
	void RobotSystem::step(double dt, double simTime) {
		if (!_hasRobot) return;
		const size_t n = _robot.joints.size();
		_simTime = simTime;

		// 
		mathlib::VecX x = packState();

		// Define the derivative function
		auto f = [&](double t, const mathlib::VecX& xIn) { return _dynamics->derivative(t, xIn); };
		auto step = _integrator->stepODE(_curIntMethod, x, simTime, dt, f);
		mathlib::VecX x_Next = step.x_next;

		// Unpack new state
		unpackState(x_Next);

		// Enforce joint limits
		for (auto& j : _robot.joints) {
			const double wMax = j.limits.maxOmegaRad_s;
			/*if (wMax > 0.0f) { j.omegaRad_s = glm::clamp(j.omegaRad_s, -wMax, wMax); }*/
			enforceJointLimits(j);
		}

		// FK needed for inertia
		mathlib::VecX x_f = packState();
		std::vector<Pose> T_world = _kinematics->computeForwardKinematics_fromState(x_f);
		std::vector<Pose> jointWorldPose = _kinematics->calcJointWorldPoses(T_world, _robot.joints);

		// compute gravity torques for new state so logs match dynamics
		std::vector<double> tau_gravity(n, 0.0);
		std::vector<double> theta(n);
		for (size_t i = 0; i < n; ++i) theta[i] = _robot.joints[i].thetaRad;
		tau_gravity = _dynamics->computeGravityTorque(theta, T_world, x);

		// Compute effective inertia for each joint at the new state
		std::vector<double> I_eff(n, 0.0);

		// For each joint, compute the effective inertia by summing contributions from all links
		for (size_t i = 0; i < n; ++i) {
			const RobotJoint& joint = _robot.joints[i];
			// Arbitrary nonzero to avoid divide-by-zero
			if (joint.type == eJointType::FIXED) {
				I_eff[i] = 1.0;   
				continue;
			}

			// Sum contributions to effective inertia from all links for joint i
			for (size_t k = i; k < _robot.links.size(); ++k) {
				I_eff[i] += _dynamics->computeJointInertiaContribution(
					_robot.joints[i],
					_robot.links[k],
					jointWorldPose[i], // pose of joint i in world frame
					T_world[k]
				);
			}
			// Floor effective inertia to avoid singularities
			I_eff[i] = std::max(I_eff[i], 1e-6);
		}

		for (size_t i = 0; i < n; ++i) {
			const auto& joint = _robot.joints[i];
			const auto& link  = _robot.links[i + 1];

			// Current states
			const double theta = joint.thetaRad;
			const double omega = joint.omegaRad_s;
			const double eta   = joint.eta;

			// Use integrated reference (baseline truth)
			const double thetaRef = joint.thetaRefRad;
			const double omegaRef = joint.omegaRefRad_s;
			const double alphaRef = joint.alphaRefRad_s2;

			// Compute joint metrics
			RobotMetrics m = computeJointMetrics(
				joint, link, I_eff[i],
				theta, omega, eta,         
				thetaRef, omegaRef, alphaRef,
				0.0,tau_gravity[i]
			);

			// Log metrics to buffer if logging is enabled
			auto* buf = _logBuffer;

			// If logging is enabled, store metrics in the buffer for this joint
			if (buf) {
				// Sim Metadata
				buf->sim_time.push_back(simTime);
				buf->dt_taken.push_back(step.dt_taken);
				buf->dt_sug.push_back(step.dt_sug);
				// States
				buf->theta.push_back(m.theta);
				buf->omega.push_back(m.omega);
				buf->alpha.push_back(m.alpha);
				buf->err.push_back(m.err);
				buf->err_d.push_back(m.err_d);
				// Dynamics
				buf->I_eff.push_back(m.I_eff);
				buf->tau.push_back(m.tau);
				buf->tau_fb.push_back(m.tau_fb);
				buf->tau_coriolis.push_back(m.tau_coriolis);
				buf->tau_gravity.push_back(m.tau_gravity);
				buf->tau_damping.push_back(m.tau_damping);
				buf->tau_friction.push_back(m.tau_friction);
				buf->tau_barrier.push_back(m.tau_barrier);
				buf->tau_sat.push_back(m.tau_sat);
				// Limit flags and info
				buf->clamp_theta.push_back((double)_clampTheta[i]);
				buf->clamp_omega.push_back((double)_clampOmega[i]);
				buf->sat_flag.push_back(m.sat_flag);
				// Joint Index
				buf->joint_index.push_back((int)i);
			}
		}

		// Update base pose if free-floating
		if (_baseIsFree) {
			integrateBaseTranslation(dt);
			updateBaseRootPose();
		}

		// Update kinematics
		updateRobotKinematics();
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
				j.thetaRefRad    = clampJointAngle(j, s.q); // set ref angle
				j.omegaRefRad_s  = s.qd;
				j.alphaRefRad_s2 = s.qdd;
			}
			// Store inputs
			else {
				j.alphaRefRad_s2 = 0.0f;
				j.omegaRefRad_s = 0.0f;
			}

			auto* buf = _refBuffer;
			if (buf) {
				// Sim Metadata
				buf->sim_time.push_back(t);
				// Reference states
				buf->theta_ref.push_back(j.thetaRefRad);
				buf->omega_ref.push_back(j.omegaRefRad_s);
				buf->alpha_ref.push_back(j.alphaRefRad_s2);
				// Joint Index
				buf->joint_index.push_back((int)i);
			}
		}
	}

	// --- ROBOT LOADING AND RESET METHODS ---

	// Method to load a robot model by name
	void RobotSystem::loadRobot(const std::string& name) {
		clearRobot();

		// Construct path to robot JSON file
		const std::filesystem::path jsonPath = paths::assets() / "objects" / "Robotic_Arm_Models" / name / (name + ".json");
		if (!std::filesystem::exists(jsonPath)) {
			LOG_ERROR("Robot JSON file not found -> %s", jsonPath.string().c_str());
			D_ERROR("Robot JSON file not found -> %s", jsonPath.string().c_str());
			return;
		}

		// Load robot model from JSON
		_robot = robots::RobotLoader::loadFromJSON(jsonPath.string());
		_dynamics->setRobot(_robot);
		_kinematics->setRobot(_robot);

		_loadedName = name;
		_baseIsFree = false;

		// Check if any joint is free-floating to determine if the base is free
		for (const auto& joint : _robot.joints) {
			if (joint.type == eJointType::FREE) {
				_baseIsFree = true;
				break;
			}
		}

		// Set base frame and home position
		glm::mat4 baseFrameGLM = toGlm(_robot.baseFrame);
		_robotRootHome = baseFrameGLM;
		_robotRootPose = _robotRootHome;

		// Initialize joint home positions
		_robotQHome = _robot.makeJointVector();
		_robotHomeValid = true;

		instantiateRobotLinks();
		buildLinkIndex();
	
		// Declare that we have a robot loaded
		_hasRobot = true;
		// Reset robot
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
			joint.omegaRad_s = 0.0f;
			joint.thetaRefRad = joint.thetaRad;
			joint.omegaRefRad_s = 0.0f;
			joint.alphaRefRad_s2 = 0.0f;
		}

		// Reset base state if free-floating
		_basePos = Vec3(0, 0, 0);
		_baseVel = Vec3(0, 0, 0);
		_baseAcc = Vec3(0, 0, 0);

		// Assuming base orientation is represented as a yaw angle for simplicity
		_baseYaw = 0.0;
		_baseYawRate = 0.0;
		_baseYawAcc = 0.0;

		// Reset adaptive integrator so it doesn't carry a stale step size
		_integrator->resetAdaptiveState();

		updateRobotKinematics();
		D_INFO("Robot reset to home position.");
		D_SUCCESS("Robot reset to home position.");
	}

	// Method to clear the current robot from the scene
	void RobotSystem::clearRobot() {
		if (!_hasRobot) return;

		// Remove robot objects from _objects
		for (auto& link : _robot.links) {
			for (auto* dead : link.attachedObjects) {
				if (!dead) continue;
				_objects.erase(
					std::remove_if(_objects.begin(), _objects.end(),
						[&](const std::unique_ptr<scene::Object>& obj) { return obj.get() == dead; }),
					_objects.end()
				);
			}
			link.attachedObjects.clear();
			link.attachedObject = nullptr;
		}

		_robot.links.clear();
		_robot.joints.clear();
		_linkIndex.clear();
		_hasRobot = false;

		LOG_INFO("Old robot model removed");
		D_WARN("Old robot model removed");
	}

	void RobotSystem::stopAll() {
		if (!_hasRobot) return;
		for (auto& joint : _robot.joints) {
			joint.omegaRad_s = 0.0f;
			joint.thetaRefRad = joint.thetaRad;
		}
	}

	integration::IntegrationService* RobotSystem::getIntegrator() {
		return _integrator.get();
	}

	const integration::IntegrationService* RobotSystem::getIntegrator() const {
		return _integrator.get();
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
	void RobotSystem::updateRobotKinematics() {
		if (!_hasRobot) return;
		std::vector<glm::mat4> world(_robot.links.size(), glm::mat4(1.0f));

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

			const glm::mat4& T_parent = world[pIdx];


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
				glm::mat4 T_joint = glm::translate(glm::mat4(1.0f), toGlm(j.origin_xyz));
				glm::mat4 R_joint = glm::mat4_cast(toGlm(j.origin_q));

				// Compute child link pose in world frame
				glm::mat4 T_child = T_parent * T_joint * R_joint;

				// Apply joint rotation for revolute joints
				if (j.type == eJointType::REVOLUTE) {
					glm::mat4 R_q = glm::rotate(glm::mat4(1.0f), static_cast<float>(j.thetaRad), glm::normalize(toGlm(j.axis)));
					T_child = T_child * R_q;
				}
				else if (j.type == eJointType::PRISMATIC) {
					glm::mat4 T_q = glm::translate(glm::mat4(1.0f), glm::normalize(toGlm(j.axis)) * static_cast<float>(j.thetaRad));
					T_child = T_child * T_q;
				}

				// FIXED joints: no motion
				world[cIdx] = T_child;

				st.push(j.child);
			}
		}

		// Update attached objects (visuals) based on FK results
		for (int i = 0; i < (int)_robot.links.size(); ++i) {
			auto& link = _robot.links[i];
			if (link.attachedObjects.empty()) continue;

			for (auto* obj : link.attachedObjects) {
				if (!obj) continue;
				scene::Mesh* mesh = obj->getMesh();
				if (!mesh) { continue; }

				// Engine-aligned robots (CAD-authored):
				// Mesh local frame already represents link frame.
				// FK must be baked directly into the mesh, not the object.
				if (_robot.baseFrameIsEngineAligned) {
					mesh->localTransform = glm::mat4(1.0f);
					mesh->localTransform = world[i];
					continue;
				}

				// FK: world -> link frame
				const glm::mat4& T_link = world[i];

				// Visual origin: link frame -> visual frame
				const Vec3& vt = link.visual.origin_xyz;
				const Vec3& vr = link.visual.origin_rpy;

				glm::mat4 T_visual(1.0f);
				T_visual = glm::translate(T_visual, glm::vec3(vt.x(), vt.y(), vt.z()));
				T_visual *= glm::mat4_cast(toGlm(_kinematics->rpyRadToQuat(vr)));

				glm::mat4 M = T_link * T_visual;

				// Apply mesh-local ONCE
				if (mesh->hasLocalTransform()) {
					M = M * mesh->localTransform;
				}

				obj->transform.position = glm::vec3(M[3]);
				obj->transform.rotQ = glm::quat_cast(M);

				LOG_INFO_ONCE("Mesh local determinant: %.3f",
					glm::determinant(mesh->localTransform));
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
				outAngle = joint.thetaRad;
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
				joint.thetaRad = clampJointAngle(joint, angleRad); // clamp to joint limits
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
				outOmega = joint.omegaRad_s;
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
				joint.omegaRad_s = omegaRad;
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
				outTargetRad = joint.thetaRefRad;
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
				if (joint.limits.continuous) { joint.thetaRefRad = wrapRad(targetRad); }
				else { joint.thetaRefRad = clampJointAngle(joint, targetRad); } // clamp to joint 
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
				maxOmegaRad = joint.limits.maxOmegaRad_s;
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
				joint.limits.maxOmegaRad_s = maxOmegaRad;
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
				double t = joint.thetaRefRad + deltaRad;
				if (joint.limits.continuous) { t = wrapRad(t); }
				else { t = glm::clamp(t, joint.limits.minAngle, joint.limits.maxAngle); }
				joint.thetaRefRad = t; // clamp to joint limits
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
				joint.omegaRefRad_s = omegaRefRad;
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
				joint.alphaRefRad_s2 = alphaRefRad;
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
			joint.omegaRefRad_s = 0.0f;
			joint.alphaRefRad_s2 = 0.0f;
		}
		return true;
	}

	// Method to check if a specific robot joint is at its target angle within a tolerance (radians)
	bool RobotSystem::isJointAtTargetRad(const std::string& childLink, double tolRad) const {
		if (!_hasRobot) { return false; }
		if (tolRad < 0.0f) { tolRad = -tolRad; }

		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				double err = joint.thetaRefRad - joint.thetaRad;
				if (joint.limits.continuous) { err = wrapToPi(err); }
				err = std::abs(err);
				return err <= tolRad;
			}
		}
		return false;
	}

	// Method to check if a specific robot joint is at its target angle within a tolerance (degrees)
	bool RobotSystem::isJointAtTargetDeg(const std::string& childLink, double tolDeg) const { 
		return isJointAtTargetRad(childLink, glm::radians(tolDeg)); 
	}

	// Method to check if a specific robot joint is near a target angle within a tolerance (radians)
	bool RobotSystem::isJointNearAngleRad(const std::string& childLink, double targetRad, double tolRad) const {
		if (!_hasRobot) { return false; }
		tolRad = std::abs(tolRad);

		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				double err = targetRad - joint.thetaRad;
				if (joint.limits.continuous) { err = wrapToPi(err); }
				return std::abs(err) <= tolRad;
			}
		}
		return false;
	}

	// Method to check if a specific robot joint is near a target angle within a tolerance (degrees)
	bool RobotSystem::isJointNearAngleDeg(const std::string& childLink, double targetDeg, double tolDeg) const { 
		return isJointNearAngleRad(childLink, glm::radians(targetDeg), glm::radians(tolDeg));
	}

	// --- ROBOT LINK AND ROOT POSE METHODS ---

	// Method to set the rotation angle of a specific robot link angle in degrees
	bool RobotSystem::setRobotLinkRotation(const std::string& childLinkName, double angleDeg) {
		for (auto& j : _robot.joints) {
			if (j.child == childLinkName) {
				j.thetaRad = glm::radians(angleDeg);
				updateRobotKinematics();
				return true;
			}
		}
		return false;
	}

	// Method to set the robot root pose in world coordinates
	void RobotSystem::setRobotRootPose(const glm::vec3& pos, const glm::quat& rot) {
		glm::mat4 T = glm::translate(glm::mat4(1.0f), pos);
		glm::mat4 R = glm::mat4_cast(rot);
		_robotRootPose = T * R;
	}

	// Method to set the robot root home pose in world coordinates
	void RobotSystem::setRobotRootHome(const glm::vec3& pos, const glm::quat& rot) {
		glm::mat4 T = glm::translate(glm::mat4(1.0f), pos);
		glm::mat4 R = glm::mat4_cast(rot);
		_robotRootHome = T * R;
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
				drive += -j.omegaRad_s;
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
		glm::mat4 T = glm::translate(glm::mat4(1.0f),
			glm::vec3(
				(float)_basePos.x(),
				(float)_basePos.y(),
				(float)_basePos.z()
			)
		);

		glm::mat4 R = glm::rotate(
			glm::mat4(1.0f),
			(float)_baseYaw,
			glm::vec3(0, 1, 0)
		);

		_robotRootPose = T * R * _robotRootHome;
	}

	// --- ROBOT SYSTEM CONFIGURATION METHODS ---

	// Method to set the gravity strength for the robot system
	void RobotSystem::setGravity(double g) {
		_gravity = g;
		_dynamics->setGravity(g);
	}

	// Set the torque mode for the robot system
	void RobotSystem::setTorqueMode(eTorqueMode mode) {
		_torqueMode = mode;
		_dynamics->setTorqueMode(mode);
	}
} // namespace robots