#include "pch.h"

#include "Robots/RobotSystem.h"
#include "Robots/RobotLoader.h"
#include "Scene/Object.h"
#include "Scene/Mesh.h"

#include <stack>

#include <glm/gtc/matrix_transform.hpp>
#include <Core/Utils.h>
#include <kinematics/Forward_Kinematics.h>
#include "Robots/TrajectoryManager.h"

#include "EngineLib/LogMacros.h"

#include "Platform/DataManager.h"

using namespace mathlib;
using namespace constants;

namespace robots {
	// Constructor
	RobotSystem::RobotSystem(std::vector<std::unique_ptr<scene::Object>>& objects, spawnFn meshLoader)
		: _integrator(std::make_unique<integration::IntegrationService>()), _curIntMethod(integration::eIntegrationMethod::Euler), 
		_objects(objects), _loadMeshReturn(std::move(meshLoader)) {
		if (!_integrator) { LOG_WARN("RobotSystem got null IntegrationService*"); }
	}

	// --- HELPER METHODS ---

	// Method to clamp a joint angle to its limits
	float RobotSystem::clampJointAngle(const RobotJoint& joint, float angleRad) {
		if (joint.limits.continuous) { return wrapRad(angleRad); }
		else { return glm::clamp(angleRad, joint.limits.minAngle, joint.limits.maxAngle); }
	}

	// Method to wrap an angle in radians to the range [-pi, pi]
	float RobotSystem::wrapToPi(float angleRad) {
		angleRad = std::fmod(angleRad + static_cast<float>(PI_d), static_cast<float>(TWO_PI_d));
		if (angleRad < 0.0f) { angleRad += static_cast<float>(TWO_PI_d); }
		return angleRad - static_cast<float>(PI_d);
	}

	// Method to wrap an angle in radians to the range [0, 2pi]
	float RobotSystem::wrapRad(float angleRad) {
		angleRad = fmod(angleRad, static_cast<float>(TWO_PI_d));
		if (angleRad < 0.0f) { angleRad += static_cast<float>(TWO_PI_d); }
		return angleRad;
	}

	// Method to apply a soft velocity barrier to joint torque
	static void applyOmegaBarrier(double& tau, double omega, double wMax, double I_eff) {
		if (wMax <= 0.0) return;

		const double absw = std::abs(omega);
		const double wSoft = 0.90 * wMax;

		if (absw <= wSoft) return;

		// How deep into the "soft zone" are we? 0..1
		const double t = (absw - wSoft) / (wMax - wSoft);
		const double gain = 2.0 * I_eff;  // tune factor; units make sense as torque per (rad/s)

		// Quadratic ramp gives gentle start and strong near limit
		const double wall = gain * (t * t) * (absw - wSoft);

		// Apply opposing torque to reduce |omega|
		tau -= wall * (omega >= 0.0 ? 1.0 : -1.0);
	}

	// Method to compute effective inertia about a joint axis
	double RobotSystem::computeJointAxisInertia(const RobotJoint& joint, const RobotLink& link) const {
		// Inertia matrix
		const robots::Inertia& inertial = link.inertial.inertia;
		
		glm::mat3 I_link(
			inertial.ixx, inertial.ixy, inertial.ixz,
			inertial.ixy, inertial.iyy, inertial.iyz,
			inertial.ixz, inertial.iyz, inertial.izz
		);

		// Transform to world frame
		glm::mat3 R = glm::mat3(glm::mat3_cast(joint.origin_q));
		glm::mat3 I_world = R * I_link * glm::transpose(R);

		// Joint axis in world frame
		glm::vec3 a = glm::normalize(joint.axis);

		// Effective inertia
		double I_eff = glm::dot(a, I_world * a);
		I_eff = std::max(I_eff, 1e-6); // avoid division by zero
		return I_eff;
	}

	// --- ROBOT STATE INTEGRATION METHODS ---

	// Method to create Object instances for each robot link
	void RobotSystem::instantiateRobotLinks() {
		for (auto& link : _robot.links) {
			auto objs = _loadMeshReturn(link.visual.meshFile);
			if (objs.empty()) { continue; }

			scene::Object* obj = objs[0];

			obj->category = scene::ObjectCategory::RobotLink;
			obj->transform.scale = glm::vec3(_robot.scale);
			link.attachedObject = obj;

			LOG_INFO_ONCE("Instantiated %zu robot links", _robot.links.size());
			D_INFO_ONCE("Instantiated %zu robot links", _robot.links.size());
		}
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
		mathlib::VecX x(2 * n);

		// Pack angles and velocities
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];

			// Current states
			x[i] = (double)j.thetaRad;
			x[i + n] = (double)j.omegaRad_s;
		}
		return x;
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
			float theta_in = (float)x[i];
			float omega_in = (float)x[i + n];

			// Enforce joint limits
			float theta_out = clampJointAngle(j, theta_in);
			float wMax = std::abs(j.limits.maxOmegaRad_s); // max |omega|
			float omega_out = omega_in;

			// Clamp omega if necessary
			if (wMax > 0.0f) { omega_out = glm::clamp(omega_in, -wMax, wMax); }

			// Record clamping
			_clampTheta[i] = (theta_in != theta_out) ? 1 : 0;
			_clampOmega[i] = (omega_in != omega_out) ? 1 : 0;

			// Update joint states
			_robot.joints[i].thetaRad = theta_out;
			_robot.joints[i].omegaRad_s = omega_out;
		}
	}

	mathlib::VecX RobotSystem::packRefState() const {
		const size_t n = (int)_robot.joints.size();
		mathlib::VecX x(2 * n);
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];

			// Pack reference angles and velocities
			x[i] = (double)j.thetaRefRad;
			x[i + n] = (double)j.omegaRefRad_s;
		}
		return x;
	}

	void RobotSystem::unpackRefState(const mathlib::VecX& x) {
		const size_t n = (int)_robot.joints.size();
		for (size_t i = 0; i < n; ++i) {
			auto& j = _robot.joints[i];
			j.thetaRefRad = (float)x[i];
			j.omegaRefRad_s = (float)x[i + n];
			j.thetaRefRad = clampJointAngle(j, j.thetaRefRad);
		}
	}

	JointMetrics RobotSystem::computeJointMetrics(const RobotJoint& joint, const RobotLink& link, double theta, double omega, double thetaRef, double omegaRef, double alphaRef) const {
		JointMetrics m{};

		// References
		m.theta = theta;
		m.omega = omega;
		m.thetaRef = thetaRef;
		m.omegaRef = omegaRef;
		m.alphaRef = alphaRef;

		// Errors
		m.err = m.thetaRef - theta;
		m.err_d = m.omegaRef - omega;
			
		m.I_eff = computeJointAxisInertia(joint, link);
		if (!std::isfinite(m.I_eff) || m.I_eff < 1e-9) m.I_eff = 1e-9;

		const double wn = joint.wn_target;
		const double z = joint.zeta_target;

		double k_p = m.I_eff * wn * wn;
		double k_d = 2.0 * z * m.I_eff * wn;

		m.kp = k_p;
		m.kd = k_d;

		// PD -> u(t) = I_eff * a_ref + k_p * e(t) + k_d * de(t) 
		double tau_motor = m.I_eff * m.alphaRef + k_p * m.err + k_d * m.err_d; // control torque

		// Passive dynamics
		const double c = joint.dynamics.damping;
		const double mu = joint.dynamics.friction;
		const double v_eps = 1e-2; // small velocity threshold

		double tau_loss = 0.0;
		tau_loss += c * omega; // viscous damping
		tau_loss += mu * std::tanh(omega / v_eps); // Coulomb friction (smooth approx)

		double tau_g = 0.0; // gravity torque (not added yet)

		m.tau = tau_motor - tau_loss - tau_g; // net torque

		// Effort clamp
		if (joint.limits.maxEffort > 0.0f) {
			const double e_max = joint.limits.maxEffort;
			m.tau = std::clamp(m.tau, -e_max, e_max);
		}

		// Velocity soft limit
		const double wMax = std::abs(joint.limits.maxOmegaRad_s);

		// Apply soft velocity barrier
		applyOmegaBarrier(m.tau, omega, wMax, m.I_eff);
		// Final angular acceleration
		m.alpha = m.tau / m.I_eff;

		return m;
	}

	// Derivative function for ODE integration
	mathlib::VecX RobotSystem::deriv(const control::TrajectoryManager& traj, double t, const mathlib::VecX& x) const {
		const size_t n = static_cast<int>(_robot.joints.size());
		mathlib::VecX dx(2 * n);

		// For each joint
		for (size_t i = 0; i < n; ++i) {
			// Current states
			const double theta = x[i];
			const double omega = x[i + n];

			// Current joint
			const RobotJoint& joint = _robot.joints[i];
			const RobotLink& link = _robot.links[i+1];

			// Use integrated reference (baseline truth)
			const double thetaRef = (double)joint.thetaRefRad;
			const double omegaRef = (double)joint.omegaRefRad_s;
			const double alphaRef = (double)joint.alphaRefRad_s2;

			// Compute joint metrics
			JointMetrics m = computeJointMetrics(joint, link, theta, omega, thetaRef, omegaRef, alphaRef);

			// Fill in derivatives
			dx[i] =		  omega;	// dtheta/dt = omega
			dx[i + n] = m.alpha;	// domega/dt = alpha		
		}
		return dx;
	}

	// Method to enforce joint limits after integration
	void RobotSystem::enforceJointLimits(RobotJoint& j) {
		if (j.limits.continuous) { return; }

		const float lo = j.limits.minAngle;
		const float hi = j.limits.maxAngle;

		if (j.thetaRad < lo) { j.thetaRad = lo; if (j.omegaRad_s < 0.0f) { j.omegaRad_s = 0.0f; }}
		if (j.thetaRad > hi) { j.thetaRad = hi; if (j.omegaRad_s > 0.0f) { j.omegaRad_s = 0.0f; }}
	}

	// Method to advance the robot state by dt using the selected integrator
	void RobotSystem::step(const control::TrajectoryManager& traj, double dt, double simTime) {
		if (!_hasRobot) return;

		_simTime = simTime;

		// Pack current state
		mathlib::VecX x = packState(); // current state vector

		// Define the derivative function
		auto f = [&](double t, const mathlib::VecX& xIn) { return deriv(traj, t, xIn); };
		mathlib::VecX x_Next = _integrator->stepODE(_curIntMethod, x, simTime, dt, f);
		//
		//LOG_INFO("preClamp theta_next=%g rad (%g deg), omega_next=%g rad/s",
		//	x_Next[0], glm::degrees((float)x_Next[0]), x_Next[(int)_robot.joints.size()]);

		// Unpack new state
		unpackState(x_Next);

		// Enforce joint limits
		for (auto& j : _robot.joints) {
			const float wMax = j.limits.maxOmegaRad_s;
			if (wMax > 0.0f) { j.omegaRad_s = glm::clamp(j.omegaRad_s, -wMax, wMax); }
			enforceJointLimits(j);
		}

		for (size_t i = 0; i < (int)_robot.joints.size(); ++i) {
			const auto& joint = _robot.joints[i];
			const auto& link  = _robot.links[i + 1];

			// Current states
			const double theta = (double)joint.thetaRad;
			const double omega = (double)joint.omegaRad_s;

			// Use integrated reference (baseline truth)
			const double thetaRef = (double)joint.thetaRefRad;
			const double omegaRef = (double)joint.omegaRefRad_s;
			const double alphaRef = (double)joint.alphaRefRad_s2;

			// Compute joint metrics
			JointMetrics m = computeJointMetrics(joint, link, theta, omega, thetaRef, omegaRef, alphaRef);

			HDF5_SIM_DATA("robot_joint_control",
				(data::FieldList{
					// Simulation info
					{"sim_time",    simTime},
					{"dt",          dt},
					//  Joint identification
					{"joint_name",  std::string(joint.name)},
					{"joint_child", std::string(joint.child)},
					{"joint_parent",std::string(joint.parent)},
					// Limit values
					{"minAngle",    (double)joint.limits.minAngle},
					{"maxAngle",    (double)joint.limits.maxAngle},
					{"wMax",        (double)joint.limits.maxOmegaRad_s},
					{"maxEffort",   (double)joint.limits.maxEffort},
					// Dynamics values
					{"damping",     (double)joint.dynamics.damping},
					{"friction",    (double)joint.dynamics.friction},
					// Control gains
					{"k_p",         m.kp},
					{"k_d",         m.kd},
					// State and control values
					{"theta",       m.theta},
					{"theta_ref",   m.thetaRef},
					{"err",         m.err},
					{"omega",       m.omega},
					{"omega_ref",   m.omegaRef},
					{"err_d",       m.err_d},
					{"alpha_ref",   m.alphaRef},
					{"torque",      m.tau},
					{"I_eff",       m.I_eff},
					{"alpha",       m.alpha},
					// Clamping info
					{"clamp_theta", (double)_clampTheta[i]},
					{"clamp_omega", (double)_clampOmega[i]}
				})
			);
		}

		// Update kinematics
		updateRobotKinematics();
	}

	void RobotSystem::stepReference(control::TrajectoryManager& traj, double dt, double t) {
		if (!_hasRobot) return;
		
		const int n = (int)_robot.joints.size();
		if (n <= 0) { return; }

		// Sample trajectories ("ground truth" inputs)
		for (int i = 0; i < n; ++i) {
			RobotJoint& j = _robot.joints[i];
			control::TrajState s{};
			// Try to evaluate trajectory
			if (traj.tryEval(std::string(j.child), t, s)) {
				j.thetaRefRad = clampJointAngle(j, (float)s.q); // set ref angle
				j.omegaRefRad_s = (float)s.qd;
				j.alphaRefRad_s2 = (float)s.qdd;
			}
			// Store inputs
			else {
				j.alphaRefRad_s2 = 0.0f;
				j.omegaRefRad_s = 0.0f;
			}

			HDF5_REF_DATA("robot_joint_reference",
				(data::FieldList{
					{"sim_time", t},
					{"dt", dt},
					{"joint_name", std::string(j.name)},
					{"joint_child", std::string(j.child)},
					{"joint_parent", std::string(j.parent)},
					// "traj_*" == the truth in Option A
					{"traj_theta_ref", (double)j.thetaRefRad},
					{"traj_omega_ref", (double)j.omegaRefRad_s},
					{"traj_alpha_ref", (double)j.alphaRefRad_s2},
					// "theta_ref" etc also same truth
					{"theta_ref", (double)j.thetaRefRad},
					{"omega_ref", (double)j.omegaRefRad_s},
					{"alpha_ref", (double)j.alphaRefRad_s2},
				})
			);
		}
	}

	// --- ROBOT LOADING AND RESET METHODS ---

	// Method to load a robot model by name
	void RobotSystem::loadRobot(const std::string& name) {
		clearRobot();

		std::string jsonPath = "Engine/assets/objects/Robotic_Arm_Models/" + name + "/" + name + ".json";
		_robot = robots::RobotLoader::loadFromJSON(jsonPath);
		_hasRobot = true;
		_loadedName = name;

		_robotRootHome = glm::mat4(1.0f);
		_robotRootHome = glm::rotate(_robotRootHome, glm::radians(-90.0f), glm::vec3(1, 0, 0));
		_robotRootHome = glm::translate(_robotRootHome, glm::vec3(0.0f, 0.0f, 0.0f));
		_robotRootPose = _robotRootHome;

		_robotQHome = _robot.makeJointVector();
		_robotHomeValid = true;

		instantiateRobotLinks();
		buildLinkIndex();

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

		updateRobotKinematics();
		D_INFO("Robot reset to home position.");
		D_SUCCESS("Robot reset to home position.");
	}

	// Method to clear the current robot from the scene
	void RobotSystem::clearRobot() {
		if (!_hasRobot) return;

		// Remove robot objects from _objects
		for (auto& link : _robot.links) {
			if (link.attachedObject) {
				// find and erase matching object
				_objects.erase(
					std::remove_if( _objects.begin(), _objects.end(), 
						[&](const std::unique_ptr<scene::Object>& obj) { return obj.get() == link.attachedObject; }),
					_objects.end()
				);
			}
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

	// --- ROBOT KINEMATICS AND JOINT STATE METHODS ---

	void RobotSystem::updateRobotKinematics() {
		if (!_hasRobot) return;

		std::vector<glm::mat4> world(_robot.links.size(), glm::mat4(1.0f));
		int rootIdx = _linkIndex.at("link00");
		world[rootIdx] = _robotRootPose;

		// parent -> children joints
		std::unordered_map<std::string, std::vector<const RobotJoint*>> children;
		children.reserve(_robot.joints.size());
		for (const auto& j : _robot.joints) children[j.parent].push_back(&j);

		std::stack<std::string> st;
		st.push("link00");

		while (!st.empty()) {
			std::string parentName = st.top(); st.pop();
			int pIdx = _linkIndex.at(parentName);

			auto it = children.find(parentName);
			if (it == children.end()) continue;

			for (const RobotJoint* jp : it->second) {
				const RobotJoint& j = *jp;
				int cIdx = _linkIndex.at(j.child);

				glm::mat4 T = glm::translate(glm::mat4(1.0f), j.origin_xyz);
				glm::mat4 R0 = glm::mat4_cast(j.origin_q);

				// axis_frame == "joint" means axis is in the joint frame AFTER origin rotation
				glm::vec3 axisWrtParent = glm::normalize(glm::vec3(R0 * glm::vec4(j.axis, 0.0f)));
				glm::mat4 Rq = glm::rotate(glm::mat4(1.0f), j.thetaRad, axisWrtParent);
				world[cIdx] = world[pIdx] * T * R0 * Rq;

				st.push(j.child);
			}
		}

		for (int i = 0; i < (int)_robot.links.size(); ++i) {
			if (auto* obj = _robot.links[i].attachedObject) {
				if (auto* mesh = obj->getMesh()) { mesh->localTransform = world[i]; }
			}
		}
	}

	// Method to get the angle of a specific robot joint
	bool RobotSystem::tryGetJointAngleRad(const std::string& childLink, float& outAngle) const {
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
	bool RobotSystem::trySetJointAngleRad(const std::string& childLink, float angleRad) {
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
	bool RobotSystem::tryGetJointOmegaRad(const std::string& childLink, float& outOmega) const {
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
	bool RobotSystem::trySetJointOmegaRad(const std::string& childLink, float omegaRad) {
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

	// Method to get the target angle (reference) of a specific robot joint in radians
	bool RobotSystem::tryGetJointTargetRad(const std::string& childLink, float& outTargetRad) const {
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
	bool RobotSystem::trySetJointTargetRad(const std::string& childLink, float targetRad) {
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

	//	Method to set the maximum angular velocity of a specific robot joint in radians
	bool RobotSystem::trySetJointOmegaMaxRad(const std::string& childLink, float maxOmegaRad) {
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
	bool RobotSystem::tryAddJointTargetRad(const std::string& childLink, float deltaRad) {
		if (!_hasRobot) { return false; }
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				float t = joint.thetaRefRad + deltaRad;
				if (joint.limits.continuous) { t = wrapRad(t); }
				else { t = glm::clamp(t, joint.limits.minAngle, joint.limits.maxAngle); }
				joint.thetaRefRad = t; // clamp to joint limits
				return true;
			}
		}
		return false;
	}

	// Method to set the reference angular velocity of a specific robot joint in radians
	bool RobotSystem::trySetJointOmegaRefRad(const std::string& childLink, float omegaRefRad) {
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
	bool RobotSystem::trySetJointAlphaRefRad(const std::string& childLink, float alphaRefRad) {
		if (!_hasRobot) { return false; }
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				joint.alphaRefRad_s2 = alphaRefRad;
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
	bool RobotSystem::isJointAtTargetRad(const std::string& childLink, float tolRad) const {
		if (!_hasRobot) { return false; }
		if (tolRad < 0.0f) { tolRad = -tolRad; }

		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				float err = joint.thetaRefRad - joint.thetaRad;
				if (joint.limits.continuous) { err = wrapToPi(err); }
				err = std::abs(err);
				return err <= tolRad;
			}
		}
		return false;
	}

	// Method to check if a specific robot joint is at its target angle within a tolerance (degrees)
	bool RobotSystem::isJointAtTargetDeg(const std::string& childLink, float tolDeg) const { 
		return isJointAtTargetRad(childLink, glm::radians(tolDeg)); 
	}

	// Method to check if a specific robot joint is near a target angle within a tolerance (radians)
	bool RobotSystem::isJointNearAngleRad(const std::string& childLink, float targetRad, float tolRad) const {
		if (!_hasRobot) { return false; }
		tolRad = std::abs(tolRad);

		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				float err = targetRad - joint.thetaRad;
				if (joint.limits.continuous) { err = wrapToPi(err); }
				return std::abs(err) <= tolRad;
			}
		}
		return false;
	}

	// Method to check if a specific robot joint is near a target angle within a tolerance (degrees)
	bool RobotSystem::isJointNearAngleDeg(const std::string& childLink, float targetDeg, float tolDeg) const { 
		return isJointNearAngleRad(childLink, glm::radians(targetDeg), glm::radians(tolDeg));
	}

	// --- ROBOT LINK AND ROOT POSE METHODS ---

	// Method to set the rotation angle of a specific robot link angle in degrees
	bool RobotSystem::setRobotLinkRotation(const std::string& childLinkName, float angleDeg) {
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
		glm::mat4 Align = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1, 0, 0));
		_robotRootPose = (T * R) * Align;
	}

	// Method to set the robot root home pose in world coordinates
	void RobotSystem::setRobotRootHome(const glm::vec3& pos, const glm::quat& rot) {
		glm::mat4 T = glm::translate(glm::mat4(1.0f), pos);
		glm::mat4 R = glm::mat4_cast(rot);
		glm::mat4 Align = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1, 0, 0));
		_robotRootPose = (T * R) * Align;
		_robotRootPose = _robotRootHome;
	}
} // namespace robots