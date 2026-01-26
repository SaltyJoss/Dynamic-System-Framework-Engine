#include "pch.h"

#include "Robots/RobotSystem.h"
#include "Robots/RobotLoader.h"
#include "Scene/Object.h"
#include "Scene/Mesh.h"

#include <stack>

#include <glm/gtc/matrix_transform.hpp>
#include <Core/Utils.h>
#include <kinematics/Forward_Kinematics.h>

#include "EngineLib/LogMacros.h"

using namespace mathlib;
using namespace constants;

namespace robots {
	// Constructor
	RobotSystem::RobotSystem(std::vector<std::unique_ptr<scene::Object>>& objects, spawnFn meshLoader)
		: _integrator(std::make_unique<integration::IntegrationService>()), _refSolver(std::make_unique<integration::ReferenceSolver>()), 
		  _curIntMethod(integration::eIntegrationMethod::Euler), _objects(objects), _loadMeshReturn(std::move(meshLoader)) {
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
		angleRad = std::fmod(angleRad + PI, TWO_PI);
		if (angleRad < 0.0f) angleRad += TWO_PI;
		return angleRad - PI;
	}

	// Method to wrap an angle in radians to the range [0, 2pi]
	float RobotSystem::wrapRad(float angleRad) {
		angleRad = fmod(angleRad, TWO_PI);
		if (angleRad < 0.0f) angleRad += TWO_PI;
		return angleRad;
	}


	// Convert mathlib::Pose to glm::mat4
	static glm::mat4 poseToGlm(const mathlib::Pose& T) {
		glm::mat4 M(1.0f);
		for (int r = 0; r < 4; ++r) { for (int c = 0; c < 4; ++c) { M[c][r] = static_cast<float>(T(r, c)); } }
		return M;
	}

	// Method to create a rotation matrix about a pivot point
	static glm::mat4 rotAboutPivot(const glm::vec3& pivot, const glm::vec3& axisUnit, float angleRad) {
		glm::mat4 T = glm::translate(glm::mat4(1.0f), pivot);
		glm::mat4 Ti = glm::translate(glm::mat4(1.0f), -pivot);
		glm::mat4 R = glm::rotate(glm::mat4(1.0f), angleRad, axisUnit);
		return T * R * Ti;
	}

	// Method to compute the world transforms of all robot links at zero joint angles (Not needed nor used currently)
	static std::vector<glm::mat4> computeVisualZeroWorld(const RobotModel& robot, const std::unordered_map<std::string, int>& linkIndx, const glm::mat4& rootPose) {
		std::vector<glm::mat4> world(robot.links.size(), glm::mat4(1.0f));

		int rootIndx = linkIndx.at("link00"); // assume first link is root (follows my convention)
		world[rootIndx] = rootPose;

		// Build parent -> list of outgoing joints
		std::unordered_map<std::string, std::vector<const RobotJoint*>> children;
		children.reserve(robot.joints.size());
		for (const auto& j : robot.joints) { children[j.parent].push_back(&j); }

		// DFS (or BFS)
		std::stack<std::string> st;
		st.push("link00"); // start from root
		world[linkIndx.at("link00")] = rootPose; // set root pose
		
		// Traverse the tree, !st.empty() ensures we process all links, including branches (meaning multiple children)
		while (!st.empty()) {
			std::string parentName = st.top(); st.pop();
			int parentIndx = linkIndx.at(parentName);

			auto it = children.find(parentName);
			if (it == children.end()) continue;

			for (const RobotJoint* jp : it->second) {
				const RobotJoint& joint = *jp;
				int childIndx = linkIndx.at(joint.child);

				glm::mat4 T = glm::translate(glm::mat4(1.0f), joint.origin_xyz);
				glm::mat4 R = glm::mat4_cast(joint.origin_q);
				glm::mat4 Rq = glm::rotate(glm::mat4(1.0f), joint.angleRad, glm::normalize(joint.axis));

				world[childIndx] = world[parentIndx] * T * R * Rq;
			
				st.push(joint.child);
			}
		}

		return world;
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
		const int n = static_cast<int>(_robot.joints.size());
		mathlib::VecX x(2 * n);
		for (int i = 0; i < n; ++i) {
			x[i]	 = static_cast<double>(_robot.joints[i].angleRad);
			x[i + n] = static_cast<double>(_robot.joints[i].omegaRad_s);
		}
		return x;
	}
	
	// Method to unpack state vector into robot joints
	void RobotSystem::unpackState(const mathlib::VecX& x) {
		const int n = static_cast<int>(_robot.joints.size());
		for (int i = 0; i < n; ++i) {
			float theta = static_cast<float>(x[i]);
			float omega = static_cast<float>(x[i + n]);

			theta = clampJointAngle(_robot.joints[i], theta);

			float wMax = std::abs(_robot.joints[i].limits.maxOmegaRad_s); // max |omega|
			if (wMax > 0.0f) { omega = glm::clamp(omega, -wMax, wMax); }

			_robot.joints[i].angleRad = theta;
			_robot.joints[i].omegaRad_s = omega;
		}
	}

	// Derivative function for ODE integration
	mathlib::VecX RobotSystem::deriv(double t, const mathlib::VecX& x) const {
		const int n = static_cast<int>(_robot.joints.size());
		mathlib::VecX dx(2 * n);
		const double c = 2.0; // damping [1/s] -> placeholder for now

		for (int i = 0; i < n; ++i) {
			const double theta = x[i];
			const double omega = x[i + n];

			// Targets and gains
			const RobotJoint& joint = _robot.joints[i];
			const double thetaRef = static_cast<double>(joint.thetaRefRad);
			const double k_p = static_cast<double>(joint.k_p);
			const double k_d = static_cast<double>(joint.k_d);

			// Error
			double err = thetaRef - theta;

			// PD 
			double tau = k_p * err - k_d * omega; // control torque

			// Passive dynamics
			const double damping = static_cast<double>(joint.dynamics.damping);
			const double friction = static_cast<double>(joint.dynamics.friction);

			tau -= damping * omega;
			
			// Friction model
			const double v_eps = 1e-2; // small velocity threshold
			if (std::abs(omega) > v_eps) { tau -= friction * sgn(omega); } // Coulomb friction
			else { tau -= friction * (omega / v_eps); } // linear region near zero

			// Effort clamp
			if (joint.limits.maxEffort > 0.0f) {
				const double e = static_cast<double>(joint.limits.maxEffort);
				if (tau > e) { tau = e; }
				if (tau < -e) { tau = -e; }
			}

			// Effective inertia (assumed 1.0 as placeholder, I aim to extend this later)
			const double I_eff = 1.0;	// TODO: per joint effective inertia
			double alpha = tau / I_eff; // angular acceleration

			// Omega clamp
			const double wMax = static_cast<double>(joint.limits.maxOmegaRad_s);
			if (wMax > 0.0) {
				if ((omega >= wMax && alpha > 0.0) || (omega <= -wMax && alpha < 0.0)) { alpha = 0.0; }
			}

			dx[i] = omega;		// dtheta/dt = omega
			dx[i + n] = alpha;	// domega/dt = alpha

			LOG_INFO("j03 theta=%.4f ref=%.4f err=%.4f omega=%.6f kp=%.2f kd=%.2f fric=%.4f damp=%.4f tau=%.4f",
				theta, thetaRef, err, omega, k_p, k_d, friction, damping, tau);
			SIM_ROTATE("j03 theta=%.4f ref=%.4f err=%.4f omega=%.6f kp=%.2f kd=%.2f fric=%.4f damp=%.4f tau=%.4f",
				theta, thetaRef, err, omega, k_p, k_d, friction, damping, tau);
		}
		return dx;
	}

	// Method to enforce joint limits after integration
	void RobotSystem::enforceJointLimits(RobotJoint& j) {
		if (j.limits.continuous) { return; }

		const float lo = j.limits.minAngle;
		const float hi = j.limits.maxAngle;

		if (j.angleRad < lo) { j.angleRad = lo; if (j.omegaRad_s < 0.0f) { j.omegaRad_s = 0.0f; }}
		if (j.angleRad > hi) { j.angleRad = hi; if (j.omegaRad_s > 0.0f) { j.omegaRad_s = 0.0f; }}
	}

	// Method to advance the robot state by dt using the selected integrator
	void RobotSystem::step(double dt, double simTime) {
		if (!_hasRobot) return;

		_simTime = simTime;

		// Pack current state
		mathlib::VecX x = packState(); // current state vector

		// Define the derivative function
		auto f = [&](double t, const mathlib::VecX& xIn) { return deriv(t, xIn); };
		mathlib::VecX x_Next = _integrator->stepODE(_curIntMethod, x, simTime, dt, f);
		
		// Unpack new state
		unpackState(x_Next);

		// Enforce joint limits
		for (auto& j : _robot.joints) {
			const float wMax = j.limits.maxOmegaRad_s;
			if (wMax > 0.0f) { j.omegaRad_s = glm::clamp(j.omegaRad_s, -wMax, wMax); }
			enforceJointLimits(j);
		}

		// Update kinematics
		updateRobotKinematics();
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
			joint.thetaRefRad = joint.angleRad;
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
			joint.thetaRefRad = joint.angleRad;
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
				glm::mat4 Rq = glm::rotate(glm::mat4(1.0f), j.angleRad, axisWrtParent);
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
				outAngle = joint.angleRad;
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
				joint.angleRad = clampJointAngle(joint, angleRad); // clamp to joint limits
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

	// Method to check if a specific robot joint is at its target angle within a tolerance (radians)
	bool RobotSystem::isJointAtTargetRad(const std::string& childLink, float tolRad) const {
		if (!_hasRobot) { return false; }
		if (tolRad < 0.0f) { tolRad = -tolRad; }

		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				float err = joint.thetaRefRad - joint.angleRad;
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
				float err = targetRad - joint.angleRad;
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
				j.angleRad = glm::radians(angleDeg);
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
} // namespace robot