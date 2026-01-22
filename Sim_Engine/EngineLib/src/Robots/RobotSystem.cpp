#include "pch.h"

#include "Robots/RobotSystem.h"
#include "Robots/RobotLoader.h"
#include "Scene/Object.h"
#include "Scene/Mesh.h"

#include <stack>

#include <glm/gtc/matrix_transform.hpp>
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
		for (int r = 0; r < 4; ++r) {
			for (int c = 0; c < 4; ++c) { M[c][r] = static_cast<float>(T(r, c)); }
		}
		return M;
	}

	static std::vector<glm::mat4> computeVisualZeroWorld(const RobotModel& robot, const std::unordered_map<std::string, int>& linkIndx, const glm::mat4& rootPose) {
		std::vector<glm::mat4> world(robot.links.size(), glm::mat4(1.0f));

		int rootIndx = linkIndx.at("link00"); // assume first link is root (follows my convention)
		world[rootIndx] = rootPose;

		// Build parent -> list of outgoing joints
		std::unordered_map<std::string, std::vector<const RobotJoint*>> children;
		children.reserve(robot.joints.size());
		for (const auto& j : robot.joints)
			children[j.parent].push_back(&j);

		// DFS (or BFS)
		std::stack<std::string> st;
		st.push("link00");
		world[linkIndx.at("link00")] = rootPose;

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
		mathlib::VecX dxdt(2 * n);
		const double c = 2.0; // damping [1/s] -> placeholder for now

		for (int i = 0; i < n; ++i) {
			const double theta = x[i];
			const double omega = x[i + n];

			const RobotJoint& joint = _robot.joints[i];
			double thetaRef = static_cast<double>(joint.thetaRefRad);
			double err = thetaRef - theta;

			if (joint.limits.continuous) { err = robots::RobotSystem::wrapToPi(static_cast<float>(err)); } // wrap error for continuous joints

			dxdt[i] = omega; // dtheta/dt = omega

			// omega' = k_p * err - k_d * omega - c * omega -> PD control with damping
			const double k_p = static_cast<double>(joint.k_p);
			const double k_d = static_cast<double>(joint.k_d);
			double domega_dt = k_p * err - k_d * omega - c * omega;

			dxdt[i + n] = domega_dt;
		}
		return dxdt;
	}

	// Method to advance the robot state by dt using the selected integrator
	void RobotSystem::step(double dt, double simTime) {
		if (!_hasRobot) return;
		mathlib::VecX x = packState();

		auto f = [&](double t, const mathlib::VecX& xIn) { return deriv(t, xIn); };
		mathlib::VecX xNext = _integrator->stepODE(_curIntMethod, x, simTime, dt, f);
		
		unpackState(xNext);
		updateRobotKinematics();
	}

	// --- ROBOT LOADING AND RESET METHODS ---

	// Method to load a robot model by name
	void RobotSystem::loadRobot(const std::string& name) {
		clearRobot();

		std::string jsonPath = "Engine/assets/Objects/Robotic_Arm_Models/" + name + "/" + name + ".json";

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

		// Update kinematics to reflect initial state
		VecX q0 = _robot.makeJointVector();   // all angles at their defaults
		kinematics::Forward_Kinematics fk;
		std::vector<mathlib::Pose> TdH0 = fk.linkTransforms(_robot.dhParams, q0);

		// visual zero configuration world transforms
		std::vector<glm::mat4>  Wvis0 = computeVisualZeroWorld(_robot, _linkIndex, _robotRootPose);

		// Build Fix_i
		int rootIndx = _linkIndex["link00"]; // my JSON convention
		_robot.links[rootIndx].dhToMeshFix = glm::inverse(_robotRootPose) * Wvis0[rootIndx];

		for (int i = 0; i < static_cast<int>(TdH0.size()); ++i) {
			const int linkNumber = i + 1; // link00, link01, ...

			const std::string linkName = (linkNumber < 10) ? ("link0" + std::to_string(linkNumber)) : ("link" + std::to_string(linkNumber));
			auto it = _linkIndex.find(linkName);

			if (it == _linkIndex.end()) { continue; }
			const int linkIndx = it->second;

			glm::mat4 Tdh0 = poseToGlm(TdH0[i]);
			glm::mat4 Wdh0 = _robotRootPose * Tdh0;
			glm::mat4 Wv0 = Wvis0[linkIndx];

			_robot.links[linkIndx].dhToMeshFix = glm::inverse(Wdh0) * Wv0; // Fix_i = (T0_i)^-1 * Wv0
		}

		updateRobotKinematics();
		LOG_INFO("Loaded robot model -> %s", name.c_str());
		D_SUCCESS("Loaded robot model -> %s", name.c_str());

		LOG_INFO("links=%zu joints=%zu dhParams=%zu TdH0=%zu",
			_robot.links.size(), _robot.joints.size(), _robot.dhParams.size(), TdH0.size());
	}

	// Method to reset the robot to its home position
	void RobotSystem::resetRobot() {
		if (!_hasRobot || !_robotHomeValid) { return; }
		_robotRootPose = _robotRootHome;
		_robot.setJointVector(_robotQHome);

		for (auto& joint : _robot.joints) { /*I shall be adding state reset here :)*/ }

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
					std::remove_if(
						_objects.begin(),
						_objects.end(),
						[&](const std::unique_ptr<scene::Object>& obj) { return obj.get() == link.attachedObject; }
					),
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

	// --- ROBOT KINEMATICS AND JOINT STATE METHODS ---

	// Method to update robot link transforms based on joint angles (NEEDS TO BE REVISED BASED ON ROBOT STRUCTURE)
	void RobotSystem::updateRobotKinematics() {
		if (!_hasRobot) return;

		VecX q = _robot.makeJointVector();

		//for (size_t i = 0; i < _robot.joints.size(); ++i) { LOG_INFO("joints[%zu] name=%s angleRad=%.6f", i, _robot.joints[i].name.c_str(), _robot.joints[i].angle); }
		//LOG_INFO("q = [%.6f %.6f %.6f %.6f %.6f %.6f]", q[0], q[1], q[2], q[3], q[4], q[5]);

		kinematics::Forward_Kinematics fk;
		std::vector<mathlib::Pose> TdH = fk.linkTransforms(_robot.dhParams, q);

		auto worldDir = [](const glm::mat4& M, const glm::vec3& v) {
			return glm::normalize(glm::vec3(M * glm::vec4(v, 0.0f)));
			};

		auto zAxisFrom = [](const glm::mat4& M) {
			// GLM is column-major: M[2] is the 3rd column = local Z axis in world
			return glm::normalize(glm::vec3(M[2]));
			};

		// Build T0_prev for each joint.
		// Standard DH: joint i rotates about z_{i-1}, so use the PREVIOUS frame.
		// For i=0 (joint01), z_{0} is base Z, so use root pose.
		for (int i = 0; i < (int)_robot.joints.size(); ++i) {
			glm::mat4 T0_prev = (i == 0) ? _robotRootPose : (_robotRootPose * poseToGlm(TdH[i - 1]));

			glm::vec3 axis_joint = glm::normalize(_robot.joints[i].axis);
			glm::vec3 axis_world = glm::normalize(glm::vec3(glm::mat4_cast(_robot.joints[i].origin_q) * glm::vec4(axis_joint, 0.0f)));

			glm::vec3 axis_dh_world = zAxisFrom(T0_prev);
			float dot = glm::dot(axis_dh_world, axis_world);

			LOG_INFO("joint%02d axis: dh=(%.3f %.3f %.3f) json=(%.3f %.3f %.3f) dot=%.3f",
				i + 1,
				axis_dh_world.x, axis_dh_world.y, axis_dh_world.z,
				axis_world.x, axis_world.y, axis_world.z, dot
			);
		}

		std::vector<glm::mat4> world(_robot.links.size(), glm::mat4(1.0f));
		int rootIndx = _linkIndex["link00"];
		world[rootIndx] = _robotRootPose;

		// link00 (base)
		{
			auto* obj = _robot.links[rootIndx].attachedObject;
			if (obj && obj->getMesh()) {
				obj->getMesh()->localTransform = world[rootIndx];
			}
		}

		// link01, link02, ...
		for (int i = 0; i < static_cast<int>(_robot.joints.size()); ++i) {
			const std::string& linkName = _robot.joints[i].child;

			auto it = _linkIndex.find(linkName);
			if (it == _linkIndex.end()) continue;
			int linkIndx = it->second;

			glm::mat4 T0_i = poseToGlm(TdH[i]);
			glm::mat4 meshFix = glm::mat4_cast(_robot.links[linkIndx].dhToMeshFix);

			world[linkIndx] = _robotRootPose * T0_i * meshFix;
		}

		for (size_t i = 0; i < _robot.links.size(); i++) {
			auto* obj = _robot.links[i].attachedObject;
			if (!obj) { continue; }
			auto* mesh = obj->getMesh();
			if (!mesh) { continue; }
			mesh->localTransform = world[i];
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

	// Method to set the target angle (reference) of a specific robot joint in degrees
	bool RobotSystem::trySetJointTargetDeg(const std::string& childLink, float targetDeg) {
		if (!_hasRobot) { return false; }
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				float targetRad = glm::radians(targetDeg);
				if (joint.limits.continuous) { targetRad = wrapRad(targetRad); }
				else { targetRad = glm::clamp(targetRad, joint.limits.minAngle, joint.limits.maxAngle); }
				joint.thetaRefRad = targetRad; // clamp to joint limits
				return true;
			}
		}
		return false;
	}

	// Method to set the maximum angular velocity of a specific robot joint in degrees
	bool RobotSystem::trySetJointOmegeMaxDeg(const std::string& childLink, float maxOmegaDeg) {
		if (!_hasRobot) { return false; }
		float maxOmegaRad = glm::radians(maxOmegaDeg);
		for (auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				joint.limits.maxOmegaRad_s = std::abs(maxOmegaRad); // |omega[max]|
				return true;
			}
		}
		return false;
	}

	// --- ROBOT LINK AND ROOT POSE METHODS ---

	// Method to set the rotation angle of a specific robot link angle in degrees
	void RobotSystem::setRobotLinkRotation(const std::string& linkName, float angle) {
		if (!_hasRobot) { return; }

		auto it = _linkIndex.find(linkName);
		if (it == _linkIndex.end()) { return; }

		// Find joint child matching linkName
		for (auto& joint : _robot.joints) {
			if (joint.child == linkName) {
				float a = glm::radians(angle); // stores angle in radians
				joint.angleRad = clampJointAngle(joint, a); // clamp to joint limits
				D_INFO_ONCE("%s -> %.2f degrees.", linkName.c_str(), angle);
				return;
			}
		}
		LOG_WARN_ONCE("No joint found for link %s to set rotation.", linkName.c_str());
		D_WARN_ONCE("No joint found for link %s to set rotation.", linkName.c_str());
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