#include "pch.h"

#include "Robots/RobotSystem.h"
#include "Robots/RobotLoader.h"
#include "Scene/Object.h"
#include "Scene/Mesh.h"

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
		if (joint.continuous) { return wrapRad(angleRad); }
		else { return glm::clamp(angleRad, joint.minAngle, joint.maxAngle); }
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

	// --- ROBOT STATE INTEGRATION METHODS ---

	// Method to create Object instances for each robot link
	void RobotSystem::instantiateRobotLinks() {
		for (auto& link : _robot.links) {
			auto objs = _loadMeshReturn(link.meshFile);
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
			x[i]	 = static_cast<double>(_robot.joints[i].angle);
			x[i + n] = static_cast<double>(_robot.joints[i].omega);
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

			float wMax = std::abs(_robot.joints[i].maxOmega); // max |omega|
			if (wMax > 0.0f) { omega = glm::clamp(omega, -wMax, wMax); }

			_robot.joints[i].angle = theta;
			_robot.joints[i].omega = omega;
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
			double thetaRef = static_cast<double>(joint.thetaRef);
			double err = thetaRef - theta;

			if (joint.continuous) { err = robots::RobotSystem::wrapToPi(static_cast<float>(err)); } // wrap error for continuous joints

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

		{
			VecX q = _robot.makeJointVector();   // all angles at their defaults
			kinematics::Forward_Kinematics fk;

			mathlib::Pose T_ee = fk.FK(_robot.dhParams, q);

			LOG_INFO("FK zero config EE: x=%.4f y=%.4f z=%.4f", T_ee(0, 3), T_ee(1, 3), T_ee(2, 3));
			D_DEBUG("FK zero config EE: x=%.4f y=%.4f z=%.4f", T_ee(0, 3), T_ee(1, 3), T_ee(2, 3));
		}

		for (std::size_t i = 0; i < _robot.dhParams.size(); ++i) {
			const auto& p = _robot.dhParams[i];
			LOG_INFO("DH[%zu]: a=%.4f alpha=%.4f d=%.4f theta=%.4f type=%s",
				i, p.a, p.alpha, p.d, p.theta,
				p.type == kinematics::JointType::Revolute ? "R" : "P");
			D_DEBUG("DH[%zu]: a=%.4f alpha=%.4f d=%.4f theta=%.4f type=%s",
				i, p.a, p.alpha, p.d, p.theta,
				p.type == kinematics::JointType::Revolute ? "R" : "P");
		}

		instantiateRobotLinks();
		buildLinkIndex();

		LOG_INFO("Loaded robot model -> %s", name.c_str());
		D_SUCCESS("Loaded robot model -> %s", name.c_str());
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

		// Get current joint angles as Eigen vector
		VecX q = _robot.makeJointVector();

		kinematics::Forward_Kinematics fk;
		std::vector<mathlib::Pose> TdH = fk.linkTransforms(_robot.dhParams, q);

		// World transforms for each link
		std::vector<glm::mat4> world(_robot.links.size(), glm::mat4(1.0f));
		int rootIndx = _linkIndex["link00"];  // All robotic arms have link00 as root link (my JSON convention)
		world[rootIndx] = _robotRootPose;


		for (int i = 0; i < static_cast<int>(TdH.size()); ++i) {
			const int linkNumber = i + 1; // link01, link02, ...
			const std::string childName = (linkNumber < 10) ? ("link0" + std::to_string(linkNumber)) : ("link" + std::to_string(linkNumber));
			auto it = _linkIndex.find(childName);
			if (it == _linkIndex.end()) { continue; }
			const int childIndx = it->second;

			glm::mat4 T0_i = poseToGlm(TdH[i]);
			glm::mat4 meshFix = _robot.links[childIndx].meshFix;

			world[childIndx] = world[rootIndx] * T0_i * meshFix;
		}

		for (size_t i = 0; i < _robot.links.size(); i++) {
			auto* obj = _robot.links[i].attachedObject;
			if (!obj) { continue; }
			auto* mesh = obj->getMesh();
			if (!mesh) { continue; }

			mesh->localTransform = world[i];
		}


		{
			// Find end-effector link index
			const int eeLinkNumber = static_cast<int>(TdH.size()); // link0n
			const std::string eeName = (eeLinkNumber < 10) ? ("link0" + std::to_string(eeLinkNumber)) : ("link" + std::to_string(eeLinkNumber));
			auto itEE = _linkIndex.find(eeName);
			// Calculate and log robot reach
			if (itEE != _linkIndex.end()) {
				const int eeIndx = itEE->second;
				glm::vec3 p_Base = glm::vec3(world[rootIndx][3]); // position of base link
				glm::vec3 p_EE = glm::vec3(world[eeIndx][3]);     // position of end-effector link
				float reach = glm::length(p_EE - p_Base);
				LOG_INFO_ONCE("Robot reach: %.4f metres", reach);
			}
		}
	}

	// Method to get the angle of a specific robot joint
	bool RobotSystem::tryGetJointAngleRad(const std::string& childLink, float& outAngle) const {
		if (!_hasRobot) { return false; }
		// Find joint child matching childLink
		for (const auto& joint : _robot.joints) {
			if (joint.child == childLink) {
				outAngle = joint.angle;
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
				joint.angle = clampJointAngle(joint, angleRad); // clamp to joint limits
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
				outOmega = joint.omega;
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
				joint.omega = omegaRad;
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
				if (joint.continuous) { targetRad = wrapRad(targetRad); }
				else { targetRad = glm::clamp(targetRad, joint.minAngle, joint.maxAngle); }
				joint.thetaRef = targetRad; // clamp to joint limits
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
				joint.maxOmega = std::abs(maxOmegaRad); // |omega[max]|
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
				joint.angle = clampJointAngle(joint, a); // clamp to joint limits
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