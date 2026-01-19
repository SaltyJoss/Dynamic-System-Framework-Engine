#include "pch.h"

#include "Robots/RobotSystem.h"
#include "Robots/RobotLoader.h"
#include "Scene/Object.h"
#include "Scene/Mesh.h"

#include <glm/gtc/matrix_transform.hpp>
#include <kinematics/Forward_Kinematics.h>

#include "EngineLib/LogMacros.h"

namespace robots {
	RobotSystem::RobotSystem(std::vector<std::unique_ptr<scene::Object>>& objects, spawnFn meshLoader)
		: _objects(objects), _loadMeshReturn(std::move(meshLoader)) {
	}

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

	// Method to update robot link transforms based on joint angles (NEEDS TO BE REVISED BASED ON ROBOT STRUCTURE)
	void RobotSystem::updateRobotKinematics() {
		if (!_hasRobot) return;

		// Get current joint angles as Eigen vector
		VecX q = _robot.makeJointVector();

		// World transforms for each link
		std::vector<glm::mat4> world(_robot.links.size(), glm::mat4(1.0f));

		glm::vec3 pBase = glm::vec3(world[_linkIndex["link00"]][3]);
		glm::vec3 pEE = glm::vec3(world[_linkIndex["link06"]][3]);

		float reach = glm::length(pEE - pBase);
		LOG_INFO_ONCE("Reach link00->link06 origin = %.3f world units", reach);

		int rootIdx = _linkIndex["link00"];  // Z1 root link (base static link)
		world[rootIdx] = _robotRootPose;

		// Sort joints in parent-to-child order
		std::vector<RobotJoint> sorted = _robot.joints;

		std::sort(sorted.begin(), sorted.end(),
			[&](const RobotJoint& a, const RobotJoint& b) {
				int a_parent_indx = _linkIndex[a.parent];
				int b_parent_indx = _linkIndex[b.parent];
				return a_parent_indx < b_parent_indx;
			});

		for (auto& joint : sorted) {
			int parent = _linkIndex[joint.parent];
			int child = _linkIndex[joint.child];

			glm::mat4 T_offset = glm::translate(glm::mat4(1.0f), joint.offset); // meters
			glm::mat4 R_joint = glm::rotate(glm::mat4(1.0f), joint.angle, glm::normalize(joint.axis));
			glm::mat4 R_align = glm::mat4_cast(joint.quat); // from JSON
			world[child] = world[parent] * T_offset * R_align * R_joint;
		}

		// Update link object transforms
		for (size_t i = 0; i < _robot.links.size(); i++) {
			auto* obj = _robot.links[i].attachedObject;
			auto* mesh = obj->getMesh();
			if (!mesh) continue;

			mesh->localTransform = world[i];
		}
	}

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

	float RobotSystem::clampJointAngle(const RobotJoint& joint, float angleRad) {
		if (joint.continuous) { return wrapRad(angleRad); }
		else { return glm::clamp(angleRad, joint.minAngle, joint.maxAngle); }
	}

	float RobotSystem::wrapToPi(float angleRad) {
		angleRad = std::fmod(angleRad + PI, TWO_PI);
		if (angleRad < 0.0f) angleRad += TWO_PI;
		return angleRad - PI;
	}

	float RobotSystem::wrapRad(float angleRad) {
		angleRad = fmod(angleRad, TWO_PI);
		if (angleRad < 0.0f) angleRad += TWO_PI;
		return angleRad;
	}

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

	void RobotSystem::setRobotRootPose(const glm::vec3& pos, const glm::quat& rot) {
		glm::mat4 T = glm::translate(glm::mat4(1.0f), pos);
		glm::mat4 R = glm::mat4_cast(rot);
		glm::mat4 Align = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1, 0, 0));
		_robotRootPose = (T * R) * Align;
	}

	void RobotSystem::setRobotRootHome(const glm::vec3& pos, const glm::quat& rot) {
		glm::mat4 T = glm::translate(glm::mat4(1.0f), pos);
		glm::mat4 R = glm::mat4_cast(rot);
		glm::mat4 Align = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f), glm::vec3(1, 0, 0));
		_robotRootPose = (T * R) * Align;
		_robotRootPose = _robotRootHome;
	}

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
						[&](const std::unique_ptr<scene::Object>& obj) {
							return obj.get() == link.attachedObject;
						}),
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
} // namespace robot