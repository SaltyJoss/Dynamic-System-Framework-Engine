
#include "pch.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <kinematics/Forward_Kinematics.h>
#include "Robots/RobotLoader.h"
#include "Scene/Object.h"
#include "Scene/MeshLoader.h"
#include "EngineLib/LogMacros.h"

#include <nlohmann/json.hpp>

using json = nlohmann::json;
using kinematics::DH_Params;
using kinematics::JointType;

namespace robots {
	// --- Static Helper Functions ---

	// Find a link by name in the robot model
	static RobotLink* findLink(std::vector<RobotLink>& links, const std::string& name) {
		for (auto& link : links) { if (link.name == name) { return &link; } }
		LOG_WARN("Link %s not found in robot model", name.c_str());
		return nullptr;
	}

	// Parse quaternion from joint data
	static glm::quat parseQuaternion(const json& jointData) {
		if (jointData.contains("quat") && jointData["quat"].is_array() && jointData["quat"].size() == 4) {
			const auto& q = jointData["quat"];
			return glm::normalize(glm::quat(
				q[0].get<float>(), // w
				q[1].get<float>(), // x
				q[2].get<float>(), // y
				q[3].get<float>()  // z
			));
		}

		return glm::quat(1.0, 0.0, 0.0, 0.0);
	}

	// Parse DH joint type from string
	static JointType parseDHType(const std::string& s) {
		std::string t = s;
		for (char& c : t) { c = static_cast<char>(std::tolower((unsigned char)c)); }
		if (t == "revolute" || t == "r") { return JointType::Revolute; }
		return JointType::Prismatic;
	}

	// --- RobotLoader Implementation ---

	// Load a robot model from a JSON file
	robots::RobotModel RobotLoader::loadFromJSON(const std::string& filepath) {
		RobotModel robot;
		LOG_INFO("Loading robot model from JSON: %s", filepath.c_str());

		std::ifstream file(filepath);
		if (!file.is_open()) {
			LOG_ERROR("Failed to open JSON file: %s", filepath.c_str());
			return robot;
		}
		json data = json::parse(file);

		robot.name = data["name"].get<std::string>();
		robot.scale = data["scale"].get<float>();

		for (auto& linkData : data["links"]) {
			RobotLink link;
			link.name = linkData["name"].get<std::string>();
			link.meshFile = linkData["mesh"].get<std::string>();
			robot.links.push_back(link);
		}

		// Load joints
		for (auto& jointData : data["joints"]) {
			RobotJoint joint;

			// Load basic joint info
			joint.name = jointData["name"].get<std::string>();
			joint.parent = jointData["parent"].get<std::string>();
			joint.child = jointData["child"].get<std::string>();

			// not used currently - will delete when confirmed that new logic works
			joint.axis = glm::vec3(
				jointData["axis"][0].get<float>(),
				jointData["axis"][1].get<float>(),
				jointData["axis"][2].get<float>()
			);
			// not used currently - will delete when confirmed that new logic works
			joint.offset = glm::vec3(
				jointData["offset"][0].get<float>(),
				jointData["offset"][1].get<float>(),
				jointData["offset"][2].get<float>()
			);

			// Load initial orientation quaternion
			joint.quat = parseQuaternion(jointData);
			
			// Load joint limits
			json limits = jointData.contains("limit") ? jointData["limit"] : json::object();

			if (limits.contains("continuous")) { joint.continuous = limits["continuous"].get<bool>(); }
			else { joint.continuous = false; }

			if (limits.contains("max_speed") && limits["max_speed"].is_number()) { joint.maxOmega = glm::radians(limits["max_speed"].get<float>()); }
			else { joint.maxOmega = glm::radians(180.0f); } // default 180 deg/s
			
			if (!joint.continuous) {
				if (limits.contains("lower_limit") && limits["lower_limit"].is_number()) { joint.minAngle = glm::radians(limits["lower_limit"].get<float>()); }
				if (limits.contains("upper_limit") && limits["upper_limit"].is_number()) { joint.maxAngle = glm::radians(limits["upper_limit"].get<float>()); }
			} else {
				joint.minAngle = glm::radians(-359.9f); // practically continuous, not full 360 to avoid singularity
				joint.maxAngle = glm::radians( 359.9f);	// practically continuous, not full 360 to avoid singularity
			}

			robot.joints.push_back(joint);

			// Load DH parameters (kinematics)
			DH_Params dhp{};
			// Load DH parameters if available
			if (jointData.contains("dh")) {
				auto& dhData = jointData["dh"];

				dhp.a	  = dhData["a"].get<double>();
				dhp.alpha = dhData["alpha"].get<double>();
				dhp.d	  = dhData["d"].get<double>();
				dhp.theta = dhData["theta"].get<double>();

				std::string typeStr = dhData.value("type", "revolute");
				dhp.type = parseDHType(typeStr);

			} else {
				LOG_WARN("Joint %s missing 'dh' block; using zero DH", joint.name.c_str());
				dhp.a = 0.0; dhp.alpha = 0.0; dhp.d = 0.0; dhp.theta = 0.0; dhp.type = JointType::Revolute;
			}

			robot.dhParams.push_back(dhp);

	
			
			LOG_INFO("Joint: %s | Parent: %s, | Child: %s, | Continuous: %s, | Max Speed: %.2f, | Min Angle: %.2f, | Max Angle: %.2f",
				joint.name.c_str(), joint.parent.c_str(), joint.child.c_str(), joint.continuous ? "True" : "False", joint.maxOmega, joint.minAngle, joint.maxAngle);
			LOG_INFO("Joint %s mesh fix quat = (w=%.3f x=%.3f y=%.3f z=%.3f)", joint.name.c_str(), joint.quat.w, joint.quat.x, joint.quat.y, joint.quat.z);
			LOG_INFO("DH Params for Joint %s: a=%.4f, alpha=%.4f, d=%.4f, theta=%.4f, type=%s",
				joint.name.c_str(), dhp.a, dhp.alpha, dhp.d, dhp.theta,
				dhp.type == JointType::Revolute ? "revolute" : "prismatic");

			D_INFO("Joint: %s | Parent: %s, | Child: %s, | Continuous: %s, | Max Speed: %.2f, | Min Angle: %.2f, | Max Angle: %.2f",
				joint.name.c_str(), joint.parent.c_str(), joint.child.c_str(), joint.continuous ? "True" : "False", joint.maxOmega, joint.minAngle, joint.maxAngle);
			D_INFO("Joint %s mesh fix quat = (w=%.3f x=%.3f y=%.3f z=%.3f)", joint.name.c_str(), joint.quat.w, joint.quat.x, joint.quat.y, joint.quat.z);
			D_INFO("DH Params for Joint %s: a=%.4f, alpha=%.4f, d=%.4f, theta=%.4f, type=%s",
				joint.name.c_str(), dhp.a, dhp.alpha, dhp.d, dhp.theta,
				dhp.type == JointType::Revolute ? "revolute" : "prismatic");
		}

		LOG_INFO("Robot loaded: %d links, %d joints", (int)robot.links.size(), (int)robot.joints.size());
		D_SUCCESS("Robot loaded: %d links, %d joints", (int)robot.links.size(), (int)robot.joints.size());
		return robot;
	}
}