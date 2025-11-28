
#include "pch.h"
#include "Robots/RobotLoader.h"
#include "Scene/Object.h"
#include "Scene/MeshLoader.h"
#include "EngineLib/LogMacros.h"

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace robots {
	RobotModel RobotLoader::loadFromJSON(const std::string& filepath) {
		RobotModel robot;
		LOG_INFO("Loading robot model from JSON: %s", filepath.c_str());

		std::ifstream file(filepath);
		if (!file.is_open()) {
			LOG_ERROR("Failed to open JSON file: %s", filepath.c_str());
			return robot;
		}
		json data = json::parse(file);

		robot.scale = data["scale"].get<float>();

		for (auto& linkData : data["links"]) {
			RobotLink link;
			link.name = linkData["name"].get<std::string>();
			link.meshFile = linkData["mesh"].get<std::string>();
			robot.links.push_back(link);
		}

		for (auto& jointData : data["joints"]) {
			RobotJoint joint;

			joint.name = jointData["name"].get<std::string>();
			joint.parent = jointData["parent"].get<std::string>();
			joint.child = jointData["child"].get<std::string>();

			joint.axis = glm::vec3(
				jointData["axis"][0].get<float>(),
				jointData["axis"][1].get<float>(),
				jointData["axis"][2].get<float>()
			);

			joint.offset = glm::vec3(
				jointData["offset"][0].get<float>(),
				jointData["offset"][1].get<float>(),
				jointData["offset"][2].get<float>()
			);
			
			json limits = jointData.contains("limit") ? jointData["limit"] : json::object();

			if (limits.contains("continuous")) { joint.continuous = limits["continuous"].get<bool>(); }
			else { joint.continuous = false; }
			
			if (!joint.continuous) {
				if (limits.contains("lower_limit") && limits["lower_limit"].is_number()) { joint.minAngle = limits["lower_limit"].get<float>(); }
				if (limits.contains("upper_limit") && limits["upper_limit"].is_number()) { joint.maxAngle = limits["upper_limit"].get<float>(); }
			} else {
				joint.minAngle = -360.0f;
				joint.maxAngle = 360.0f;
			}

			// joint.minAngle = glm::radians(joint.minAngle);
			// joint.maxAngle = glm::radians(joint.maxAngle);

			robot.joints.push_back(joint);

			LOG_INFO("Joint: %s \n\t\t| Parent: %s, \n\t\t| Child: %s, \n\t\t| Continuous: %s, \n\t\t| Min: %.2f, \n\t\t| Max: %.2f",
				joint.name.c_str(),
				joint.parent.c_str(),
				joint.child.c_str(),
				joint.continuous ? "True" : "False",
				joint.minAngle,
				joint.maxAngle
			);
			D_SUCCESS("Joint: %s \n\t\t\t| Parent: %s\n\t\t\t| Child: %s, \n\t\t\t| Continuous: %s, \n\t\t\t| Min: %.2f, \n\t\t\t| Max: %.2f",
				joint.name.c_str(),
				joint.parent.c_str(),
				joint.child.c_str(),
				joint.continuous ? "True" : "False",
				joint.minAngle,
				joint.maxAngle
			);
		}

		LOG_INFO("Robot loaded: %d links, %d joints", (int)robot.links.size(), (int)robot.joints.size());
		return robot;
	}
}