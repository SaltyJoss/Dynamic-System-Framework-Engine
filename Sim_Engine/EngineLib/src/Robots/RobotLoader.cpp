
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

			/*joint.continuous = jointData["continuous"].get<bool>();
			if (!joint.continuous) {
				joint.minAngle = jointData["minAngle"].get<float>();
				joint.maxAngle = jointData["maxAngle"].get<float>();
			} else {
				joint.minAngle = -FLT_MAX;
				joint.maxAngle = FLT_MAX;
			}

			joint.minAngle = glm::radians(joint.minAngle);
			joint.maxAngle = glm::radians(joint.maxAngle);*/

			robot.joints.push_back(joint);
		}

		LOG_INFO("Robot loaded: %d links, %d joints",
			(int)robot.links.size(),
			(int)robot.joints.size());

		return robot;
	}
}