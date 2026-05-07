// DSFE_GUI RobotRenderer.h
#pragma once

#include <unordered_map>
#include <functional>
#include <vector>
#include <string>

#include <core/Types.h>

namespace scene { class Object; }
namespace robots { struct RobotModel; }

struct LinkRenderData {
	std::vector<scene::Object*> visuals; // Visual objects associated with this link
	std::vector<scene::Object*> collisions; // Collision objects associated with this link (not implemented yet)
};

class RobotRenderer {
public:
	using spawnFn = std::function<std::vector<scene::Object*>(const std::string&)>; // function type for loading meshes

	void instantiateRobotLinks(const robots::RobotModel& robot);
	void applyTransforms(const robots::RobotModel& robot, const std::vector<mathlib::Mat4>& world);

	//void clearRobot();

private:
	std::unordered_map<std::string, LinkRenderData> linkRenderMap; // Map from link names to their render data
	spawnFn _loadMeshReturn;
};