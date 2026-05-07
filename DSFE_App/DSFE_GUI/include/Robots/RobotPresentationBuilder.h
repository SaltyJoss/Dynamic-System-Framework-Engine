// DSFE_GUI RobotPresentationBuilder.h
#pragma once

#include <unordered_map>
#include <vector>
#include <memory>
#include <string>

namespace scene { class Object; }
namespace robots { struct RobotModel; }

struct RobotRenderBinding {
	std::unordered_map<std::string, std::vector<scene::Object*>> linkVisuals; // Map from link names to their visual objects
};

class RobotPresentationBuilder {
public:
	static RobotRenderBinding build(const robots::RobotModel& model);
};