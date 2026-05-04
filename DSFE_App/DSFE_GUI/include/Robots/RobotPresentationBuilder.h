// DSFE_GUI RobotPresentationBuilder.h
#pragma once

#include "RobotModel.h"
#include <unordered_map>
#include <vector>
#include <memory>

namespace scene { class Object; }

struct RobotRenderBinding {
	std::unordered_map<std::string, std::vector<scene::Object*>> linkVisuals; // Map from link names to their visual objects
};

class RobotPresentationBuilder {
public:
	static RobotRenderBinding build(const RobotModel& model);
};