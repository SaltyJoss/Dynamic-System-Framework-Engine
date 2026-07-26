// DSFE_GUI RigidBodyPresentationBuilder.h
#pragma once

#include <unordered_map>
#include <vector>
#include <memory>
#include <string>

namespace scene { class Object; }
namespace systems { struct RigidBodyModel; }

struct RigidBodyRenderBinding {
	std::vector<std::unique_ptr<scene::Object>> ownedObjects; // Objects owned by this binding (for proper memory management)
	std::unordered_map<std::string, std::vector<scene::Object*>> linkVisuals; // Map from link names to their visual objects
};

class RigidBodyPresentationBuilder {
public:
	static RigidBodyRenderBinding build(const systems::RigidBodyModel& model);
};