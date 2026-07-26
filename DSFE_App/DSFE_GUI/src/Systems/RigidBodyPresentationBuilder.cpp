// DSFE_GUI RobotPresentationBuilder.cpp
#include "Robots/RobotPresentationBuilder.h"

#include "Robots/RobotModel.h"

#include "Assets/MeshLoader.h"
#include "Scene/Object.h"

#include <filesystem>
#include "Platform/Paths.h"

using namespace systems;

namespace fs = std::filesystem;

// Build a RobotRenderBinding from a RobotModel by loading the visual meshes for each link
RobotRenderBinding RobotPresentationBuilder::build(const systems::RigidBodyModel& model) {
	RigidBodyRenderBinding binding;
	assets::MeshLoader loader;
	for (const auto& link : model.links) {
		auto& visuals = binding.linkVisuals[link.name];
		for (const auto& mesh : link.visual.meshEntries) {
			fs::path fullPath = paths::assets() / "objects" / "Robotic_Arm_Models" / mesh.meshFile;
			auto meshes = loader.load(fullPath.string());
			for (auto& m : meshes) {
				auto obj = std::make_unique<scene::Object>(m);
				obj->name = link.name;
				obj->category = scene::ObjectCategory::RobotLink;
				obj->transform.scale = glm::vec3(model.scale);
				visuals.push_back(obj.get()); // Store raw pointer for rendering
				binding.ownedObjects.push_back(std::move(obj)); // Cache unique_ptr for memory management
			}
		}
	}
	return binding;
}