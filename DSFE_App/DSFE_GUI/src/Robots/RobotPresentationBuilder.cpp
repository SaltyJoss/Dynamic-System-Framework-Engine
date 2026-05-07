// DSFE_GUI RobotPresentationBuilder.cpp
#include "Robots/RobotPresentationBuilder.h"

#include "Robots/RobotModel.h"

#include "Assets/MeshLoader.h"
#include "Scene/Object.h"

using namespace robots;

// Build a RobotRenderBinding from a RobotModel by loading the visual meshes for each link
RobotRenderBinding RobotPresentationBuilder::build(const robots::RobotModel& model) {
	RobotRenderBinding binding;
	assets::MeshLoader loader;
	for (const auto& link : model.links) {
		auto& visuals = binding.linkVisuals[link.name];

		for (const auto& mesh : link.visual.meshEntries) {
			auto meshes = loader.load(mesh.meshFile);

			for (auto& m : meshes) {
				auto obj = std::make_unique<scene::Object>(m);
				visuals.push_back(obj.get()); // Store raw pointer for rendering
				binding.ownedObjects.push_back(std::move(obj)); // Cache unique_ptr for memory management
			}
		}
	}
	return binding;
}