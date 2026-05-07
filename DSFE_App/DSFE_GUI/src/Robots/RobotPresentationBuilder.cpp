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
		for (const auto& mesh : link.visual.meshEntries) {
			auto obj = loader.load(mesh.meshFile);
			auto& visuals = binding.linkVisuals[link.name];
			visuals.insert(visuals.end(), obj.begin(), obj.end());
		}
	}
	return binding;
}