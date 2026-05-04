// DSFE_GUI RobotPresentationBuilder.cpp
#include "RobotPresentationBuilder.h"

#include "Assets/MeshLoader.h"
#include "Scene/Object.h"

RobotRenderBinding RobotPresentationBuilder::build(const RobotModel& model) {
	RobotRenderBinding binding;
	assets::MeshLoader loader;
	for (const auto& [linkName, link] : model.links) {
		for (const auto& mesh : link.visual.meshEnteries) {
			auto* obj = loader.load(mesh.meshFile);
			binding.linkObjects[linkName].push_back(obj);
		}
	}
	return binding;
}