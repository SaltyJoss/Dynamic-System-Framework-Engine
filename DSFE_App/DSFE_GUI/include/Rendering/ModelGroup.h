// DSFE_GUI ModelGroup.h
#pragma once

#include <vector>
#include <memory>
#include <string>
#include <glm/mat4x4.hpp>

// Represents a group of 3D model components with associated metadata.
struct ModelGroup {
	std::string name;
	std::vector<std::string> objectIDs;
	std::vector<std::string> parentIndex;
	std::vector<glm::mat4> localTransforms;
	std::vector<std::string> rawNodeNames;
	std::vector<std::string> semanticNames;
};