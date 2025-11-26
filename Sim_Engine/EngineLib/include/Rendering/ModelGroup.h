#pragma once

// ============================================
//            File: ModelGroup.h
// ============================================
// Structs representing a group of 3D model components with associated metadata.
//
// Summary:
// ============================================
//
// structs:
// --------------------------------------------
// ModelGroup
//      -> Represents a group of 3D model components with associated metadata.
// --------------------------------------------
//
// Internal State Variables:
// --------------------------------------------
// std::string name
//      -> Name of the model group.
// std::vector<std::string> objectIDs
//      -> Vector of object IDs associated with the model group.
// std::vector<std::string> parentIndex
//      -> Vector of parent indices for the model group components.
// std::vector<glm::mat4> localTransforms
//      -> Vector of local transformation matrices for the model group components.
// std::vector<std::string> rawNodeNames
//      -> Vector of raw node names for the model group components.
// std::vector<std::string> semanticNames
//      -> Vector of semantic names for the model group components.
// --------------------------------------------
//
// ============================================

#include "EngineCore.h"
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