#pragma once

//=============================================
//            File: MeshLoader.h
//=============================================
// Class responsible for loading 3D mesh files using the Assimp library.
// 
// Summary:
// ============================================
// 
// public:
// --------------------------------------------
// std::vector<std::shared_ptr<elements::Mesh>> load(const std::string& filepath)
//      -> Loads a mesh from the specified file path and returns a vector of shared pointers to Mesh objects.
// --------------------------------------------
// 
// private:
// --------------------------------------------
// std::vector<std::shared_ptr<elements::Mesh>> _imported
//      -> Vector of shared pointers to imported Mesh objects.
// std::shared_ptr<elements::Mesh> processMesh(aiMesh* mesh)
//      -> Processes an aiMesh and returns a shared pointer to a Mesh object.
// void processNode(aiNode* node, const aiScene* scene, const glm::mat4& parentTransform)
//      -> Processes an aiNode and its children, applying transformations and extracting meshes.
// --------------------------------------------
//
// ============================================

#include "EngineCore.h"

#include <assimp/scene.h>
#include <vector>
#include <memory>
#include <string>

namespace ai {
	class Node;
	class Scene;
	class Mesh;
}

namespace elements {
	class Mesh;
}

namespace gui {
	class ENGINE_API MeshLoader {
	public:
		std::vector<std::shared_ptr<elements::Mesh>> load(const std::string& filepath);

	private:
		std::vector<std::shared_ptr<elements::Mesh>> _imported;
		std::shared_ptr<elements::Mesh> processMesh(aiMesh* mesh);

		void processNode(aiNode* node, const aiScene* scene, const glm::mat4& parentTransform);
	};
}