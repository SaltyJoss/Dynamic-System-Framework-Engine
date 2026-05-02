#pragma once
// File:   MeshLoader.h
// GitHub: SaltyJoss
// -----
// Initially templated off a tutorial:
// GitHub: jayanam/jgl_demos/JGL_MeshLoader
#pragma warning(disable : 4251)

#include "EngineCore.h"
#include "Assets/VertexHolder.h"
#include <assimp/scene.h>
#include <vector>
#include <memory>
#include <string>

// Forward Declarations for Assimp types
namespace ai {
	class Node;
	class Scene;
	class Mesh;
}

// Forward Declarations for Mesh.h
namespace scene { class DSFE_API Mesh; }

namespace assets {
	class DSFE_API MeshLoader {
	public:
		// Loads a mesh from the specified file path and returns a vector of shared pointers to Mesh objects
		std::vector<std::shared_ptr<scene::Mesh>> load(const std::string& filepath);

	private:
		// Stores the meshes that have been imported during the loading process
		std::vector<std::shared_ptr<scene::Mesh>> _imported;
		// Helper method to process an Assimp mesh and convert it into a shared pointer to a scene::Mesh object
		std::shared_ptr<scene::Mesh> processMesh(aiMesh* mesh, const aiScene* scene);

		// Helper method to recursively process an Assimp node and its children, applying the parent transformation to each mesh
		void processNode(aiNode* node, const aiScene* scene, const glm::mat4& parentTransform);
	};
} // namespace gui