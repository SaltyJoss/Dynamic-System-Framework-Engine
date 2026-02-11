#include "pch.h"
// File:   MeshLoader.cpp
// GitHub: SaltyJoss
#include "Scene/MeshLoader.h"
#include "Scene/VertexHolder.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include "Scene/Mesh.h"

#include "EngineLib/LogMacros.h"

namespace gui {
	// Load a mesh from the specified file path and return a vector of shared pointers to Mesh objects
	std::vector<std::shared_ptr<scene::Mesh>> MeshLoader::load(const std::string& filepath) {
		_imported.clear();

		const uint32_t importFlags =
			aiProcess_Triangulate |
			aiProcess_JoinIdenticalVertices |
			aiProcess_FindInvalidData |
			aiProcess_GenNormals |
			aiProcess_ImproveCacheLocality |
			aiProcess_OptimizeMeshes |
			aiProcess_ValidateDataStructure;
		Assimp::Importer importer;
		const aiScene* scene = importer.ReadFile(filepath.c_str(), importFlags);

		// Check if the import was successful and if the scene contains a root node
		if (!scene || !scene->mRootNode) {
			LOG_ERROR("Failed to load mesh from %s: %s", filepath.c_str(), importer.GetErrorString());
			return {};
		}

		glm::mat4 rootTransform(1.0f);
		processNode(scene->mRootNode, scene, rootTransform);

		return _imported;
	}

	// Helper method to recursively process an Assimp node and its children, applying the parent transformation to each mesh
	void MeshLoader::processNode(aiNode* node, const aiScene* scene, const glm::mat4& parentTransform) {
		// convert aiMatrix4x4s to glm::mat4
		aiMatrix4x4 a = node->mTransformation;
		// Assimp uses row-major order, while GLM uses column-major order, so we need to transpose the matrix
		glm::mat4 nodeTransform = glm::mat4(
			a.a1, a.b1, a.c1, a.d1,
			a.a2, a.b2, a.c2, a.d2,
			a.a3, a.b3, a.c3, a.d3,
			a.a4, a.b4, a.c4, a.d4
		);

		glm::mat4 globalTransform = parentTransform * nodeTransform;

		// Process meshes in this node
		for (unsigned int i = 0; i < node->mNumMeshes; i++)
		{
			aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
			auto m = processMesh(mesh);
			m->localTransform = globalTransform;
			_imported.push_back(std::move(m));
		}

		// Recursively process child nodes
		for (unsigned int i = 0; i < node->mNumChildren; i++) {
			processNode(node->mChildren[i], scene, globalTransform);
		}
	}

	// Helper method to process an Assimp mesh and convert it into a shared pointer to a scene::Mesh object
	std::shared_ptr<scene::Mesh> MeshLoader::processMesh(aiMesh* mesh) {
		auto result = std::make_shared<scene::Mesh>();
		unsigned int indexOffset = 0;

		// Get the vertices of the mesh and add them to the result mesh, applying the local transform to the vertex positions and normals
		for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
			scene::VertexHolder vh;
			vh._pos = glm::vec3(mesh->mVertices[i].x, mesh->mVertices[i].y, mesh->mVertices[i].z);
			vh._normal = mesh->mNormals
				? glm::vec3(mesh->mNormals[i].x, mesh->mNormals[i].y, mesh->mNormals[i].z)
				: glm::vec3(0.0f, 1.0f, 0.0f);

			if (mesh->mTextureCoords[0]) {
				vh._texCoord = glm::vec2(mesh->mTextureCoords[0][i].x,
					mesh->mTextureCoords[0][i].y);
			}
			else { vh._texCoord = glm::vec2(0.0f); }

			result->addVertex(vh);
		}

		// Get the indices for the faces of the mesh and add them to the result mesh, applying the index offset to account for previously added vertices
		for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
			const aiFace& face = mesh->mFaces[i];
			for (unsigned int j = 0; j < face.mNumIndices; ++j) {
				result->addVertexIndex(face.mIndices[j] + indexOffset);
			}
		}

		// Update the index offset for the next mesh
		result->init();
		return result;
	}
}