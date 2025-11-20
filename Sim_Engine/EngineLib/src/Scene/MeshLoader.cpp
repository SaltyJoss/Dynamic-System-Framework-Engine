
#include "pch.h"

#include "Scene/MeshLoader.h"
#include "Scene/VertexHolder.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include "Scene/Mesh.h"

#include "EngineLib/LogMacros.h"

namespace gui {
	std::vector<std::shared_ptr<elements::Mesh>> MeshLoader::load(const std::string& filepath) {
		std::vector<std::shared_ptr<elements::Mesh>> result;

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
		if (!scene || !scene->mRootNode) {
			LOG_ERROR("Failed to load mesh from %s: %s", filepath.c_str(), importer.GetErrorString());
			return result;
		}

		processNode(scene->mRootNode, scene, result);
		return result;
	}

	void MeshLoader::processNode(aiNode* node, const aiScene* scene, std::vector<std::shared_ptr<elements::Mesh>>& out) {
		for (unsigned int i = 0; i < node->mNumMeshes; i++) {
			aiMesh* aiMesh = scene->mMeshes[node->mMeshes[i]];
			auto mesh = processMesh(aiMesh);
			if (mesh) { out.push_back(mesh); }
				
		}
		for (unsigned int i = 0; i < node->mNumChildren; i++) {
			processNode(node->mChildren[i], scene, out);
		}
	}

	std::shared_ptr<elements::Mesh> MeshLoader::processMesh(aiMesh* mesh) {
		auto result = std::make_shared<elements::Mesh>();
		unsigned int indexOffset = 0;

		// vertices
		for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
			elements::VertexHolder vh;
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

		// indices
		for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
			const aiFace& face = mesh->mFaces[i];
			for (unsigned int j = 0; j < face.mNumIndices; ++j) {
				result->addVertexIndex(face.mIndices[j] + indexOffset);
			}
		}

		result->init();
		return result;
	}
}