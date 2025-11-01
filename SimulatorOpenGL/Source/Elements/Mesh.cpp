#include "ch.h"
#include "Mesh.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include "Render/OpenGLBufferManager.h"

namespace elements {
	void Mesh::init() {
		_rndrBffrMngr = std::make_unique<render::OpenGLVertexIndexBuffer>();
		createBuffers();
	}

	elements::Mesh::~Mesh() { deleteBuffers(); }

	bool Mesh::load(const std::string& path)
	{
		const uint32_t _meshImportFlags =
			aiProcess_CalcTangentSpace |
			aiProcess_Triangulate |
			aiProcess_SortByPType |
			aiProcess_GenNormals |
			aiProcess_GenUVCoords |
			aiProcess_OptimizeMeshes |
			aiProcess_ValidateDataStructure;

		Assimp::Importer Importer;

		const aiScene* scene = Importer.ReadFile(path.c_str(), _meshImportFlags);

		if (scene && scene->HasMeshes()) {
			_vertexIndices.clear();
			_vertices.clear();

			auto* mesh = scene->mMeshes[0];

			for (uint32_t i = 0; i < mesh->mNumVertices; i++) {
				VertexHolder vh;
				vh._pos = { mesh->mVertices[i].x, mesh->mVertices[i].y ,mesh->mVertices[i].z };
				vh._normal = { mesh->mNormals[i].x, mesh->mNormals[i].y ,mesh->mNormals[i].z };

				addVertex(vh);
			}
			for (size_t i = 0; i < mesh->mNumFaces; i++) {
				aiFace face = mesh->mFaces[i];
				for (size_t j = 0; j < face.mNumIndices; j++) { addVertexIndex(face.mIndices[j]); }		
			}

			init();
			return true;
		}

		return false;
	}

	void Mesh::createBuffers() { _rndrBffrMngr->createBuffers(_vertices, _vertexIndices); }
	void Mesh::deleteBuffers() { _rndrBffrMngr->deleteBuffers(); }
	void Mesh::bind() { _rndrBffrMngr->bind(); }
	void Mesh::unbind() { _rndrBffrMngr->unbind(); }
	void Mesh::render() { _rndrBffrMngr->draw((int) _vertexIndices.size()); }

}