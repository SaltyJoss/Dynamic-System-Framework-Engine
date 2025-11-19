
#include "pch.h"

#include "Scene/Mesh.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include "Rendering/OpenGLBufferManager.h"

#include "EngineLib/LogMacros.h"

namespace elements {
	void Mesh::init() {
		_rndrBffrMngr = std::make_unique<render::OpenGLVertexIndexBuffer>();

		createBuffers();
	}

	elements::Mesh::~Mesh() { deleteBuffers(); LOG_INFO("Buffers deleted in destructor");}

	bool Mesh::load(const std::string& path) {
		LOG_INFO("Loading mesh from %s", path.c_str());

		const uint32_t _meshImportFlags =
			aiProcess_Triangulate |
			aiProcess_JoinIdenticalVertices |
			aiProcess_RemoveRedundantMaterials |
			aiProcess_FindInvalidData |
			aiProcess_GenNormals |
			aiProcess_ImproveCacheLocality |
			aiProcess_OptimizeMeshes |
			aiProcess_ValidateDataStructure;

		Assimp::Importer Importer;

		const aiScene* scene = Importer.ReadFile(path.c_str(), _meshImportFlags);

		if (!scene) { LOG_ERROR("Failed to load mesh: %s", Importer.GetErrorString()); return false; }
		if (!scene->HasMeshes()) { LOG_WARN("Scene has no meshes"); return false; }

		if (scene && scene->HasMeshes()) {
			_vertexIndices.clear();
			_vertices.clear();

			auto* mesh = scene->mMeshes[0];
			LOG_INFO("Mesh has %d vertices and %d faces", mesh->mNumVertices, mesh->mNumFaces);

			for (uint32_t i = 0; i < mesh->mNumVertices; i++) {
				VertexHolder vh;
				vh._pos = { mesh->mVertices[i].x, mesh->mVertices[i].y ,mesh->mVertices[i].z };
				vh._normal = { mesh->mNormals[i].x, mesh->mNormals[i].y ,mesh->mNormals[i].z };

				if (mesh->mTextureCoords[0]) {
					vh._texCoord = {
						mesh->mTextureCoords[0][i].x,
						mesh->mTextureCoords[0][i].y
					};
					if (i < 5) {     // only print first 5 so you don't spam 100k logs
						LOG_INFO("UV[%d] = %f %f",
							i,
							mesh->mTextureCoords[0][i].x,
							mesh->mTextureCoords[0][i].y);
					}
				}
				else {
					LOG_WARN("Mesh has NO UVs at all (mesh->mTextureCoords[0] == NULL)");
					vh._texCoord = glm::vec2(0.0f);
				}

				addVertex(vh);
			}
			LOG_INFO("Vertices added: %zu", _vertices.size());
			for (size_t i = 0; i < mesh->mNumFaces; i++) {
				aiFace face = mesh->mFaces[i];
				for (size_t j = 0; j < face.mNumIndices; j++) { addVertexIndex(face.mIndices[j]); }		
			}
			LOG_INFO("Vertex indices added: %zu", _vertexIndices.size());

			for (auto& v : _vertices) {
				v._normal = -v._normal;
			}
			LOG_INFO("Mesh normals flipped");

			init();
			LOG_INFO("Mesh initialized successfully");

			return true;
		}

		return false;
	}

	void elements::Mesh::integrate(float dt, float floorY) {
		if (_isStatic) return;

		_velocity += _acceleration * dt;
		_position += _velocity * dt;

		// Floor collision
		if (_position.y < floorY) {
			_position.y = floorY;
			_velocity.y = 0.0f;
		}
	}

	void Mesh::createBuffers() { _rndrBffrMngr->createBuffers(_vertices, _vertexIndices); }
	void Mesh::deleteBuffers() { _rndrBffrMngr->deleteBuffers(); }
	void Mesh::bind() { _rndrBffrMngr->bind(); }
	void Mesh::unbind() { _rndrBffrMngr->unbind(); }
	void Mesh::render() { _rndrBffrMngr->draw((int) _vertexIndices.size()); }
	void Mesh::clear() {
		if (_rndrBffrMngr) _rndrBffrMngr->deleteBuffers();
		_vertices.clear();
		_vertexIndices.clear();
	}
}