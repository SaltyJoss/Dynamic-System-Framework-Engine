#include "pch.h"
// File:   Mesh.cpp
// GitHub: SaltyJoss
#include "Assets/MeshLoader.h"
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include "Scene/Mesh.h"
#include "Rendering/OpenGLBufferManager.h"

#include "EngineLib/LogMacros.h"

namespace scene {
	// Mesh Initialisation
	void Mesh::init() {
		_rndrBffrMngr = std::make_unique<render::OpenGLVertexIndexBuffer>();
		createBuffers();
	}

	// Load mesh data from file using MeshLoader
	bool Mesh::load(const std::string& path) {
		assets::MeshLoader loader;
		loader.load(path);
		if (loader.load(path).empty()) {
			LOG_ERROR("Mesh load failed for file: %s", path.c_str());
			return false;
		}
		return true;
	}

	// Create GPU buffers from CPU vertex and index data
	void Mesh::createBuffers() { _rndrBffrMngr->createBuffers(_vertices, _indices); }
	// Delete GPU buffers
	void Mesh::deleteBuffers() { _rndrBffrMngr->deleteBuffers(); }

	// Bind the mesh (binds the GPU buffers)
	void Mesh::bind() { _rndrBffrMngr->bind(); }
	// Unbind the mesh (unbinds the GPU buffers)
	void Mesh::unbind() { _rndrBffrMngr->unbind(); }

	// Render the mesh using the current GPU buffers
	void Mesh::render() { 
		if (!_rndrBffrMngr) {
			LOG_ERROR("VertexIndexBuffer is nullptr");
			return;
		}
		_rndrBffrMngr->draw((int)_indices.size());
	}	

	// Clean up CPU and GPU buffers
	void Mesh::clean() {
		if (_rndrBffrMngr) _rndrBffrMngr->deleteBuffers();
		_vertices.clear();
		_indices.clear();
	}
}