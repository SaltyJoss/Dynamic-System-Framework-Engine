
#include "pch.h"
#include "Scene/MeshLoader.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include "Scene/Mesh.h"

#include "Rendering/OpenGLBufferManager.h"

#include "EngineLib/LogMacros.h"

namespace elements {
	void Mesh::init() {
		_rndrBffrMngr = std::make_unique<render::OpenGLVertexIndexBuffer>();

		createBuffers();
	}

	bool Mesh::load(const std::string& path) {
		gui::MeshLoader loader;
		loader.load(path);
		
		if (loader.load(path).empty()) {
			LOG_ERROR("Mesh load failed for file: %s", path.c_str());
			return false;
		}

		return true;
	}

	void Mesh::createBuffers() { _rndrBffrMngr->createBuffers(_vertices, _indices); }
	void Mesh::deleteBuffers() { _rndrBffrMngr->deleteBuffers(); }
	void Mesh::bind() { _rndrBffrMngr->bind(); }
	void Mesh::unbind() { _rndrBffrMngr->unbind(); }
	void Mesh::render() { _rndrBffrMngr->draw((int) _indices.size()); }	
	void Mesh::clean() {
		if (_rndrBffrMngr) _rndrBffrMngr->deleteBuffers();
		_vertices.clear();
		_indices.clear();
	}
}