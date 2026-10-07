// DSFE_GUI Mesh.cpp
#include "Assets/MeshLoader.h"
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include "Scene/Mesh.h"

#include "EngineLib/LogMacros.h"

namespace scene {
	// Mesh Initialisation
	void Mesh::init() {}

	// Load mesh data from file using MeshLoader
	bool Mesh::load(const std::string& path) {
		assets::MeshLoader loader;
		auto meshes = loader.load(path);
		if (meshes.empty()) {
			LOG_ERROR("Mesh load failed for file: %s", path.c_str());
			return false;
		}
		return true;
	}

	// Bind the mesh (binds the GPU buffers)
	void Mesh::bind() {}
	// Unbind the mesh (unbinds the GPU buffers)
	void Mesh::unbind() {}

	// Render the mesh using the current GPU buffers
	void Mesh::render() { 
	}	

	// Clean up CPU and GPU buffers
	void Mesh::clean() {
	}
}