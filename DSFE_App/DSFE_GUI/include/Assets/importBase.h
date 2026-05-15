#pragma once
// File:    importBase.h
// GitHub:  SaltyJoss
// -----
// Initially templated off a tutorial:
// GitHub: jayanam/jgl_demos/JGL_MeshLoader
#include "EngineCore.h"

#include "Scene/Mesh.h"
#include "Platform/Logger.h"

// Base interface for mesh importers, to be implemented by specific format importers (e.g., OBJ, FBX, GLTF)
namespace mesh_import {
	struct IMeshImporter { virtual bool fromFile(const std::string& filepath, scene::Mesh* pMesh) = 0; };
}