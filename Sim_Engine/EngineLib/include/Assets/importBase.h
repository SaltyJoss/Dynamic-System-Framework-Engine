#pragma once

// =============================================
// 				File: importBase.h	
// =============================================
// Interface for mesh importers.
//
// Summary:
// =============================================
//
// structures and enumerations:
// ---------------------------------------------
// struct IMeshImporter
//		-> Interface for mesh importers with a method to load meshes from files.
// ---------------------------------------------
//
// ============================================
// 
// IMeshImporter:
// --------------------------------------------
// virtual bool fromFile(const std::string& filepath, scene::Mesh* pMesh)
// 		-> Pure virtual function to load a mesh from a file into the provided Mesh object.
// -------------------------------------------- 
// 
// Built upon code from:
// ============================================
//	 GitHub: jayanam/jgl_demos/JGL_MeshLoader
// ============================================
//
// ============================================
//			  GitHub: saltyjoss
// ============================================

#include "EngineCore.h"

#include "Scene/Mesh.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace mesh_import {

	struct IMeshImporter { virtual bool fromFile(const std::string& filepath, scene::Mesh* pMesh) = 0; };
}