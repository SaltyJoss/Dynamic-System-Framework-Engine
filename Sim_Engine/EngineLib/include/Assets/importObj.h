#pragma once
// File:    importObj.h
// GitHub:  SaltyJoss
#include "EngineCore.h"

#include "importBase.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;

namespace mesh_import {

	class ENGINE_API ObjMeshImporter : public IMeshImporter {
	public:
		virtual bool fromFile(const std::string& filepath, scene::Mesh* pMesh) override;
	};
}
