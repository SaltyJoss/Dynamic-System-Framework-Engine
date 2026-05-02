#pragma once
// File:    importObj.h
// GitHub:  SaltyJoss
#include "EngineCore.h"

#include "importBase.h"
#include "Platform/Logger.h"

extern DSFE_API Debug gLog;

namespace mesh_import {

	class DSFE_API ObjMeshImporter : public IMeshImporter {
	public:
		virtual bool fromFile(const std::string& filepath, scene::Mesh* pMesh) override;
	};
}
