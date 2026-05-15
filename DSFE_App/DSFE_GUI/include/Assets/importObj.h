// DSFE_GUI ImportObj.h
#pragma once

#include "importBase.h"
#include "Platform/Logger.h"

namespace mesh_import {

	class ObjMeshImporter : public IMeshImporter {
	public:
		virtual bool fromFile(const std::string& filepath, scene::Mesh* pMesh) override;
	};
}
