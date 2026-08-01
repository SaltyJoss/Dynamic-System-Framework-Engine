/*
 * File: Systems/RigidBodyLoader.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include "Systems/RigidBodyModel.h"

namespace systems {
	class DSFE_API RigidBodyLoader {
	public:
		static RigidBodyModel load(const std::string& fp);
		static RigidBodyModel loadFromJSON(const std::string& fp);
		static RigidBodyModel loadFromURDF(const std::string& fp);
	};
} // namespace systems