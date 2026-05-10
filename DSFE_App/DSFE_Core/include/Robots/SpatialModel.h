// DSFE_Core SpatialModel.h
#pragma once

#include "EngineCore.h"

#include <unordered_map>
#include <core/SpatialMath.h>
#include "Robots/RobotModel.h"

namespace robots {
	// Spatial joint struct
	struct DSFE_API SpatialJoint {
		int parent = -1;

		eJointType type = eJointType::FIXED;
		
		mathlib::SpatialMat Xtree;
		mathlib::SpatialMat inertia;
		mathlib::SpatialVec S;

		std::string name;
	};

	// Spatial model struct
	struct DSFE_API SpatialModel {
		std::vector<SpatialJoint> joints;
		std::unordered_map<std::string, int> linkNameToIndex;
	};
}