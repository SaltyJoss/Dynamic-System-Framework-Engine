// DSFE_Core SpatialModel.h
#pragma once

#include "EngineCore.h"

#include <unordered_map>
#include <core/SpatialMath.h>
#include "Robots/RobotModel.h"

namespace robots {
	// Spatial joint struct
	template<typename Scalar>
	struct SpatialJoint {
		int parent = -1;

		eJointType type = eJointType::FIXED;
		
		mathlib::SpatialMat_T<Scalar> Xtree;
		mathlib::SpatialMat_T<Scalar> inertia;
		mathlib::SpatialVec_T<Scalar> S;

		std::string name;
	};

	// Spatial model struct
	template<typename Scalar>
	struct SpatialModel {
		std::vector<SpatialJoint<Scalar>> joints;
		std::unordered_map<std::string, int> linkNameToIndex;
	};
}