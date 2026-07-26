/*
 * File: Systems/SpatialModel.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include <unordered_map>
#include <core/SpatialMath>
#include "Systems/RigidBodyModel.h"

namespace systems {
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

		template<typename ScalarT>
		SpatialModel<ScalarT> cast() const;
	};
} // namespace systems
#include "SpatialModelCast.inl"