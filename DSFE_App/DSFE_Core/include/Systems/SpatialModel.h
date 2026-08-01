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
		mathlib::Quat_T<Scalar> free_qref{Scalar(1),Scalar(0),Scalar(0),Scalar(0)}; // free joint reference orientation
		int nfDOF = 1; // number of degrees of freedom for this joint (1 for revolute/prismatic, 0 for fixed, 6 for free)

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
#include "Systems/SpatialModelCast.inl"