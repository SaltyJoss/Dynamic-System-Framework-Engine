// DSFE_Core SpatialDynamics.h
#pragma once

#include "EngineCore.h"
#include "Robots/SpatialModel.h"
#include <core/SpatialMath.h>

namespace robots {
	class DSFE_API SpatialDynamics {
	public:
		static void computeSpatialVelocities(
			const SpatialModel& model,
			const mathlib::VecX& q,
			const mathlib::VecX& qd,
			std::vector<mathlib::SpatialVec>& v_out,
			std::vector<mathlib::SpatialMat>& Xup_out
		);
	};
}