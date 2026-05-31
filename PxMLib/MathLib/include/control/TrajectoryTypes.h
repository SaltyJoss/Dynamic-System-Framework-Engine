// PxM/
#pragma once

#include <core/MathLib.h>

using namespace mathlib;

namespace control {
	template<typename Scalar>
	struct TrajState {
		Scalar q = Scalar(0);   // Desired position at this trajectory point
		Scalar qd = Scalar(0);  // Desired velocity at this trajectory point
		Scalar qdd = Scalar(0); // Desired acceleration at this trajectory point
	};

	template<typename Scalar>
	struct TrajTimeSpan {
		Scalar t0 = Scalar(0); // Start time of the trajectory segment
		Scalar tf = Scalar(0); // End time of the trajectory segment
	};
} // namespace control