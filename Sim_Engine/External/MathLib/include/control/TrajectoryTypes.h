#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"

using namespace mathlib;

namespace control {
	struct TrajState {
		double q = 0.0;   // Desired position at this trajectory point
		double qd = 0.0;  // Desired velocity at this trajectory point
		double qdd = 0.0; // Desired acceleration at this trajectory point
	};

	struct TrajTimeSpan {
		double t0 = 0.0; // Start time of the trajectory segment
		double tf = 0.0; // End time of the trajectory segment
	};
} // namespace control