#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"

using namespace mathlib;

namespace control {
	/// <summary>
	/// Structure to hold PID controller gains.
	/// </summary>
	struct MATHLIB_API PID_Gains {
		VecX Kp; // Proportional gains
		VecX Ki; // Integral gains
		VecX Kd; // Derivative gains
	};

	/// <summary>
	/// Structure to hold the state of the PID controller.
	/// </summary>
	struct MATHLIB_API PID_State {
		VecX integral;      // Integral of the error
		VecX prev_error;    // Previous error for derivative calculation
		bool first_update = true; // Flag to check if it's the first update
	};

	/// <summary>
	/// Updates the PID controller state and computes the control output.
	/// </summary>
	/// <param name="gains">The PID gains.</param>
	/// <param name="state">The current state of the PID controller.</param>
	/// <param name="error">The current error signal.</param>
	/// <param name="dt">The time step since the last update.</param>
	/// <param name="out_u">The computed control output.</param>
	void MATHLIB_API PID(const PID_Gains& gains, PID_State& state, const VecX& error, double dt, VecX& out_u);
} // namespace control