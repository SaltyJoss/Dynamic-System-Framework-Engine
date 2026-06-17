#pragma once

#include <core/MathLib.h>

using namespace mathlib;

namespace control {
	/// <summary>
	/// Structure to hold PID controller gains.
	/// </summary>
	template<typename Scalar>
	struct PID_Gains_T {
		VecX_T<Scalar> Kp; // Proportional gains
		VecX_T<Scalar> Ki; // Integral gains
		VecX_T<Scalar> Kd; // Derivative gains
	};

	/// <summary>
	/// Structure to hold the state of the PID controller.
	/// </summary>
	template<typename Scalar>
	struct PID_State_T {
		VecX_T<Scalar> integral;      // Integral of the error
		VecX_T<Scalar> prev_error;    // Previous error for derivative calculation
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
	template<typename Scalar>
	void PID(
		const PID_Gains_T<Scalar>& gains, PID_State_T<Scalar>& state,
		const VecX_T<Scalar>& error, Scalar dt, VecX_T<Scalar>& out_u
	) {
		out_u.resize(error.size());	// Ensure the output vector is the correct size

		VecX_T<Scalar> P = gains.Kp.cwiseProduct(error);	// Proportional term
		state.integral += error * dt;	// Integral term
		VecX_T<Scalar> I = gains.Ki.cwiseProduct(state.integral);	// Compute integral term
		VecX_T<Scalar> D; // Derivative term

		if (state.first_update) {
			D = VecX_T<Scalar>::Zero(error.size());
			state.first_update = false;
		}
		else {
			VecX_T<Scalar> deriv = (error - state.prev_error) / dt;
			D = gains.Kd.cwiseProduct(deriv);
		}

		state.prev_error = error;	// Update previous error
		out_u = P + I + D;	// Compute total control output
	}
} // namespace control