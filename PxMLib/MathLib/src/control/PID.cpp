#include "pch.h"
#include "control/PID.h"

namespace control {
	/// <inheritdoc/>
	void PID(const PID_Gains& gains, PID_State& state, const VecX& error, double dt, VecX& out_u) {

		out_u.resize(error.size());	// Ensure the output vector is the correct size

		VecX P = gains.Kp.cwiseProduct(error);	// Proportional term
		state.integral += error * dt;	// Integral term
		VecX I = gains.Ki.cwiseProduct(state.integral);	// Compute integral term
		VecX D;	// Derivative term

		if (state.first_update) {
			D = VecX::Zero(error.size());
			state.first_update = false;
		}
		else {
			VecX derivative = (error - state.prev_error) / dt;
			D = gains.Kd.cwiseProduct(derivative);
		}

		state.prev_error = error;	// Update previous error
		out_u = P + I + D;	// Compute total control output
	}
}