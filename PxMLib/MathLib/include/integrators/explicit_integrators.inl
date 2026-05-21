// PxM/MathLib explicit_integrators.inl
#pragma once

namespace integration {
	// Euler method
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::eulerStep(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f
	) {
		return x + dt * f(t, x);
	}

	// Second-order Runge-Kutta method (Midpoint)
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::midpointStep(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f
	) {
		mathlib::VecX_T<Scalar> k1 = dt * f(t, x);
		mathlib::VecX_T<Scalar> k2 = dt * f(t + dt / Scalar(2), x + k1 / Scalar(2));
		return x + k2;
	}

	// Second-order Runge-Kutta method (Heun)
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::heunStep(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f
	) {
		mathlib::VecX_T<Scalar> k1 = dt * f(t, x);
		mathlib::VecX_T<Scalar> k2 = dt * f(t + dt, x + k1);
		return x + (k1 + k2) / Scalar(2);
	}

	// Second-order Runge-Kutta method (Ralston)
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::ralstonStep(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f
	) {
		mathlib::VecX_T<Scalar> k1 = dt * f(t, x);
		mathlib::VecX_T<Scalar> k2 = dt * f(t + (Scalar(2) / Scalar(3)) * dt, x + (Scalar(2) / Scalar(3)) * k1);
		return x + (k1 + Scalar(3) * k2) / Scalar(4);
	}

	// Fourth-order Runge-Kutta method
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::rk4Step(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar dt,
		Func&& f
	) {
		mathlib::VecX_T<Scalar> k1 = dt * f(t, x);
		mathlib::VecX_T<Scalar> k2 = dt * f(t + dt / Scalar(2), x + k1 / Scalar(2));
		mathlib::VecX_T<Scalar> k3 = dt * f(t + dt / Scalar(2), x + k2 / Scalar(2));
		mathlib::VecX_T<Scalar> k4 = dt * f(t + dt, x + k3);
		return x + (k1 + Scalar(2) * k2 + Scalar(2) * k3 + k4) / Scalar(6);
	}

	// RK45 method with adaptive step size (Dormand-Prince)
	template<typename Scalar, typename Func>
	mathlib::VecX_T<Scalar> NumericalIntegrator::rk45Step(
		const mathlib::VecX_T<Scalar>& x,
		Scalar t,
		Scalar& dt,
		Scalar& dt_used,
		Func&& f,
		Scalar rtol,
		Scalar atol
	) {
		const Scalar safety = 0.9;	// safety factor to prevent aggressive step size changes
		const Scalar fac_min = 0.2;	// minimum factor for reducing step size
		const Scalar fac_max = 5.0;	// maximum factor for increasing step size
		const Scalar h_min = 1e-10;	// minimum allowed step size
		const Scalar h_max = 1.0;	// maximum allowed step size

		dt = std::clamp(dt, h_min, h_max);

		// Try up to 25 attempts to find an acceptable step size
		for (int attempt = 0; attempt < 25; ++attempt) {
			// Butcher tableau for Dormand-Prince method (7 stages, 5th order, SHOULD probably be precomputed as static constants, in a matrix or equiv)

			// Coefficients for error estimation
			const Scalar c2 = 1.0 / 5.0;
			const Scalar c3 = 3.0 / 10.0;
			const Scalar c4 = 4.0 / 5.0;
			const Scalar c5 = 8.0 / 9.0;
			const Scalar c6 = 1.0;
			const Scalar c7 = 1.0;

			// Dormand-Prince coefficients
			const Scalar a21 = 1.0 / 5.0;
			const Scalar a31 = 3.0 / 40.0;
			const Scalar a32 = 9.0 / 40.0;
			const Scalar a41 = 44.0 / 45.0;
			const Scalar a42 = -56.0 / 15.0;
			const Scalar a43 = 32.0 / 9.0;
			const Scalar a51 = 19372.0 / 6561.0;
			const Scalar a52 = -25360.0 / 2187.0;
			const Scalar a53 = 64448.0 / 6561.0;
			const Scalar a54 = -212.0 / 729.0;
			const Scalar a61 = 9017.0 / 3168.0;
			const Scalar a62 = -355.0 / 33.0;
			const Scalar a63 = 46732.0 / 5247.0;
			const Scalar a64 = 49.0 / 176.0;
			const Scalar a65 = -5103.0 / 18656.0;
			const Scalar a71 = 35.0 / 384.0;
			const Scalar a72 = 0.0;
			const Scalar a73 = 500.0 / 1113.0;
			const Scalar a74 = 125.0 / 192.0;
			const Scalar a75 = -2187.0 / 6784.0;
			const Scalar a76 = 11.0 / 84.0;

			// Weights for 4th and 5th order estimates
			const Scalar b1 = 35.0 / 384.0;
			const Scalar b2 = 0.0;
			const Scalar b3 = 500.0 / 1113.0;
			const Scalar b4 = 125.0 / 192.0;
			const Scalar b5 = -2187.0 / 6784.0;
			const Scalar b6 = 11.0 / 84.0;
			const Scalar b1s = 5179.0 / 57600.0;
			const Scalar b2s = 0.0;
			const Scalar b3s = 7571.0 / 16695.0;
			const Scalar b4s = 393.0 / 640.0;
			const Scalar b5s = -92097.0 / 339200.0;
			const Scalar b6s = 187.0 / 2100.0;
			const Scalar b7s = 1.0 / 40.0;

			// Compute the Runge-Kutta stages (DP -> 7 stages)
			const mathlib::VecX_T<Scalar> k1 = f(t, x);
			const mathlib::VecX_T<Scalar> k2 = f(t + c2 * dt, x + dt * (a21 * k1));
			const mathlib::VecX_T<Scalar> k3 = f(t + c3 * dt, x + dt * (a31 * k1 + a32 * k2));
			const mathlib::VecX_T<Scalar> k4 = f(t + c4 * dt, x + dt * (a41 * k1 + a42 * k2 + a43 * k3));
			const mathlib::VecX_T<Scalar> k5 = f(t + c5 * dt, x + dt * (a51 * k1 + a52 * k2 + a53 * k3 + a54 * k4));
			const mathlib::VecX_T<Scalar> k6 = f(t + c6 * dt, x + dt * (a61 * k1 + a62 * k2 + a63 * k3 + a64 * k4 + a65 * k5));
			const mathlib::VecX_T<Scalar> k7 = f(t + c7 * dt, x + dt * (a71 * k1 + a72 * k2 + a73 * k3 + a74 * k4 + a75 * k5 + a76 * k6));

			// Compute 4th and 5th order estimates
			const mathlib::VecX_T<Scalar> y5 = x + dt * ((b1 * k1) + (b3 * k3) + (b4 * k4) + (b5 * k5) + (b6 * k6));   // 5th order estimate
			const mathlib::VecX_T<Scalar> y4 = x + dt * ((b1s * k1) + (b3s * k3) + (b4s * k4) + (b5s * k5) + (b6s * k6) + (b7s * k7)); // 4th order estimate

			const mathlib::VecX_T<Scalar> e = y5 - y4; // Error estimate

			// Compute the error norm
			Scalar errNorm = 0.0;
			for (int i = 0; i < e.size(); ++i) {
				Scalar sc = atol + rtol * std::max<Scalar>(std::abs(x(i)), std::abs(y5(i)));
				const Scalar r = e(i) / sc;
				errNorm += r * r;
			}
			Scalar err = std::sqrt(errNorm / e.size());

			// Adaptive step size control

			// Accept
			if (err <= 1.0 && std::isfinite(err)) {
				dt_used = dt; // Store the actual step size used for this step

				// Update step size for next iteration
				const Scalar denom = std::max<Scalar>(err, 1e-10); // prevent division by zero
				Scalar fac = safety * std::pow(denom, -0.2);	   // exponent for 5th order method
				fac = std::clamp(fac, fac_min, fac_max);		   // limit step size change
				dt = std::clamp(dt * fac, h_min, h_max);		   // update step size
				return y5;
			}
			// Reject
			else {
				Scalar denom = (std::isfinite(err)
					? std::max<Scalar>(err, 1e-16) : 1e16);	 // prevent division by zero & NaN
				Scalar fac = safety * std::pow(denom, -0.2); // exponent for 4th order method
				fac = std::clamp(fac, fac_min, fac_max);
				dt = std::clamp(dt * fac, h_min, h_max);
			}
		}
		throw std::runtime_error("RK45 failed to converge after maximum attempts");
	}
}