#define NOMINMAX

#include "pch.h"
#include "integrators/numerical_integrators.h"

// Numerical integration methods
namespace integration {
	// Ordinary Differential Equation (ODE) solvers
	// ----------------------------------------------------------------
	// Euler method
	VecX ODE::eulerStep(const VecX& x, double t, double dt, const std::function<VecX(double, const VecX&)>& f) {
		return x + dt * f(t, x);
	}

	// Second-order Runge-Kutta method (Midpoint method)
	VecX ODE::midpointStep(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f) {
		VecX k1 = dt * f(t, x);
		VecX k2 = dt * f(t + dt / 2.0, x + k1 / 2.0);
		return x + k2;
	}

	// Second-order Runge-Kutta method (Heun's method)
	VecX ODE::heunStep(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f) {
		VecX k1 = dt * f(t, x);
		VecX k2 = dt * f(t + dt, x + k1);
		return x + (k1 + k2) / 2.0;
	}

	// Second-order Runge-Kutta method (Ralston's method)
	VecX ODE::ralstonStep(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f) {
		VecX k1 = dt * f(t, x);
		VecX k2 = dt * f(t + (2.0 / 3.0) * dt, x + (2.0 / 3.0) * k1);
		return x + (k1 + 3.0 * k2) / 4.0;
	}

	// Fourth-order Runge-Kutta method
	VecX ODE::rk4Step(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f) {
		VecX k1 = dt * f(t, x);
		VecX k2 = dt * f(t + dt / 2.0, x + k1 / 2.0);
		VecX k3 = dt * f(t + dt / 2.0, x + k2 / 2.0);
		VecX k4 = dt * f(t + dt, x + k3);
		return x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
	}

	// RK45 method with adaptive step size (Dormand-Prince)
	VecX ODE::rk45Step(const VecX& x, double t, double& dt, std::function<VecX(double, const VecX&)> f, double rtol, double atol) {
		const double safety = 0.9;
		const double fac_min = 0.2;
		const double fac_max = 5.0;
		const double h_min = 1e-10;
		const double h_max = 1.0;

		dt = std::clamp(dt, h_min, h_max);

		// Try up to 25 attempts to find an acceptable step size
		for (int attempt = 0; attempt < 25; ++attempt) {
			// Butcher tableau for Dormand-Prince method

			// Coefficients for error estimation
			const double c2 = 1.0 / 5.0;
			const double c3 = 3.0 / 10.0;
			const double c4 = 4.0 / 5.0;
			const double c5 = 8.0 / 9.0;
			const double c6 = 1.0;
			const double c7 = 1.0;

			// Dormand-Prince coefficients
			const double a21 =  1.0 / 5.0;
			const double a31 =  3.0 / 40.0;
			const double a32 =  9.0 / 40.0;
			const double a41 =  44.0 / 45.0;
			const double a42 = -56.0 / 15.0;
			const double a43 =  32.0 / 9.0;
			const double a51 =  19372.0 / 6561.0;
			const double a52 = -25360.0 / 2187.0;
			const double a53 =  64448.0 / 6561.0;
			const double a54 = -212.0 / 729.0;
			const double a61 =  9017.0 / 3168.0;
			const double a62 = -355.0 / 33.0;
			const double a63 =  46732.0 / 5247.0;
			const double a64 =  49.0 / 176.0;
			const double a65 = -5103.0 / 18656.0;
			const double a71 =  35.0 / 384.0;
			const double a72 =  0.0;
			const double a73 =  500.0 / 1113.0;
			const double a74 =  125.0 / 192.0;
			const double a75 = -2187.0 / 6784.0;
			const double a76 =  11.0 / 84.0;

			// Weights for 4th and 5th order estimates
			const double b1  =  35.0 / 384.0;
			const double b2  =  0.0;
			const double b3  =  500.0 / 1113.0;
			const double b4  =  125.0 / 192.0;
			const double b5  = -2187.0 / 6784.0;
			const double b6  =  11.0 / 84.0;
			const double b1s =  5179.0 / 57600.0;
			const double b2s =  0.0;
			const double b3s =  7571.0 / 16695.0;
			const double b4s =  393.0 / 640.0;
			const double b5s = -92097.0 / 339200.0;
			const double b6s =  187.0 / 2100.0;
			const double b7s =  1.0 / 40.0;

			// Compute the Runge-Kutta stages (DP -> 7 stages)
			const VecX k1 = f(t, x);
			const VecX k2 = f(t + c2 * dt, x + dt * (a21 * k1));
			const VecX k3 = f(t + c3 * dt, x + dt * (a31 * k1 + a32 * k2));
			const VecX k4 = f(t + c4 * dt, x + dt * (a41 * k1 + a42 * k2 + a43 * k3));
			const VecX k5 = f(t + c5 * dt, x + dt * (a51 * k1 + a52 * k2 + a53 * k3 + a54 * k4));
			const VecX k6 = f(t + c6 * dt, x + dt * (a61 * k1 + a62 * k2 + a63 * k3 + a64 * k4 + a65 * k5));
			const VecX k7 = f(t + c7 * dt, x + dt * (a71 * k1 + a72 * k2 + a73 * k3 + a74 * k4 + a75 * k5 + a76 * k6));

			// Compute 4th and 5th order estimates
			const VecX y5 = x + dt * ((b1 * k1) + (b3 * k3) + (b4 * k4) + (b5 * k5) + (b6 * k6));   // 5th order estimate
			const VecX y4 = x + dt * ((b1s * k1) + (b3s * k3) + (b4s * k4) + (b5s * k5) + (b6s * k6) + (b7s * k7)); // 4th order estimate

			const VecX e = y5 - y4; // Error estimate

			// Compute the error norm
			double errNorm = 0.0;
			for (int i = 0; i < e.size(); ++i) {
				double sc = atol + rtol * std::max<double>(std::abs(x(i)), std::abs(y5(i)));
				const double r = e(i) / sc;
				errNorm += r * r;
			}
			double err = std::sqrt(errNorm / e.size());

			// Adaptive step size control
			if (err <= 1.0 && std::isfinite(err)) {
				// accept
				const double denom = std::max<double>(err, 1e-10);		// prevent division by zero
				double fac = safety * std::pow(denom, -0.2);	// exponent for 5th order method
				fac = std::clamp(fac, fac_min, fac_max);		// limit step size change
				dt = std::clamp(dt * fac, h_min, h_max);		// update step size
				return y5;
			}
			else {
				// reject
				double denom = (std::isfinite(err) ? std::max<double>(err, 1e-16) : 1e16);	// prevent division by zero & NaN
				double fac = safety * std::pow(denom, -0.2);							// exponent for 4th order method
				fac = std::clamp(fac, fac_min, fac_max);
				dt = std::clamp(dt * fac, h_min, h_max);
			}
		}

		// If it reaches here, it failed to converge after many attempts
		// Just putting this here as a policy choice honestly, return the best effort or throw
		throw std::runtime_error("RK45 failed to converge after maximum attempts");
		//return x;
	}

	// Partial Differential Equation (PDE) solvers
	// ----------------------------------------------------------------
	// Finite Difference Method
	VecX PDE::fdmStep(const VecX& u, double dx, double dt, double alpha) {
		int n = (int)u.size();
		VecX u_new = u;
		double r = alpha * dt / (dx * dx);
		for (int i = 1; i < n - 1; ++i) {
			u_new(i) = u(i) + r * (u(i + 1) - 2 * u(i) + u(i - 1));
		}
		return u_new;
	}
}