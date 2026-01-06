#define NOMINMAX

#include "pch.h"
#include "integrators/numerical_integrators.h"

// Numerical integration methods
namespace integration {
	// Ordinary Differential Equation (ODE) solvers
	// ----------------------------------------------------------------
	// Euler method
	VecX ODE::eulerStep(const VecX& x, const VecX& dxdt, double dt) {
		return x + dxdt * dt;
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
		// Safe Guards
		if (dt < 1e-10) {
			dt = 1e-10; // Minimum step size
		}
		if (dt > 1.0) {
			dt = 1.0; // Maximum step size
		}

		// Dormand-Prince coefficients
		double a11 =  1 / 5;
		double a21 =  3 / 40;
		double a22 =  9 / 40;
		double a31 =  44 / 45;
		double a32 = -56 / 15;
		double a33 =  32 / 9;
		double a41 =  19372 / 6561;
		double a42 = -25360 / 2187;
		double a43 =  64448 / 6561;
		double a44 = -212 / 729;
		double a51 =  9017 / 3168;
		double a52 = -355 / 33;
		double a53 =  46732 / 5247;
		double a54 =  49 / 176;
		double a55 = -5103 / 18656;
		double a61 =  35 / 384;
		double a62 =  0;
		double a63 =  500 / 1113;
		double a64 =  125 / 192;
		double a65 = -2187 / 6784;
		double a66 =  11 / 84;
		// Coefficients for error estimation
		double c1 = 1 / 5;
		double c2 = 3 / 10;
		double c3 = 4 / 5;
		double c4 = 8 / 9;
		double c5 = 1;
		double c6 = 1;
		// Weights for 4th and 5th order estimates
		double b1 =  35 / 384;
		double b2 =  0;
		double b3 =  500 / 1113;
		double b4 =  125 / 192;
		double b5 = -2187 / 6784;
		double b6 =  11 / 84;
		double b1s = 517 / 57600;
		double b2s = 0;
		double b3s = 7571 / 16695;
		double b4s = 393 / 640;
		double b5s = -92097 / 339200;
		double b6s = 187 / 2100;

		// Compute the Runge-Kutta stages
		VecX k1 = dt * f(t, x);
		VecX k2 = dt * f(t + c1 * dt, x + a11 * k1);
		VecX k3 = dt * f(t + c2 * dt, x + a21 * k1 + a22 * k2);
		VecX k4 = dt * f(t + c3 * dt, x + a31 * k1 + a32 * k2 + a33 * k3);
		VecX k5 = dt * f(t + c4 * dt, x + a41 * k1 + a42 * k2 + a43 * k3 + a44 * k4);
		VecX k6 = dt * f(t + c5 * dt, x + a51 * k1 + a52 * k2 + a53 * k3 + a54 * k4 + a55 * k5);

		// Compute 4th and 5th order estimates
		VecX y4 = x + b1 * k1 + b2 * k2 + b3 * k3 + b4 * k4 + b5 * k5 + b6 * k6;   // 4th order estimate
		VecX y5 = x + b1s * k1 + b2s * k2 + b3s * k3 + b4s * k4 + b5s * k5 + b6s * k6; // 5th order estimate

		VecX error = y5 - y4; // Error estimate

		// Compute the error norm
		double errNorm = 0.0;
		for (int i = 0; i < error.size(); ++i) {
			double sc = atol + rtol * std::max<double>(std::abs(x(i)), std::abs(y5(i)));
			errNorm += (error(i) / sc) * (error(i) / sc);
		}
		errNorm = std::sqrt(errNorm / error.size());

		// Adaptive step size control
		double a = 0.9;
		if (errNorm <= 1.0) {
			// Accept the step
			dt *= std::min<double>(5.0, a * std::pow(errNorm, -0.2));
			return y5;
		} else {
			// Reject the step and reduce dt
			dt *= a * std::pow(errNorm, -0.25);
			return rk45Step(x, t, dt, f, rtol, atol); // Retry with new dt
		}
	}

	// Partial Differential Equation (PDE) solvers
	// ----------------------------------------------------------------
	// Finite Difference Method
	VecX PDE::fdmStep(const VecX& u, double dx, double dt, double alpha) {
		int n = u.size();
		VecX u_new = u;
		double r = alpha * dt / (dx * dx);
		for (int i = 1; i < n - 1; ++i) {
			u_new(i) = u(i) + r * (u(i + 1) - 2 * u(i) + u(i - 1));
		}
		return u_new;
	}
}