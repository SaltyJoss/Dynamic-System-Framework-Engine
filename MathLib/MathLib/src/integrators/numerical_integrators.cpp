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