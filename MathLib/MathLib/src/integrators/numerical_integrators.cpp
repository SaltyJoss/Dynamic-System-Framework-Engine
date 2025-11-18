#include "pch.h"
#include "integrators/numerical_integrators.h"

#pragma message("MATHLIB_BUILD = " _CRT_STRINGIZE(MATHLIB_BUILD))
#pragma message("MATHLIB_API = " _CRT_STRINGIZE(MATHLIB_API))

// Numerical integration methods
namespace integration {
	// Ordinary Differential Equation (ODE) solvers
	// ----------------------------------------------------------------
	// Euler method
	VectorXd ODE::eulerStep(const VectorXd& x, const VectorXd& dxdt, double dt) {
		return x + dxdt * dt;
	}

	// Second-order Runge-Kutta method (Heun / Midpoint method)
	VectorXd ODE::rk2Step(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f) {
		VectorXd k1 = dt * f(t, x);
		VectorXd k2 = dt * f(t + dt / 2.0, x + k1 / 2.0);
		return x + k2;
	}

	// Fourth-order Runge-Kutta method
	VectorXd ODE::rk4Step(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f) {
		VectorXd k1 = dt * f(t, x);
		VectorXd k2 = dt * f(t + dt / 2.0, x + k1 / 2.0);
		VectorXd k3 = dt * f(t + dt / 2.0, x + k2 / 2.0);
		VectorXd k4 = dt * f(t + dt, x + k3);
		return x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
	}

	// Partial Differential Equation (PDE) solvers
	// ----------------------------------------------------------------
	// Finite Difference Method
	VectorXd PDE::fdmStep(const VectorXd& u, double dx, double dt, double alpha) {
		/*stub*/
	}
}