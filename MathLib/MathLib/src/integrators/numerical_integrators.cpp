#include "pch.h"
#include "integrators/numerical_integrators.h"

// Numerical integration methods
namespace integration {
	// Ordinary Differential Equation (ODE) solvers
	// ----------------------------------------------------------------
	// Euler method
	VectorXd ODE::eulerStep(const VectorXd& x, const VectorXd& dxdt, double dt) {
		return x + dxdt * dt;
	}

	// Second-order Runge-Kutta method (Midpoint method)
	VectorXd ODE::midpointStep(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f) {
		VectorXd k1 = dt * f(t, x);
		VectorXd k2 = dt * f(t + dt / 2.0, x + k1 / 2.0);
		return x + k2;
	}

	// Second-order Runge-Kutta method (Heun's method)
	VectorXd ODE::heunStep(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f) {
		VectorXd k1 = dt * f(t, x);
		VectorXd k2 = dt * f(t + dt, x + k1);
		return x + (k1 + k2) / 2.0;
	}

	// Second-order Runge-Kutta method (Ralston's method)
	VectorXd ODE::ralstonStep(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f) {
		VectorXd k1 = dt * f(t, x);
		VectorXd k2 = dt * f(t + (2.0 / 3.0) * dt, x + (2.0 / 3.0) * k1);
		return x + (k1 + 3.0 * k2) / 4.0;
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
		int n = u.size();
		VectorXd u_new = u;
		double r = alpha * dt / (dx * dx);
		for (int i = 1; i < n - 1; ++i) {
			u_new(i) = u(i) + r * (u(i + 1) - 2 * u(i) + u(i - 1));
		}
		return u_new;
	}
}