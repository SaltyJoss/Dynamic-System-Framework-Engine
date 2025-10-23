#include <Eigen/Dense>
#include <functional>
using namespace Eigen;

// Numerical integration methods
namespace mathlib::NumericalIntegrators {
// Ordinary Differential Equation (ODE) solvers
namespace ODE{
	// Euler method
	VectorXd eulerStep(const VectorXd& x, const VectorXd& dxdt, double dt) {
		return x + dxdt * dt;
	}

	// Second-order Runge-Kutta method (Heun / Midpoint method)
	VectorXd rk2Step(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f) {
		VectorXd k1 = dt * f(t, x);
		VectorXd k2 = dt * f(t + dt / 2.0, x + k1 / 2.0);
		return x + k2;
	}

	// Fourth-order Runge-Kutta method
	VectorXd rk4Step(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f) {
		VectorXd k1 = dt * f(t, x);
		VectorXd k2 = dt * f(t + dt / 2.0, x + k1 / 2.0);
		VectorXd k3 = dt * f(t + dt / 2.0, x + k2 / 2.0);
		VectorXd k4 = dt * f(t + dt, x + k3);
		return x + (k1 + 2.0 * k2 + 2.0 * k3 + k4)/6.0;
	}
}
}