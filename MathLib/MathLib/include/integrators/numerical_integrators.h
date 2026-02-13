#pragma once
// File:   numerical_integrators.h
// GitHub: SaltyJoss
#include "MathLibAPI.h"

#include "core/Types.h"
#include <functional>

using namespace mathlib;

// Numerical integration methods
namespace integration {
	// Ordinary Differential Equation (ODE) solvers
	class MATHLIB_API ODE {
	public:
		// Euler method
		VecX eulerStep(const VecX& x, double t, double dt, const std::function<VecX(double, const VecX&)>& f);

		// Second-order Runge-Kutta method (Midpoint)
		VecX midpointStep(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f);

		// Second-order Runge-Kutta method (Heun)
		VecX heunStep(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f);

		// Second-order Runge-Kutta method (Ralston)
		VecX ralstonStep(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f);

		// Fourth-order Runge-Kutta method
		VecX rk4Step(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f);

		// RK45 method with adaptive step size (Dormand-Prince)
		VecX rk45Step(const VecX& x, double t, double& dt, double& dt_used, std::function<VecX(double, const VecX&)> f, double rtol, double atol);
	};

	// Partial Differential Equation (PDE) solvers --> Not going to use really in my current scope, just thought to include for completeness
	class MATHLIB_API PDE {
	public:
		// Finite Difference Method (FDM) for discrete points in space
		VecX fdmStep(const VecX& u, double dx, double dt, double alpha);
	};
}