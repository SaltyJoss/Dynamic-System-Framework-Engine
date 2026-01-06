#pragma once

// ==============================================================
// File: numerical_integrators.h
// =============================================================
// Header file for numerical integrators including ODE and PDE solvers
//
// Summary:
// =============================================================
// eulerStep(const VecX& x, const VecX& dxdt, double dt)
//		-> Performs a single Euler integration step for ODEs - useful for simple, quick approximations.
// midpointStep(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f)
// 		-> Implements the second-order Runge-Kutta method (Midpoint) for improved accuracy over Euler (Uses midpoint slope).
// heunStep(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f)
// 		-> Implements the second-order Runge-Kutta method (Heun) for better accuracy in ODE integration (Uses average slope [trapezoidal-esc approach]).
// ralstonStep(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f)
// 		-> Implements the second-order Runge-Kutta method (Ralston) for better accuracy in ODE integration (Uses optimised weights).
// rk4Step(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f)
//		-> Implements the fourth-order Runge-Kutta method for high-accuracy ODE integration.
// fdmStep(const VecX& u, double dx, double dt, double alpha)
//		-> Applies the Finite Difference Method for solving PDEs, is not used for simulations that simulate continuous systems like a robot arm because they typically involve spatial discretisation.
// =============================================================

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
		VecX eulerStep(const VecX& x, const VecX& dxdt, double dt);

		// Second-order Runge-Kutta method (Midpoint)
		VecX midpointStep(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f);

		// Second-order Runge-Kutta method (Heun)
		VecX heunStep(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f);

		// Second-order Runge-Kutta method (Ralston)
		VecX ralstonStep(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f);

		// Fourth-order Runge-Kutta method
		VecX rk4Step(const VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f);

		// RK45 method with adaptive step size (Dormand-Prince)
		VecX rk45Step(const VecX& x, double t, double& dt, std::function<VecX(double, const VecX&)> f, double rtol, double atol);
	};

	// Partial Differential Equation (PDE) solvers --> Not going to use really in my current scope, just thought to include for completeness
	class MATHLIB_API PDE {
	public:
		// Finite Difference Method (FDM) for discrete points in space
		VecX fdmStep(const VecX& u, double dx, double dt, double alpha);
	};
}