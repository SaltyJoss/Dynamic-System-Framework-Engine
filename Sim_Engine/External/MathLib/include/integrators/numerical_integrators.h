#pragma once

// ==============================================================
// File: numerical_integrators.h
// =============================================================
// Header file for numerical integrators including ODE and PDE solvers
//
// Summary:
// =============================================================
// eulerStep(const VectorXd& x, const VectorXd& dxdt, double dt)
//		-> Performs a single Euler integration step for ODEs - useful for simple, quick approximations.
// midpointStep(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f)
// 		-> Implements the second-order Runge-Kutta method (Midpoint) for improved accuracy over Euler (Uses midpoint slope).
// heunStep(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f)
// 		-> Implements the second-order Runge-Kutta method (Heun) for better accuracy in ODE integration (Uses average slope [trapezoidal-esc approach]).
// ralstonStep(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f)
// 		-> Implements the second-order Runge-Kutta method (Ralston) for better accuracy in ODE integration (Uses optimised weights).
// rk4Step(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f)
//		-> Implements the fourth-order Runge-Kutta method for high-accuracy ODE integration.
// fdmStep(const VectorXd& u, double dx, double dt, double alpha)
//		-> Applies the Finite Difference Method for solving PDEs, is not used for simulations that simulate continuous systems like a robot arm because they typically involve spatial discretisation.
// =============================================================

#include "MathLibAPI.h"

#include <Eigen/Dense>
#include <functional>
using namespace Eigen;

// Numerical integration methods
namespace integration {
	// Ordinary Differential Equation (ODE) solvers
	class MATHLIB_API ODE {
	public:
		// Euler method
		VectorXd eulerStep(const VectorXd& x, const VectorXd& dxdt, double dt);

		// Second-order Runge-Kutta method (Midpoint)
		VectorXd midpointStep(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f);

		// Second-order Runge-Kutta method (Heun)
		VectorXd heunStep(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f);

		// Second-order Runge-Kutta method (Ralston)
		VectorXd ralstonStep(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f);

		// Fourth-order Runge-Kutta method
		VectorXd rk4Step(const VectorXd& x, double t, double dt, std::function<VectorXd(double, const VectorXd&)> f);
	};

	// Partial Differential Equation (PDE) solvers --> Not going to use really in my current scope, just thought to include for completeness
	class MATHLIB_API PDE {
	public:
		// Finite Difference Method (FDM) for discrete points in space
		VectorXd fdmStep(const VectorXd& u, double dx, double dt, double alpha);
	};
}