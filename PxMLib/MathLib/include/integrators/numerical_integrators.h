#pragma once
// File:   numerical_integrators.h
// GitHub: SaltyJoss
#include "MathLibAPI.h"

#include <core/Types.h>
#include <core/Types_tpl.h>
#include <core/DualNumbers.h>
#include <string>
#include <functional>
#include <iostream>

// Numerical integration methods
namespace integration {
	constexpr bool USE_AD_JACOBIANS = true;

	// Ordinary Differential Equation (ODE) solvers
	class MATHLIB_API NumericalIntegrator {
	public:
		// Euler method
		template<typename Scalar, typename Func>
		mathlib::VecX_T<Scalar> eulerStep(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f
		);

		// Second-order Runge-Kutta method (Midpoint)
		template<typename Scalar, typename Func>
		mathlib::VecX_T<Scalar> midpointStep(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f
		);

		// Second-order Runge-Kutta method (Heun)
		template<typename Scalar, typename Func>
		mathlib::VecX_T<Scalar> heunStep(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f
		);

		// Second-order Runge-Kutta method (Ralston)
		template<typename Scalar, typename Func>
		mathlib::VecX_T<Scalar> ralstonStep(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f);

		// Fourth-order Runge-Kutta method
		template<typename Scalar, typename Func>
		mathlib::VecX_T<Scalar> rk4Step(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f
		);

		// RK45 method with adaptive step size (Dormand-Prince)
		template<typename Scalar, typename Func>
		mathlib::VecX_T<Scalar> rk45Step(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar& dt,
			Scalar& dt_used,
			Func&& f,
			Scalar rtol,
			Scalar atol
		);

		// Implicit Euler method
		template<typename Scalar, typename Func, typename JacFunc = std::nullptr_t>
		mathlib::VecX_T<Scalar> implicitEuler(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f,
			JacFunc&& jac = nullptr,
			int maxIter = 8,
			Scalar tol = Scalar(1e-6)
		);
		// Overload without Jacobian
		template<typename Scalar, typename Func>
		mathlib::VecX_T<Scalar> ImplicitEuler(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f,
			int maxIter,
			Scalar tol
		) {
			return ImplicitEuler(
				x,
				t,
				dt,
				std::forward<Func>(f),
				nullptr,
				maxIter,
				tol
			);
		}

		// Implicit Midpoint method
		template<typename Scalar, typename Func, typename JacFunc = std::nullptr_t>
		mathlib::VecX_T<Scalar> implicitMidpoint(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f,
			JacFunc&& jac = nullptr,
			int maxIter = 10,
			Scalar tol = Scalar(1e-7)
		);
		// Overload without Jacobian
		template<typename Scalar, typename Func>
		mathlib::VecX_T<Scalar> ImplicitMidpoint(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f,
			int maxIter,
			Scalar tol
		) {
			return ImplicitMidpoint(
				x,
				t,
				dt,
				std::forward<Func>(f),
				nullptr,
				maxIter,
				tol
			);
		}

		// Gauss-Legendre Runge-Kutta method (2 stages, 4th order)
		template<typename Scalar, typename Func, typename JacFunc = std::nullptr_t>
		mathlib::VecX_T<Scalar> GLRK2(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f,
			JacFunc&& jac = nullptr,
			int maxIter = 50,
			Scalar tol = Scalar(1e-6)
		);
		// Overload without Jacobian
		template<typename Scalar, typename Func>
		mathlib::VecX_T<Scalar> GLRK2(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f,
			int maxIter,
			Scalar tol
		) {
			return GLRK2(
				x,
				t,
				dt,
				std::forward<Func>(f),
				nullptr,
				maxIter,
				tol
			);
		}

		// Gauss-Legendre Runge-Kutta method (3 stages, 6th order)
		template<typename Scalar, typename Func, typename JacFunc = std::nullptr_t>
		mathlib::VecX_T<Scalar> GLRK3(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f,
			JacFunc&& jac = nullptr,
			int maxIter = 150,
			Scalar tol = Scalar(1e-14)
		);
		// Overload without Jacobian
		template<typename Scalar, typename Func>
		mathlib::VecX_T<Scalar> GLRK3(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f,
			int maxIter,
			Scalar tol
		) {
			return GLRK3(
				x,
				t,
				dt,
				std::forward<Func>(f),
				nullptr,
				maxIter,
				tol
			);
		}

	private:

		// Newton-Raphson solver for systems of nonlinear equations g(x) = 0
		template<typename Scalar, typename EvalG, typename EvalJ>
		mathlib::VecX_T<Scalar> newtonRaphson(
			EvalG&& eval_g,
			EvalJ&& eval_j,
			mathlib::VecX_T<Scalar> x0,
			int maxIter,
			Scalar tol
		);

		// Finite difference approximation of the Jacobian matrix df/dx for a vector-valued function f: R^n -> R^m at a point x
		template<typename Scalar, typename Func>
		mathlib::MatX_T<Scalar> finiteDifferenceJacobian(
			Func&& f,
			Scalar t,
			const mathlib::VecX_T<Scalar>& x
		);

		template<typename Scalar, typename Func>
		mathlib::MatX_T<Scalar> automaticDifferenceJacobian(
			Func&& f,
			Scalar t,
			const mathlib::VecX_T<Scalar>& x
		);
	};
}// namespace integration

#include "numerical_integrators.inl"