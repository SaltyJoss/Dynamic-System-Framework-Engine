// PxMLib/MathLib numerical_integrators.h
#pragma once
#include <core/MathLib.h>
#include <string>
#include <functional>
#include <iostream>
#include <format>
#include <sstream>

// Numerical integration methods
namespace integration {
	constexpr bool USE_AD_JACOBIANS = false;
	// Ordinary Differential Equation (ODE) solvers
	class NumericalIntegrator {
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
		// AD version of Implicit Euler method
		template<typename Scalar, typename Func>
		mathlib::VecX_T<Scalar> implicitEuler_AD(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f,
			int maxIter,
			typename mathlib::DualTraits<Scalar>::BaseScalar tol
		);

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
		// AD version of Implicit Midpoint method
		template<typename Scalar, typename Func>
		mathlib::VecX_T<Scalar> implicitMidpoint_AD(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f,
			int maxIter,
			typename mathlib::DualTraits<Scalar>::BaseScalar tol
		);

		// Gauss-Legendre Runge-Kutta method (2 stages, 4th order)
		template<typename Scalar, typename Func, typename JacFunc = std::nullptr_t>
		mathlib::VecX_T<Scalar> GLRK2(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f,
			JacFunc&& jac = nullptr,
			int maxIter = 80,
			Scalar tol = Scalar(1e-10)
		);
		// AD version of GLRK2
		template<typename Scalar, typename Func>
		mathlib::VecX_T<Scalar> GLRK2_AD(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f,
			int maxIter,
			typename mathlib::DualTraits<Scalar>::BaseScalar tol
		);

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
		// AD version of GLRK3
		template<typename Scalar, typename Func>
		mathlib::VecX_T<Scalar> GLRK3_AD(
			const mathlib::VecX_T<Scalar>& x,
			Scalar t,
			Scalar dt,
			Func&& f,
			int maxIter,
			typename mathlib::DualTraits<Scalar>::BaseScalar tol
		);

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
		// AD version of Newton-Raphson solver
		template<typename Scalar, typename EvalG, typename EvalJ>
		mathlib::VecX_T<Scalar> newtonRaphson_AD(
			EvalG&& eval_g,
			EvalJ&& eval_j,
			mathlib::VecX_T<Scalar> x0,
			int maxIter,
			typename mathlib::DualTraits<Scalar>::BaseScalar tol
		);

		// Finite difference approximation of the Jacobian matrix df/dx for a vector-valued function f: R^n -> R^m at a point x
		template<typename Scalar, typename Func>
		mathlib::MatX_T<Scalar> finiteDifferenceJacobian(
			Func&& f,
			Scalar t,
			const mathlib::VecX_T<Scalar>& x
		);

		template<typename RealScalar, size_t NVar, typename Func>
		mathlib::MatX_T<RealScalar> automaticDifferenceJacobian(
			Func&& f,
			mathlib::DualNumber_T<RealScalar, NVar> t,
			const mathlib::VecX_T<mathlib::DualNumber_T<RealScalar, NVar>>& x
		);
	};
}// namespace integration

#include "explicit_integrators.inl"
#include "implicit_integrators.inl"
#include "auto_diff_integrators.inl"