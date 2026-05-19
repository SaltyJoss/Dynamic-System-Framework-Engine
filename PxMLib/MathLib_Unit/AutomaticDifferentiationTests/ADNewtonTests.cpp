// MathLib_UnitTests ADNewtonTests.cpp

#include "TestHarness.h"
#include <core/DualNumbers.h>
#include <cmath>
#include <functional>

using namespace mathlib;

namespace {
	// Simple nonlinear function: f(x) = x^2 - 2, which has a root at sqrt(2)
	template<typename Scalar, size_t NVar>
	DualNumber_T<Scalar, NVar> nonlinearFunc(const DualNumber_T<Scalar, NVar>& x) {
		return pow(x, Scalar(2)) - DualNumber_T<Scalar, NVar>(Scalar(2));
	};

	// Analytical derivative of the nonlinear function: f'(x) = 2x
	template<typename Scalar, size_t NVar>
	DualNumber_T<Scalar, NVar> nonlinearFuncDerivative(const DualNumber_T<Scalar, NVar>& x) {
		return Scalar(2) * x;
	};

	// Helper function to perform a single Newton-Raphson iteration using dual numbers
	template<typename Scalar, size_t NVar>
	DualNumber_T<Scalar, NVar> newtonRaphsonStep(
		const DualNumber_T<Scalar, NVar>& x
	) {
		auto g = nonlinearFunc<Scalar, NVar>(x);
		auto J = nonlinearFuncDerivative<Scalar, NVar>(x);
		return x - g / J;
	}
}

// Test that the Newton-Raphson step using dual numbers correctly finds the root of the nonlinear function and that the Jacobian is computed correctly.
TEST("AD Newton-Raphson", ADNewton_RootFinding) {
	DualNumber_T<double, 1> x(1.0, { 1.0 });
	for (int iter = 0; iter < 10; ++iter) {
		x = newtonRaphsonStep<double, 1>(x);
	}
	double root = x.real;
	ASSERT_TRUE(std::abs(root - std::sqrt(2.0)) < 1e-6, "Root finding error too large");
}
// Test that the Newton-Raphson step using dual numbers converges to the root of the nonlinear function from different initial guesses.
TEST("AD Newton-Raphson", ADNewton_Convergence) {
	std::vector<double> x0 = { 0.5, 1.0, 1.5, 2.0 };
	for (double x_guess : x0) {
		DualNumber_T<double, 1> x(x_guess, { 1.0 });
		for (int iter = 0; iter < 10; ++iter) {
			x = newtonRaphsonStep(x);
		}
		double root = x.real;
		double residual = std::abs(nonlinearFunc<double, 1>(x).real);

		std::string errorMsg = "Root finding error too large for initial guess " + std::to_string(x_guess);
		ASSERT_TRUE(std::abs(root - std::sqrt(2.0)) < 1e-6, errorMsg.c_str());
		ASSERT_TRUE(residual < 1e-10, "Residual is too large after Newton solve");
	}
}
// Test that the Jacobian computed using dual numbers matches the analytical derivative of the nonlinear function.
TEST("AD Newton-Raphson", ADNewton_JacobianAccuracy) {
	DualNumber_T<double, 1> x(1.0, { 1.0 });
	DualNumber_T<double, 1> g = nonlinearFunc<double, 1>(x);
	double computedJacobian = g.dual[0];
	double expectedJacobian = nonlinearFuncDerivative<double, 1>(x).real;
	ASSERT_TRUE(std::abs(computedJacobian - expectedJacobian) < 1e-6, "Jacobian accuracy error too large");
}