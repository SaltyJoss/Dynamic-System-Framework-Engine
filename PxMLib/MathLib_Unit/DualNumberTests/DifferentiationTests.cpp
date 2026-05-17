// MathLib_UnitTests DifferentiationTests.cpp

#include "TestHarness.h"
#include <core/DualNumbers.h>
#include <cmath>
#include <functional>

using namespace mathlib;

namespace {
	// Test that the derivative of a simple polynomial function is computed correctly using dual numbers
	TEST("Differentiation", DualNumber_PolynomialDerivative) {
		auto f = [](const DualNumber_T<double, 1>& x) {
			return 3.0 * x * x + 2.0 * x + 1.0; // f(x) = 3x^2 + 2x + 1, f'(x) = 6x + 2
		};
		DualNumber_T<double, 1> x(1.0, { 1.0 }); // Set dual part to 1 to compute derivative
		DualNumber_T<double, 1> y = f(x);
		ASSERT_TRUE(std::abs(y.real - 6.0) < 1e-12, "Polynomial function value incorrect");
		ASSERT_TRUE(std::abs(y.dual[0] - 8.0) < 1e-12, "Polynomial derivative incorrect");
	}
	// Test that the derivative of x^2 is computed correctly by the product rule using dual numbers
	TEST("Differentiation", DualNumber_ProductRule_xSquared) {
		auto f = [](const DualNumber_T<double, 1>& x) {
			return x * x; // f(x) = x^2
		};
		DualNumber_T<double, 1> x(2.0, { 1.0 }); // Set dual part to 1 to compute derivative
		DualNumber_T<double, 1> y = f(x);
		ASSERT_TRUE(std::abs(y.real - 4.0) < 1e-12, "Product rule function value incorrect");
		ASSERT_TRUE(std::abs(y.dual[0] - 4.0) < 1e-12, "Product rule derivative incorrect");
	}
	// Test that the sine function's derivative is computed correctly using dual numbers
	TEST("Differentiation", DualNumber_SineDerivative) {
		auto f = [](const DualNumber_T<double, 1>& x) {
			return sin(x);
		};
		DualNumber_T<double, 1> x(0.5, { 1.0 }); // Set dual part to 1 to compute derivative
		DualNumber_T<double, 1> y = f(x);
		ASSERT_TRUE(std::abs(y.real - std::sin(0.5)) < 1e-12, "Sine function value incorrect");
		ASSERT_TRUE(std::abs(y.dual[0] - std::cos(0.5)) < 1e-12, "Sine derivative incorrect");
	}
	// Test that the cosine function's derivative is computed correctly using dual numbers
	TEST("Differentiation", DualNumber_CosineDerivative) {
		auto f = [](const DualNumber_T<double, 1>& x) {
			return cos(x);
		};
		DualNumber_T<double, 1> x(0.5, { 1.0 }); // Set dual part to 1 to compute derivative
		DualNumber_T<double, 1> y = f(x);
		ASSERT_TRUE(std::abs(y.real - std::cos(0.5)) < 1e-12, "Cosine function value incorrect");
		ASSERT_TRUE(std::abs(y.dual[0] + std::sin(0.5)) < 1e-12, "Cosine derivative incorrect");
	}
	// Test that the tangent function's derivative is computed correctly using dual numbers
	TEST("Differentiation", DualNumber_TangentDerivative) {
		auto f = [](const DualNumber_T<double, 1>& x) {
			return tan(x);
		};
		DualNumber_T<double, 1> x(0.5, { 1.0 }); // Set dual part to 1 to compute derivative
		DualNumber_T<double, 1> y = f(x);
		ASSERT_TRUE(std::abs(y.real - std::tan(0.5)) < 1e-12, "Tangent function value incorrect");
		double sec2 = 1.0 / (std::cos(0.5) * std::cos(0.5));
		ASSERT_TRUE(std::abs(y.dual[0] - sec2) < 1e-12, "Tangent derivative incorrect");
	}
	// Test that the derivative of the hyperbolic tangent function is computed correctly using dual numbers
	TEST("Differentiation", DualNumber_HyperbolicTangentDerivative) {
		auto f = [](const DualNumber_T<double, 1>& x) {
			return tanh(x);
		};
		DualNumber_T<double, 1> x(0.5, { 1.0 }); // Set dual part to 1 to compute derivative
		DualNumber_T<double, 1> y = f(x);
		ASSERT_TRUE(std::abs(y.real - std::tanh(0.5)) < 1e-12, "Hyperbolic tangent function value incorrect");
		double sech2 = 1.0 / (std::cosh(0.5) * std::cosh(0.5));
		ASSERT_TRUE(std::abs(y.dual[0] - sech2) < 1e-12, "Hyperbolic tangent derivative incorrect");
	}
	// Test that the partial derivatives of a multivariable function are computed correctly using dual numbers
	TEST("Differentiation", DualNumber_MultivariableFunction) {
		auto f = [](const DualNumber_T<double, 2>& x, const DualNumber_T<double, 2>& y) {
			return 3.0 * x * x + 2.0 * y + sin(x); // f(x, y) = 3x^2 + 2y + sin(x)
		};

		DualNumber_T<double, 2> x(2.0, { 1.0, 0.0 }); // Set dual part to (1, 0) to compute partial derivative with respect to x
		DualNumber_T<double, 2> y(3.0, { 0.0, 1.0 }); // Set dual part to (0, 1) to compute partial derivative with respect to y

		double dfdx = 6.0 * x.real + std::cos(x.real); // df/dx = 6x + cos(x)
		double dfdy = 2.0; // df/dy = 2

		ASSERT_TRUE(std::abs(f(x, y).real - (3.0 * 4.0 + 2.0 * 3.0 + std::sin(2.0))) < 1e-12, "Multivariable function value incorrect");
		ASSERT_TRUE(std::abs(f(x, y).dual[0] - dfdx) < 1e-12, "Partial derivative with respect to x incorrect");
		ASSERT_TRUE(std::abs(f(x, y).dual[1] - dfdy) < 1e-12, "Partial derivative with respect to y incorrect");
	}
	// Test that the exponential function's derivative is computed correctly using dual numbers
	TEST("Differentiation", DualNumber_ExponentialDerivative) {
		auto f = [](const DualNumber_T<double, 1>& x) {
			return exp(x);
		};
		DualNumber_T<double, 1> x(0.5, { 1.0 }); // Set dual part to 1 to compute derivative
		DualNumber_T<double, 1> y = f(x);
		double expected0 = std::exp(0.5); // f(x) = exp(x) -> f'(x) = exp(x)
		ASSERT_TRUE(std::abs(y.real - expected0) < 1e-12, "Exponential function value incorrect");
		ASSERT_TRUE(std::abs(y.dual[0] - expected0) < 1e-12, "Exponential derivative incorrect");
	}
}