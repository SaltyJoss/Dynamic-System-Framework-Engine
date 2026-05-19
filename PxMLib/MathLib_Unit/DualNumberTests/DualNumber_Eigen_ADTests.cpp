#include "TestHarness.h"
#include <core/DualNumbers.h>
#include <integrators/numerical_integrators.h>
#include <core/constants.h>

using namespace mathlib;
using namespace constants;

namespace {
	using Dual = DualNumber_T<double, 1>;
	constexpr double EPS = 1e-9;

	integration::NumericalIntegrator integrator;
}

TEST("DualNumber_T Eigen AD", PolynomialDerivative) {
	Dual x(2.0);
	x.dual[0] = 1.0;
	Dual y = x * x + 3.0 * x + 1.0; 
	ASSERT_TRUE(y.real == 11.0, "Polynomial evaluation real part incorrect");
	ASSERT_TRUE(y.dual[0] == 7.0, "Polynomial derivative dual part incorrect");
}
TEST("DualNumber_T Eigen AD", SinDerivative) {
	Dual x(PI_d / 4); // π/4
	x.dual[0] = 1.0;
	Dual y = sin(x);
	ASSERT_TRUE(std::abs(y.real - std::sin(PI_d / 4)) < EPS, "Sine evaluation real part incorrect");
	ASSERT_TRUE(std::abs(y.dual[0] - std::cos(PI_d / 4)) < EPS, "Sine derivative dual part incorrect");
}
TEST("DualNumber_T Eigen AD", CosDerivative) {
	Dual x(PI_d / 4); // π/4
	x.dual[0] = 1.0;
	Dual y = cos(x);
	ASSERT_TRUE(std::abs(y.real - std::cos(PI_d / 4)) < EPS, "Cosine evaluation real part incorrect");
	ASSERT_TRUE(std::abs(y.dual[0] + std::sin(PI_d / 4)) < EPS, "Cosine derivative dual part incorrect");
}
TEST("DualNumber_T Eigen AD", TanDerivative) {
	Dual x(PI_d / 4); // π/4
	x.dual[0] = 1.0;
	Dual y = tan(x);
	double expectedReal = std::tan(PI_d / 4);
	double expectedDual = 1.0 / (std::cos(PI_d / 4) * std::cos(PI_d / 4)); // sec^2(x)
	ASSERT_TRUE(std::abs(y.real - expectedReal) < EPS, "Tangent evaluation real part incorrect");
	ASSERT_TRUE(std::abs(y.dual[0] - expectedDual) < EPS, "Tangent derivative dual part incorrect");
}
TEST("DualNumber_T Eigen AD", ExpDerivative) {
	Dual x(1.0);
	x.dual[0] = 1.0;
	Dual y = exp(x);
	ASSERT_TRUE(std::abs(y.real - std::exp(1.0)) < EPS, "Exponential evaluation real part incorrect");
	ASSERT_TRUE(std::abs(y.dual[0] - std::exp(1.0)) < EPS, "Exponential derivative dual part incorrect");
}
TEST("DualNumber_T Eigen AD", LogDerivative) {
	Dual x(2.0);
	x.dual[0] = 1.0;
	Dual y = log(x);
	ASSERT_TRUE(std::abs(y.real - std::log(2.0)) < EPS, "Logarithm evaluation real part incorrect");
	ASSERT_TRUE(std::abs(y.dual[0] - 0.5) < EPS, "Logarithm derivative dual part incorrect");
}

//TEST("DualNumber_T Eigen AD", AutomaticDifferenceJacobian) {
//	// Define a simple function f: R^2 -> R^2 for testing
//	auto f = [](Dual t, const VecX_T<Dual>& x) -> VecX_T<Dual> {
//		VecX_T<Dual> y(2);
//		y(0) = x(0) * x(0) + t; // f1(x, t) = x1^2 + t
//		y(1) = x(0) * x(1);      // f2(x, t) = x1 * x2
//		return y;
//	};
//	Dual t = Dual(1.0, { 0.5 }); // Perturbation in time
//	VecX_T<Dual> x(2);
//	x << Dual(2.0, { 0.5 }), Dual(3.0, { 0.5 }); // Perturbations in state
//	// Compute the Jacobian using automatic differentiation
//	mathlib::MatX_T<double> J_ad = integration::NumericalIntegrator::automaticDifferenceJacobian(f, t, x);
//	// Compute the expected Jacobian manually
//	mathlib::MatX_T<double> J_expected(2, 2);
//	J_expected << 4.0, 0.0,
//		3.0, 2.0;
//	// Check that the computed Jacobian matches the expected Jacobian within a tolerance
//	for (int i = 0; i < J_ad.rows(); ++i) {
//		for (int j = 0; j < J_ad.cols(); ++j) {
//
//			ASSERT_TRUE(std::abs(J_ad(i, j) - J_expected(i, j)) < EPS,
//				"AD Jacobian entry (" + std::to_string(i) + ", " + std::to_string(j) + ") is incorrect");
//		}
//	}
//}