#include "TestHarness.h"
#include <core/MathLib.h>
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
	ASSERT_TRUE(abs(y.real - sin(PI_d / 4.0)) < EPS, "Sine evaluation real part incorrect");
	ASSERT_TRUE(abs(y.dual[0] - cos(PI_d / 4.0)) < EPS, "Sine derivative dual part incorrect");
}
TEST("DualNumber_T Eigen AD", CosDerivative) {
	Dual x(PI_d / 4); // π/4
	x.dual[0] = 1.0;
	Dual y = cos(x);
	ASSERT_TRUE(abs(y.real - cos(PI_d / 4)) < EPS, "Cosine evaluation real part incorrect");
	ASSERT_TRUE(abs(y.dual[0] + sin(PI_d / 4)) < EPS, "Cosine derivative dual part incorrect");
}
TEST("DualNumber_T Eigen AD", TanDerivative) {
	Dual x(PI_d / 4.0);
	x.dual[0] = 1.0;
	Dual y = tan(x);
	double expectedReal = tan<double>(PI_d / 4.0);
	double expectedDual = 1.0 / (cos(PI_d / 4.0) * cos(PI_d / 4.0)); // sec^2(x)
	ASSERT_TRUE(abs(y.real - expectedReal) < EPS, "Tangent evaluation real part incorrect");
	ASSERT_TRUE(abs(y.dual[0] - expectedDual) < EPS, "Tangent derivative dual part incorrect");
}
TEST("DualNumber_T Eigen AD", ExpDerivative) {
	Dual x(1.0);
	x.dual[0] = 1.0;
	Dual y = exp(x);
	ASSERT_TRUE(abs(y.real - exp(1.0)) < EPS, "Exponential evaluation real part incorrect");
	ASSERT_TRUE(abs(y.dual[0] - exp(1.0)) < EPS, "Exponential derivative dual part incorrect");
}
TEST("DualNumber_T Eigen AD", LogDerivative) {
	Dual x(2.0);
	x.dual[0] = 1.0;
	Dual y = log(x);
	ASSERT_TRUE(abs<double>(y.real - log(2.0)) < EPS, "Logarithm evaluation real part incorrect");
	ASSERT_TRUE(abs<double>(y.dual[0] - 0.5) < EPS, "Logarithm derivative dual part incorrect");
}