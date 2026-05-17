// MathLib_UnitTests TranscendentalTests.cpp

#include "TestHarness.h"
#include <core/DualNumbers.h>
#include <cmath>
#include <functional>

using namespace mathlib;

namespace {
	// Test that the sine function works correctly for dual
	TEST("Transcendental Functions", DualNumber_Sine) {
		DualNumber_T<double, 2> a(0.5, { 0.1, 0.05 });
		DualNumber_T<double, 2> s = sin(a);
		ASSERT_TRUE(std::abs(s.real - std::sin(0.5)) < 1e-12, "Sine real part incorrect");
		ASSERT_TRUE(std::abs(s.dual[0] - (std::cos(0.5) * 0.1)) < 1e-12, "Sine dual[0] part incorrect");
		ASSERT_TRUE(std::abs(s.dual[1] - (std::cos(0.5) * 0.05)) < 1e-12, "Sine dual[1] part incorrect");
	}
	// Test that the sine function works correctly for dual numbers defined with float type
	TEST("Transcendental Functions", DualNumber_Sine_f) {
		DualNumber_T<float, 2> a(0.5f, { 0.1f, 0.05f });
		DualNumber_T<float, 2> s = sin(a);
		ASSERT_TRUE(std::abs(s.real - std::sin(0.5f)) < 1e-5f, "Sine real part incorrect");
		ASSERT_TRUE(std::abs(s.dual[0] - (std::cos(0.5f) * 0.1f)) < 1e-5f, "Sine dual[0] part incorrect");
		ASSERT_TRUE(std::abs(s.dual[1] - (std::cos(0.5f) * 0.05f)) < 1e-5f, "Sine dual[1] part incorrect");
	}
	// Test that the cosine function works correctly for dual
	TEST("Transcendental Functions", DualNumber_Cosine) {
		DualNumber_T<double, 2> a(0.5, { 0.1, 0.05 });
		DualNumber_T<double, 2> c = cos(a);
		ASSERT_TRUE(std::abs(c.real - std::cos(0.5)) < 1e-12, "Cosine real part incorrect");
		ASSERT_TRUE(std::abs(c.dual[0] - (-std::sin(0.5) * 0.1)) < 1e-12, "Cosine dual[0] part incorrect");
		ASSERT_TRUE(std::abs(c.dual[1] - (-std::sin(0.5) * 0.05)) < 1e-12, "Cosine dual[1] part incorrect");
	}
	// Test that the cosine function works correctly for dual numbers defined with float type
	TEST("Transcendental Functions", DualNumber_Cosine_f) {
		DualNumber_T<float, 2> a(0.5f, { 0.1f, 0.05f });
		DualNumber_T<float, 2> c = cos(a);
		ASSERT_TRUE(std::abs(c.real - std::cos(0.5f)) < 1e-5f, "Cosine real part incorrect");
		ASSERT_TRUE(std::abs(c.dual[0] - (-std::sin(0.5f) * 0.1f)) < 1e-5f, "Cosine dual[0] part incorrect");
		ASSERT_TRUE(std::abs(c.dual[1] - (-std::sin(0.5f) * 0.05f)) < 1e-5f, "Cosine dual[1] part incorrect");
	}
	// Test that the tangent function works correctly for dual
	TEST("Transcendental Functions", DualNumber_Tangent) {
		DualNumber_T<double, 2> a(0.5, { 0.1, 0.05 });
		DualNumber_T<double, 2> t = tan(a);
		ASSERT_TRUE(std::abs(t.real - std::tan(0.5)) < 1e-12, "Tangent real part incorrect");
		double sec2 = 1.0 / (std::cos(0.5) * std::cos(0.5));
		ASSERT_TRUE(std::abs(t.dual[0] - (sec2 * 0.1)) < 1e-12, "Tangent dual[0] part incorrect");
		ASSERT_TRUE(std::abs(t.dual[1] - (sec2 * 0.05)) < 1e-12, "Tangent dual[1] part incorrect");
	}
	// Test that the tangent function works correctly for dual numbers defined with float type
	TEST("Transcendental Functions", DualNumber_Tangent_f) {
		DualNumber_T<float, 2> a(0.5f, { 0.1f, 0.05f });
		DualNumber_T<float, 2> t = tan(a);
		ASSERT_TRUE(std::abs(t.real - std::tan(0.5f)) < 1e-5f, "Tangent real part incorrect");
		float sec2 = 1.0f / (std::cos(0.5f) * std::cos(0.5f));
		ASSERT_TRUE(std::abs(t.dual[0] - (sec2 * 0.1f)) < 1e-5f, "Tangent dual[0] part incorrect");
		ASSERT_TRUE(std::abs(t.dual[1] - (sec2 * 0.05f)) < 1e-5f, "Tangent dual[1] part incorrect");
	}
	// Test that the sine and cosine functions satisfy the Pythagorean identity for dual numbers
	TEST("Transcendental Functions", DualNumber_PythagoreanIdentity) {
		DualNumber_T<double, 2> a(0.5, { 0.1, 0.05 });
		DualNumber_T<double, 2> s = sin(a);
		DualNumber_T<double, 2> c = cos(a);
		double sumReal = s.real * s.real + c.real * c.real;
		double sumDual0 = 2.0 * (s.real * s.dual[0] + c.real * c.dual[0]);
		double sumDual1 = 2.0 * (s.real * s.dual[1] + c.real * c.dual[1]);
		ASSERT_TRUE(std::abs(sumReal - 1.0) < 1e-12, "Pythagorean identity real part incorrect");
		ASSERT_TRUE(std::abs(sumDual0) < 1e-12, "Pythagorean identity dual[0] part incorrect");
		ASSERT_TRUE(std::abs(sumDual1) < 1e-12, "Pythagorean identity dual[1] part incorrect");
	}
	// Test that the tangent function can be expressed as sine divided by cosine for dual numbers
	TEST("Transcendental Functions", DualNumber_TangentAsSineOverCosine) {
		DualNumber_T<double, 2> a(0.5, { 0.1, 0.05 });
		DualNumber_T<double, 2> s = sin(a);
		DualNumber_T<double, 2> c = cos(a);
		DualNumber_T<double, 2> t = tan(a);
		DualNumber_T<double, 2> t_from_sc = s / c;
		ASSERT_TRUE(std::abs(t.real - t_from_sc.real) < 1e-12, "Tangent as sine/cosine real part incorrect");
		ASSERT_TRUE(std::abs(t.dual[0] - t_from_sc.dual[0]) < 1e-12, "Tangent as sine/cosine dual[0] part incorrect");
		ASSERT_TRUE(std::abs(t.dual[1] - t_from_sc.dual[1]) < 1e-12, "Tangent as sine/cosine dual[1] part incorrect");
	}
	// Tests that the hyperbolic tangent function works correctly for dual numbers
	TEST("Transcendental Functions", DualNumber_HyperbolicTangent) {
		DualNumber_T<double, 2> a(0.5, { 0.1, 0.05 });
		DualNumber_T<double, 2> t = tanh(a);
		ASSERT_TRUE(std::abs(t.real - std::tanh(0.5)) < 1e-12, "Hyperbolic tangent real part incorrect");
		double sech2 = 1.0 / (std::cosh(0.5) * std::cosh(0.5)); // sech^2(x) = 1 - tanh^2(x) <-> cosh has a more stable implementation for large x
		ASSERT_TRUE(std::abs(t.dual[0] - (sech2 * 0.1)) < 1e-12, "Hyperbolic tangent dual[0] part incorrect");
		ASSERT_TRUE(std::abs(t.dual[1] - (sech2 * 0.05)) < 1e-12, "Hyperbolic tangent dual[1] part incorrect");
	}
	// Tests that the exponential function works correctly for dual numbers
	TEST("Transcendental Functions", DualNumber_Exponential) {
		DualNumber_T<double, 2> a(0.5, { 0.1, 0.05 });
		DualNumber_T<double, 2> e = exp(a);
		double expReal = std::exp(0.5);
		ASSERT_TRUE(std::abs(e.real - expReal) < 1e-12, "Exponential real part incorrect");
		ASSERT_TRUE(std::abs(e.dual[0] - (expReal * 0.1)) < 1e-12, "Exponential dual[0] part incorrect");
		ASSERT_TRUE(std::abs(e.dual[1] - (expReal * 0.05)) < 1e-12, "Exponential dual[1] part incorrect");
	}
}