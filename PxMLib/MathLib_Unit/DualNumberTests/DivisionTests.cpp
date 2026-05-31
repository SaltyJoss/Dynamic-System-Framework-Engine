// MathLib_UnitTests DivisionTests.cpp

#include "TestHarness.h"
#include <core/MathLib.h>
#include <cmath>
#include <functional>

using namespace mathlib;

namespace {
	using mathlib::abs;

	// Test that division of dual numbers works with variable value of two
	TEST("Division", DualNumber_Division_1Var) {
		DualNumber_T<double, 1> a(1.0, { 0.5 });
		DualNumber_T<double, 1> b(2.0, { 0.1 });
		DualNumber_T<double, 1> c = a / b;
		double expected0 = (0.5 * 2.0 - 1.0 * 0.1) / (2.0 * 2.0);
		ASSERT_TRUE(abs<double>(c.real - 0.5) < 1e-12, "Division real part incorrect");
		ASSERT_TRUE(abs<double>(c.dual[0] - expected0) < 1e-12, "Division dual[0] part incorrect");
	}
	// Test that division of dual numbers works with variable value of two
	TEST("Division", DualNumber_Division_2Var) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> b(2.0, { 0.1, 0.05 });
		DualNumber_T<double, 2> c = a / b;
		double expected0 = (0.5 * 2.0 - 1.0 * 0.1) / (2.0 * 2.0);
		double expected1 = (0.25 * 2.0 - 1.0f * 0.05) / (2.0 * 2.0);
		ASSERT_TRUE(abs<double>(c.real - 0.5) < 1e-12, "Division real part incorrect");
		ASSERT_TRUE(abs<double>(c.dual[0] - expected0) < 1e-12, "Division dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(c.dual[1] - expected1) < 1e-12, "Division dual[1] part incorrect");
	}
	// Test that division of a dual number defined with a float and double type works correctly for dual numbers
	TEST("Division", DualNumber_Division_f) {
		DualNumber_T<float, 1> a(1.0f, { 0.5f });
		DualNumber_T<float, 1> b(2.0f, { 0.1f });
		DualNumber_T<float, 1> c = a / b;
		float expected0 = (0.5f * 2.0f - 1.0f * 0.1f) / (2.0f * 2.0f);
		ASSERT_TRUE(abs<float>(c.real - 0.5f) < 1e-5f, "Division real part incorrect");
		ASSERT_TRUE(abs<float>(c.dual[0] - expected0) < 1e-5f, "Division dual[0] part incorrect");
	}
	// Test that division by the identity element works correctly for dual numbers
	TEST("Division", DualNumber_DivisionIdentity) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> identity(1.0, { 0.0, 0.0 });
		DualNumber_T<double, 2> left = a / identity;
		ASSERT_TRUE(abs<double>(left.real - a.real) < 1e-12, "Division by identity real part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[0] - a.dual[0]) < 1e-12, "Division by identity dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[1] - a.dual[1]) < 1e-12, "Division by identity dual[1] part incorrect");
	}
	// Test that division by zero throws an exception for dual numbers
	TEST("Division", DualNumber_DivisionByScalar) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		double scalar = 2.0;
		DualNumber_T<double, 2> left = a / scalar;
		ASSERT_TRUE(abs<double>(left.real - 0.5) < 1e-12, "Division by scalar real part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[0] - 0.25) < 1e-12, "Division by scalar dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[1] - 0.125) < 1e-12, "Division by scalar dual[1] part incorrect");
	}
	// Test that division by a negative scalar works correctly for dual numbers
	TEST("Division", DualNumber_DivisionByNegativeScalar) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		double scalar = -2.0;
		DualNumber_T<double, 2> left = a / scalar;
		ASSERT_TRUE(abs<double>(left.real + 0.5) < 1e-12, "Division by negative scalar real part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[0] + 0.25) < 1e-12, "Division by negative scalar dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[1] + 0.125) < 1e-12, "Division by negative scalar dual[1] part incorrect");
	}
	// Test that division by zero throws an exception for dual numbers
	TEST("Division", DualNumber_DivisionByZero) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		double scalar = 0.0;
		try {
			DualNumber_T<double, 2> left = a / scalar;
			ASSERT_TRUE(false, "Division by zero did not throw an exception");
		}
		catch (const std::exception&) {
			// Expected to catch an exception
		}
	}
	// Test that division by a dual number with zero real part throws an exception for dual numbers
	TEST("Division", DualNumber_DivisionByZeroReal) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> zeroReal(0.0, { 0.1, 0.05 });
		try {
			DualNumber_T<double, 2> left = a / zeroReal;
			ASSERT_TRUE(false, "Division by dual number with zero real part did not throw an exception");
		}
		catch (const std::exception&) {
			// Expected to catch an exception
		}
	}
}