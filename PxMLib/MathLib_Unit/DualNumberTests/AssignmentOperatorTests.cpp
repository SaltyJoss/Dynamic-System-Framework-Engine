// MathLib_UnitTests AssignmentOperatorTests.cpp

#include "TestHarness.h"
#include <core/DualNumbers.h>
#include <cmath>
#include <functional>

using namespace mathlib;

namespace {
	// Test that addition assignment works correctly for dual numbers
	TEST("Assignment Operators", DualNumber_AdditionAssignment) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> b(2.0, { 0.1, 0.05 });
		a += b;
		ASSERT_TRUE(std::abs(a.real - 3.0) < 1e-12, "Addition assignment real part incorrect");
		ASSERT_TRUE(std::abs(a.dual[0] - 0.6) < 1e-12, "Addition assignment dual[0] part incorrect");
		ASSERT_TRUE(std::abs(a.dual[1] - 0.3) < 1e-12, "Addition assignment dual[1] part incorrect");
	}
	// Test that subtraction assignment works correctly for dual numbers
	TEST("Assignment Operators", DualNumber_SubtractionAssignment) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> b(2.0, { 0.1, 0.05 });
		a -= b;
		ASSERT_TRUE(std::abs(a.real + 1.0) < 1e-12, "Subtraction assignment real part incorrect");
		ASSERT_TRUE(std::abs(a.dual[0] - 0.4) < 1e-12, "Subtraction assignment dual[0] part incorrect");
		ASSERT_TRUE(std::abs(a.dual[1] - 0.2) < 1e-12, "Subtraction assignment dual[1] part incorrect");
	}
	// Test that addition assignment and subtraction assignment are inverses for dual numbers
	TEST("Assignment Operators", DualNumber_AddSubAssignmentInverse) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> b(2.0, { 0.1, 0.05 });
		a += b;
		a -= b;
		ASSERT_TRUE(std::abs(a.real - 1.0) < 1e-12, "Add/Sub assignment inverse real part incorrect");
		ASSERT_TRUE(std::abs(a.dual[0] - 0.5) < 1e-12, "Add/Sub assignment inverse dual[0] part incorrect");
		ASSERT_TRUE(std::abs(a.dual[1] - 0.25) < 1e-12, "Add/Sub assignment inverse dual[1] part incorrect");
	}
	// Test that multiplication assignment works correctly for dual numbers
	TEST("Assignment Operators", DualNumber_MultiplicationAssignment) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> b(2.0, { 0.1, 0.05 });
		a *= b;
		ASSERT_TRUE(std::abs(a.real - 2.0) < 1e-12, "Multiplication assignment real part incorrect");
		ASSERT_TRUE(std::abs(a.dual[0] - (1.0 * 0.1 + 2.0 * 0.5)) < 1e-12, "Multiplication assignment dual[0] part incorrect");
		ASSERT_TRUE(std::abs(a.dual[1] - (1.0 * 0.05 + 2.0 * 0.25)) < 1e-12, "Multiplication assignment dual[1] part incorrect");
	}
	// Test that division assignment works correctly for dual numbers
	TEST("Assignment Operators", DualNumber_DivisionAssignment) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> b(2.0, { 0.1, 0.05 });
		a /= b;
		double expected0 = (0.5f * 2.0f - 1.0f * 0.1f) / (2.0f * 2.0f);
		double expected1 = (0.25f * 2.0f - 1.0f * 0.05f) / (2.0f * 2.0f);
		ASSERT_TRUE(std::abs(a.real - 0.5) < 1e-12, "Division real part incorrect");
		ASSERT_TRUE(std::abs(a.dual[0] - expected0) < 1e-12, "Division dual[0] part incorrect");
		ASSERT_TRUE(std::abs(a.dual[1] - expected1) < 1e-12, "Division dual[1] part incorrect");
	}
	// Test that division assignment by zero throws an exception for dual numbers
	TEST("Assignment Operators", DualNumber_DivisionAssignmentByZero) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		double scalar = 0.0;
		try {
			a /= scalar;
			ASSERT_TRUE(false, "Division assignment by zero did not throw an exception");
		}
		catch (const std::exception&) {
			// Expected to catch an exception
		}
	}
	// Test that division assignment by a dual number with zero real part throws an exception for dual numbers
	TEST("Assignment Operators", DualNumber_DivisionAssignmentByZeroReal) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> zeroReal(0.0, { 0.1, 0.05 });
		try {
			a /= zeroReal;
			ASSERT_TRUE(false, "Division assignment by dual number with zero real part did not throw an exception");
		}
		catch (const std::exception&) {
			// Expected to catch an exception
		}
	}
}