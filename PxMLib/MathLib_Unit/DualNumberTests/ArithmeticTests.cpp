// MathLib_UnitTests ArithmeticTests.cpp

#include "TestHarness.h"
#include <core/MathLib.h>
#include <cmath>
#include <functional>

using namespace mathlib;

namespace {
	// Test that addition of dual numbers works correctly
	TEST("Basic Arithmetic", DualNumber_Addition) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> b(2.0, { 0.1, 0.05 });
		DualNumber_T<double, 2> c = a + b;
		ASSERT_TRUE(abs<double>(c.real - 3.0) < 1e-12, "Addition real part incorrect");
		ASSERT_TRUE(abs<double>(c.dual[0] - 0.6) < 1e-12, "Addition dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(c.dual[1] - 0.3) < 1e-12, "Addition dual[1] part incorrect");
	}

	// Test subtraction of dual numbers works correctly
	TEST("Basic Arithmetic", DualNumber_Subtraction) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> b(2.0, { 0.1, 0.05 });
		DualNumber_T<double, 2> c = a - b;
		ASSERT_TRUE(abs<double>(c.real + 1.0) < 1e-12, "Subtraction real part incorrect");
		ASSERT_TRUE(abs<double>(c.dual[0] - 0.4) < 1e-12, "Subtraction dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(c.dual[1] - 0.2) < 1e-12, "Subtraction dual[1] part incorrect");
	}
	// Test that addition and subtraction are inverses for dual numbers
	TEST("Basic Arithmetic", DualNumber_AddSubInverse) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> b(2.0, { 0.1, 0.05 });
		DualNumber_T<double, 2> c = (a + b) - b;
		ASSERT_TRUE(abs<double>(c.real - a.real) < 1e-12, "Add/Sub inverse real part incorrect");
		ASSERT_TRUE(abs<double>(c.dual[0] - a.dual[0]) < 1e-12, "Add/Sub inverse dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(c.dual[1] - a.dual[1]) < 1e-12, "Add/Sub inverse dual[1] part incorrect");
	}

	// Test that multiplication of dual numbers works correctly
	TEST("Multiplication", DualNumber_Multiplication_1Var) {
		DualNumber_T<double, 1> a(1.0, { 0.5 });
		DualNumber_T<double, 1> b(2.0, { 0.1 });
		DualNumber_T<double, 1> c = a * b;
		ASSERT_TRUE(abs<double>(c.real - 2.0) < 1e-12, "Multiplication real part incorrect");
		ASSERT_TRUE(abs<double>(c.dual[0] - (1.0 * 0.1 + 2.0 * 0.5)) < 1e-12, "Multiplication dual part incorrect");
	}
	// Test that multiplication of dual numbers with variable value of two works correctly
	TEST("Multiplication", DualNumber_Multiplication_2Var) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> b(2.0, { 0.1, 0.05 });
		DualNumber_T<double, 2> c = a * b;
		ASSERT_TRUE(abs<double>(c.real - 2.0) < 1e-12, "Multiplication real part incorrect");
		ASSERT_TRUE(abs<double>(c.dual[0] - (1.0 * 0.1 + 2.0 * 0.5)) < 1e-12, "Multiplication dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(c.dual[1] - (1.0 * 0.05 + 2.0 * 0.25)) < 1e-12, "Multiplication dual[1] part incorrect");
	}
	// Test that multiplication of dual numbers works correctly with float type
	TEST("Multiplication", DualNumber_Multiplication_f) {
		DualNumber_T<float, 1> a(1.0f, { 0.5f });
		DualNumber_T<float, 1> b(2.0f, { 0.1f });
		DualNumber_T<float, 1> c = a * b;
		ASSERT_TRUE(abs<float>(c.real - 2.0f) < 1e-5f, "Multiplication real part incorrect");
		ASSERT_TRUE(abs<float>(c.dual[0] - (1.0f * 0.1f + 2.0f * 0.5f)) < 1e-5f, "Multiplication dual part incorrect");
	}
	// Test that multiplication distributes over addition for dual numbers
	TEST("Multiplication", DualNumber_Distributivity) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> b(2.0, { 0.1, 0.05 });
		DualNumber_T<double, 2> c(3.0, { 0.2, 0.1 });
		DualNumber_T<double, 2> left = a * (b + c);
		DualNumber_T<double, 2> right = a * b + a * c;
		ASSERT_TRUE(abs<double>(left.real - right.real) < 1e-12, "Distributivity real part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[0] - right.dual[0]) < 1e-12, "Distributivity dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[1] - right.dual[1]) < 1e-12, "Distributivity dual[1] part incorrect");
	}
	// Test that multiplication is commutative for dual numbers
	TEST("Multiplication", DualNumber_Commutativity) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> b(2.0, { 0.1, 0.05 });
		DualNumber_T<double, 2> ab = a * b;
		DualNumber_T<double, 2> ba = b * a;
		ASSERT_TRUE(abs<double>(ab.real - ba.real) < 1e-12, "Commutativity real part incorrect");
		ASSERT_TRUE(abs<double>(ab.dual[0] - ba.dual[0]) < 1e-12, "Commutativity dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(ab.dual[1] - ba.dual[1]) < 1e-12, "Commutativity dual[1] part incorrect");
	}
	// Test that multiplication is associative for dual numbers
	TEST("Multiplication", DualNumber_Associativity) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> b(2.0, { 0.1, 0.05 });
		DualNumber_T<double, 2> c(3.0, { 0.2, 0.1 });
		DualNumber_T<double, 2> ab_c = (a * b) * c;
		DualNumber_T<double, 2> a_bc = a * (b * c);
		ASSERT_TRUE(abs<double>(ab_c.real - a_bc.real) < 1e-12, "Associativity real part incorrect");
		ASSERT_TRUE(abs<double>(ab_c.dual[0] - a_bc.dual[0]) < 1e-12, "Associativity dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(ab_c.dual[1] - a_bc.dual[1]) < 1e-12, "Associativity dual[1] part incorrect");
	}
	// Test that multiplication by the identity element works correctly for dual numbers
	TEST("Multiplication", DualNumber_Identity) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> identity(1.0, { 0.0, 0.0 });
		DualNumber_T<double, 2> left = a * identity;
		DualNumber_T<double, 2> right = identity * a;
		ASSERT_TRUE(abs<double>(left.real - a.real) < 1e-12, "Identity left real part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[0] - a.dual[0]) < 1e-12, "Identity left dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[1] - a.dual[1]) < 1e-12, "Identity left dual[1] part incorrect");
		ASSERT_TRUE(abs<double>(right.real - a.real) < 1e-12, "Identity right real part incorrect");
		ASSERT_TRUE(abs<double>(right.dual[0] - a.dual[0]) < 1e-12, "Identity right dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(right.dual[1] - a.dual[1]) < 1e-12, "Identity right dual[1] part incorrect");
	}
	// Test that multiplication by zero works correctly for dual numbers
	TEST("Multiplication", DualNumber_ZeroMultiplication) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		DualNumber_T<double, 2> zero(0.0, { 0.0, 0.0 });
		DualNumber_T<double, 2> left = a * zero;
		DualNumber_T<double, 2> right = zero * a;
		ASSERT_TRUE(abs<double>(left.real) < 1e-12, "Zero multiplication left real part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[0]) < 1e-12, "Zero multiplication left dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[1]) < 1e-12, "Zero multiplication left dual[1] part incorrect");
		ASSERT_TRUE(abs<double>(right.real) < 1e-12, "Zero multiplication right real part incorrect");
		ASSERT_TRUE(abs<double>(right.dual[0]) < 1e-12, "Zero multiplication right dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(right.dual[1]) < 1e-12, "Zero multiplication right dual[1] part incorrect");
	}
	// Test that multiplication by a scalar works correctly for dual numbers
	TEST("Multiplication", DualNumber_ScalarMultiplication) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		double scalar = 2.0;
		DualNumber_T<double, 2> left = a * scalar;
		DualNumber_T<double, 2> right = scalar * a;
		ASSERT_TRUE(abs<double>(left.real - 2.0) < 1e-12, "Scalar multiplication left real part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[0] - 1.0) < 1e-12, "Scalar multiplication left dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[1] - 0.5) < 1e-12, "Scalar multiplication left dual[1] part incorrect");
		ASSERT_TRUE(abs<double>(right.real - 2.0) < 1e-12, "Scalar multiplication right real part incorrect");
		ASSERT_TRUE(abs<double>(right.dual[0] - 1.0) < 1e-12, "Scalar multiplication right dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(right.dual[1] - 0.5) < 1e-12, "Scalar multiplication right dual[1] part incorrect");
	}
	// Test that multiplication by a negative scalar works correctly for dual numbers
	TEST("Multiplication", DualNumber_NegativeScalarMultiplication) {
		DualNumber_T<double, 2> a(1.0, { 0.5, 0.25 });
		double scalar = -2.0;
		DualNumber_T<double, 2> left = a * scalar;
		DualNumber_T<double, 2> right = scalar * a;
		ASSERT_TRUE(abs<double>(left.real + 2.0) < 1e-12, "Negative scalar multiplication left real part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[0] + 1.0) < 1e-12, "Negative scalar multiplication left dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(left.dual[1] + 0.5) < 1e-12, "Negative scalar multiplication left dual[1] part incorrect");
		ASSERT_TRUE(abs<double>(right.real + 2.0) < 1e-12, "Negative scalar multiplication right real part incorrect");
		ASSERT_TRUE(abs<double>(right.dual[0] + 1.0) < 1e-12, "Negative scalar multiplication right dual[0] part incorrect");
		ASSERT_TRUE(abs<double>(right.dual[1] + 0.5) < 1e-12, "Negative scalar multiplication right dual[1] part incorrect");
	}
}