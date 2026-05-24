#include "TestHarness.h"
#include <core/MathLib.h>

using namespace mathlib;

namespace {
	using Dual = DualNumber_T<double, 1>;
	constexpr double EPS = 1e-9;
}

TEST("DualNumber_T Basic Function", DefaultConstruction) {
	Dual d;
	ASSERT_TRUE(d.real == 0.0, "Default real part should be 0");
	ASSERT_TRUE(d.dual[0] == 0.0, "Default dual part should be 0");
}

TEST("DualNumber_T Basic Function", ArithmeticOperations_Additions) {
	Dual a(1.0, { 0.5 });
	Dual b(2.0, { 1.5 });
	// Test addition
	Dual c = a + b;
	ASSERT_TRUE(c.real == 3.0, "Addition real part incorrect");
	ASSERT_TRUE(c.dual[0] == 2.0, "Addition dual part incorrect");
}

TEST("DualNumber_T Basic Function", ArithmeticOperations_Subtraction) {
	Dual a(1.0, { 0.5 });
	Dual b(2.0, { 1.5 });
	// Test subtraction
	Dual c = a - b;
	ASSERT_TRUE(c.real == -1.0, "Subtraction real part incorrect");
	ASSERT_TRUE(c.dual[0] == -1.0, "Subtraction dual part incorrect");
}

TEST("DualNumber_T Basic Function", ArithmeticOperations_Multiplication) {
	Dual a(1.0, { 0.5 });
	Dual b(2.0, { 1.5 });
	// Test multiplication
	Dual c = a * b;
	ASSERT_TRUE(c.real == 2.0, "Multiplication real part incorrect");
	ASSERT_TRUE(c.dual[0] == 2.5, "Multiplication dual part incorrect");
}

TEST("DualNumber_T Basic Function", ArithmeticOperations_Division) {
	Dual a(1.0, { 0.5 });
	Dual b(2.0, { 1.5 });
	// Test division
	Dual c = a / b;
	ASSERT_TRUE(std::abs(c.real - 0.5) < EPS, "Division real part incorrect");
	ASSERT_TRUE(std::abs(c.dual[0] - 0.375) < EPS, "Division dual part incorrect");
}

TEST("DualNumber_T Basic Function", ScalarPromotion_Multiplcation) {
	Dual a(1.0, { 0.5 });
	double scalar = 2.0;
	// Test scalar multiplication
	Dual c = a * scalar;
	ASSERT_TRUE(c.real == 2.0, "Scalar multiplication real part incorrect");
	ASSERT_TRUE(c.dual[0] == 1.0, "Scalar multiplication dual part incorrect");
}

TEST("DualNumber_T Basic Function", ScalarPromotion_Division) {
	Dual a(1.0, { 0.5 });
	double scalar = 2.0;
	// Test scalar division
	Dual c = a / scalar;
	ASSERT_TRUE(c.real == 0.5, "Scalar division real part incorrect");
	ASSERT_TRUE(c.dual[0] == 0.25, "Scalar division dual part incorrect");
}

TEST("DualNumber_T Basic Function", ScalarPromotion_Division_ScalarByDual) {
	Dual a(2.0, { 1.5 });
	double scalar = 1.0;
	// Test scalar divided by dual
	Dual c = scalar / a;
	ASSERT_TRUE(c.real == 0.5, "Scalar by dual division real part incorrect");
	ASSERT_TRUE(c.dual[0] == -0.375, "Scalar by dual division dual part incorrect");
}

TEST("DualNumber_T Basic Function", ComparisonOperators_GreaterThan) {
	Dual a(1.0, { 0.5 });
	Dual b(2.0, { 1.5 });
	// Test greater than
	ASSERT_TRUE(b.real > a.real, "Greater than comparison failed");
}
TEST("DualNumber_T Basic Function", ComparisonOperators_LessThan) {
	Dual a(1.0, { 0.5 });
	Dual b(2.0, { 1.5 });
	// Test less than
	ASSERT_TRUE(a.real < b.real, "Less than comparison failed");
}
TEST("DualNumber_T Basic Function", ComparisonOperators_Equality) {
	Dual a(1.0, { 0.5 });
	Dual b(1.0, { 0.5 });
	// Test equality
	ASSERT_TRUE(a.real == b.real && a.dual[0] == b.dual[0], "Equality comparison failed");
}
TEST("DualNumber_T Basic Function", ComparisonOperators_Inequality) {
	Dual a(1.0, { 0.5 });
	Dual b(1.0, { 0.6 });
	// Test inequality
	ASSERT_TRUE(a.real == b.real && a.dual[0] != b.dual[0], "Inequality comparison failed");
}
TEST("DualNumber_T Basic Function", ComparisonOperators_GreaterThanOrEqual) {
	Dual a(1.0, { 0.5 });
	Dual b(1.0, { 0.5 });
	// Test greater than or equal
	ASSERT_TRUE(a.real >= b.real && a.dual[0] >= b.dual[0], "Greater than or equal comparison failed");
}
TEST("DualNumber_T Basic Function", ComparisonOperators_LessThanOrEqual) {
	Dual a(1.0, { 0.5 });
	Dual b(1.0, { 0.5 });
	// Test less than or equal
	ASSERT_TRUE(a.real <= b.real && a.dual[0] <= b.dual[0], "Less than or equal comparison failed");
}