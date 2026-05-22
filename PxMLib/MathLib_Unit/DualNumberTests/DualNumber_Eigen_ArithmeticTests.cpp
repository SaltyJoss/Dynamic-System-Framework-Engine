#include "TestHarness.h"
#include <core/DualNumbers.h>

using namespace mathlib;

namespace {
	using Dual = DualNumber_T<double, 1>;
	constexpr double EPS = 1e-9;
}

TEST("DualNumber_T Eigen Arithmetic", MatrixConstruction) {
	Eigen::Matrix<Dual, 2, 2> M;
	M.setZero();
	ASSERT_TRUE(M(0, 0).real == 0.0 && M(0, 0).dual[0] == 0.0, "M(0, 0) should be zero");
	ASSERT_TRUE(M(1, 1).real == 0.0 && M(1, 1).dual[0] == 0.0, "M(1, 1) should be zero");
}

TEST("DualNumber_T Eigen Arithmetic", MatrixAddition) {
	Eigen::Matrix<Dual, 2, 2> A, B;
	A(0, 0) = Dual(1.0, { 0.5 });
	A(1, 1) = Dual(2.0, { 1.5 });
	B(0, 0) = Dual(3.0, { 2.5 });
	B(1, 1) = Dual(4.0, { 3.5 });
	Eigen::Matrix<Dual, 2, 2> C = A + B;
	ASSERT_TRUE(C(0, 0).real == 4.0 && C(0, 0).dual[0] == 3.0, "C(0, 0) should be the sum of A and B");
	ASSERT_TRUE(C(1, 1).real == 6.0 && C(1, 1).dual[0] == 5.0, "C(1, 1) should be the sum of A and B");
}

TEST("DualNumber_T Eigen Arithmetic", MatrixSubtraction) {
	Eigen::Matrix<Dual, 2, 2> A, B;
	A(0, 0) = Dual(1.0, { 0.5 });
	A(1, 1) = Dual(2.0, { 1.5 });
	B(0, 0) = Dual(3.0, { 2.5 });
	B(1, 1) = Dual(4.0, { 3.5 });
	Eigen::Matrix<Dual, 2, 2> C = A - B;
	ASSERT_TRUE(C(0, 0).real == -2.0 && C(0, 0).dual[0] == -2.0,
		"C(0, 0) should be the difference of A and B");
	ASSERT_TRUE(C(1, 1).real == -2.0 && C(1, 1).dual[0] == -2.0,
		"C(1, 1) should be the difference of A and B");
}

TEST("DualNumber_T Eigen Arithmetic", MatrixMultiplication) {
	Eigen::Matrix<Dual, 2, 2> A, B;
	A(0, 0) = Dual(1.0, { 0.5 });
	A(1, 1) = Dual(2.0, { 1.5 });
	B(0, 0) = Dual(3.0, { 2.5 });
	B(1, 1) = Dual(4.0, { 3.5 });
	// (a + a'ε)(b + b'ε) = ab + (ab' + a'b)ε
	Eigen::Matrix<Dual, 2, 2> C = A * B;
	ASSERT_TRUE(C(0, 0).real == 3.0 && C(0, 0).dual[0] == 4.0,
		"C(0, 0) should be the product of A and B");
	ASSERT_TRUE(C(1, 1).real == 8.0 && C(1, 1).dual[0] == 13.0,
		"C(1, 1) should be the product of A and B");
}

TEST("DualNumber_T Eigen Arithmetic", DotProduct) {
	Eigen::Matrix<Dual, 2, 1> a, b;
	a << Dual(1.0, { 0.5 }), Dual(2.0, { 1.5 });
	b << Dual(3.0, { 2.5 }), Dual(4.0, { 3.5 });
	Dual dotProduct = a.dot(b);
	ASSERT_TRUE(dotProduct.real == 11.0 && dotProduct.dual[0] == 17.0,
		"The dot product should be the sum of the products of corresponding elements");
}