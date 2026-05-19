#include "TestHarness.h"
#include <core/DualNumbers.h>

using namespace mathlib;

namespace {
	using Dual = DualNumber_T<double, 1>;
	constexpr double EPS = 1e-9;
}

TEST("DualNumber_T Eigen Solvers", PartialPivLU) {
	Eigen::Matrix<Dual, 2, 2> A;
	Eigen::Matrix<Dual, 2, 1> b;

	A(0, 0) = Dual(4.0, { 0.5 });
	A(0, 1) = Dual(2.0, { 0.5 });
	A(1, 0) = Dual(1.0, { 0.5 });
	A(1, 1) = Dual(3.0, { 0.5 });
	b << Dual(10.0, { 1.0 }), Dual(11.0, { 1.0 });

	auto solver = A.partialPivLu();

	auto x = solver.solve(b);
	
	ASSERT_TRUE(std::isfinite(x(0).real), "Real part of solution should be finite");
	ASSERT_TRUE(std::isfinite(x(0).dual[0]), "Dual part of solution should be finite");
}

TEST("DualNumber_T Eigen Solvers", MaxCoeff) {
	Eigen::Matrix<Dual, 2, 2> A;

	A(0, 0) = Dual(4.0);
	A(0, 1) = Dual(2.0);
	A(1, 0) = Dual(1.0);
	A(1, 1) = Dual(3.0);

	auto m = A.maxCoeff();

	ASSERT_TRUE(std::isfinite(m.real), "Max coefficient real part should be finite");
}

TEST("DualNumber_T Eigen Solvers", CwiseAbsMaxCoeff) {
	Eigen::Matrix<Dual, 2, 2> A;

	A(0, 0) = Dual(4.0);
	A(0, 1) = Dual(2.0);
	A(1, 0) = Dual(1.0);
	A(1, 1) = Dual(3.0);

	auto m = A.cwiseAbs().maxCoeff();

	ASSERT_TRUE(std::isfinite(m), "CwiseAbsMaxCoeff real part should be finite");
}