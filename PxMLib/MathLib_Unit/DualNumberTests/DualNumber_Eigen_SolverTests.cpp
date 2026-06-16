#include "TestHarness.h"
#include <core/MathLib.h>

using namespace mathlib;

namespace {
	using Dual = DualNumber_T<double, 1>;
	constexpr double EPS = 1e-9;
}

TEST("DualNumber_T Eigen Solvers", MaxCoeff) {
	Eigen::Matrix<Dual, 2, 2> A;

	A(0, 0) = Dual(4.0);
	A(0, 1) = Dual(2.0);
	A(1, 0) = Dual(1.0);
	A(1, 1) = Dual(3.0);

	auto m = A.maxCoeff();

	ASSERT_TRUE(isfinite(m.real), "Max coefficient real part should be finite");
}
TEST("DualNumber_T Eigen Solvers", CwiseAbsMaxCoeff) {
	Eigen::Matrix<Dual, 2, 2> A;

	A(0, 0) = Dual(4.0);
	A(0, 1) = Dual(2.0);
	A(1, 0) = Dual(1.0);
	A(1, 1) = Dual(3.0);

	auto m = A.cwiseAbs().maxCoeff();

	ASSERT_TRUE(isfinite(m), "CwiseAbsMaxCoeff real part should be finite");
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
	ASSERT_TRUE(isfinite(x(0).real), "Real part of solution should be finite");
	ASSERT_TRUE(isfinite(x(0).dual[0]), "Dual part of solution should be finite");
}

TEST("DualNumber_T Eigen Solvers", LDLTComputeOnly) {
	Eigen::Matrix<Dual, 2, 2> A;
	A <<
		Dual(4.0), Dual(1.0),
		Dual(1.0), Dual(3.0);

	Eigen::LDLT<Eigen::Matrix<Dual, 2, 2>> ldlt;
	ldlt.compute(A);
	SUCCEED();
}

TEST("DualNumber_T Eigen Solvers", LDLT) {
	Eigen::Matrix<Dual, 2, 2> A;
	Eigen::Matrix<Dual, 2, 1> b;
	A(0, 0) = Dual(4.0, { 0.5 });
	A(0, 1) = Dual(2.0, { 0.5 });
	A(1, 0) = Dual(1.0, { 0.5 });
	A(1, 1) = Dual(3.0, { 0.5 });
	b << Dual(10.0, { 1.0 }), Dual(11.0, { 1.0 });
	auto solver = A.ldlt();
	auto x = solver.solve(b);
	ASSERT_TRUE(isfinite<double>(x(0).real), "Real part of solution should be finite");
	ASSERT_TRUE(isfinite<double>(x(0).dual[0]), "Dual part of solution should be finite");
}

TEST("DualNumber_T Eigen Solvers", FullPivLUComputeAndSolve_DecompWithDualCasts) {
	using Dual = DualNumber_T<double, 2>;

	mathlib::MatX_T<double> J(2, 2);
	J <<
		1.0, 1000.0,
		0.0, 1.0;
	mathlib::VecX_T<double> b(2);
	b << -5.0, 2.0;

	Eigen::FullPivLU<mathlib::MatX_T<double>> solver;
	solver.compute(J);
	ASSERT_TRUE(solver.isInvertible(), "FullPivLU failed to invert the system matrix.");

	mathlib::VecX_T<double> delta = solver.solve(b);
	// x + 1000y = -5
	ASSERT_NEAR(delta(0), -2005.0, 1e-7); // x = -2005
	ASSERT_NEAR(delta(1), 2.0, 1e-7); // y = 2

	mathlib::VecX_T<Dual> x(2);
	x(0) = Dual(10.0, { 1.0, 0.0 });
	x(1) = Dual(5.0,  { 0.0, 1.0 });
	x += delta.template cast<Dual>();
	ASSERT_NEAR(x(0).real , -1995.0, 1e-7);
	ASSERT_NEAR(x(1).real, 7.0, 1e-7);
}
