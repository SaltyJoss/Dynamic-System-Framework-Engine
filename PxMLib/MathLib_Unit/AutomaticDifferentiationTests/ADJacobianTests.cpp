// MathLib_UnitTests ADJacobianTests.cpp

#include "TestHarness.h"
#include <core/DualNumbers.h>
#include <cmath>
#include <functional>

using namespace mathlib;

namespace {
	// Simple vector-valued function: f(x) = [x^2 + y, sin(x) + y^2]
	template<typename Scalar, size_t NVar>
	VecX_T<DualNumber_T<Scalar, NVar>> vectorFunction(
		const VecX_T<DualNumber_T<Scalar, NVar>>& x
	) {
		VecX_T<DualNumber_T<Scalar, NVar>> f(2);
		f(0) = x(0) * x(0) + x(1);		// f1 = x^2 + y
		f(1) = sin(x(0)) + x(1) * x(1); // f2 = sin(x) + y^2
		return f;
	}

	// Dense coupled nonlinear system: F(x) = [x^2 + xy - y, sin(xy) + y^3]
	template<typename Scalar, size_t NVar>
	VecX_T<DualNumber_T<Scalar, NVar>> coupledNonlinearSystem(
		const VecX_T<DualNumber_T<Scalar, NVar>>& x
	) {
		VecX_T<DualNumber_T<Scalar, NVar>> F(2);
		F(0) = x(0) * x(0) + x(0) * x(1) - x(1);  // F1 = x^2 + xy - y
		F(1) = sin(x(0) * x(1)) + pow(x(1), Scalar(3)); // F2 = sin(xy) + y^3
		return F;
	}

	// Analytical Jacobian of a simple vector-valued function f(x) = [x^2 + y, sin(x) + y^2]
	template<typename Scalar, size_t NVar>
	void analyticalJacobian(
		const VecX_T<DualNumber_T<Scalar, NVar>>& x,
		MatX_T<Scalar>& J_out
	) {
		double x0 = x(0).real;
		double y0 = x(1).real;
		J_out(0, 0) = Scalar(2) * x0;		// df1/dx
		J_out(0, 1) = Scalar(1);			// df1/dy
		J_out(1, 0) = std::cos(x0); // df2/dx
		J_out(1, 1) = Scalar(2) * y0;     // df2/dy
	}

	// Finite difference approximation of the Jacobian for the same vector-valued function, to compare against the AD-computed Jacobian.
	template<typename Scalar, size_t NVar>
	void finiteDifferenceJacobian(
		const VecX_T<DualNumber_T<Scalar, NVar>>& x,
		MatX_T<Scalar>& J_out
	) {
		Scalar h = (Scalar)1e-6;
		VecX_T<DualNumber_T<Scalar, NVar>> f0 = vectorFunction<Scalar, NVar>(x);
		for (int i = 0; i < x.size(); ++i) {
			auto x_perturbed = x;
			x_perturbed(i).real += h;
			auto f_perturbed = vectorFunction<Scalar, NVar>(x_perturbed);
			for (int j = 0; j < x.size(); ++j) {
				J_out(j, i) = (f_perturbed(j).real - f0(j).real) / h;
			}
		}
	}

	// Helper function to construct the Jacobian matrix using dual numbers for a simple vector-valued function, to be used in the AD Jacobian tests.
	template<typename Scalar, size_t NVar>
	void constructADJacobian(
		const VecX_T<DualNumber_T<Scalar, NVar>>& x,
		MatX_T<Scalar>& J_ad
	) {
		VecX_T<DualNumber_T<Scalar, NVar>> f = vectorFunction<Scalar, NVar>(x);
		for (int i = 0; i < f.size(); ++i) {
			for (int j = 0; j < NVar; ++j) {
				J_ad(i, j) = f(i).dual[j];
			}
		}
	}

	// Test that the Jacobian computed using dual numbers matches the analytical Jacobian for a simple vector-valued function.
	TEST("AD Jacobian", ADJacobian_anaylticalMatch) {
		VecX_T<DualNumber_T<double, 2>> x(2);
		x(0) = DualNumber_T<double, 2>(1.0, { 1.0, 0.0 }); // x = 1.0 with dual part for df/dx
		x(1) = DualNumber_T<double, 2>(2.0, { 0.0, 1.0 }); // y = 2.0 with dual part for df/dy
		MatX_T<double> J_ad(2, 2);
		MatX_T<double> J_analytical(2, 2);
		analyticalJacobian<double, 2>(x, J_analytical);
		constructADJacobian<double, 2>(x, J_ad);
		for (int i = 0; i < J_analytical.rows(); ++i) {
			for (int j = 0; j < J_analytical.cols(); ++j) {
				std::string errorMsg = "Jacobian mismatch at (" + std::to_string(i)
					+ ", " + std::to_string(j)
					+ "): " + std::to_string(J_analytical(i, j))
					+ " vs " + std::to_string(J_ad(i, j));
				ASSERT_TRUE(std::abs(J_analytical(i, j) - J_ad(i, j)) < 1e-6, errorMsg.c_str());
				ASSERT_TRUE(std::isfinite(J_ad(i, j)), "AD Jacobian contains non-finite value");
			}
		}
		ASSERT_TRUE(J_analytical.isApprox(J_ad, 1e-6), "Analytical and AD Jacobians do not match");
	}

	// Test that the Jacobian computed using dual numbers matches a finite difference approximation of the Jacobian for the same vector-valued function.
	TEST("AD Jacobian", ADJacobian_finiteDifferenceMatch) {
		VecX_T<DualNumber_T<double, 2>> x(2);
		x(0) = DualNumber_T<double, 2>(1.0, { 1.0, 0.0 }); // x = 1.0 with dual part for df/dx
		x(1) = DualNumber_T<double, 2>(2.0, { 0.0, 1.0 }); // y = 2.0 with dual part for df/dy
		MatX_T<double> J_ad(2, 2);
		MatX_T<double> J_fd(2, 2);
		finiteDifferenceJacobian<double, 2>(x, J_fd);
		constructADJacobian(x, J_ad);
		for (int i = 0; i < J_fd.rows(); ++i) {
			for (int j = 0; j < J_fd.cols(); ++j) {
				std::string errorMsg = "Jacobian mismatch at (" + std::to_string(i)
					+ ", " + std::to_string(j)
					+ "): " + std::to_string(J_fd(i, j))
					+ " vs " + std::to_string(J_ad(i, j));
				ASSERT_TRUE(std::abs(J_fd(i, j) - J_ad(i, j)) < 1e-5, errorMsg.c_str());
				ASSERT_TRUE(std::isfinite(J_ad(i, j)), "AD Jacobian contains non-finite value");
			}
		}
		ASSERT_TRUE(J_fd.isApprox(J_ad, 1e-6), "Finite difference and AD Jacobians do not match");
	}

	// Tests the cross-variable propagation of derivatives in a coupled nonlinear system, ensuring that the Jacobian captures the interactions between variables correctly.
	TEST("AD Jacobian", ADJacobian_coupledNonlinearSystem) {
		VecX_T<DualNumber_T<double,2>> x(2);
		x(0) = DualNumber_T<double, 2>(1.0, { 1.0, 0.0 }); // x = 1.0 with dual part for df/dx
		x(1) = DualNumber_T<double, 2>(2.0, { 0.0, 1.0 }); // y = 2.0 with dual part for df/dy
		VecX_T<DualNumber_T<double, 2>> F = coupledNonlinearSystem<double, 2>(x);
		MatX_T<double> J_ad(2, 2);
		for (int i = 0; i < F.size(); ++i) {
			for (int j = 0; j < 2; ++j) {
				J_ad(i, j) = F(i).dual[j];
			}
		}
		MatX_T<double> J_expected(2, 2);
		J_expected(0, 0) = 2.0 * x(0).real + x(1).real; // dF1/dx
		J_expected(0, 1) = x(0).real - 1.0;              // dF1/dy
		J_expected(1, 0) = std::cos(x(0).real * x(1).real) * x(1).real; // dF2/dx
		J_expected(1, 1) = std::cos(x(0).real * x(1).real) * x(0).real + 3.0 * pow(x(1).real, 2.0); // dF2/dy
		for (int i = 0; i < J_ad.rows(); ++i) {
			for (int j = 0; j < J_ad.cols(); ++j) {
				std::string errorMsg = "Coupled system Jacobian mismatch at (" + std::to_string(i)
					+ ", " + std::to_string(j)
					+ "): " + std::to_string(J_expected(i, j))
					+ " vs " + std::to_string(J_ad(i, j));
				ASSERT_TRUE(std::abs(J_expected(i, j) - J_ad(i, j)) < 1e-6, errorMsg.c_str());
				ASSERT_TRUE(std::isfinite(J_ad(i, j)), "AD Jacobian contains non-finite value");
			}
		}
		ASSERT_TRUE(J_expected.isApprox(J_ad, 1e-6), "Expected and AD Jacobians do not match for coupled nonlinear system");
	}

	// Tests the conditioning of the Jacobian matrix computed via automatic differentiation, ensuring that it does not contain excessively large or small values that could indicate numerical instability.
	TEST("AD Jacobian", ADJacobian_numericalStability) {
		VecX_T<DualNumber_T<double, 2>> x(2);
		x(0) = DualNumber_T<double, 2>(1.0, { 1.0, 0.0 }); // x = 1.0 with dual part for df/dx
		x(1) = DualNumber_T<double, 2>(2.0, { 0.0, 1.0 }); // y = 2.0 with dual part for df/dy
		VecX_T<DualNumber_T<double, 2>> f = vectorFunction<double, 2>(x);
		MatX_T<double> J_ad(2, 2);
		J_ad(0, 0) = f(0).dual[0]; // df1/dx
		J_ad(0, 1) = f(0).dual[1]; // df1/dy
		J_ad(1, 0) = f(1).dual[0]; // df2/dx
		J_ad(1, 1) = f(1).dual[1]; // df2/dy
		for (int i = 0; i < J_ad.rows(); ++i) {
			for (int j = 0; j < J_ad.cols(); ++j) {
				std::string errorMsg = "AD Jacobian contains non-finite value at (" + std::to_string(i)
					+ ", " + std::to_string(j)
					+ "): " + std::to_string(J_ad(i, j));
				ASSERT_TRUE(std::isfinite(J_ad(i, j)), errorMsg.c_str());
				ASSERT_TRUE(std::abs(J_ad(i, j)) < 1e6, "AD Jacobian contains excessively large value");
			}
		}
		ASSERT_TRUE(J_ad.norm() < 1e6, "AD Jacobian norm is excessively large, indicating potential numerical instability");
	}

	// Tests the consistency of the Jacobian computed via automatic differentiation across multiple evaluations, ensuring that repeated computations yield the same results.
	TEST("AD Jacobian", ADJacobian_consistency) {
		VecX_T<DualNumber_T<double, 2>> x(2);
		x(0) = DualNumber_T<double, 2>(1.0, { 1.0, 0.0 }); // x = 1.0 with dual part for df/dx
		x(1) = DualNumber_T<double, 2>(2.0, { 0.0, 1.0 }); // y = 2.0 with dual part for df/dy
		MatX_T<double> J_first(2, 2);
		MatX_T<double> J_second(2, 2);
		VecX_T<DualNumber_T<double, 2>> f_first = vectorFunction<double, 2>(x);
		J_first(0, 0) = f_first(0).dual[0]; // df1/dx
		J_first(0, 1) = f_first(0).dual[1]; // df1/dy
		J_first(1, 0) = f_first(1).dual[0]; // df2/dx
		J_first(1, 1) = f_first(1).dual[1]; // df2/dy
		VecX_T<DualNumber_T<double, 2>> f_second = vectorFunction<double, 2>(x);
		J_second(0, 0) = f_second(0).dual[0]; // df1/dx
		J_second(0, 1) = f_second(0).dual[1]; // df1/dy
		J_second(1, 0) = f_second(1).dual[0]; // df2/dx
		J_second(1, 1) = f_second(1).dual[1]; // df2/dy
		for (int i = 0; i < J_first.rows(); ++i) {
			for (int j = 0; j < J_first.cols(); ++j) {
				std::string errorMsg = "Inconsistent AD Jacobian at (" + std::to_string(i)
					+ ", " + std::to_string(j)
					+ "): " + std::to_string(J_first(i, j))
					+ " vs " + std::to_string(J_second(i, j));
				ASSERT_TRUE(std::abs(J_first(i, j) - J_second(i, j)) < 1e-6, errorMsg.c_str());
				ASSERT_TRUE(std::isfinite(J_first(i, j)) && std::isfinite(J_second(i, j)), "AD Jacobian contains non-finite value");
			}
		}
		ASSERT_TRUE(J_first.isApprox(J_second, 1e-6), "AD Jacobians from repeated evaluations do not match");
	}
}