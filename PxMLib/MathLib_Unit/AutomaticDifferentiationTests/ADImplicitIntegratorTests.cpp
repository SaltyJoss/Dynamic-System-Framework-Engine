// MathLib_UnitTests ADImplicitIntegratorTests.cpp

#include "TestHarness.h"
#include <core/DualNumbers.h>
#include <integrators/numerical_integrators.h>
#include <cmath>
#include <functional>
#include <core/constants.h>

using namespace mathlib;
using namespace constants;

namespace {
	constexpr double tol_low = 1e-3;
	constexpr double tol_high = 1e-6;

	integration::NumericalIntegrator integrator;

	// Exponential Decay function for dual numbers
	template<typename Scalar, size_t NVar>
	VecX_T<DualNumber_T<Scalar, NVar>> expDecay(
		DualNumber_T<Scalar, NVar> /*t*/,
		const VecX_T<DualNumber_T<Scalar, NVar>>& x
	) {
		VecX_T<DualNumber_T<Scalar, NVar>> dx = x;
		for (int i = 0; i < x.size(); ++i) { dx(i) = -x(i); }
		return dx;
	}

	// Harmonic Oscillator function for dual numbers
	template<typename Scalar, size_t NVar>
	VecX_T<DualNumber_T<Scalar, NVar>> harmonicOsc(
		DualNumber_T<Scalar, NVar> /*t*/,
		const VecX_T<DualNumber_T<Scalar, NVar>>& s
	) {
		VecX_T<DualNumber_T<Scalar, NVar>> d(2);
		d(0) = s(1);
		d(1) = -s(0);
		return d;
	}

	// Linear Decay function for dual numbers
	template<typename Scalar, size_t NVar>
	VecX_T<DualNumber_T<Scalar, NVar>> linearDecay(
		DualNumber_T<Scalar, NVar> /*t*/,
		const VecX_T<DualNumber_T<Scalar, NVar>>& x
	) {
		VecX_T<DualNumber_T<Scalar, NVar>> dx = x;
		for (int i = 0; i < x.size(); ++i) { dx(i) = -Scalar(2) * x(i); }
		return dx;
	}

	// Nonlinear Decay function for dual numbers
	template<typename Scalar, size_t NVar>
	VecX_T<DualNumber_T<Scalar, NVar>> nonlinearDecay(
		DualNumber_T<Scalar, NVar> /*t*/,
		const VecX_T<DualNumber_T<Scalar, NVar>>& x
	) {
		VecX_T<DualNumber_T<Scalar, NVar>> dx = x;
		for (int i = 0; i < x.size(); ++i) { dx(i) = -x(i) * x(i); }
		return dx;
	}

	// Stiff Decay function for dual numbers
	template<typename Scalar, size_t NVar>
	VecX_T<DualNumber_T<Scalar, NVar>> stiffDecay(
		DualNumber_T<Scalar, NVar> /*t*/,
		const VecX_T<DualNumber_T<Scalar, NVar>>& x
	) {
		VecX_T<DualNumber_T<Scalar, NVar>> dx = x;
		for (int i = 0; i < x.size(); ++i) { dx(i) = -Scalar(100) * x(i); }
		return dx;
	}

	// Helper function to integrate from t=0 to t=T using N steps of the given step function, for dual numbers
	template<typename Scalar, size_t NVar, typename StepFn>
	VecX_T<DualNumber_T<Scalar, NVar>> integrateToTime(
		StepFn stepFn,
		const VecX_T<DualNumber_T<Scalar, NVar>>& x0,
		Scalar T,
		int N
	) {
		DualNumber_T<Scalar, NVar> dt(T / static_cast<Scalar>(N));
		DualNumber_T<Scalar, NVar> t(0.0);
		VecX_T<DualNumber_T<Scalar, NVar>> x = x0;
		for (int i = 0; i < N; ++i) { x = stepFn(x, t, dt); t += dt; }
		return x;
	}

	// Helper function to verify that the exponential decay ODE is integrated correctly, and that the sensitivity of the solution with respect to the initial condition matches the analytical derivative.
	template<typename StepFn>
	void verifyExpDecaySensitivity(
		StepFn stepFn,
		double tol,
		double exactFinal,
		int N
	) {
		using Dual = DualNumber_T<double, 1>;
		VecX_T<Dual> x0(1);
		x0(0) = Dual(1.0, { 1.0 });
		VecX_T<Dual> r = integrateToTime(stepFn, x0, 1.0, N);
		double eps = 1e-6;
		VecX_T<Dual> x0_plus(1), x0_minus(1);
		x0_plus(0) = Dual(1.0 + eps, {0.0});
		x0_minus(0) = Dual(1.0 - eps, {0.0});
		VecX_T<Dual> r_plus = integrateToTime(stepFn, x0_plus, 1.0, N);
		VecX_T<Dual> r_minus = integrateToTime(stepFn, x0_minus, 1.0, N);
		double expectedSensitivity = (r_plus(0).real - r_minus(0).real) / (2.0 * eps);
		ASSERT_TRUE(std::abs(r(0).real - exactFinal) < tol, "State error too large");
		ASSERT_TRUE(std::abs(r(0).dual[0] - expectedSensitivity) < 5e-3, "AD decay sensitivity error too large");
		ASSERT_TRUE(std::abs(r(0).dual[0]) > 0.0, "Sensitivity vanished during propagation");
	}

	// Helper function to verify that a harmonic oscillator ODE is integrated correctly, and that the sensitivity of the solution with respect to the initial condition matches the analytical derivative.
	template<typename StepFn>
	void verifyHarmonicOscillatorSensitivity(
		StepFn stepFn,
		double tol,
		int N
	) {
		using Dual = DualNumber_T<double, 2>;
		VecX_T<Dual> x0(2);
		x0(0) = Dual(1.0, { 1.0, 0.0 }); // Initial position with sensitivity
		x0(1) = Dual(0.0, { 0.0, 1.0 }); // Initial velocity with sensitivity
		VecX_T<Dual> r = integrateToTime(stepFn, x0, TWO_PI_d, N);
		double eps = 1e-6;

		VecX_T<Dual> x0_u_plus(2), x0_u_minus(2);
		x0_u_plus(0) = Dual(1.0 + eps, {0.0, 0.0});
		x0_u_plus(1) = Dual(0.0, { 0.0, 0.0 });
		x0_u_minus(0) = Dual(1.0 - eps, {0.0, 0.0});
		x0_u_minus(1) = Dual(0.0, { 0.0, 0.0 });
		VecX_T<Dual> r_u_plus = integrateToTime(stepFn, x0_u_plus, TWO_PI_d, N);
		VecX_T<Dual> r_u_minus = integrateToTime(stepFn, x0_u_minus, TWO_PI_d, N);
		double expected_du_du0 = (r_u_plus(0).real - r_u_minus(0).real) / (2.0 * eps);

		VecX_T<Dual> x0_v_plus(2), x0_v_minus(2);
		x0_v_plus(0) = Dual(1.0, { 0.0, 0.0 });
		x0_v_plus(1) = Dual(0.0 + eps, { 0.0, 0.0 });
		x0_v_minus(0) = Dual(1.0, { 0.0, 0.0 });
		x0_v_minus(1) = Dual(0.0 - eps, { 0.0, 0.0 });
		VecX_T<Dual> r_v_plus = integrateToTime(stepFn, x0_v_plus, TWO_PI_d, N);
		VecX_T<Dual> r_v_minus = integrateToTime(stepFn, x0_v_minus, TWO_PI_d, N);
		double expected_dv_dv0 = (r_v_plus(1).real - r_v_minus(1).real) / (2.0 * eps);

		double expectedPosition = std::cos(TWO_PI_d);
		double expectedVelocity = -std::sin(TWO_PI_d);
		ASSERT_TRUE(std::abs(r(0).real - expectedPosition) < tol, "Harmonic oscillator position state error too large");
		ASSERT_TRUE(std::abs(r(1).real - expectedVelocity) < tol, "Harmonic oscillator velocity state error too large");
		ASSERT_TRUE(std::abs(r(0).dual[0] - expected_du_du0) < 5e-3, "Position sensitivity AD track broken");
		ASSERT_TRUE(std::abs(r(1).dual[1] - expected_dv_dv0) < 5e-3, "Velocity sensitivity AD track broken");
	}
}

// AD Implicit Euler Tests
// Test case for the GLRK2 method on the exponenital decay ODE.
TEST("AD Implicit Euler Method", ImplicitEuler_ExponentialDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 1.0;
	DualNumber_T<double, 1> dt(T / 1000.0, { 0.0 });
	DualNumber_T<double, 1> t(0.0, { 0.0 });
	for (int i = 0; i < 1000; ++i) {
		x = integrator.implicitEuler_AD(x, t, dt, expDecay<double, 1>, 10, 1e-5);
		t += dt;
	}
	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "Implicit Euler exponential decay error too large");
}
// Test case for the implicit Euler method on the exponential decay ODE.
TEST("AD Implicit Euler Method", ImplicitEuler_ExponentialDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
		) {
		return integrator.implicitEuler_AD(x, t, dt, expDecay<double, 1>, 50, 1e-5);
	};
	verifyExpDecaySensitivity(stepFn, 1e-2, std::exp(-1.0), 100);
}
// Test case for the Implicit Midpoint method on the linear decay ODE.
TEST("AD Implicit Euler Method", ImplicitEuler_LinearDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 0.5;
	DualNumber_T<double, 1> dt(T / 500.0);
	DualNumber_T<double, 1> t(0.0);
	for (int i = 0; i < 500; ++i) {
		x = integrator.implicitEuler_AD(x, t, dt, linearDecay<double, 1>, 25, 1e-5);
		t += dt;
	}
	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "Implicit Euler linear decay error too large");
}
// Test case for the implicit Euler method on the linear decay ODE.
TEST("AD Implicit Euler Method", ImplicitEuler_LinearDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
		) {
		return integrator.implicitEuler_AD(x, t, dt, linearDecay<double, 1>, 50, 1e-5);
	};
	verifyExpDecaySensitivity(stepFn, 1e-2, std::exp(-2.0), 1000);
}
// Test case for the Implicit Euler method on the stiff decay ODE.
TEST("AD Implicit Euler Method", ImplicitEuler_StiffDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 0.5;
	DualNumber_T<double, 1> dt(T / 500.0);
	DualNumber_T<double, 1> t(0.0);
	double expected = std::exp(-50.0);
	auto stiff_f = [](auto t, const auto& x) {
		auto dx = x;
		dx(0) = -100.0 * x(0);
		return dx;
	};
	for (int i = 0; i < 500; ++i) {
		x = integrator.implicitEuler_AD(x, t, dt, stiff_f, 25, 1e-5);
		t += dt;
	}
	ASSERT_TRUE(std::isfinite(x(0).real), "Implicit Euler produced a non-finite result.");
	ASSERT_TRUE(x(0).real < 1e-12, "Implicit Euler did not sufficiently damp stiff mode");
}
// Test case for the implicit Euler method on the stiff decay ODE.
TEST("AD Implicit Euler Method", ImplicitEuler_StiffDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
		) {
		return integrator.implicitEuler_AD(x, t, dt, stiffDecay<double, 1>, 50, 1e-5);
	};
	verifyExpDecaySensitivity(stepFn, 1e-2, std::exp(-100.0), 1000);
}
// Test large step performance of implicit midpoint
TEST("AD Implicit Euler Method", ImplicitEuler_Stability_LargeStep) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 1.0 });
	double T = 1.0;
	DualNumber_T<double, 1> t(0.0);
	DualNumber_T<double, 1> dt(1.0 / 150.0);
	DualNumber_T<double, 1> prev = x(0);
	for (int i = 0; i < 10; ++i) {
		x = integrator.implicitEuler_AD(x, t, dt, stiffDecay<double, 1>, 50, 1e-6);
		t += dt;
		ASSERT_TRUE(std::abs(x(0).real) < std::abs(prev.real), "Not monotone");
		ASSERT_TRUE(std::abs(x(0).dual[0]) < std::abs(prev.dual[0]), "Not strictly decaying");
		prev = x(0);
	}
}

// AD Implicit Midpoint Tests
// Test case for the Implicit Midpoint method on the exponenital decay ODE.
TEST("AD Implicit Midpoint Method", ImplicitMidpoint_ExponentialDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 1.0;
	DualNumber_T<double, 1> dt(T / 1000.0, { 0.0 });
	DualNumber_T<double, 1> t(0.0, { 0.0 });
	for (int i = 0; i < 1000; ++i) {
		x = integrator.implicitMidpoint_AD(x, t, dt, expDecay<double, 1>, 10, 1e-6);
		t += dt;
	}
	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "Implicit Midpoint exponential decay error too large");
}
// Test case for the implicit midpoint method on the exponential decay ODE.
TEST("AD Implicit Midpoint Method", ImplicitMidpoint_ExponentialDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
	) {
		return integrator.implicitMidpoint_AD(x, t, dt, expDecay<double, 1>, 20, 1e-6);
	};
	verifyExpDecaySensitivity(stepFn, 1e-2, std::exp(-1.0), 1000);
}
// Test case for the Implicit Midpoint method on the harmonic oscillator ODE.
TEST("AD Implicit Midpoint Method", ImplicitMidpoint_HarmonicOscillator_EnergyPreservation) {
	VecX_T<DualNumber_T<double, 2>> x(2);
	x(0) = DualNumber_T<double, 2>(1.0, { 0.0, 0.0 });
	x(1) = DualNumber_T<double, 2>(0.0, { 0.0, 0.0 });
	double E0 = 0.5 * (x(0).real * x(0).real + x(1).real * x(1).real);
	double T = 5.0;
	DualNumber_T<double, 2> dt(T / 2500.0);
	DualNumber_T<double, 2> t(0.0);
	for (int i = 0; i < 2500; ++i) {
		x = integrator.implicitMidpoint_AD(x, t, dt, harmonicOsc<double, 2>, 10, 1e-6);
		t += dt;
	}
	double Ef = 0.5 * (x(0).real * x(0).real + x(1).real * x(1).real);
	double drift = std::abs(Ef - E0);
	ASSERT_TRUE(drift < tol_high * (1.0 + E0), "Implicit Midpoint energy drift too large for SHO");
}
// Test case for the implicit midpoint method on the harmonic oscillator ODE.
TEST("AD Implicit Midpoint Method", ImplicitMidpoint_HarmonicOscillatorSensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 2>>& x,
		DualNumber_T<double, 2> t,
		DualNumber_T<double, 2> dt
	) {
		return integrator.implicitMidpoint_AD(x, t, dt, harmonicOsc<double, 2>, 20, 1e-6);
	};
	verifyHarmonicOscillatorSensitivity(stepFn, 1e-2, 1000);
}
// Test case for the Implicit Midpoint method on the linear decay ODE.
TEST("AD Implicit Midpoint Method", ImplicitMidpoint_LinearDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 0.5;
	DualNumber_T<double, 1> dt(T / 500.0);
	DualNumber_T<double, 1> t(0.0);
	for (int i = 0; i < 500; ++i) {
		x = integrator.implicitMidpoint_AD(x, t, dt, linearDecay<double, 1>, 10, 1e-6);
		t += dt;
	}
	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "Implicit Midpoint linear decay error too large");
}
// Test case for the implicit midpoint method on the linear decay ODE.
TEST("AD Implicit Midpoint Method", ImplicitMidpoint_LinearDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
	) {
		return integrator.implicitMidpoint_AD(x, t, dt, linearDecay<double, 1>, 20, 1e-6);
	};
	verifyExpDecaySensitivity(stepFn, 1e-2, std::exp(-2.0), 1000);
}
// Test case for the Implicit Midpoint method on the stiff decay ODE.
TEST("AD Implicit Midpoint Method", ImplicitMidpoint_StiffDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 0.5;
	DualNumber_T<double, 1> dt(T / 500.0);
	DualNumber_T<double, 1> t(0.0);
	double expected = std::exp(-50.0);
	auto stiff_f = [](auto t, const auto& x) { // why is this necessary? Why can't I just pass stiffDecay<double, 1> directly? the answer is that the template parameters can't be deduced from the function pointer, but they can be deduced from the 
		auto dx = x;
		dx(0) = -100.0 * x(0);
		return dx;
	};
	for (int i = 0; i < 500; ++i) {
		x = integrator.implicitMidpoint_AD(x, t, dt, stiffDecay<double, 1>, 10, 1e-6);
		t += dt;
	}
	ASSERT_TRUE(std::isfinite(x(0).real), "Implicit Midpoint produced a non-finite result.");
	ASSERT_TRUE(x(0).real < 1e-12, "Implicit Midpoint did not sufficiently damp stiff mode");
}
// Test case for the implicit midpoint method on the stiff decay ODE.
TEST("AD Implicit Midpoint Method", ImplicitMidpoint_StiffDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
	) {
		return integrator.implicitMidpoint_AD(x, t, dt, stiffDecay<double, 1>, 20, 1e-6);
	};
	verifyExpDecaySensitivity(stepFn, 1e-2, std::exp(-100.0), 1000);
}
// Test large step performance of implicit midpoint
TEST("AD Implicit Midpoint Method", ImplicitMidpoint_Stability_LargeStep) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 1.0 });
	double T = 1.0;
	DualNumber_T<double, 1> t(0.0);
	DualNumber_T<double, 1> dt(1.0 / 10.0);
	DualNumber_T<double, 1> prev = x(0);
	for (int i = 0; i < 10; ++i) {
		x = integrator.implicitMidpoint_AD(x, t, dt, stiffDecay<double, 1>, 50, 1e-6);
		t += dt;
		ASSERT_TRUE(std::abs(x(0).real) < std::abs(prev.real), "Not monotone");
		ASSERT_TRUE(std::abs(x(0).dual[0]) < std::abs(prev.dual[0]), "Not strictly decaying");
		prev = x(0);
	}
}

// AD GLRK2 Tests
// Test case for the GLRK2 method on the exponenital decay ODE.
TEST("AD GLRK2 Method", GLRK2_ExponentialDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 1.0;
	DualNumber_T<double, 1> dt(T / 1000.0, { 0.0 });
	DualNumber_T<double, 1> t(0.0, { 0.0 });
	for (int i = 0; i < 1000; ++i) {
		x = integrator.GLRK2_AD(x, t, dt, expDecay<double, 1>, 50, 1e-6);
		t += dt;
	}
	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "GLRK3 exponential decay error too large");
}
// Test case for the GLRK2 method on the exponential decay ODE.
TEST("AD GLRK2 Method", GLRK2_ExponentialDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
	) {
		return integrator.GLRK2_AD(x, t, dt, expDecay<double, 1>, 50, 1e-6);
	};
	verifyExpDecaySensitivity(stepFn, 1e-4, std::exp(-1.0), 1000);
}
// Test case for the GLRK2 method on the harmonic oscillator ODE.
TEST("AD GLRK2 Method", GLRK2_HarmonicOscillator_EnergyPreservation) {
	VecX_T<DualNumber_T<double, 2>> x(2);
	x(0) = DualNumber_T<double, 2>(1.0, { 0.0, 0.0 });
	x(1) = DualNumber_T<double, 2>(0.0, { 0.0, 0.0 });
	double E0 = 0.5 * (x(0).real * x(0).real + x(1).real * x(1).real);
	double T = 5.0;
	DualNumber_T<double, 2> dt(T / 2500.0);
	DualNumber_T<double, 2> t(0.0);
	for (int i = 0; i < 2500; ++i) {
		x = integrator.GLRK2_AD(x, t, dt, harmonicOsc<double, 2>, 50, 1e-6);
		t += dt;
	}
	double Ef = 0.5 * (x(0).real * x(0).real + x(1).real * x(1).real);
	double drift = std::abs(Ef - E0);
	ASSERT_TRUE(drift < tol_high * (1.0 + E0), "GLRK2 energy drift too large for SHO");
}
// Test case for the GLRK2 method on the harmonic oscillator ODE.
TEST("AD GLRK2 Method", GLRK2_HarmonicOscillatorSensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 2>>& x,
		DualNumber_T<double, 2> t,
		DualNumber_T<double, 2> dt
	) {
		return integrator.GLRK2_AD(x, t, dt, harmonicOsc<double, 2>, 50, 1e-6);
	};
	verifyHarmonicOscillatorSensitivity(stepFn, 1e-4, 1000);
}
// Test case for the GLRK2 method on the linear decay ODE.
TEST("AD GLRK2 Method", GLRK2_LinearDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 0.5;
	DualNumber_T<double, 1> dt(T / 500.0);
	DualNumber_T<double, 1> t(0.0);
	for (int i = 0; i < 500; ++i) {
		x = integrator.GLRK2_AD(x, t, dt, linearDecay<double, 1>, 50, 1e-6);
		t += dt;
	}
	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "GLRK2 linear decay error too large");
}
// Test case for the GLRK2 method on the linear decay ODE.
TEST("AD GLRK2 Method", GLRK2_LinearDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
	) {
		return integrator.GLRK2_AD(x, t, dt, linearDecay<double, 1>, 50, 1e-6);
	};
	verifyExpDecaySensitivity(stepFn, 1e-4, std::exp(-2.0), 1000);
}
// Test case for the GLRK2 method on the stiff decay ODE.
TEST("AD GLRK2 Method", GLRK2_StiffDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 0.5;
	DualNumber_T<double, 1> dt(T / 500.0);
	DualNumber_T<double, 1> t(0.0);
	double expected = std::exp(-50.0);
	auto stiff_f = [](auto t, const auto& x) {
		auto dx = x;
		dx(0) = -100.0 * x(0);
		return dx;
	};
	for (int i = 0; i < 500; ++i) {
		x = integrator.GLRK2_AD(x, t, dt, stiff_f, 50, 1e-10);
		t += dt;
	}
	ASSERT_TRUE(std::isfinite(x(0).real), "GLRK2 produced a non-finite result.");
	ASSERT_TRUE(x(0).real < 1e-12, "GLRK2 did not sufficiently damp stiff mode");
}
// Test case for the GLRK2 method on the stiff decay ODE.
TEST("AD GLRK2 Method", GLRK2_StiffDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
	) {
		return integrator.GLRK2_AD(x, t, dt, stiffDecay<double, 1>, 50, 1e-6);
	};
	verifyExpDecaySensitivity(stepFn, 1e-4, std::exp(-100.0), 1000);
}
// Test case for the GLRK2 method on the nonlinear decay ODE.
TEST("AD GLRK2 Method", GLRK2_NonlinearDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 0.5;
	DualNumber_T<double, 1> dt(T / 500.0);
	DualNumber_T<double, 1> t(0.0);
	double expected = 1.0 / (1.0 + T);
	auto nl_f = [](auto, const auto& x) {
		auto dx = x;
		dx(0) = -x(0) * x(0);
		return dx;
	};
	for (int i = 0; i < 500; ++i) {
		x = integrator.GLRK2_AD(x, t, dt, nl_f, 50, 1e-6);
		t += dt;
	}
	double rel_err = std::abs(x(0).real - expected) / expected;
	ASSERT_TRUE(rel_err < tol_low, "GLRK2 nonlinear decay error too large");
}
// Test case for the GLRK2 method on the stiff nonlinear decay ODE.
TEST("AD GLRK2 Method", GLRK2_StiffNonlinearDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 0.5;
	DualNumber_T<double, 1> dt(T / 500.0);
	DualNumber_T<double, 1> t(0.0);
	auto stiff_nl_f = [](auto t, const auto& x) {
		auto dx = x;
		dx(0) = -x(0) * (100.0 + x(0));
		return dx;
	};
	for (int i = 0; i < 500; ++i) {
		x = integrator.GLRK2_AD(x, t, dt, stiff_nl_f, 50, 1e-6);
		t += dt;
	}
	double expected = 100.0 / (101.0 * std::exp(100.0 * T) - 1.0);
	double rel_err = std::abs(x(0).real - expected);
	ASSERT_TRUE(rel_err < 1e-12, "GLRK2 stiff nonlinear decay error too large");
}
// Test large step performance of GLRK2
TEST("AD GLRK2 Method", GLRK2_Stability_LargeStep) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 1.0 });
	double T = 1.0;
	DualNumber_T<double, 1> t(0.0);
	DualNumber_T<double, 1> dt(1.0 / 10.0);
	DualNumber_T<double, 1> prev = x(0);
	for (int i = 0; i < 10; ++i) {
		x = integrator.GLRK2_AD(x, t, dt, stiffDecay<double, 1>, 50, 1e-6);
		t += dt;
		ASSERT_TRUE(std::abs(x(0).real) < std::abs(prev.real), "Not monotone");
		ASSERT_TRUE(std::abs(x(0).dual[0]) < std::abs(prev.dual[0]), "Not strictly decaying");
		prev = x(0);
	}
}

// AD GLRK3 Tests
// Test case for the GLRK3 method on the exponenital decay ODE.
TEST("AD GLRK3 Method", GLRK3_ExponentialDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 1.0;
	DualNumber_T<double, 1> dt(T / 1000.0, { 0.0 });
	DualNumber_T<double, 1> t(0.0, { 0.0 });
	for (int i = 0; i < 1000; ++i) {
		x = integrator.GLRK3_AD(x, t, dt, expDecay<double, 1>, 80, 1e-7);
		t += dt;
	}
	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "GLRK3 exponential decay error too large");
}
// Test case for the GLRK3 method on the exponential decay ODE sensitivity.
TEST("AD GLRK3 Method", GLRK3_ExponentialDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
	) {
		auto f = [](auto t, const auto& x) { return expDecay(t, x); };
		return integrator.GLRK3_AD(x, t, dt, f, 80, 1e-7);
	};
	verifyExpDecaySensitivity(stepFn, 1e-4, std::exp(-1.0), 1000);
}
// Test case for the GLRK3 method on the harmonic oscillator ODE.
TEST("AD GLRK3 Method", GLRK3_HarmonicOscillator_EnergyPreservation) {
	VecX_T<DualNumber_T<double, 2>> x(2);
	x(0) = DualNumber_T<double, 2>(1.0, { 0.0, 0.0 });
	x(1) = DualNumber_T<double, 2>(0.0, { 0.0, 0.0 });
	double E0 = 0.5 * (x(0).real * x(0).real + x(1).real * x(1).real);
	double T = 5.0;
	DualNumber_T<double, 2> dt(T / 2500.0);
	DualNumber_T<double, 2> t(0.0);
	for (int i = 0; i < 2500; ++i) {
		x = integrator.GLRK3_AD(x, t, dt, harmonicOsc<double, 2>, 80, 1e-7);
		t += dt;
	}
	double Ef = 0.5 * (x(0).real * x(0).real + x(1).real * x(1).real);
	double drift = std::abs(Ef - E0);
	ASSERT_TRUE(drift < tol_high * (1.0 + E0), "GLRK3 energy drift too large for SHO");
}
// Test case for the GLRK3 method on the harmonic oscillator ODE.
TEST("AD GLRK3 Method", GLRK3_HarmonicOscillatorSensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 2>>& x,
		DualNumber_T<double, 2> t,
		DualNumber_T<double, 2> dt
	) {
		auto f = [](auto t, const auto& x) { return harmonicOsc(t, x); };
		return integrator.GLRK3_AD(x, t, dt, f, 80, 1e-7);
	};
	verifyHarmonicOscillatorSensitivity(stepFn, 1e-4, 1000);
}
// Test case for the GLRK3 method on the linear decay ODE.
TEST("AD GLRK3 Method", GLRK3_LinearDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 0.5;
	DualNumber_T<double, 1> dt(T / 500.0);
	DualNumber_T<double, 1> t(0.0);
	for (int i = 0; i < 500; ++i) {
		x = integrator.GLRK3_AD(x, t, dt, linearDecay<double, 1>, 80, 1e-7);
		t += dt;
	}
	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "GLRK3 linear decay error too large");
}
// Test case for the GLRK3 method on the linear decay ODE.
TEST("AD GLRK3 Method", GLRK3_LinearDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
	) {
		auto f = [](auto t, const auto& x) { return linearDecay(t, x); };
		return integrator.GLRK3_AD(x, t, dt, f, 80, 1e-7);
	};
	verifyExpDecaySensitivity(stepFn, 1e-12, std::exp(-2.0), 1000);
}
// Test case for the GLRK3 method on the stiff decay ODE.
TEST("AD GLRK3 Method", GLRK3_StiffDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 0.5;
	DualNumber_T<double, 1> dt(T / 500.0);
	DualNumber_T<double, 1> t(0.0);
	double expected = std::exp(-50.0);
	auto stiff_f = [](auto t, const auto& x) {
		auto dx = x;
		dx(0) = -100.0 * x(0);
		return dx;
	};
	for (int i = 0; i < 500; ++i) {
		x = integrator.GLRK3_AD(x, t, dt, stiff_f, 150, 1e-12);
		t += dt;
	}
	ASSERT_TRUE(std::isfinite(x(0).real), "GLRK3 produced a non-finite result.");
	ASSERT_TRUE(x(0).real < 1e-14, "GLRK3 did not sufficiently damp stiff mode");
}
// Test case for the GLRK3 method on the stiff decay ODE.
TEST("AD GLRK3 Method", GLRK3_StiffDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
	) {
		auto f = [](auto t, const auto& x) { return stiffDecay(t, x); };
		return integrator.GLRK3_AD(x, t, dt, f, 150, 1e-12);
	};
	verifyExpDecaySensitivity(stepFn, 1e-12, std::exp(-100.0), 1000);
}
// Test case for the GLRK3 method on the nonlinear decay ODE.
TEST("AD GLRK3 Method", GLRK3_NonlinearDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 0.5;
	DualNumber_T<double, 1> dt(T / 500.0);
	DualNumber_T<double, 1> t(0.0);
	double expected = 1.0 / (1.0 + T);
	auto nl_f = [](auto t, const auto& x) {
		auto dx = x;
		dx(0) = -x(0) * x(0);
		return dx;
	};
	for (int i = 0; i < 500; ++i) {
		x = integrator.GLRK3_AD(x, t, dt, nl_f, 80, 1e-7);
		t += dt;
	}
	double rel_err = std::abs(x(0).real -expected) / expected;
	ASSERT_TRUE(rel_err < tol_low, "GLRK3 nonlinear decay error too large");
}
// Test case for the GLRK3 method on the stiff nonlinear decay ODE.
TEST("AD GLRK3 Method", GLRK3_StiffNonlinearDecay) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
	double T = 0.5;
	DualNumber_T<double, 1> dt(T / 500.0);
	DualNumber_T<double, 1> t(0.0);
	auto stiff_nl_f = [](auto t, const auto& x) {
		auto dx = x.eval();
		dx(0) = -x(0) * (100.0 + x(0));
		return dx;
	};
	for (int i = 0; i < 500; ++i) {
		x = integrator.GLRK3_AD(x, t, dt, stiff_nl_f, 80, 1e-7);
		t += dt;
	}
	double expected = 100.0 / (101.0 * std::exp(100.0 * T) - 1.0);
	double rel_err = std::abs(x(0).real - expected);
	ASSERT_TRUE(rel_err < 1e-12, "GLRK3 stiff nonlinear decay error too large");
}
// Test case for the GLRK3 method on the stiff decay ODE.
TEST("AD GLRK3 Method", GLRK3_Stability_LargeStep) {
	VecX_T<DualNumber_T<double, 1>> x(1);
	x(0) = DualNumber_T<double, 1>(1.0, { 1.0 });
	double T = 1.0;
	DualNumber_T<double, 1> t(0.0);
	DualNumber_T<double, 1> dt(1.0 / 10.0);
	DualNumber_T<double, 1> prev = x(0);
	for (int i = 0; i < 10; ++i) {
		x = integrator.GLRK3_AD(x, t, dt, stiffDecay<double, 1>, 150, 1e-12);
		t += dt;
		ASSERT_TRUE(std::abs(x(0).real) < std::abs(prev.real), "Not monotone");
		ASSERT_TRUE(std::abs(x(0).dual[0]) < std::abs(prev.dual[0]), "Not strictly decaying");
		prev = x(0);
	}
}