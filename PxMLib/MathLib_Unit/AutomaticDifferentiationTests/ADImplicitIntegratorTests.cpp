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
		DualNumber_T<Scalar, NVar> dt(T / Scalar(N));
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
		int N
	) {
		VecX_T<DualNumber_T<double, 1>> x0(1);
		x0(0) = DualNumber_T<double, 1>(1.0, { 1.0 }); // Initial condition with derivative 1.0
		VecX_T<DualNumber_T<double, 1>> r = integrateToTime(stepFn, x0, 1.0, N);
		double expectedValue = std::exp(-1.0);
		ASSERT_TRUE(std::abs(r(0).real - expectedValue) < tol, "Exponential decay error too large");
		ASSERT_TRUE(std::abs(r(0).dual[0] - expectedValue) < tol, "Exponential decay sensitivity error too large");
		ASSERT_TRUE(std::abs(r(0).dual[0]) > 0.0, "Sensitivity vanished during propagation");
	}

	// Helper function to verify that a harmonic oscillator ODE is integrated correctly, and that the sensitivity of the solution with respect to the initial condition matches the analytical derivative.
	template<typename StepFn>
	void verifyHarmonicOscillatorSensitivity(
		StepFn stepFn,
		double tol,
		int N
	) {
		VecX_T<DualNumber_T<double, 2>> x0(2);
		x0(0) = DualNumber_T<double, 2>(1.0, { 1.0, 0.0 }); // Initial position with sensitivity
		x0(1) = DualNumber_T<double, 2>(0.0, { 0.0, 1.0 }); // Initial velocity with sensitivity
		VecX_T<DualNumber_T<double, 2>> r = integrateToTime(stepFn, x0, TWO_PI_d, N);
		double expectedPosition = std::cos(TWO_PI_d);
		double expectedVelocity = -std::sin(TWO_PI_d);
		double expectedVelocitySens = std::cos(TWO_PI_d);
		ASSERT_TRUE(std::abs(r(0).real - expectedPosition) < tol, "Harmonic oscillator position error too large");
		ASSERT_TRUE(std::abs(r(1).real - expectedVelocity) < tol, "Harmonic oscillator velocity error too large");
		ASSERT_TRUE(std::abs(r(0).dual[0] - expectedPosition) < tol, "Harmonic oscillator position sensitivity error too large");
		ASSERT_TRUE(std::abs(r(1).dual[1] - expectedVelocitySens) < tol, "Harmonic oscillator velocity sensitivity error too large");
		ASSERT_TRUE(std::abs(r(0).dual[0]) > 0.0, "Position sensitivity vanished during propagation");
		ASSERT_TRUE(std::abs(r(1).dual[1]) > 0.0, "Velocity sensitivity vanished during propagation");
	}
}

// AD Implicit Euler Tests
// Test case for the GLRK2 method on the exponenital decay ODE.
//TEST("AD Implicit Euler Method", ImplicitEuler_ExponentialDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 1.0;
//	DualNumber_T<double, 1> dt(T / 1000.0, { 0.0 });
//	DualNumber_T<double, 1> t(0.0, { 0.0 });
//	for (int i = 0; i < 1000; ++i) {
//		x = integrator.implicitEuler_AD(x, t, dt, expDecay<double, 1>, 8, DualNumber_T<double, 1>(1e-6, { 0.0 }));
//		t += dt;
//	}
//	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "Implicit Euler exponential decay error too large");
//}
//// Test case for the implicit Euler method on the exponential decay ODE.
//TEST("AD Implicit Euler Method", ImplicitEuler_ExponentialDecaySensitivity) {
//	auto stepFn = [&](
//		const VecX_T<DualNumber_T<double, 1>>& x,
//		DualNumber_T<double, 1> t,
//		DualNumber_T<double, 1> dt
//		) {
//		return integrator.implicitEuler_AD(x, t, dt, expDecay<double, 1>, 8, DualNumber_T<double, 1>(1e-6, { 0.0 }));
//	};
//	verifyExpDecaySensitivity(stepFn, 1e-2, 1000);
//}
//// Test case for the Implicit Midpoint method on the linear decay ODE.
//TEST("AD Implicit Euler Method", ImplicitEuler_LinearDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 0.5;
//	DualNumber_T<double, 1> dt(T / 500.0);
//	DualNumber_T<double, 1> t(0.0);
//	for (int i = 0; i < 500; ++i) {
//		x = integrator.implicitEuler_AD(x, t, dt, linearDecay<double, 1>, 50, DualNumber_T<double, 1>(1e-6, { 0.0 }));
//		t += dt;
//	}
//	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "Implicit Euler linear decay error too large");
//}
//// Test case for the implicit Euler method on the linear decay ODE.
//TEST("AD Implicit Euler Method", ImplicitEuler_LinearDecaySensitivity) {
//	auto stepFn = [&](
//		const VecX_T<DualNumber_T<double, 1>>& x,
//		DualNumber_T<double, 1> t,
//		DualNumber_T<double, 1> dt
//		) {
//		return integrator.implicitEuler_AD(x, t, dt, expDecay<double, 1>, 8, DualNumber_T<double, 1>(1e-6, { 0.0 }));
//	};
//	verifyExpDecaySensitivity(stepFn, 1e-2, 1000);
//}
//// Test case for the Implicit Euler method on the stiff decay ODE.
//TEST("AD Implicit Euler Method", ImplicitEuler_StiffDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 0.5;
//	DualNumber_T<double, 1> dt(T / 500.0);
//	DualNumber_T<double, 1> t(0.0);
//	double expected = std::exp(-50.0);
//	auto stiff_f = [](auto t, const auto& x) {
//		auto dx = x;
//		dx(0) = -100.0 * x(0);
//		return dx;
//	};
//	for (int i = 0; i < 500; ++i) {
//		x = integrator.implicitEuler_AD(x, t, dt, stiff_f, 50, DualNumber_T<double, 1>(1e-6, { 0.0 }));
//		t += dt;
//	}
//	ASSERT_TRUE(std::isfinite(x(0).real), "Implicit Euler produced a non-finite result.");
//	ASSERT_TRUE(x(0).real < 1e-12, "Implicit Euler did not sufficiently damp stiff mode");
//}
//// Test case for the implicit Euler method on the stiff decay ODE.
//TEST("AD Implicit Euler Method", ImplicitEuler_StiffDecaySensitivity) {
//	auto stepFn = [&](
//		const VecX_T<DualNumber_T<double, 1>>& x,
//		DualNumber_T<double, 1> t,
//		DualNumber_T<double, 1> dt
//		) {
//		return integrator.implicitEuler_AD(x, t, dt, stiffDecay<double, 1>, 50, DualNumber_T<double, 1>(1e-6, { 0.0 }));
//	};
//	verifyExpDecaySensitivity(stepFn, 1e-2, 1000);
//}
//// Test large step performance of implicit midpoint
//TEST("AD Implicit Euler Method", ImplicitEuler_Stability_LargeStep) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 1.0 });
//	double T = 1.0;
//	DualNumber_T<double, 1> t(0.0);
//	DualNumber_T<double, 1> dt(1.0 / 10.0);
//	DualNumber_T<double, 1> prev = x(0);
//	for (int i = 0; i < 10; ++i) {
//		x = integrator.implicitEuler_AD(x, t, dt, stiffDecay<double, 1>, 50, DualNumber_T<double, 1>(1e-6, { 0.0 }));
//		t += dt;
//		ASSERT_TRUE(std::abs(x(0).real) < std::abs(prev.real), "Not monotone");
//		ASSERT_TRUE(std::abs(x(0).dual[0]) < std::abs(prev.dual[0]), "Not strictly decaying");
//		prev = x(0);
//	}
//}

//// AD Implicit Midpoint Tests
//// Test case for the Implicit Midpoint method on the exponenital decay ODE.
//TEST("AD Implicit Midpoint Method", ImplicitMidpoint_ExponentialDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 1.0;
//	DualNumber_T<double, 1> dt(T / 1000.0, { 0.0 });
//	DualNumber_T<double, 1> t(0.0, { 0.0 });
//	for (int i = 0; i < 1000; ++i) {
//		x = integrator.implicitMidpoint_AD(x, t, dt, expDecay<double, 1>, 8, DualNumber_T<double, 1>(1e-6, { 0.0 }));
//		t += dt;
//	}
//	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "Implicit Midpoint exponential decay error too large");
//}
//// Test case for the implicit midpoint method on the exponential decay ODE.
//TEST("AD Implicit Midpoint Method", ImplicitMidpoint_ExponentialDecaySensitivity) {
//	auto stepFn = [&](
//		const VecX_T<DualNumber_T<double, 1>>& x,
//		DualNumber_T<double, 1> t,
//		DualNumber_T<double, 1> dt
//	) {
//		return integrator.implicitMidpoint_AD(x, t, dt, expDecay<double, 1>, 8, DualNumber_T<double, 1>(1e-6, { 0.0 }));
//	};
//	verifyExpDecaySensitivity(stepFn, 1e-3, 1000);
//}
//// Test case for the Implicit Midpoint method on the harmonic oscillator ODE.
//TEST("AD Implicit Midpoint Method", ImplicitMidpoint_HarmonicOscillator_EnergyPreservation) {
//	VecX_T<DualNumber_T<double, 2>> x(2);
//	x(0) = DualNumber_T<double, 2>(1.0, { 0.0, 0.0 });
//	x(1) = DualNumber_T<double, 2>(0.0, { 0.0, 0.0 });
//	double E0 = 0.5 * (x(0).real * x(0).real + x(1).real * x(1).real);
//	double T = 5.0;
//	DualNumber_T<double, 2> dt(T / 2500.0);
//	DualNumber_T<double, 2> t(0.0);
//	for (int i = 0; i < 2500; ++i) {
//		x = integrator.implicitMidpoint_AD(x, t, dt, harmonicOsc<double, 2>, 10, DualNumber_T<double, 2>(1e-7, { 0.0, 0.0 }));
//		t += dt;
//	}
//	double Ef = 0.5 * (x(0).real * x(0).real + x(1).real * x(1).real);
//	double drift = std::abs(Ef - E0);
//	ASSERT_TRUE(drift < tol_high * (1.0 + E0), "Implicit Midpoint energy drift too large for SHO");
//}
//// Test case for the implicit midpoint method on the harmonic oscillator ODE.
//TEST("AD Implicit Midpoint Method", ImplicitMidpoint_HarmonicOscillatorSensitivity) {
//	auto stepFn = [&](
//		const VecX_T<DualNumber_T<double, 2>>& x,
//		DualNumber_T<double, 2> t,
//		DualNumber_T<double, 2> dt
//	) {
//		return integrator.implicitMidpoint_AD(x, t, dt, harmonicOsc<double, 2>, 10, DualNumber_T<double, 2>(1e-7, { 0.0, 0.0 }));
//	};
//	verifyHarmonicOscillatorSensitivity(stepFn, 1e-3, 1000);
//}
//// Test case for the Implicit Midpoint method on the linear decay ODE.
//TEST("AD Implicit Midpoint Method", ImplicitMidpoint_LinearDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 0.5;
//	DualNumber_T<double, 1> dt(T / 500.0);
//	DualNumber_T<double, 1> t(0.0);
//	for (int i = 0; i < 500; ++i) {
//		x = integrator.implicitMidpoint_AD(x, t, dt, linearDecay<double, 1>, 10, DualNumber_T<double, 1>(1e-6, { 0.0 }));
//		t += dt;
//	}
//	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "Implicit Midpoint linear decay error too large");
//}
//// Test case for the implicit midpoint method on the linear decay ODE.
//TEST("AD Implicit Midpoint Method", ImplicitMidpoint_LinearDecaySensitivity) {
//	auto stepFn = [&](
//		const VecX_T<DualNumber_T<double, 1>>& x,
//		DualNumber_T<double, 1> t,
//		DualNumber_T<double, 1> dt
//	) {
//		return integrator.implicitMidpoint_AD(x, t, dt, linearDecay<double, 1>, 10, DualNumber_T<double, 1>(1e-6, { 0.0 }));	
//	};
//	verifyExpDecaySensitivity(stepFn, 1e-3, 1000);
//}
//// Test case for the Implicit Midpoint method on the stiff decay ODE.
//TEST("AD Implicit Midpoint Method", ImplicitMidpoint_StiffDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 0.5;
//	DualNumber_T<double, 1> dt(T / 500.0);
//	DualNumber_T<double, 1> t(0.0);
//	double expected = std::exp(-50.0);
//	auto stiff_f = [](auto t, const auto& x) { // why is this necessary? Why can't I just pass stiffDecay<double, 1> directly? the answer is that the template parameters can't be deduced from the function pointer, but they can be deduced from the 
//		auto dx = x;
//		dx(0) = -100.0 * x(0);
//		return dx;
//	};
//	for (int i = 0; i < 500; ++i) {
//		x = integrator.implicitMidpoint_AD(x, t, dt, stiffDecay<double, 1>, 10, DualNumber_T<double, 1>(1e-6, { 0.0 }));
//		t += dt;
//	}
//	ASSERT_TRUE(std::isfinite(x(0).real), "Implicit Midpoint produced a non-finite result.");
//	ASSERT_TRUE(x(0).real < 1e-12, "Implicit Midpoint did not sufficiently damp stiff mode");
//}
//// Test case for the implicit midpoint method on the stiff decay ODE.
//TEST("AD Implicit Midpoint Method", ImplicitMidpoint_StiffDecaySensitivity) {
//	auto stepFn = [&](
//		const VecX_T<DualNumber_T<double, 1>>& x,
//		DualNumber_T<double, 1> t,
//		DualNumber_T<double, 1> dt
//	) {
//		return integrator.implicitMidpoint_AD(x, t, dt, stiffDecay<double, 1>, 10, DualNumber_T<double, 1>(1e-6, { 0.0 }));
//	};
//	verifyExpDecaySensitivity(stepFn, 1e-3, 1000);
//}
//// Test large step performance of implicit midpoint
//TEST("AD Implicit Midpoint Method", ImplicitMidpoint_Stability_LargeStep) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 1.0 });
//	double T = 1.0;
//	DualNumber_T<double, 1> t(0.0);
//	DualNumber_T<double, 1> dt(1.0 / 10.0);
//	DualNumber_T<double, 1> prev = x(0);
//	for (int i = 0; i < 10; ++i) {
//		x = integrator.implicitMidpoint_AD(x, t, dt, stiffDecay<double, 1>, 10, DualNumber_T<double, 1>(1e-6, { 0.0 }));
//		t += dt;
//		ASSERT_TRUE(std::abs(x(0).real) < std::abs(prev.real), "Not monotone");
//		ASSERT_TRUE(std::abs(x(0).dual[0]) < std::abs(prev.dual[0]), "Not strictly decaying");
//		prev = x(0);
//	}
//}

//// AD GLRK2 Tests
//// Test case for the GLRK2 method on the exponenital decay ODE.
//TEST("AD GLRK2 Method", GLRK2_ExponentialDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 1.0;
//	DualNumber_T<double, 1> dt(T / 1000.0, { 0.0 });
//	DualNumber_T<double, 1> t(0.0, { 0.0 });
//	for (int i = 0; i < 1000; ++i) {
//		x = integrator.GLRK2(x, t, dt, expDecay<double, 1>, 50, 1e-6);
//		t += dt;
//	}
//	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "GLRK3 exponential decay error too large");
//}
//// Test case for the GLRK2 method on the exponential decay ODE.
//TEST("AD GLRK2 Method", GLRK2_ExponentialDecaySensitivity) {
//	auto stepFn = [&](
//		const VecX_T<DualNumber_T<double, 1>>& x,
//		DualNumber_T<double, 1> t,
//		DualNumber_T<double, 1> dt
//	) {
//		return integrator.GLRK2(x, t, dt, expDecay<double, 1>);
//	};
//	verifyExpDecaySensitivity(stepFn, 1e-4, 1000);
//}
//// Test case for the GLRK2 method on the harmonic oscillator ODE.
//TEST("AD GLRK2 Method", GLRK2_HarmonicOscillator_EnergyPreservation) {
//	VecX_T<DualNumber_T<double, 2>> x(2);
//	x(0) = DualNumber_T<double, 2>(1.0, { 0.0, 0.0 });
//	x(1) = DualNumber_T<double, 2>(0.0, { 0.0, 0.0 });
//	double E0 = 0.5 * (x(0).real * x(0).real + x(1).real * x(1).real);
//	double T = 5.0;
//	DualNumber_T<double, 2> dt(T / 2500.0);
//	DualNumber_T<double, 2> t(0.0);
//	for (int i = 0; i < 2500; ++i) {
//		x = integrator.GLRK2(x, t, dt, harmonicOsc<double, 2>, 50, 1e-6);
//		t += dt;
//	}
//	double Ef = 0.5 * (x(0).real * x(0).real + x(1).real * x(1).real);
//	double drift = std::abs(Ef - E0);
//	ASSERT_TRUE(drift < tol_high * (1.0 + E0), "GLRK2 energy drift too large for SHO");
//}
//// Test case for the GLRK2 method on the harmonic oscillator ODE.
//TEST("AD GLRK2 Method", GLRK2_HarmonicOscillatorSensitivity) {
//	auto stepFn = [&](
//		const VecX_T<DualNumber_T<double, 2>>& x,
//		DualNumber_T<double, 2> t,
//		DualNumber_T<double, 2> dt
//	) {
//		return integrator.GLRK2(x, t, dt, harmonicOsc<double, 2>);
//	};
//	verifyHarmonicOscillatorSensitivity(stepFn, 1e-4, 1000);
//}
//// Test case for the GLRK2 method on the linear decay ODE.
//TEST("AD GLRK2 Method", GLRK2_LinearDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 0.5;
//	DualNumber_T<double, 1> dt(T / 500.0);
//	DualNumber_T<double, 1> t(0.0);
//	for (int i = 0; i < 500; ++i) {
//		x = integrator.GLRK2(x, t, dt, linearDecay<double, 1>, 50, 1e-6);
//		t += dt;
//	}
//	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "GLRK2 linear decay error too large");
//}
//// Test case for the GLRK2 method on the linear decay ODE.
//TEST("AD GLRK2 Method", GLRK2_LinearDecaySensitivity) {
//	auto stepFn = [&](
//		const VecX_T<DualNumber_T<double, 1>>& x,
//		DualNumber_T<double, 1> t,
//		DualNumber_T<double, 1> dt
//	) {
//		return integrator.GLRK2(x, t, dt, linearDecay<double, 1>);
//	};
//	verifyExpDecaySensitivity(stepFn, 1e-4, 1000);
//}
//// Test case for the GLRK2 method on the stiff decay ODE.
//TEST("AD GLRK2 Method", GLRK2_StiffDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 0.5;
//	DualNumber_T<double, 1> dt(T / 500.0);
//	DualNumber_T<double, 1> t(0.0);
//	double expected = std::exp(-50.0);
//	auto stiff_f = [](auto t, const auto& x) {
//		auto dx = x;
//		dx(0) = -100.0 * x(0);
//		return dx;
//	};
//	for (int i = 0; i < 500; ++i) {
//		x = integrator.GLRK2(x, t, dt, stiff_f, 50, 1e-10);
//		t += dt;
//	}
//	ASSERT_TRUE(std::isfinite(x(0).real), "GLRK2 produced a non-finite result.");
//	ASSERT_TRUE(x(0).real < 1e-12, "GLRK2 did not sufficiently damp stiff mode");
//}
//// Test case for the GLRK2 method on the stiff decay ODE.
//TEST("AD GLRK2 Method", GLRK2_StiffDecaySensitivity) {
//	auto stepFn = [&](
//		const VecX_T<DualNumber_T<double, 1>>& x,
//		DualNumber_T<double, 1> t,
//		DualNumber_T<double, 1> dt
//	) {
//		return integrator.GLRK2(x, t, dt, stiffDecay<double, 1>);
//	};
//	verifyExpDecaySensitivity(stepFn, 1e-4, 1000);
//}
//// Test case for the GLRK2 method on the nonlinear decay ODE.
//TEST("AD GLRK2 Method", GLRK2_NonlinearDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 0.5;
//	DualNumber_T<double, 1> dt(T / 500.0);
//	DualNumber_T<double, 1> t(0.0);
//	double expected = 1.0 / (1.0 + T);
//	auto nl_f = [](auto, const auto& x) {
//		auto dx = x;
//		dx(0) = -x(0) * x(0);
//		return dx;
//	};
//	for (int i = 0; i < 500; ++i) {
//		x = integrator.GLRK2(x, t, dt, nl_f, 50, 1e-6);
//		t += dt;
//	}
//	double rel_err = std::abs(x(0).real - expected) / expected;
//	ASSERT_TRUE(rel_err < tol_low, "GLRK2 nonlinear decay error too large");
//}
//// Test case for the GLRK2 method on the stiff nonlinear decay ODE.
//TEST("AD GLRK2 Method", GLRK2_StiffNonlinearDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 0.5;
//	DualNumber_T<double, 1> dt(T / 500.0);
//	DualNumber_T<double, 1> t(0.0);
//	double expected = 1.0 / 51.0;
//	auto stiff_nl_f = [](auto t, const auto& x) {
//		auto dx = x;
//		dx(0) = -100.0 * x(0) - x(0) * x(0);
//		return dx;
//	};
//	for (int i = 0; i < 500; ++i) {
//		x = integrator.GLRK2(x, t, dt, stiff_nl_f, 50, 1e-6);
//		t += dt;
//	}
//	double rel_err = std::abs(x(0).real - expected) / expected;
//	ASSERT_TRUE(rel_err < tol_low, "GLRK2 stiff nonlinear decay error too large");
//}
//// Test large step performance of GLRK2
//TEST("AD GLRK2 Method", GLRK2_Stability_LargeStep) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 1.0 });
//	double T = 1.0;
//	DualNumber_T<double, 1> t(0.0);
//	DualNumber_T<double, 1> dt(1.0 / 10.0);
//	DualNumber_T<double, 1> prev = x(0);
//	for (int i = 0; i < 10; ++i) {
//		x = integrator.GLRK2(x, t, dt, stiffDecay<double, 1>);
//		t += dt;
//		ASSERT_TRUE(std::abs(x(0).real) < std::abs(prev.real), "Not monotone");
//		ASSERT_TRUE(std::abs(x(0).dual[0]) < std::abs(prev.dual[0]), "Not strictly decaying");
//		prev = x(0);
//	}
//}

//// AD GLRK3 Tests
//// Test case for the GLRK3 method on the exponenital decay ODE.
//TEST("AD GLRK3 Method", GLRK3_ExponentialDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 1.0;
//	DualNumber_T<double, 1> dt(T / 1000.0, { 0.0 });
//	DualNumber_T<double, 1> t(0.0, { 0.0 });
//	for (int i = 0; i < 1000; ++i) {
//		x = integrator.GLRK3(x, t, dt, expDecay<double, 1>, 80, 1e-7);
//		t += dt;
//	}
//	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "GLRK3 exponential decay error too large");
//}
//// Test case for the GLRK3 method on the exponential decay ODE sensitivity.
//TEST("AD GLRK3 Method", GLRK3_ExponentialDecaySensitivity) {
//	auto stepFn = [&](
//		const VecX_T<DualNumber_T<double, 1>>& x,
//		DualNumber_T<double, 1> t,
//		DualNumber_T<double, 1> dt
//	) {
//		return integrator.GLRK3(x, t, dt, expDecay<double, 1>);
//	};
//	verifyExpDecaySensitivity(stepFn, 1e-4, 1000);
//}
//// Test case for the GLRK3 method on the harmonic oscillator ODE.
//TEST("AD GLRK3 Method", GLRK3_HarmonicOscillator_EnergyPreservation) {
//	VecX_T<DualNumber_T<double, 2>> x(2);
//	x(0) = DualNumber_T<double, 2>(1.0, { 0.0, 0.0 });
//	x(1) = DualNumber_T<double, 2>(0.0, { 0.0, 0.0 });
//	double E0 = 0.5 * (x(0).real * x(0).real + x(1).real * x(1).real);
//	double T = 5.0;
//	DualNumber_T<double, 2> dt(T / 2500.0);
//	DualNumber_T<double, 2> t(0.0);
//	for (int i = 0; i < 2500; ++i) {
//		x = integrator.GLRK3(x, t, dt, harmonicOsc<double, 2>, 80, 1e-7);
//		t += dt;
//	}
//	double Ef = 0.5 * (x(0).real * x(0).real + x(1).real * x(1).real);
//	double drift = std::abs(Ef - E0);
//	ASSERT_TRUE(drift < tol_high * (1.0 + E0), "GLRK3 energy drift too large for SHO");
//}
//// Test case for the GLRK3 method on the harmonic oscillator ODE.
//TEST("AD GLRK3 Method", GLRK3_HarmonicOscillatorSensitivity) {
//	auto stepFn = [&](
//		const VecX_T<DualNumber_T<double, 2>>& x,
//		DualNumber_T<double, 2> t,
//		DualNumber_T<double, 2> dt
//	) {
//		return integrator.GLRK3(x, t, dt, harmonicOsc<double, 2>);
//	};
//	verifyHarmonicOscillatorSensitivity(stepFn, 1e-4, 1000);
//}
//// Test case for the GLRK3 method on the linear decay ODE.
//TEST("AD GLRK3 Method", GLRK3_LinearDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 0.5;
//	DualNumber_T<double, 1> dt(T / 500.0);
//	DualNumber_T<double, 1> t(0.0);
//	for (int i = 0; i < 500; ++i) {
//		x = integrator.GLRK3(x, t, dt, linearDecay<double, 1>, 80, 1e-7);
//		t += dt;
//	}
//	ASSERT_TRUE(std::abs(x(0).real - std::exp(-1.0)) < tol_low, "GLRK3 linear decay error too large");
//}
//// Test case for the GLRK3 method on the linear decay ODE.
//TEST("AD GLRK3 Method", GLRK3_LinearDecaySensitivity) {
//	auto stepFn = [&](
//		const VecX_T<DualNumber_T<double, 1>>& x,
//		DualNumber_T<double, 1> t,
//		DualNumber_T<double, 1> dt
//		) {
//		return integrator.GLRK3(x, t, dt, linearDecay<double, 1>);
//	};
//	verifyExpDecaySensitivity(stepFn, 1e-12, 1000);
//}
//// Test case for the GLRK3 method on the stiff decay ODE.
//TEST("AD GLRK3 Method", GLRK3_StiffDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 0.5;
//	DualNumber_T<double, 1> dt(T / 500.0);
//	DualNumber_T<double, 1> t(0.0);
//	double expected = std::exp(-50.0);
//	auto stiff_f = [](auto t, const auto& x) {
//		auto dx = x;
//		dx(0) = -100.0 * x(0);
//		return dx;
//	};
//	for (int i = 0; i < 500; ++i) {
//		x = integrator.GLRK3(x, t, dt, stiff_f, 150, 1e-12);
//		t += dt;
//	}
//	ASSERT_TRUE(std::isfinite(x(0).real), "GLRK3 produced a non-finite result.");
//	ASSERT_TRUE(x(0).real < 1e-14, "GLRK3 did not sufficiently damp stiff mode");
//}
//// Test case for the GLRK3 method on the stiff decay ODE.
//TEST("AD GLRK3 Method", GLRK3_StiffDecaySensitivity) {
//	auto stepFn = [&](
//		const VecX_T<DualNumber_T<double, 1>>& x,
//		DualNumber_T<double, 1> t,
//		DualNumber_T<double, 1> dt
//	) {
//		return integrator.GLRK3(x, t, dt, stiffDecay<double, 1>);
//	};
//	verifyExpDecaySensitivity(stepFn, 1e-12, 1000);
//}
//// Test case for the GLRK3 method on the nonlinear decay ODE.
//TEST("AD GLRK3 Method", GLRK3_NonlinearDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 0.5;
//	DualNumber_T<double, 1> dt(T / 500.0);
//	DualNumber_T<double, 1> t(0.0);
//	double expected = 1.0 / (1.0 + T);
//	auto nl_f = [](auto t, const auto& x) {
//		auto dx = x;
//		dx(0) = -x(0) * x(0);
//		return dx;
//	};
//	for (int i = 0; i < 500; ++i) {
//		x = integrator.GLRK3(x, t, dt, nl_f, 80, 1e-7);
//		t += dt;
//	}
//	double rel_err = std::abs(x(0).real -expected) / expected;
//	ASSERT_TRUE(rel_err < tol_low, "GLRK3 nonlinear decay error too large");
//}
//// Test case for the GLRK3 method on the stiff nonlinear decay ODE.
//TEST("AD GLRK3 Method", GLRK3_StiffNonlinearDecay) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 0.0 });
//	double T = 0.5;
//	DualNumber_T<double, 1> dt(T / 500.0);
//	DualNumber_T<double, 1> t(0.0);
//	double expected = 1.0 / 51.0;
//	auto stiff_nl_f = [](auto t, const auto& x) {
//		auto dx = x.eval();
//		dx(0) = -100.0 * x(0) - x(0) * x(0);
//		return dx;
//	};
//	for (int i = 0; i < 500; ++i) {
//		x = integrator.GLRK3(x, t, dt, stiff_nl_f, 80, 1e-7);
//		t += dt;
//	}
//	double rel_err = std::abs(x(0).real - expected) / expected;
//	ASSERT_TRUE(rel_err < tol_low, "GLRK3 stiff nonlinear decay error too large");
//}
//// Test case for the GLRK3 method on the stiff decay ODE.
//TEST("AD GLRK3 Method", GLRK3_Stability_LargeStep) {
//	VecX_T<DualNumber_T<double, 1>> x(1);
//	x(0) = DualNumber_T<double, 1>(1.0, { 1.0 });
//	double T = 1.0;
//	DualNumber_T<double, 1> t(0.0);
//	DualNumber_T<double, 1> dt(1.0 / 10.0);
//	DualNumber_T<double, 1> prev = x(0);
//	for (int i = 0; i < 10; ++i) {
//		x = integrator.GLRK3(x, t, dt, stiffDecay<double, 1>);
//		t += dt;
//		ASSERT_TRUE(std::abs(x(0).real) < std::abs(prev.real), "Not monotone");
//		ASSERT_TRUE(std::abs(x(0).dual[0]) < std::abs(prev.dual[0]), "Not strictly decaying");
//		prev = x(0);
//	}
//}