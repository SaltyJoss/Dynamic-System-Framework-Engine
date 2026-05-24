// MathLib_UnitTests ADExplicitIntegratorTests.cpp

#include "TestHarness.h"
#include <core/MathLib.h>
#include <integrators/numerical_integrators.h>
#include <cmath>
#include <functional>
#include <core/constants.h>

using namespace mathlib;
using namespace constants;

namespace {
	// Simple Exponential Decay Function: dx/dt = -x, which should decay to exp(-1) at t=1
	template<typename Scalar, size_t NVar>
	VecX_T<DualNumber_T<Scalar, NVar>> expDecay(
		DualNumber_T<Scalar, NVar>,
		const VecX_T<DualNumber_T<Scalar, NVar>>& x
	) {
		VecX_T<DualNumber_T<Scalar, NVar>> dx = x;
		for (int i = 0; i < x.size(); ++i) { dx(i) = -x(i); }
		return dx;
	}

	// Simple Harmonic Oscillator: dx/dt = v, dv/dt = -x, which should preserve energy
	template<typename Scalar, size_t NVar>
	VecX_T<DualNumber_T<Scalar, NVar>> harmonicOsc(
		DualNumber_T<Scalar, NVar>,
		const VecX_T<DualNumber_T<Scalar, NVar>>& s
	) {
		VecX_T<DualNumber_T<Scalar, NVar>> d(2);
		d(0) = s(1);
		d(1) = -s(0);
		return d;
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

	template<typename StepFn>
	void verifyHarmonicOscillatorSensitivity(
		StepFn stepFn,
		double stateTol,
		double sensitivityTol,
		double energyTol,
		int N
	) {
		VecX_T<DualNumber_T<double, 2>> x0(2);
		x0(0) = DualNumber_T<double, 2>(1.0, { 1.0, 0.0 }); // Initial position with sensitivity to itself
		x0(1) = DualNumber_T<double, 2>(0.0, { 0.0, 1.0 }); // Initial velocity with sensitivity to itself
		VecX_T<DualNumber_T<double, 2>> r = integrateToTime(stepFn, x0, 2.0 * PI_d, N);
		double energy = 0.5 * (r(0).real * r(0).real + r(1).real * r(1).real);

		ASSERT_TRUE(std::abs(r(0).real - 1.0) < stateTol, "Harmonic oscillator position error too large");
		ASSERT_TRUE(std::abs(r(1).real - 0.0) < stateTol, "Harmonic oscillator velocity error too large");
		ASSERT_TRUE(std::abs(r(0).dual[0] - 1.0) < sensitivityTol, "Harmonic oscillator position sensitivity error too large");
		ASSERT_TRUE(std::abs(r(1).dual[1] - 1.0) < sensitivityTol, "Harmonic oscillator velocity sensitivity error too large");
		ASSERT_TRUE(std::abs(r(0).dual[1]) < sensitivityTol, "Harmonic oscillator position sensitivity to velocity should be near zero");
		ASSERT_TRUE(std::abs(r(1).dual[0]) < sensitivityTol, "Harmonic oscillator velocity sensitivity to position should be near zero");
		ASSERT_TRUE(std::abs(energy - 0.5) < energyTol, "Harmonic oscillator energy error too large");
	}

	integration::NumericalIntegrator integrator;
}

// Test that the Euler method correctly integrates the exponential decay ODE, and that the dual number derivatives match the analytical derivatives.
TEST("AD Euler Method", Euler_ExponentialDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
		) {
		return integrator.eulerStep(x, t, dt, expDecay<double, 1>);
	};

	verifyExpDecaySensitivity(stepFn, 1e-2, 1000);
}
// Test that the Euler method correctly integrates a simple harmonic oscillator, and that the dual number derivatives match the analytical derivatives.
TEST("AD Euler Method", Euler_HarmonicOscillatorSensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 2>>& x,
		DualNumber_T<double, 2> t,
		DualNumber_T<double, 2> dt
		) {
		return integrator.eulerStep(x, t, dt, harmonicOsc<double, 2>);
	};
	verifyHarmonicOscillatorSensitivity(stepFn, 5e-2, 5e-2, 1e-1, 2000);
}

// Tests that the Midpoint method (RK2) correctly integrates the exponential decay ODE, and that the dual number derivatives match the analytical derivatives.
TEST("AD Midpoint Method (RK2)", Midpoint_ExponentialDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
		) {
		return integrator.midpointStep(x, t, dt, expDecay<double, 1>);
	};
	verifyExpDecaySensitivity(stepFn, 1e-5, 500);
}
// Test that the Midpoint method (RK2) correctly integrates a simple harmonic oscillator, and that the dual number derivatives match the analytical derivatives.
TEST("AD Midpoint Method (RK2)", Midpoint_HarmonicOscillatorSensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 2>>& x,
		DualNumber_T<double, 2> t,
		DualNumber_T<double, 2> dt
		) {
		return integrator.midpointStep(x, t, dt, harmonicOsc<double, 2>);
	};
	verifyHarmonicOscillatorSensitivity(stepFn, 5e-4, 5e-4, 5e-3, 1000);
}

// Tests that the Heun method (RK2) correctly integrates the exponential decay ODE, and that the dual number derivatives match the analytical derivatives.
TEST("AD Heun Method (RK2)", Heun_ExponentialDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
		) {
		return integrator.heunStep(x, t, dt, expDecay<double, 1>);
	};
	verifyExpDecaySensitivity(stepFn, 1e-5, 500);
}
// Test that the Heun method (RK2) correctly integrates a simple harmonic oscillator, and that the dual number derivatives match the analytical derivatives.
TEST("AD Heun Method (RK2)", Heun_HarmonicOscillatorSensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 2>>& x,
		DualNumber_T<double, 2> t,
		DualNumber_T<double, 2> dt
		) {
		return integrator.heunStep(x, t, dt, harmonicOsc<double, 2>);
	};
	verifyHarmonicOscillatorSensitivity(stepFn, 5e-4, 5e-4, 5e-3, 1000);
}

// Tests that the Ralston method (RK2) correctly integrates the exponential decay ODE, and that the dual number derivatives match the analytical derivatives.
TEST("AD Ralston Method (RK2)", Ralston_ExponentialDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
		) {
		return integrator.ralstonStep(x, t, dt, expDecay<double, 1>);
	};
	verifyExpDecaySensitivity(stepFn, 1e-5, 500);
}
// Test that the Ralston method (RK2) correctly integrates a simple harmonic oscillator, and that the dual number derivatives match the analytical derivatives.
TEST("AD Ralston Method (RK2)", Ralston_HarmonicOscillatorSensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 2>>& x,
		DualNumber_T<double, 2> t,
		DualNumber_T<double, 2> dt
		) {
		return integrator.ralstonStep(x, t, dt, harmonicOsc<double, 2>);
	};
	verifyHarmonicOscillatorSensitivity(stepFn, 5e-4, 5e-4, 5e-3, 1000);
}

// Tests that the RK4 method correctly integrates the exponential decay ODE, and that the dual number derivatives match the analytical derivatives.
TEST("AD RK4 Method", RK4_ExponentialDecaySensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 1>>& x,
		DualNumber_T<double, 1> t,
		DualNumber_T<double, 1> dt
		) {
		return integrator.rk4Step(x, t, dt, expDecay<double, 1>);
	};
	verifyExpDecaySensitivity(stepFn, 1e-6, 200);
}
// Test that the RK4 method correctly integrates a simple harmonic oscillator, and that the dual number derivatives match the analytical derivatives.
TEST("AD RK4 Method", RK4_HarmonicOscillatorSensitivity) {
	auto stepFn = [&](
		const VecX_T<DualNumber_T<double, 2>>& x,
		DualNumber_T<double, 2> t,
		DualNumber_T<double, 2> dt
		) {
		return integrator.rk4Step(x, t, dt, harmonicOsc<double, 2>);
	};
	verifyHarmonicOscillatorSensitivity(stepFn, 1e-5, 1e-5, 1e-6, 100);
}

// RK45 method is not tested due to further complexity with using an adapative step, implementation will be tested separately LATER once confirmed these work.