// File:   ExplicitMethodTests.cpp
// GitHub: SaltyJoss
// Tests for explicit integration methods: Euler, Midpoint, Heun, Ralston, RK4

#include "TestHarness.h"
#include <integrators/numerical_integrators.h>
#include <cmath>
#include <functional>

using namespace mathlib;

namespace {
	// Simple Exponential Decay Function: dx/dt = -x, which should decay to exp(-1) at t=1
	auto expDecay = [](double, const VecX& x) -> VecX { return -x; };

	// Simple Harmonic Oscillator: dx/dt = v, dv/dt = -x, which should preserve energy
	auto harmonicOsc = [](double, const VecX& s) -> VecX {
		VecX d(2);
		d(0) = s(1);
		d(1) = -s(0);
		return d;
		};

	// Helper function to integrate from t=0 to t=T using N steps of the given step function
	template<typename StepFn>
	VecX integrateToTime(StepFn stepFn, const VecX& x0, double T, int N) {
		double dt = T / N;
		VecX x = x0;
		double t = 0.0;
		for (int i = 0; i < N; ++i) { x = stepFn(x, t, dt); t += dt; }
		return x;
	}

	integration::NumericalIntegrator integrator;
}

// Forward Euler Tests
// Exponential Decay Test
TEST("Euler Method", Euler_ExponentialDecay_SmallStep)
{
	VecX x0(1); x0 << 1.0;
	VecX r = integrateToTime([](const VecX& x, double t, double dt) { return integrator.eulerStep(x, t, dt, expDecay); }, x0, 1.0, 1000);
	ASSERT_TRUE(std::abs(r(0) - std::exp(-1.0)) < 1e-2, "Euler exponential decay error too large");
}
// Simple Harmonic Oscillator Test
TEST("Euler Method", Euler_HarmonicOscillator_SmallStep)
{
	VecX x0(2); x0 << 1.0, 0.0;
	VecX r = integrateToTime([](const VecX& x, double t, double dt) { return integrator.eulerStep(x, t, dt, harmonicOsc); }, x0, 2.0, 2000);
	ASSERT_TRUE(std::abs(r(0) - std::cos(2.0)) < 5e-2, "Euler SHO position error too large");
	ASSERT_TRUE(std::abs(r(1) + std::sin(2.0)) < 5e-2, "Euler SHO velocity error too large");
}

// Midpoint Method (RK2) Tests
// Exponential Decay Test
TEST("Midpoint Method (RK2)", Midpoint_ExponentialDecay)
{
	VecX x0(1); x0 << 1.0;
	VecX r = integrateToTime([](const VecX& x, double t, double dt) { return integrator.midpointStep(x, t, dt, expDecay); }, x0, 1.0, 500);
	ASSERT_TRUE(std::abs(r(0) - std::exp(-1.0)) < 1e-5, "Midpoint exponential decay error too large");
}
// Simple Harmonic Oscillator Test
TEST("Midpoint Method (RK2)", Midpoint_HarmonicOscillator)
{
	VecX x0(2); x0 << 1.0, 0.0;
	VecX r = integrateToTime([](const VecX& x, double t, double dt) { return integrator.midpointStep(x, t, dt, harmonicOsc); }, x0, 2.0, 1000);
	ASSERT_TRUE(std::abs(r(0) - std::cos(2.0)) < 1e-4, "Midpoint SHO position error too large");
	ASSERT_TRUE(std::abs(r(1) + std::sin(2.0)) < 1e-4, "Midpoint SHO velocity error too large");
}

// Heun's Method (RK2) Tests
// Exponential Decay Test
TEST("Heun Method (RK2)", Heun_ExponentialDecay)
{
	VecX x0(1); x0 << 1.0;
	VecX r = integrateToTime([](const VecX& x, double t, double dt) { return integrator.heunStep(x, t, dt, expDecay); }, x0, 1.0, 500);
	ASSERT_TRUE(std::abs(r(0) - std::exp(-1.0)) < 1e-5, "Heun exponential decay error too large");
}
// Simple Harmonic Oscillator Test
TEST("Heun Method (RK2)", Heun_HarmonicOscillator)
{
	VecX x0(2); x0 << 1.0, 0.0;
	VecX r = integrateToTime([](const VecX& x, double t, double dt) { return integrator.heunStep(x, t, dt, harmonicOsc); }, x0, 2.0, 1000);
	ASSERT_TRUE(std::abs(r(0) - std::cos(2.0)) < 1e-4, "Heun SHO position error too large");
	ASSERT_TRUE(std::abs(r(1) + std::sin(2.0)) < 1e-4, "Heun SHO velocity error too large");
}

// Ralston's Method (RK2) Tests
// Exponential Decay Test
TEST("Ralston Method (RK2)", Ralston_ExponentialDecay)
{
	VecX x0(1); x0 << 1.0;
	VecX r = integrateToTime([](const VecX& x, double t, double dt) { return integrator.ralstonStep(x, t, dt, expDecay); }, x0, 1.0, 500);
	ASSERT_TRUE(std::abs(r(0) - std::exp(-1.0)) < 1e-5, "Ralston exponential decay error too large");
}
// Simple Harmonic Oscillator Test
TEST("Ralston Method (RK2)", Ralston_HarmonicOscillator)
{
	VecX x0(2); x0 << 1.0, 0.0;
	VecX r = integrateToTime([](const VecX& x, double t, double dt) { return integrator.ralstonStep(x, t, dt, harmonicOsc); }, x0, 2.0, 1000);
	ASSERT_TRUE(std::abs(r(0) - std::cos(2.0)) < 1e-4, "Ralston SHO position error too large");
	ASSERT_TRUE(std::abs(r(1) + std::sin(2.0)) < 1e-4, "Ralston SHO velocity error too large");
}

// RK4 Method Tests
// Exponential Decay Test
TEST("RK4 Method", RK4_ExponentialDecay)
{
	VecX x0(1); x0 << 1.0;
	VecX r = integrateToTime([](const VecX& x, double t, double dt) { return integrator.rk4Step(x, t, dt, expDecay); }, x0, 1.0, 100);
	ASSERT_TRUE(std::abs(r(0) - std::exp(-1.0)) < 1e-9, "RK4 exponential decay error too large");
}
// Simple Harmonic Oscillator Test
TEST("RK4 Method", RK4_HarmonicOscillator)
{
	VecX x0(2); x0 << 1.0, 0.0;
	VecX r = integrateToTime([](const VecX& x, double t, double dt) { return integrator.rk4Step(x, t, dt, harmonicOsc); }, x0, 2.0, 200);
	ASSERT_TRUE(std::abs(r(0) - std::cos(2.0)) < 1e-9, "RK4 SHO position error too large");
	ASSERT_TRUE(std::abs(r(1) + std::sin(2.0)) < 1e-9, "RK4 SHO velocity error too large");
}
// RK4 with larger step size to test stability
TEST("RK4 Method", RK4_ExponentialDecay_LargeStep)
{
	VecX x0(1); x0 << 1.0;
	VecX r = integrateToTime([](const VecX& x, double t, double dt) { return integrator.rk4Step(x, t, dt, expDecay); }, x0, 1.0, 10);
	ASSERT_TRUE(std::abs(r(0) - std::exp(-1.0)) < 1e-5, "RK4 large-step exponential decay error too large");
}