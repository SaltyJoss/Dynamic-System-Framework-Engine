// File:   ImplicitMethodTests.cpp
// GitHub: SaltyJoss
// Tests for implicit integration methods: Backward Euler, Implicit Midpoint

#include "TestHarness.h"
#include <integrators/numerical_integrators.h>
#include <cmath>
#include <functional>

using namespace mathlib;

namespace {
	// Simple Exponential Decay Function: dx/dt = -x, which should decay to exp(-1) at t=1
	auto impExpDecay = [](double, const VecX& x) -> VecX { return -x; };

	// Simple Harmonic Oscillator: dx/dt = v, dv/dt = -x, which should preserve energy
	auto impHarmonicOsc = [](double, const VecX& s) -> VecX {
		VecX d(2);
		d(0) = s(1);
		d(1) = -s(0);
		return d;
	};

	// Simple Linear Decay Function: dx/dt = -2x, which should decay to exp(-2) at t=1
	auto linearDecay = [](double, const VecX& x) -> VecX { return -2.0 * x; };

	integration::ODE ode;
}

// Backward Euler Tests
// Exponential Decay Test
TEST("Backward Euler Method", BackwardEuler_ExponentialDecay)
{
	VecX x(1); x << 1.0;
	double dt = 1.0 / 1000, t = 0.0;
	for (int i = 0; i < 1000; ++i) { x = ode.backward_euler(x, t, dt, impExpDecay); t += dt; }
	ASSERT_TRUE(std::abs(x(0) - std::exp(-1.0)) < 1e-2, "Backward Euler exponential decay error too large");
}
// Linear Decay Test
TEST("Backward Euler Method", BackwardEuler_LinearDecay)
{
	VecX x(1); x << 1.0;
	double dt = 0.5 / 500, t = 0.0;
	for (int i = 0; i < 500; ++i) { x = ode.backward_euler(x, t, dt, linearDecay); t += dt; }
	ASSERT_TRUE(std::abs(x(0) - std::exp(-1.0)) < 1e-2, "Backward Euler linear decay error too large");
}
// Stability Test with Large Time Step
TEST("Backward Euler Method", BackwardEuler_Stability_LargeStep)
{
	VecX x(1); x << 1.0;
	double dt = 2.0 / 20, t = 0.0;
	for (int i = 0; i < 20; ++i) { x = ode.backward_euler(x, t, dt, impExpDecay); t += dt; }
	ASSERT_TRUE(x(0) > 0.0,          "Backward Euler should produce positive result for decay");
	ASSERT_TRUE(x(0) < 1.0,          "Backward Euler result should be less than initial value");
	ASSERT_TRUE(std::isfinite(x(0)), "Backward Euler result should be finite");
}

// Implicit Midpoint Tests
// Exponential Decay Test
TEST("Implicit Midpoint Method", ImplicitMidpoint_ExponentialDecay)
{
	VecX x(1); x << 1.0;
	double dt = 1.0 / 1000, t = 0.0;
	for (int i = 0; i < 1000; ++i) { x = ode.implicit_midpoint(x, t, dt, impExpDecay); t += dt; }
	ASSERT_TRUE(std::abs(x(0) - std::exp(-1.0)) < 1e-3, "Implicit Midpoint exponential decay error too large");
}
// Energy Preservation Test for Simple Harmonic Oscillator
TEST("Implicit Midpoint Method", ImplicitMidpoint_HarmonicOscillator_EnergyPreservation)
{
	VecX x(2); x << 1.0, 0.0;
	double E0 = 0.5 * (x(0)*x(0) + x(1)*x(1));
	double dt = 5.0 / 2500, t = 0.0;
	for (int i = 0; i < 2500; ++i) { x = ode.implicit_midpoint(x, t, dt, impHarmonicOsc); t += dt; }
	double Ef = 0.5 * (x(0)*x(0) + x(1)*x(1));
	ASSERT_TRUE(std::abs(Ef - E0) / E0 < 1e-4, "Implicit Midpoint energy drift too large for SHO");
}
// Stability Test with Large Time Step
TEST("Implicit Midpoint Method", ImplicitMidpoint_Stability_LargeStep)
{
	VecX x(1); x << 1.0;
	double dt = 2.0 / 20, t = 0.0;
	for (int i = 0; i < 20; ++i) { x = ode.implicit_midpoint(x, t, dt, impExpDecay); t += dt; }
	ASSERT_TRUE(x(0) > 0.0,          "Implicit Midpoint should produce positive result for decay");
	ASSERT_TRUE(x(0) < 1.0,          "Implicit Midpoint result should be less than initial value");
	ASSERT_TRUE(std::isfinite(x(0)), "Implicit Midpoint result should be finite");
}