// File:   AdaptiveMethodTests.cpp
// GitHub: SaltyJoss
// Tests for adaptive step-size integration: RK45 (Dormand-Prince)

#include "TestHarness.h"
#include <integrators/numerical_integrators.h>
#include <cmath>
#include <functional>

using namespace mathlib;

namespace {
	// Simple Exponential Decay Function: dx/dt = -x, which should decay to exp(-1) at t=1
	auto rk45ExpDecay = [](double, const VecX& x) -> VecX { return -x; };

	// Simple Harmonic Oscillator: d^2x/dt^2 + x = 0, which should yield cos(t) and -sin(t) for position and velocity
	auto rk45HarmonicOsc = [](double, const VecX& s) -> VecX {
		VecX d(2);
		d(0) = s(1);
		d(1) = -s(0);
		return d;
	};

	// Helper function to integrate from t=0 to t=T using RK45 with adaptive step size
	VecX integrateRK45(integration::NumericalIntegrator& integrator, const VecX& x0, double T, std::function<VecX(double, const VecX&)> f, double rtol, double atol) {
		VecX x = x0;
		double t = 0.0, dt = T / 10.0;
		for (int s = 0; t < T && s < 10000; ++s) {
			double h = std::min(dt, T - t), dt_used = 0.0;
			x = integrator.rk45Step(x, t, h, dt_used, f, rtol, atol);
			t += dt_used;
		}
		return x;
	}

	integration::NumericalIntegrator integrator;
}

// RK45 (Dormand-Prince) Tests
// Exponential Decay Test with Tight Tolerance
TEST("RK45 Adaptive Step-Size", RK45_ExponentialDecay_TightTolerance)
{
	VecX x0(1); x0 << 1.0;
	VecX r = integrateRK45(integrator, x0, 1.0, rk45ExpDecay, 1e-8, 1e-10);
	ASSERT_TRUE(std::abs(r(0) - std::exp(-1.0)) < 1e-6, "RK45 tight-tol exponential decay error too large");
}
// Exponential Decay Test with Loose Tolerance
TEST("RK45 Adaptive Step-Size", RK45_ExponentialDecay_LooseTolerance)
{
	VecX x0(1); x0 << 1.0;
	VecX r = integrateRK45(integrator, x0, 1.0, rk45ExpDecay, 1e-3, 1e-6);
	ASSERT_TRUE(std::abs(r(0) - std::exp(-1.0)) < 1e-2, "RK45 loose-tol exponential decay error too large");
}
// Simple Harmonic Oscillator Test
TEST("RK45 Adaptive Step-Size", RK45_HarmonicOscillator)
{
	VecX x0(2); x0 << 1.0, 0.0;
	VecX r = integrateRK45(integrator, x0, 2.0, rk45HarmonicOsc, 1e-8, 1e-10);
	ASSERT_TRUE(std::abs(r(0) - std::cos(2.0)) < 1e-6, "RK45 SHO position error too large");
	ASSERT_TRUE(std::abs(r(1) + std::sin(2.0)) < 1e-6, "RK45 SHO velocity error too large");
}
// Longer Integration Test
TEST("RK45 Adaptive Step-Size", RK45_LongerIntegration)
{
	VecX x0(1); x0 << 1.0;
	VecX r = integrateRK45(integrator, x0, 5.0, rk45ExpDecay, 1e-6, 1e-9);
	ASSERT_TRUE(std::abs(r(0) - std::exp(-5.0)) < 1e-5, "RK45 longer integration error too large");
}