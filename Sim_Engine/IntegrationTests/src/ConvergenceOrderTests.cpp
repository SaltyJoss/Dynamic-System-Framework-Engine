// File:   ConvergenceOrderTests.cpp
// GitHub: SaltyJoss
// Verifies the expected convergence order of each integration method.

#include "TestHarness.h"
#include <integrators/numerical_integrators.h>
#include <cmath>
#include <functional>

using namespace mathlib;

namespace {
	// Simple Exponential Decay Function: dx/dt = -x, which should decay to exp(-T) at t=T
	auto convExpDecay = [](double, const VecX& x) -> VecX { return -x; };

	// Computes the error of a given step function by integrating from t=0 to t=T and comparing to the exact solution exp(-T)
	template<typename StepFn>
	double computeError(StepFn stepFn, double T, int N) {
		double dt = T / N;
		VecX x(1); x << 1.0;
		double t = 0.0;
		for (int i = 0; i < N; ++i) { x = stepFn(x, t, dt); t += dt; }
		return std::abs(x(0) - std::exp(-T));
	}

	// Verifies that the observed convergence order is close to the expected order within a tolerance
	template<typename StepFn>
	void verifyOrder(StepFn stepFn, double T, int Nc, double expected, double tol, const char* msg) {
		double ec = computeError(stepFn, T, Nc);
		double ef = computeError(stepFn, T, Nc * 2);
		if (ef < 1e-15) return;
		double observed = std::log2(ec / ef);
		test::assertTrue(std::abs(observed - expected) < tol, msg);
	}

	integration::ODE ode;
}

// Convergence Order Tests
// Forward Euler Convergence Order Test
TEST("Convergence Order", Euler_Order1)
{
	verifyOrder([](const VecX& x, double t, double dt) { return ode.eulerStep(x, t, dt, convExpDecay); },
	1.0, 500, 1.0, 0.3, "Euler should be order 1");
}
// Midpoint (RK2) Convergence Order Test
TEST("Convergence Order", Midpoint_Order2)
{
	verifyOrder([](const VecX& x, double t, double dt) { return ode.midpointStep(x, t, dt, convExpDecay); },
	1.0, 200, 2.0, 0.3, "Midpoint should be order 2");
}
// Heun (RK2) Convergence Order Test
TEST("Convergence Order", Heun_Order2)
{
	verifyOrder([](const VecX& x, double t, double dt) { return ode.heunStep(x, t, dt, convExpDecay); },
	1.0, 200, 2.0, 0.3, "Heun should be order 2");
}
// Ralston (RK2) Convergence Order Test
TEST("Convergence Order", Ralston_Order2)
{
	verifyOrder([](const VecX& x, double t, double dt) { return ode.ralstonStep(x, t, dt, convExpDecay); },
	1.0, 200, 2.0, 0.3, "Ralston should be order 2");
}
// RK4 Convergence Order Test
TEST("Convergence Order", RK4_Order4)
{
	verifyOrder([](const VecX& x, double t, double dt) { return ode.rk4Step(x, t, dt, convExpDecay); },
	1.0, 50, 4.0, 0.3, "RK4 should be order 4");
}