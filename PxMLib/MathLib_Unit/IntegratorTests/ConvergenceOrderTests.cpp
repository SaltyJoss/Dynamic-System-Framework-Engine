// File:   ConvergenceOrderTests.cpp
// GitHub: SaltyJoss
// Verifies the expected convergence order of each integration method.

#include "TestHarness.h"
#include <integrators/numerical_integrators.h>
#include <cmath>
#include <functional>
#include <string>

using namespace mathlib;

namespace {
	// Simple Exponential Decay Function: dx/dt = -x, which should decay to exp(-T) at t=T
	auto convExpDecay = [](double, const VecX& x) -> VecX { return -x; };

	// Jacobian of the exponential decay function, which is just -1 for this simple case
	auto convExpDecayJac = [](const VecX& x, MatX& F_out) {
		F_out.setZero(1, 1);
		F_out(0, 0) = -1.0;
	};

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
	void verifyOrder(StepFn stepFn, double T, int Nc, double expected, double tol, const std::string& msg = "") {
		double ec = computeError(stepFn, T, Nc);
		double ef = computeError(stepFn, T, Nc * 2);
		if (ef < 1e-15) test::assertTrue(false, "Error too small to measure convergence");
		double observed = std::log2(ec / ef);
		std::string fullMsg;
		if (!msg.empty()) fullMsg = msg + std::string(", observed: ") + std::to_string(observed);
		else fullMsg = std::string("Observed order: ") + std::to_string(observed);
		test::assertTrue(std::abs(observed - expected) < tol, fullMsg.c_str());
	}

	integration::NumericalIntegrator integrator;
}

// Forward Euler Convergence Order Test
TEST("Euler Method", Euler_Order1) {
	verifyOrder([](const VecX& x, double t, double dt) {
		return integrator.eulerStep(x, t, dt, convExpDecay);
	}, 1.0, 500, 1.0, 0.3, "Euler should be order 1");
}
// Midpoint (RK2) Convergence Order Test
TEST("Midpoint Method (RK2)", Midpoint_Order2) {
	verifyOrder([](const VecX& x, double t, double dt) {
		return integrator.midpointStep(x, t, dt, convExpDecay);
	}, 1.0, 200, 2.0, 0.3, "Midpoint should be order 2");
}
// Heun (RK2) Convergence Order Test
TEST("Heun Method (RK2)", Heun_Order2) {
	verifyOrder([](const VecX& x, double t, double dt) {
		return integrator.heunStep(x, t, dt, convExpDecay);
	}, 1.0, 200, 2.0, 0.3, "Heun should be order 2");
}
// Ralston (RK2) Convergence Order Test
TEST("Ralston Method (RK2)", Ralston_Order2) {
	verifyOrder([](const VecX& x, double t, double dt) {
		return integrator.ralstonStep(x, t, dt, convExpDecay);
	}, 1.0, 200, 2.0, 0.3, "Ralston should be order 2");
}
// RK4 Convergence Order Test
TEST("RK4 Method", RK4_Order4) {
	verifyOrder([](const VecX& x, double t, double dt) {
		return integrator.rk4Step(x, t, dt, convExpDecay);
	}, 1.0, 50, 4.0, 0.3, "RK4 should be order 4");
}

// Implicit Euler Convergence Order Test
TEST("Implicit Euler Method", ImplicitEuler_Order1) {
	verifyOrder([](const VecX& x, double t, double dt) {
		return integrator.implicitEuler(x, t, dt, convExpDecay);
	}, 1.0, 500, 1.0, 0.3, "Implicit Euler should be order 1");
}
// Implicit Midpoint Convergence Order Test
TEST("Implicit Midpoint Method", ImplicitMidpoint_Order2)
{
	verifyOrder([](const VecX& x, double t, double dt) {
		return integrator.implicitMidpoint(x, t, dt, convExpDecay);
	}, 1.0, 200, 2.0, 0.3, "Implicit Midpoint should be order 2");
}
// GLRK2 Convergence Order Test
TEST("GLRK2 Method", GLRK2_Order4) {
	verifyOrder([](const VecX& x, double t, double dt) {
		return integrator.GLRK2(x, t, dt, convExpDecay, convExpDecayJac, 50, 1e-10);
	}, 1.0, 50, 4.0, 0.3, "GLRK2 should be order 4");
}
// GLRK3 Convergence Order Test
TEST("GLRK3 Method", GLRK3_Order6) {
	verifyOrder([](const VecX& x, double t, double dt) {
		return integrator.GLRK3(x, t, dt, convExpDecay, convExpDecayJac, 150, 1e-16);
	}, 10.0, 10, 6.0, 0.5, "GLRK3 should be order 6");
}