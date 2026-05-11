// File:   ImplicitMethodTests.cpp
// GitHub: SaltyJoss
// Tests for implicit integration methods: Backward Euler, Implicit Midpoint

#include "TestHarness.h"
#include <integrators/numerical_integrators.h>
#include <cmath>
#include <functional>

using namespace mathlib;

namespace {
	constexpr double tol_low = 1e-3;
	constexpr double tol_high = 1e-6;

	// Simple Exponential Decay Function: dx/dt = -x, which should decay to exp(-1) at t=1
	auto impExpDecay = [](double, const VecX& x) -> VecX { return -x; };

	// Jacobian for the Exponential Decay Function: dF/dx = -1
	auto impExpDecayJac = [](const VecX& x, MatX& F_out) {
		F_out.setZero(1, 1);
		F_out(0, 0) = -1.0;
	};

	// Simple Harmonic Oscillator: dx/dt = v, dv/dt = -x, which should preserve energy
	auto impHarmonicOsc = [](double, const VecX& s) -> VecX {
		VecX d(2);
		d(0) = s(1);
		d(1) = -s(0);
		return d;
	};

	// Jacobian for the Simple Harmonic Oscillator: dF/ds = [[0, 1], [-1, 0]]
	auto impHarmonicOscJac = [](const VecX& s, MatX& F_out) {
		F_out.setZero(2, 2);
		F_out(0, 1) = 1.0;
		F_out(1, 0) = -1.0;
	};

	// Simple Linear Decay Function: dx/dt = -2x, which should decay to exp(-2) at t=1
	auto linearDecay = [](double, const VecX& x) -> VecX { return -2.0 * x; };

	// Jacobian for the Linear Decay Function: dF/dx = -2
	auto linearDecayJac = [](const VecX& x, MatX& F_out) {
		F_out.setZero(1, 1);
		F_out(0, 0) = -2.0;
	};

	// Stiff Decay Function: dx/dt = -100x, which should decay to exp(-100) at t=1
	auto stiffDecayJac = [](const VecX& x, MatX& F_out) {
		F_out.setZero(1, 1);
		F_out(0, 0) = -100.0;
	};

	integration::ODE ode;
}

// Backward Euler Tests
// Exponential Decay Test
TEST("Implicit Euler Method", ImplicitEuler_ExponentialDecay) {
	VecX x(1); x << 1.0;
	double dt = 1.0 / 1000, t = 0.0;
	for (int i = 0; i < 1000; ++i) { x = ode.implicit_euler(x, t, dt, impExpDecay); t += dt; }
	ASSERT_TRUE(std::abs(x(0) - std::exp(-1.0)) < 1e-2, "Backward Euler exponential decay error too large");
}
// Linear Decay Test
TEST("Implicit Euler Method", ImplicitEuler_LinearDecay) {
	VecX x(1); x << 1.0;
	double T = 0.5, dt = T / 500, t = 0.0;
	for (int i = 0; i < 500; ++i) { x = ode.implicit_euler(x, t, dt, linearDecay); t += dt; }
	ASSERT_TRUE(std::abs(x(0) - std::exp(-1.0)) < 1e-2, "Backward Euler linear decay error too large");
}
// Stability Test with Large Time Step
TEST("Implicit Euler Method", ImplicitEuler_Stability_LargeStep) {
	VecX x(1); x << 1.0;
	double T = 2.0, dt = T / 20, t = 0.0;
	double prev = x(0);
	for (int i = 0; i < 20; ++i) {
		x = ode.implicit_euler(x, t, dt, impExpDecay);
		t += dt;
		ASSERT_TRUE(x(0) < prev * (1.0 + 1e-12), "Not monotone");
		ASSERT_TRUE(x(0) < prev, "Not strictly decaying");
		prev = x(0);
	}
}

// Implicit Midpoint Tests
// Exponential Decay Test
TEST("Implicit Midpoint Method", ImplicitMidpoint_ExponentialDecay) {
	VecX x(1); x << 1.0;
	double dt = 1.0 / 1000, t = 0.0;
	for (int i = 0; i < 1000; ++i) { x = ode.implicit_midpoint(x, t, dt, impExpDecay); t += dt; }
	ASSERT_TRUE(std::abs(x(0) - std::exp(-1.0)) < 1e-2, "Implicit Midpoint exponential decay error too large");
}
// Energy Preservation Test for Simple Harmonic Oscillator
TEST("Implicit Midpoint Method", ImplicitMidpoint_HarmonicOscillator_EnergyPreservation) {
	VecX x(2); x << 1.0, 0.0;
	double E0 = 0.5 * (x(0)*x(0) + x(1)*x(1));
	double dt = 5.0 / 2500, t = 0.0;
	for (int i = 0; i < 2500; ++i) { x = ode.implicit_midpoint(x, t, dt, impHarmonicOsc); t += dt; }
	double Ef = 0.5 * (x(0)*x(0) + x(1)*x(1));
	double drift = std::abs(Ef - E0);
	ASSERT_TRUE(drift < tol_high * (1.0 + E0), "Implicit Midpoint energy drift too large for SHO");
}
// Stability Test with Large Time Step
TEST("Implicit Midpoint Method", ImplicitMidpoint_Stability_LargeStep) {
	VecX x(1); x << 1.0;
	double T = 2.0, dt = T / 20, t = 0.0;
	double prev = x(0);
	for (int i = 0; i < 20; ++i) {
		x = ode.implicit_midpoint(x, t, dt, impExpDecay);
		t += dt;
		ASSERT_TRUE(x(0) < prev * (1.0 + 1e-12), "Not monotone");
		ASSERT_TRUE(x(0) < prev, "Not strictly decaying");
		prev = x(0);
	}
}

// Gauss-Legendre Runge-Kutta 2-Stage (GLRK2) Tests
// GLRK2 Test for Simple Harmonic Oscillator
TEST("GLRK2 Method", GLRK2_HarmonicOscillator_EnergyPreservation) {
	VecX x(2); x << 1.0, 0.0;
	double E0 = 0.5 * (x(0) * x(0) + x(1) * x(1));
	double T = 5.0, dt = T / 2500, t = 0.0;
	for (int i = 0; i < 2500; ++i) {
		x = ode.GLRK2(x, t, dt, impHarmonicOsc, 50, 1e-6, impHarmonicOscJac);
		t += dt;
	}
	double Ef = 0.5 * (x(0) * x(0) + x(1) * x(1));
	double drift = std::abs(Ef - E0);
	ASSERT_TRUE(drift < tol_high * (1.0 + E0), "GLRK2 energy drift too large for SHO");
}
// GLRK2 Test for Exponential Decay
TEST("GLRK2 Method", GLRK2_ExponentialDecay) {
	VecX x(1); x << 1.0;
	double T = 1.0, dt = T / 1000, t = 0.0;
	for (int i = 0; i < 1000; ++i) {
		x = ode.GLRK2(x, t, dt, impExpDecay, 50, 1e-6, impExpDecayJac);
		t += dt;
	}
	ASSERT_TRUE(std::abs(x(0) - std::exp(-1.0)) < tol_low, "GLRK2 exponential decay error too large");
}
// GLRK2 Stability Test with Large Time Step
TEST("GLRK2 Method", GLRK2_Stability_LargeStep) {
	VecX x(1); x << 1.0;
	double T = 2.0, dt = T / 20, t = 0.0;
	double prev = x(0);
	for (int i = 0; i < 20; ++i) {
		x = ode.GLRK2(x, t, dt, impExpDecay, 50, 1e-6, impExpDecayJac);
		t += dt;
		ASSERT_TRUE(x(0) < prev * (1.0 + 1e-12), "Not monotone");
		ASSERT_TRUE(x(0) < prev, "Not strictly decaying");
		prev = x(0);
	}
}
// GLRK2 Test for Linear Decay
TEST("GLRK2 Method", GLRK2_LinearDecay) {
	VecX x(1); x << 1.0;
	double T = 0.5, dt = T / 500, t = 0.0;
	for (int i = 0; i < 500; ++i) {
		x = ode.GLRK2(x, t, dt, linearDecay, 50, 1e-6, linearDecayJac);
		t += dt;
	}
	ASSERT_TRUE(std::abs(x(0) - std::exp(-1.0)) < tol_low, "GLRK2 linear decay error too large");
}
// GLRK2 Test for Stiff Decay
TEST("GLRK2 Method", GLRK2_StiffDecay) {
	VecX x(1); x << 1.0;
	double T = 0.5, dt = T / 500, t = 0.0;
	double expected = std::exp(-50.0);
	auto stiff_f = [](double, const VecX& x) { return -100.0 * x; };

	for (int i = 0; i < 500; ++i) {
		x = ode.GLRK2(x, t, dt, stiff_f, 50, 1e-6, stiffDecayJac);
		t += dt;
	}
	ASSERT_TRUE(x(0) > 0.0, "GLRK2 should produce positive result for decay");
	ASSERT_TRUE(std::abs(std::log(x(0)) - std::log(expected)) < tol_high, "GLRK2 stiff decay error too large");
}
// GLRK2 Test for Nonlinear Decay
TEST("GLRK2 Method", GLRK2_NonlinearDecay) {
	VecX x(1); x << 1.0;
	double T = 0.5, dt = T / 500, t = 0.0;
	double expected = 1.0 / (1.0 + T);
	auto nl_f = [](double, const VecX& x) { return -x * x; };

	auto nl_jac = [](const VecX& x, MatX& F_out) {
		F_out.setZero(1, 1);
		F_out(0, 0) = -2.0 * x(0);
	};

	for (int i = 0; i < 500; ++i) {
		x = ode.GLRK2(x, t, dt, nl_f, 50, 1e-6, nl_jac);
		t += dt;
	}
	ASSERT_TRUE(std::abs(x(0) - expected) < tol_low, "GLRK2 nonlinear decay error too large");
}
// GLRK2 Test for Stiff Nonlinear Decay
TEST("GLRK2 Method", GLRK2_StiffNonlinearDecay) {
	VecX x(1); x << 1.0;
	double T = 0.5, dt = T / 500, t = 0.0;
	auto stiff_nl_f = [](double, const VecX& x) { return -100.0 * x * x; };

	auto stiff_nl_jac = [](const VecX& x, MatX& F_out) {
		F_out.setZero(1, 1);
		F_out(0, 0) = -200.0 * x(0);
	};

	for (int i = 0; i < 500; ++i) {
		x = ode.GLRK2(x, t, dt, stiff_nl_f, 50, 1e-6, stiff_nl_jac);
		t += dt;
	}
	ASSERT_TRUE(std::abs(x(0) - (1.0 / 51.0)) < tol_low, "GLRK2 stiff nonlinear decay error too large");
}

// Gauss-Legendre Runge-Kutta 3-Stage (GLRK3) Tests
// GLRK3 Test for Simple Harmonic Oscillator
TEST("GLRK3 Method", GLRK3_HarmonicOscillator_EnergyPreservation) {
	VecX x(2); x << 1.0, 0.0;
	double E0 = 0.5 * (x(0) * x(0) + x(1) * x(1));
	double T = 5.0, dt = T / 2500, t = 0.0;
	for (int i = 0; i < 2500; ++i) {
		x = ode.GLRK3(x, t, dt, impHarmonicOsc, 80, 1e-7, impHarmonicOscJac);
		t += dt;
	}
	double Ef = 0.5 * (x(0) * x(0) + x(1) * x(1));
	double drift = std::abs(Ef - E0);
	ASSERT_TRUE(drift < tol_high * (1.0 + E0), "GLRK3 energy drift too large for SHO");
}
// GLRK3 Test for Exponential Decay
TEST("GLRK3 Method", GLRK3_ExponentialDecay) {
	VecX x(1); x << 1.0;
	double T = 1.0, dt = T / 1000, t = 0.0;
	for (int i = 0; i < 1000; ++i) {
		x = ode.GLRK3(x, t, dt, impExpDecay, 80, 1e-7, impExpDecayJac);
		t += dt;
	}
	ASSERT_TRUE(std::abs(x(0) - std::exp(-1.0)) < tol_low, "GLRK3 exponential decay error too large");
}
// GLRK3 Stability Test with Large Time Step
TEST("GLRK3 Method", GLRK3_Stability_LargeStep) {
	VecX x(1); x << 1.0;
	double T = 2.0, dt = T / 20, t = 0.0;
	double prev = x(0);
	for (int i = 0; i < 20; ++i) {
		x = ode.GLRK3(x, t, dt, impExpDecay, 80, 1e-7, impExpDecayJac);
		t += dt;
		ASSERT_TRUE(x(0) < prev * (1.0 + 1e-12), "Not monotone");
		ASSERT_TRUE(x(0) < prev, "Not strictly decaying");
		prev = x(0);
	}
}
// GLRK3 Test for Linear Decay
TEST("GLRK3 Method", GLRK3_LinearDecay) {
	VecX x(1); x << 1.0;
	double T = 0.5, dt = T / 500, t = 0.0;
	for (int i = 0; i < 500; ++i) {
		x = ode.GLRK3(x, t, dt, linearDecay, 80, 1e-7, linearDecayJac);
		t += dt;
	}
	ASSERT_TRUE(std::abs(x(0) - std::exp(-1.0)) < tol_low, "GLRK3 linear decay error too large");
}
// GLRK3 Test for Stiff Decay
TEST("GLRK3 Method", GLRK3_StiffDecay) {
	VecX x(1); x << 1.0;
	double T = 0.5, dt = T / 500, t = 0.0;
	double expected = std::exp(-50.0);
	auto stiff_f = [](double, const VecX& x) { return -100.0 * x; };

	for (int i = 0; i < 500; ++i) {
		x = ode.GLRK3(x, t, dt, stiff_f, 80, 1e-7, stiffDecayJac);
		t += dt;
	}
	ASSERT_TRUE(x(0) > 0.0, "GLRK3 should produce positive result for decay");
	ASSERT_TRUE(std::abs(std::log(x(0)) - std::log(expected)) < tol_high, "GLRK3 stiff decay error too large");
}
// GLRK3 Test for Nonlinear Decay
TEST("GLRK3 Method", GLRK3_NonlinearDecay) {
	VecX x(1); x << 1.0;
	double T = 0.5, dt = T / 500, t = 0.0;
	double expected = 1.0 / (1.0 + T);
	auto nl_f = [](double, const VecX& x) { return -x * x; };

	// DEFINED: Inline Jacobian tracking -2 * x
	auto nl_jac = [](const VecX& x, MatX& F_out) {
		F_out.setZero(1, 1);
		F_out(0, 0) = -2.0 * x(0);
	};

	for (int i = 0; i < 500; ++i) {
		x = ode.GLRK3(x, t, dt, nl_f, 80, 1e-7, nl_jac);
		t += dt;
	}
	double rel_err = std::abs(x(0) - expected) / expected;
	ASSERT_TRUE(rel_err < tol_low, "GLRK3 nonlinear decay error too large");
}
// GLRK3 Test for Stiff Nonlinear Decay
TEST("GLRK3 Method", GLRK3_StiffNonlinearDecay) {
	VecX x(1); x << 1.0;
	double T = 0.5, dt = T / 500, t = 0.0;
	double expected = 1.0 / 51.0;
	auto stiff_nl_f = [](double, const VecX& x) { return -100.0 * x * x; };

	// DEFINED: Inline Jacobian tracking -200 * x
	auto stiff_nl_jac = [](const VecX& x, MatX& F_out) {
		F_out.setZero(1, 1);
		F_out(0, 0) = -200.0 * x(0);
	};

	for (int i = 0; i < 500; ++i) {
		x = ode.GLRK3(x, t, dt, stiff_nl_f, 80, 1e-7, stiff_nl_jac);
		t += dt;
	}
	double rel_err = std::abs(x(0) - expected) / expected;
	ASSERT_TRUE(rel_err < tol_low, "GLRK3 stiff nonlinear decay error too large");
}