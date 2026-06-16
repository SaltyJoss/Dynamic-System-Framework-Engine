// DSFE_Core IntegrationMethods.h
#pragma once

namespace integration {
	enum class eIntegrationMethod {
		Euler = 0,		// First-Order Euler Method
		Midpoint = 1,	// Second-Order Runge-Kutta (Midpoint)
		Heun = 2,		// Second-Order Runge-Kutta (Heun)
		Ralston = 3,	// Second-Order Runge-Kutta (Ralston)
		RK4 = 4,		// Fourth-Order Runge-Kutta 
		RK45 = 5,		// RK45 Method with Adaptive Step Size (Dormand-Prince)
		ImplicitEuler = 6,	 // Implicit Euler Method (implicit)
		ImplicitMidpoint = 7, // Implicit Midpoint Method (implicit)
		GLRK2 = 8,		// Gauss-Legendre Runge-Kutta Method (2 stages, 4th order, implicit)
		GLRK3 = 9		// Gauss-Legendre Runge-Kutta Method (3 stages, 6th order, implicit)
	};

	enum class eAutoDiffIntegrationMethod {
		AD_ImplicitEuler = 0,
		AD_ImplicitMidpoint = 1,
		AD_GLRK2 = 2,
		AD_GLRK3 = 3
	};

	inline bool isStandardMethod(eIntegrationMethod method) {
		auto val = static_cast<int>(method);
		return val >= static_cast<int>(eIntegrationMethod::Euler) && val <= static_cast<int>(eIntegrationMethod::GLRK3);
	}
}