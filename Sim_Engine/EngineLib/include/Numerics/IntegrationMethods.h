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
		ImplicitMidpoint = 7 // Implicit Midpoint Method (implicit)
	};
}