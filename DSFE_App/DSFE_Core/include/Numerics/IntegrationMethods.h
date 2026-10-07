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

	inline std::string methodToString(eIntegrationMethod method) {
		switch (method) {
			case eIntegrationMethod::Euler: return "Euler";
			case eIntegrationMethod::Midpoint: return "Midpoint";
			case eIntegrationMethod::Heun: return "Heun";
			case eIntegrationMethod::Ralston: return "Ralston";
			case eIntegrationMethod::RK4: return "RK4";
			case eIntegrationMethod::RK45: return "RK45";
			case eIntegrationMethod::ImplicitEuler: return "Implicit Euler";
			case eIntegrationMethod::ImplicitMidpoint: return "Implicit Midpoint";
			case eIntegrationMethod::GLRK2: return "GLRK2";
			case eIntegrationMethod::GLRK3: return "GLRK3";
			default: return "Unknown Method";
		}
	}

	inline std::string ADMethodToString(eAutoDiffIntegrationMethod method) {
		switch (method) {
			case eAutoDiffIntegrationMethod::AD_ImplicitEuler: return "AD Implicit Euler";
			case eAutoDiffIntegrationMethod::AD_ImplicitMidpoint: return "AD Implicit Midpoint";
			case eAutoDiffIntegrationMethod::AD_GLRK2: return "AD GLRK2";
			case eAutoDiffIntegrationMethod::AD_GLRK3: return "AD GLRK3";
			default: return "Unknown Method";
		}
	}

	inline bool isStandardMethod(eIntegrationMethod method) {
		auto val = static_cast<int>(method);
		return val >= static_cast<int>(eIntegrationMethod::Euler) && val <= static_cast<int>(eIntegrationMethod::GLRK3);
	}
}