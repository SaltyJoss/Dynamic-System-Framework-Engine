#include "pch.h"
// File:   IntegrationService.cpp
// GitHub: SaltyJoss
#include "Numerics/IntegrationService.h"

using namespace mathlib;

namespace integration {
	// Helper to trim whitespace from string
	static inline std::string trimCopy(std::string s) {
		auto notSpace = [](unsigned char c) { return !std::isspace(c); };
		s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
		s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
		return s;
	}

	// Helper to convert string to uppercase
	static inline std::string upperCopy(std::string s) {
		std::transform(s.begin(), s.end(), s.begin(),
			[](unsigned char c) { return (unsigned char)std::toupper(c); });
		return s;
	}

	// Helper to convert string to lowercase
	static inline std::string lowerCopy(std::string s) {
		std::transform(s.begin(), s.end(), s.begin(),
			[](unsigned char c) { return (unsigned char)std::tolower(c); });
		return s;
	}

	// Helper to convert integration method enum to string
	const char* IntegrationService::toString(eIntegrationMethod m) {
		switch (m) {
			case eIntegrationMethod::Euler: return "euler";
			case eIntegrationMethod::Midpoint: return "midpoint";
			case eIntegrationMethod::Heun: return "heun";
			case eIntegrationMethod::Ralston: return "ralston";
			case eIntegrationMethod::RK4: return "rk4";
			case eIntegrationMethod::RK45: return "rk45";
			case eIntegrationMethod::ImplicitEuler: return "implicit_euler";
			case eIntegrationMethod::ImplicitMidpoint: return "implicit_midpoint";
			case eIntegrationMethod::GLRK2: return "glrk2";
			case eIntegrationMethod::GLRK3: return "glrk3";
			default: return "Unknown method";
		}
	}

	const char* DifferentiableIntegrator::toString(eAutoDiffIntegrationMethod m) {
		switch (m) {
			case eAutoDiffIntegrationMethod::AD_ImplicitEuler: return "ad_implicit_euler";
			case eAutoDiffIntegrationMethod::AD_ImplicitMidpoint: return "ad_implicit_midpoint";
			case eAutoDiffIntegrationMethod::AD_GLRK2: return "ad_glrk2";
			case eAutoDiffIntegrationMethod::AD_GLRK3: return "ad_glrk3";
			default: return "Unknown method"; 
		}
	}

	// Get integrator name
	const std::string IntegrationService::IntegratorName(eIntegrationMethod m) { return std::string(toString(m)); }
	const std::string DifferentiableIntegrator::IntegratorName(eAutoDiffIntegrationMethod m) { return std::string(toString(m)); }
	
	// Constructor
	IntegrationService::IntegrationService()
		: _integrator(std::make_unique<integration::NumericalIntegrator>()), method(eIntegrationMethod::RK4),
		_state(std::make_shared<IntegratorState>()), _rtol(1e-3), _atol(1e-6), _dt_last(), _dt_max() {
	}

	// Constructor for autodiff
	DifferentiableIntegrator::DifferentiableIntegrator()
		: _integrator(std::make_unique<integration::NumericalIntegrator>()), _m(eAutoDiffIntegrationMethod::AD_ImplicitEuler),
		_state(std::make_shared<IntegratorState>()) {
		LOG_INFO("DifferentiableIntegrator constructed. Default=%d", (int)_m);
	}
} // namespace integration