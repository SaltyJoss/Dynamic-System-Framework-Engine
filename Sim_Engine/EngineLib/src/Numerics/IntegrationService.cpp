#include "pch.h"
#include "Numerics/IntegrationService.h"
#include "EngineLib/LogMacros.h"

using namespace mathlib;

namespace integration {
	// --- HELPER METHODS ---

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

	// Parse integration method from string
	static bool tryParseMethod(std::string s, eIntegrationMethod& out) {
		s = upperCopy(trimCopy(std::move(s)));

		if (s == "EULER")		  { out = eIntegrationMethod::Euler; }
		else if (s == "MIDPOINT") { out = eIntegrationMethod::Midpoint; }
		else if (s == "HEUN")	  { out = eIntegrationMethod::Heun; }
		else if (s == "RALSTON")  { out = eIntegrationMethod::Ralston; }
		else if (s == "RK4" || s == "RK-4" || s == "RUNGEKUTTA4") { out = eIntegrationMethod::RK4; }
		else if (s == "RK45" || s == "RK4(5)" || s == "DOPRI" || s == "DORMANDPRINCE") { out = eIntegrationMethod::RK45; }
		else { return false; }

		return true;
	}

	// Helper to convert integration method enum to string
	const char* IntegrationService::toString(eIntegrationMethod m) {
		switch (m) {
			case eIntegrationMethod::Euler:
				return "Euler";
			case eIntegrationMethod::Midpoint:
				return "Midpoint";
			case eIntegrationMethod::Heun:
				return "Heun";
			case eIntegrationMethod::Ralston:
				return "Ralston";
			case eIntegrationMethod::RK4:
				return "RK4";
			case eIntegrationMethod::RK45:
				return "RK45";
			default:
				return "Unknown";
		}
	}

	// Get integrator name
	const std::string IntegrationService::IntegratorName(eIntegrationMethod m) { return std::string(toString(m)); }

	// --- INTEGRATION SERVICE METHODS ---

	// Constructor
	IntegrationService::IntegrationService() 
		: _ODE(std::make_unique<integration::ODE>()), method(eIntegrationMethod::Euler){}
	
	// Integration method dispatcher
	VecX IntegrationService::stepODE(eIntegrationMethod m, VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f) {
		if (!f) {
			D_WARN_ONCE("No derivative function provided for RK2/RK4 integration - Assuming constant derivative (Euler step)");
			return x; // could also throw an error here, but feel this is better
		}

		switch (m) {
		case eIntegrationMethod::Euler:    return _ODE->eulerStep(x, t, dt, f);
		case eIntegrationMethod::Midpoint: return _ODE->midpointStep(x, t, dt, f);
		case eIntegrationMethod::Heun:     return _ODE->heunStep(x, t, dt, f);
		case eIntegrationMethod::Ralston:  return _ODE->ralstonStep(x, t, dt, f);
		case eIntegrationMethod::RK4:      return _ODE->rk4Step(x, t, dt, f);
		case eIntegrationMethod::RK45: {
			const double rtol = 1e-6;
			const double atol = 1e-9;
			return stepAdaptiveODE(eIntegrationMethod::RK45, x, t, dt, f, rtol, atol).x_next;
		}
		default:
			LOG_WARN("Unknown integration method: %s. Defaulting to Euler.", toString(m));
			return _ODE->eulerStep(x, t, dt, f);
		}
	}

	// Adaptive step size integration method dispatcher
	StepOut IntegrationService::stepAdaptiveODE(eIntegrationMethod m, VecX& x, double t, double dt_try, std::function<VecX(double, const VecX&)> f, double rtol, double atol) {
		if (!f) {
			D_WARN_ONCE("No derivative function provided for adaptive integration - returning state unchanged");
			return { x, /*dt_taken=*/dt_try, /*dt_sug=*/dt_try }; // could also throw an error here, but feel this is better
		}
		if (m == eIntegrationMethod::RK45) {
			double dt = dt_try;
			VecX x_next = _ODE->rk45Step(x, t, dt, f, rtol, atol);
			return { x_next, /*dt_taken=*/dt_try, /*dt_sug=*/dt };
		}
		else {
			LOG_WARN("Unknown or unsupported adaptive integration method: %s. Defaulting to RK45 Method", toString(m));
			double dt = dt_try;
			VecX x_next = _ODE->rk45Step(x, t, dt, f, rtol, atol);
			return { x_next, /*dt_taken=*/dt_try, /*dt_sug=*/dt };
		}
	}
} // namespace integration