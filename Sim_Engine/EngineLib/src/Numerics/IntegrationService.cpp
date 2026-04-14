#include "pch.h"
// File:   IntegrationService.cpp
// GitHub: SaltyJoss
#include "Numerics/IntegrationService.h"

#include "EngineLib/LogMacros.h"

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
			case eIntegrationMethod::Euler:
				return "euler";
			case eIntegrationMethod::Midpoint:
				return "midpoint";
			case eIntegrationMethod::Heun:
				return "heun";
			case eIntegrationMethod::Ralston:
				return "ralston";
			case eIntegrationMethod::RK4:
				return "rk4";
			case eIntegrationMethod::RK45:
				return "rk45";
			case eIntegrationMethod::BackwardEuler:
				return "backward_euler";
			case eIntegrationMethod::ImplicitMidpoint:
				return "implicit_midpoint";
			default:
				return "Unknown";
		}
	}

	// Get integrator name
	const std::string IntegrationService::IntegratorName(eIntegrationMethod m) { return std::string(toString(m)); }

	// Constructor
	IntegrationService::IntegrationService() 
		: _ODE(std::make_unique<integration::ODE>()), method(eIntegrationMethod::RK4), _rtol(1e-3), _atol(1e-6), _dt_last(), _dt_max() {}
	
	// Integration method dispatcher
	StepOut IntegrationService::stepODE(eIntegrationMethod m, VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f) {
		if (!f) {
			D_WARN_ONCE("No derivative function provided for RK2/RK4 integration - Assuming constant derivative (Euler step)");
			return { _ODE->eulerStep(x, t, dt, f), /*dt_taken=*/dt, /*dt_sug=*/dt };
		}

		switch (m) {
		case eIntegrationMethod::Euler:    return { _ODE->eulerStep(x, t, dt, f), dt, dt };
		case eIntegrationMethod::Midpoint: return { _ODE->midpointStep(x, t, dt, f), dt, dt };
		case eIntegrationMethod::Heun:     return { _ODE->heunStep(x, t, dt, f), dt, dt };
		case eIntegrationMethod::Ralston:  return { _ODE->ralstonStep(x, t, dt, f), dt, dt };
		case eIntegrationMethod::RK4:      return { _ODE->rk4Step(x, t, dt, f), dt, dt };
		case eIntegrationMethod::RK45: {
			return stepAdaptiveODE(eIntegrationMethod::RK45, x, t, dt, f, _rtol, _atol);
		}
		// Note: Backward Euler and Implicit Midpoint are currently implemented as fixed step methods for simplicity
		// These will not work as expected, so please use without real expectatoions until I actually have time to do a full implementation.
		case eIntegrationMethod::BackwardEuler:    return { _ODE->backward_euler(x, t, dt, f), dt, dt };
		case eIntegrationMethod::ImplicitMidpoint: return { _ODE->implicit_midpoint(x, t, dt, f), dt, dt };
		default:
			LOG_WARN("Unknown integration method: %s. Defaulting to RK4.", toString(m));
			return { _ODE->rk4Step(x, t, dt, f), dt, dt };
		}
	}

	// Adaptive mehod dispatcher (currently only RK45 implemented)
	StepOut IntegrationService::stepAdaptiveODE(eIntegrationMethod m, VecX& x, double t, double dt_try, std::function<VecX(double, const VecX&)> f, double rtol, double atol) {
		if (!f) {
			D_WARN_ONCE("No derivative function provided for adaptive integration - returning state unchanged");
			return { x, dt_try, dt_try }; // could also throw an error here, but feel this is better
		}
		if (m != eIntegrationMethod::RK45) {
			LOG_WARN("Adaptive step size integration is only implemented for RK45 method. Defaulting to RK45 Method", toString(m));
		}

		// Start with the last successful step size or the initial guess
		double h_init = (_dt_last > 0.0) ? _dt_last : dt_try;

		// Enforce maximum step size if set
		if (_dt_max > 0.0) {
			h_init = std::min(h_init, _dt_max);
		}
		double h = std::min(h_init, dt_try);

		// Target end time for this adaptive step
		const double t_end = t + dt_try;	 // target end time for this step
		const double eps = 1e-12 * dt_try; // small epsilon to prevent division by zero

		// Initialise current state and time for the adaptive stepping loop
		VecX x_curr   = x;	  // current state during the adaptive step
		double t_curr = t;	  // current time during the adaptive step
		double t_total = 0.0; // total time taken for the step

		// Limit the number of substeps to prevent infinite loop
		int substeps = 0;
		const int max_substeps = 500; // safety limit

		// Loop until we reach the target end time or exceed the maximum number of substeps
		while (t_curr < t_end && substeps < max_substeps) {
			double h_try = std::min(h, t_end - t_curr);
			double dt_used = 0.0;

			VecX x_next = _ODE->rk45Step(x_curr, t_curr, h_try, dt_used, f, rtol, atol);

			// Update rk45step
			t_curr	+= dt_used; 
			t_total += dt_used; 
			x_curr	 = x_next;
			h		 = h_try;

			++substeps;
		}

		// If substep limit was reached, a warning is logged
		if (substeps >= max_substeps) { 
			LOG_WARN("Adaptive integration exceeded maximum substeps (%d) at time %f. Returning last computed state.", max_substeps, t_curr); 
		} 
		
		// Persist the last good step size for next frame
		_dt_last = h;
		return { x_curr, t_total, h };
	}
} // namespace integration