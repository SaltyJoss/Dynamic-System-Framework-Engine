#pragma once
// File:   IntegrationService.h
// GitHub: SaltyJoss
#pragma warning(disable : 4251)

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <core/constants.h>
#include <integrators/numerical_integrators.h>
#include "Numerics/IntegrationMethods.h"

#include "Platform/Logger.h"
#include "EngineLib/LogMacros.h"

namespace integration {
	// Struct representing the result of a single integration step
	struct DSFE_API StepOut {
		VecX x_next;			// next state vector
		double dt_taken = 0.0;	// actual step size taken
		double dt_sug = 0.0;	// suggested next step size
	};

	// Class representing the integration service
	class DSFE_API IntegrationService {
	public:
		// Constructor 
		IntegrationService();

		template<typename Func, typename JacFunc = std::nullptr_t>
		StepOut stepODE(eIntegrationMethod m, VecX& x, double t, double dt, Func&& f, JacFunc&& jac) {
			if constexpr (std::is_pointer_v<std::decay_t<Func>> || requires { f == nullptr; }) {
				if (f == nullptr) {
					D_WARN_ONCE("No derivative function provided for integration - Assuming constant derivative (Euler step)");
					return { _ODE->eulerStep(x, t, dt, std::forward<Func>(f)), dt, dt };
				}
			}

			switch (m) {
				// Explicit methods
				//  * currently all explicit methods use fixed step size, apart from RK45 as it is an adaptive method
				case eIntegrationMethod::Euler:    return { _ODE->eulerStep(x, t, dt, std::forward<Func>(f)), dt, dt };
				case eIntegrationMethod::Midpoint: return { _ODE->midpointStep(x, t, dt, std::forward<Func>(f)), dt, dt };
				case eIntegrationMethod::Heun:     return { _ODE->heunStep(x, t, dt, std::forward<Func>(f)), dt, dt };
				case eIntegrationMethod::Ralston:  return { _ODE->ralstonStep(x, t, dt, std::forward<Func>(f)), dt, dt };
				case eIntegrationMethod::RK4:      return { _ODE->rk4Step(x, t, dt, std::forward<Func>(f)), dt, dt };
				case eIntegrationMethod::RK45:	   return stepAdaptiveODE(eIntegrationMethod::RK45, x, t, dt, std::forward<Func>(f), _rtol, _atol);
					// Implicit methods
					//  * currently use fixed step size (no error estimation), but are likely to support adaptive stepping in the future
				case eIntegrationMethod::ImplicitEuler:    return { _ODE->implicit_euler(x, t, dt, std::forward<Func>(f)), dt, dt };
				case eIntegrationMethod::ImplicitMidpoint: return { _ODE->implicit_midpoint(x, t, dt, std::forward<Func>(f)), dt, dt };
				case eIntegrationMethod::GLRK2:			   return { _ODE->GLRK2(x, t, dt, std::forward<Func>(f), 50, 1e-10, std::forward<JacFunc>(jac)), dt, dt };
				case eIntegrationMethod::GLRK3:			   return { _ODE->GLRK3(x, t, dt, std::forward<Func>(f), 80, -1, std::forward<JacFunc>(jac)), dt, dt };
				default:
				LOG_WARN("Unknown integration method: %s. Defaulting to RK4.", toString(m));
				return { _ODE->rk4Step(x, t, dt, std::forward<Func>(f)), dt, dt };
			}
		}

		template<typename Func>
		StepOut stepAdaptiveODE(eIntegrationMethod m, VecX& x, double t, double dt_try, Func&& f, double rtol, double atol) {
			if constexpr (std::is_pointer_v<std::decay_t<Func>> || requires { f == nullptr; }) {
				if (f == nullptr) {
					LOG_WARN("No derivative function provided for adaptive integration - returning state unchanged");
					return { x, dt_try, dt_try };
				}
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
			VecX x_curr = x;	  // current state during the adaptive step
			double t_curr = t;	  // current time during the adaptive step
			double t_total = 0.0; // total time taken for the step

			// Limit the number of substeps to prevent infinite loop
			int substeps = 0;
			const int max_substeps = 500; // safety limit

			// Loop until we reach the target end time or exceed the maximum number of substeps
			while (t_curr < t_end && substeps < max_substeps) {
				double h_try = std::min(h, t_end - t_curr);
				double dt_used = 0.0;

				VecX x_next = _ODE->rk45Step(x_curr, t_curr, h_try, dt_used, std::forward<Func>(f), rtol, atol);

				// Update rk45step
				t_curr += dt_used;
				t_total += dt_used;
				x_curr = x_next;
				h = h_try;

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

		void setIntegrationMethod(eIntegrationMethod m) { method = m; }
		eIntegrationMethod getIntegrationMethod() const { return method; }

		const std::string IntegratorName(eIntegrationMethod m);

		void setAdaptiveTolerances(double rtol, double atol) { _rtol = rtol; _atol = atol; }
		void setMaxStep(double max_dt) { _dt_max = max_dt; }

		// Reset the cached adaptive step size (call on robot load/reset)
		void resetAdaptiveState() { _dt_last = 0.0; }

	private:
		const char* toString(eIntegrationMethod m);

		integration::eIntegrationMethod method;
		std::unique_ptr<integration::ODE> _ODE;

		std::string _methodStr = "RK4";

		double _rtol;
		double _atol;

		double _dt_last = 0.0; // last successful step
		double _dt_max = 0.0;  // maximum allowed step size
	};
} // namespace integration
