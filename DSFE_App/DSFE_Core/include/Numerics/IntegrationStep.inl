// DSFE_Core IntegrationStep.inl
#pragma once

namespace integration {
	template<typename Func, typename JacFunc>
	StepOut_T<double> IntegrationService::step(eIntegrationMethod m, const mathlib::VecX& x, double t, double dt, Func&& f, JacFunc&& jac) {
		if constexpr (std::is_pointer_v<std::decay_t<Func>> || requires { f == nullptr; }) {
			if (f == nullptr) {
				D_WARN_ONCE("No derivative function provided for integration - Assuming constant derivative (Euler step)");
				return { _integrator->eulerStep(x, t, dt, std::forward<Func>(f)), dt, dt };
			}
		}

		LOG_INFO_ONCE("Using integration method: %s", IntegratorName(m).c_str());

		_state->backend = eIntegrationBackend::Standard;
		_state->last_dt_taken = (_state->last_dt_taken != dt && !_state->adaptive) ? dt : _state->last_dt_taken;
		_state->last_dt_sug = (_state->last_dt_sug != dt && !_state->adaptive) ? dt : _state->last_dt_sug;
		_state->autoDiff = false;

		switch (m) {
			// Explicit methods
			//  * currently all explicit methods use fixed step size, apart from RK45 as it is an adaptive method
			case eIntegrationMethod::Euler:
				_state->name = IntegratorName(m); // all of these might add more time complexity than what is needed.
				_state->adaptive = false; _state->implicit = false;
				return { _integrator->eulerStep(x, t, dt, std::forward<Func>(f)), dt, dt };
			case eIntegrationMethod::Midpoint:
				_state->name = IntegratorName(m);
				_state->adaptive = false; _state->implicit = false;
				return { _integrator->midpointStep(x, t, dt, std::forward<Func>(f)), dt, dt };
			case eIntegrationMethod::Heun:
				_state->name = IntegratorName(m);
				_state->adaptive = false; _state->implicit = false;
				return { _integrator->heunStep(x, t, dt, std::forward<Func>(f)), dt, dt };
			case eIntegrationMethod::Ralston:
				_state->name = IntegratorName(m);
				_state->adaptive = false; _state->implicit = false;
				return { _integrator->ralstonStep(x, t, dt, std::forward<Func>(f)), dt, dt };
			case eIntegrationMethod::RK4:
				_state->name = IntegratorName(m);
				_state->adaptive = false; _state->implicit = false;
				return { _integrator->rk4Step(x, t, dt, std::forward<Func>(f)), dt, dt };
			case eIntegrationMethod::RK45:
				_state->name = IntegratorName(m);
				_state->adaptive = true; _state->implicit = false;
				return step_adaptive(eIntegrationMethod::RK45, x, t, dt, std::forward<Func>(f), _rtol, _atol);
			// Implicit methods
			//  * currently use fixed step size (no error estimation), but are likely to support adaptive stepping in the future
			case eIntegrationMethod::ImplicitEuler:
				_state->name = IntegratorName(m);
				_state->adaptive = false; _state->implicit = true;
				return { _integrator->implicitEuler(x, t, dt, std::forward<Func>(f), std::forward<JacFunc>(jac)), dt, dt };
			case eIntegrationMethod::ImplicitMidpoint:
				_state->name = IntegratorName(m);
				_state->adaptive = false; _state->implicit = true;
				return { _integrator->implicitMidpoint(x, t, dt, std::forward<Func>(f), std::forward<JacFunc>(jac)), dt, dt };
			case eIntegrationMethod::GLRK2:
				_state->name = IntegratorName(m);
				_state->adaptive = false; _state->implicit = true;
				return { _integrator->GLRK2(x, t, dt, std::forward<Func>(f), std::forward<JacFunc>(jac), 80, 1e-10), dt, dt };
			case eIntegrationMethod::GLRK3:
				_state->name = IntegratorName(m);
				_state->adaptive = false; _state->implicit = true;
				return { _integrator->GLRK3(x, t, dt, std::forward<Func>(f), std::forward<JacFunc>(jac), 150, 1e-14), dt, dt };
			default:
				LOG_WARN("Unknown integration method: %s. Defaulting to RK4.", toString(m));
				_state->name = IntegratorName(eIntegrationMethod::RK4);
				_state->adaptive = false; _state->implicit = false;
				return { _integrator->rk4Step(x, t, dt, std::forward<Func>(f)), dt, dt };
		}
	}

	template<typename Func>
	StepOut_T<double> IntegrationService::step_adaptive(eIntegrationMethod m, const mathlib::VecX& x, double t, double dt_try, Func&& f, double rtol, double atol) {
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
		mathlib::VecX x_curr = x;	  // current state during the adaptive step
		double t_curr = t;	  // current time during the adaptive step
		double t_total = 0.0; // total time taken for the step

		// Limit the number of substeps to prevent infinite loop
		int substeps = 0;
		const int max_substeps = 500; // safety limit

		if (h <= eps) {
			LOG_WARN("Adaptive integration step size is too small (%e). Returning last computed state.", h);
			return { x_curr, t_total, h };
		}

		// Loop until we reach the target end time or exceed the maximum number of substeps
		while (t_curr < t_end && substeps < max_substeps) {
			double h_try = std::min(h, t_end - t_curr);
			double dt_used = 0.0;

			mathlib::VecX x_next = _integrator->rk45Step(x_curr, t_curr, h_try, dt_used, std::forward<Func>(f), rtol, atol);

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

		_state->last_dt_taken = t_total;
		_state->last_dt_sug = h;

		// Persist the last good step size for next frame
		_dt_last = h;
		return { x_curr, t_total, h };
	}
} // namespace integration