// DSFE_Core AutoDiffStep.inl
#pragma once

namespace integration {
	template<typename Scalar, typename Func>
	StepOut_T<Scalar> DifferentiableIntegrator::step(eAutoDiffIntegrationMethod m, const mathlib::VecX_T<Scalar>& x, Scalar t, Scalar dt, Func&& f) {
		using Real = typename mathlib::DualTraits<Scalar>::BaseScalar;
		if constexpr (std::is_pointer_v<std::decay_t<Func>> || requires { f == nullptr; }) {
			if (f == nullptr) {
				D_WARN_ONCE("No derivative function provided for integration - Assuming constant derivative.");
				return { _integrator->implicitEuler_AD(x, t, dt, std::forward<Func>(f), 8, 1e-6), dt, dt };
			}
		}

		Real dt_r = mathlib::real(dt);

		_state->backend = eIntegrationBackend::AutoDiff;
		_state->autoDiff = true;
		_state->last_dt_taken = (_state->last_dt_taken != dt && !_state->adaptive) ? dt_r : _state->last_dt_taken; // Unless RK45-or other adaptive methods-it is static.
		_state->last_dt_sug = (_state->last_dt_sug != dt && !_state->adaptive) ? dt_r : _state->last_dt_sug; // Unless RK45-or other adaptive methods-it is static.

		// Though right now all methods hold the same fundamental state in AD, I am employing the same structure for consistency
		switch (m) {
			case eAutoDiffIntegrationMethod::AD_ImplicitEuler:
				_state->name = IntegratorName(m);
				_state->adaptive = false; _state->implicit = true;
				return { _integrator->implicitEuler_AD(x, t, dt, std::forward<Func>(f), 8, 1e-6), dt_r, dt_r };
			case eAutoDiffIntegrationMethod::AD_ImplicitMidpoint:
				_state->name = IntegratorName(m);
				_state->adaptive = false; _state->implicit = true;
				return { _integrator->implicitMidpoint_AD(x, t, dt, std::forward<Func>(f), 10, 1e-7), dt_r, dt_r };
			case eAutoDiffIntegrationMethod::AD_GLRK2:
				_state->name = IntegratorName(m);
				_state->adaptive = false; _state->implicit = true;
				return { _integrator->GLRK2_AD(x, t, dt, std::forward<Func>(f), 80, 1e-10), dt_r, dt_r };
			case eAutoDiffIntegrationMethod::AD_GLRK3:
				_state->name = IntegratorName(m);
				_state->adaptive = false; _state->implicit = true;
				return { _integrator->GLRK3_AD(x, t, dt, std::forward<Func>(f), 150, 1e-14), dt_r, dt_r };
			default:
				LOG_WARN("Unknown Integrator, defaulting to ImplicitEuler (AutoDiff).");
				_state->name = IntegratorName(eAutoDiffIntegrationMethod::AD_ImplicitEuler);
				_state->adaptive = false; _state->implicit = true;
				return { _integrator->implicitEuler_AD(x, t, dt, std::forward<Func>(f), 8, 1e-6), dt_r, dt_r };
		}
	}
} // namespace integration