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
		mathlib::VecX x_next;			// next state vector
		double dt_taken = 0.0;	// actual step size taken
		double dt_sug = 0.0;	// suggested next step size
	};

	// Class representing the integration service
	class DSFE_API IntegrationService {
	public:
		// Constructor 
		IntegrationService();

		template<typename Func, typename JacFunc = std::nullptr_t>
		StepOut step(eIntegrationMethod m, mathlib::VecX& x, double t, double dt, Func&& f, JacFunc&& jac);

		template<typename Func>
		StepOut step_adaptive(eIntegrationMethod m, mathlib::VecX& x, double t, double dt_try, Func&& f, double rtol, double atol);

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
		std::unique_ptr<integration::NumericalIntegrator> _integrator;

		std::string _methodStr = "RK4";

		double _rtol;
		double _atol;

		double _dt_last = 0.0; // last successful step
		double _dt_max = 0.0;  // maximum allowed step size
	};
} // namespace integration

#include "IntegrationStep.inl"
