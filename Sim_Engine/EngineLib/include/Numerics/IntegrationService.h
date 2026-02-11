#pragma once
// File:   IntegrationService.h
// GitHub: SaltyJoss
#pragma warning(disable : 4251)

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <core/constants.h>
#include <integrators/numerical_integrators.h>

#include "Platform/Logger.h"

namespace integration {
	// Integrator Methods
	enum class eIntegrationMethod {
		Euler = 0,		// First-Order Euler Method
		Midpoint = 1,	// Second-Order Runge-Kutta (Midpoint)
		Heun = 2,		// Second-Order Runge-Kutta (Heun)
		Ralston = 3,	// Second-Order Runge-Kutta (Ralston)
		RK4 = 4,		// Fourth-Order Runge-Kutta 
		RK45 = 5		// RK45 Method with Adaptive Step Size (Dormand-Prince)
	};

	// Struct representing the result of a single integration step
	struct ENGINE_API StepOut {
		VecX x_next;			// next state vector
		double dt_taken = 0.0;	// actual step size taken
		double dt_sug = 0.0;	// suggested next step size
	};


	// Class representing the integration service
	class ENGINE_API IntegrationService {
	public:
		// Constructor 
		IntegrationService();

		mathlib::VecX stepODE(eIntegrationMethod m, VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f);
		StepOut stepAdaptiveODE(eIntegrationMethod m, VecX& x, double t, double dt_try, std::function<VecX(double, const VecX&)> f, double rtol, double atol);

		void setIntegrationMethod(eIntegrationMethod m) { method = m; }
		eIntegrationMethod getIntegrationMethod() const { return method; }

		const std::string IntegratorName(eIntegrationMethod m);

	private:
		const char* toString(eIntegrationMethod m);

		integration::eIntegrationMethod method;
		std::unique_ptr<integration::ODE> _ODE;

		std::string _methodStr = "Euler";
	};
} // namespace integration
