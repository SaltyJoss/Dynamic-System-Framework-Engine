#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <core/constants.h>
#include <integrators/numerical_integrators.h>
#include "ReferenceSolver.h"
#include "Platform/Logger.h"

namespace integration {
	// Integrator Methods
	enum class eIntegrationMethod {
		Euler = 0,		// First-Order Euler Method
		Midpoint = 1,	// Second-Order Runge-Kutta (Midpoint)
		Heun = 2,		// Second-Order Runge-Kutta (Heun)
		Ralston = 3,	// Second-Order Runge-Kutta (Ralston)
		RK4 = 4			// Fourth-Order Runge-Kutta 
	};


	// Class representing the integration service
	class ENGINE_API IntegrationService {
	public:
		// Constructor 
		IntegrationService();

		mathlib::VecX stepODE(VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f);
		mathlib::VecX referenceIntegrationMethod(VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f, double rtol, double atol);

		void setIntegrationMethod(eIntegrationMethod m) { method = m; }
		eIntegrationMethod getIntegrationMethod() const { return method; }

		integration::ReferenceSolver* getReferenceSolver() const { return _refSolver.get(); }

	private:
		integration::eIntegrationMethod method;
		std::unique_ptr<integration::ODE> _ODE;
		std::unique_ptr<integration::ReferenceSolver> _refSolver;
	};
}
