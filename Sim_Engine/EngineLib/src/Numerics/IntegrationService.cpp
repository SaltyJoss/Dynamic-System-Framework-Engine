#include "pch.h"
#include "Numerics/IntegrationService.h"
#include "EngineLib/LogMacros.h"

using namespace mathlib;

namespace integration {
	IntegrationService::IntegrationService() 
		: _ODE(std::make_unique<integration::ODE>()), _refSolver(std::make_unique<integration::ReferenceSolver>()), method(eIntegrationMethod::Euler){}

	// --------------------------------------------------
	//				   INTEGRATION (ODE)
	// --------------------------------------------------
		// Integration method dispatcher
	VecX IntegrationService::stepODE(eIntegrationMethod m, VecX& x, double t, double dt, std::function<VecX(double, const VecX&)> f) {
		VecX dxdt = f(t, x); // compute derivative at current state (for Euler, but may revise euler function to do this inhouse, depends on efficiency honestly)

		if (!f) {
			// If no function provided, assume constant derivative (dxdt)
			D_WARN_ONCE("No derivative function provided for RK2/RK4 integration - Assuming constant derivative (Euler step)");
			return x + dxdt * dt;
		}

		if (m == eIntegrationMethod::Euler)			{ return _ODE->eulerStep(x, dxdt, dt); }
		else if (m == eIntegrationMethod::Midpoint) { return _ODE->midpointStep(x, t, dt, f); }
		else if (m == eIntegrationMethod::Heun)		{ return _ODE->heunStep(x, t, dt, f); }
		else if (m == eIntegrationMethod::Ralston)  { return _ODE->ralstonStep(x, t, dt, f); }
		else if (m == eIntegrationMethod::RK4)		{ return _ODE->rk4Step(x, t, dt, f); }
		else {

			LOG_WARN("Unknown integration method: %s. Defaulting to Euler Method (simplest)", method);
			return _ODE->eulerStep(x, dxdt, dt);
		}
	}
} // namespace numerics