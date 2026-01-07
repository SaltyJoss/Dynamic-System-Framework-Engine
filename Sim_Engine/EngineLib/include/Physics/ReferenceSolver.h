#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <integrators/numerical_integrators.h>
#include <integrators/IntegrationAnalysis.h>

namespace physics {
	class ENGINE_API ReferenceSolver {
	public:
		enum class eReferenceIntegrator {
			DormandPrinceRK45 = 0 // Dormand-Prince RK45 method for reference
		};

		// public struct for reference model physical states (copied from physicalstate.h for now)
		struct ENGINE_API RefPhysState {
			Vec3 theta;
			Vec3 linearVelocity;
			Vec3 angularVelocity;

			double gravity = 0.0; // default to zero-g
			double mass = 1.0; // default to 1kg
			double damping = 0.0; // default to no damping

			Mat3 inertia;
			Vec3 forces;
			Vec3 position;
			Vec3 torques;
		};

		struct ENGINE_API RefStepResult {
			VecX x_next;		// next state vector
			double dt_taken;	// actual step size taken
			double dt_sug;		// suggested next step size
		};

		struct ENGINE_API RefIntegratorDiagSample {
			double t = 0.0;   // simulation time
			Vec3 theta;       // angles (rad)
			Vec3 omega;       // angular velocity (rad/s)
		};

		RefStepResult refStep(const VecX& x, double t, double dt_try, std::function<VecX(double, const VecX&)> f, double rtol, double atol) {
			double dt_sug = dt_try;
			VecX x_next = referenceIntegrationMethod(const_cast<VecX&>(x), t, dt_sug, f, rtol, atol, refMethod);
			return { x_next, dt_try, dt_sug }; 
		}

		void setReferenceIntegrationMethod(eReferenceIntegrator m) { refMethod = m; }
		eReferenceIntegrator getReferenceIntegrationMethod() const { return refMethod; }

		VecX referenceIntegrationMethod(VecX& x, double t, double& dt, std::function<VecX(double, const VecX&)> f, double rtol, double atol, eReferenceIntegrator method) {
			switch (method) {
			case eReferenceIntegrator::DormandPrinceRK45:
			default:
				return _ODE.rk45Step(x, t, dt, f, rtol, atol);
			}
		}

	private:
		eReferenceIntegrator refMethod = eReferenceIntegrator::DormandPrinceRK45; // default reference method
		integration::ODE _ODE;
	};
}