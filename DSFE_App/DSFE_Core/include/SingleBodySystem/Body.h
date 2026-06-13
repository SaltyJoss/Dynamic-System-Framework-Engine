// DSFE_CORE Body.h
#pragma once

#include "EngineCore.h"
#include <PhysLib.h>
#include "SingleBodySystems/Dynamics.h"
#include "Numerics/IntegrationService.h"

#include "Analysis/MetricLogger.h"

namespace single_body_system {
	class SingleBodySystem { // FOR NOW, this represents a single RIGID BODY, but in the near-near-near-future I will extend this for particles (or make a separate ParticleSystem class, changing this to RigidBodySystem)
	public:
		SingleBodySystem();
		~SingleBodySystem();

		void step(double dt, double t);

		Body* body() { return _body; } // return pointer to the body
		const Body* body() const { return _body; } // return const pointer to the body

		integration::eIntegrationMethod getIntegrationMethod() const { return _curIntMethod; }
		std::string getIntegratorName() const { return _integrator->IntegratorName(_curIntMethod); }
		void setStandardIntegrator(integration::eIntegrationMethod m) { _curIntMethod = m; }

		integration::eAutoDiffIntegrationMethod AD_IntegrationMethod() const { return _curIntMethod_AD; }
		std::string AD_integratorName() const { return _AD_integrator->IntegratorName(_curIntMethod_AD); }
		void setADIntegrator(integration::eAutoDiffIntegrationMethod m) { _curIntMethod_AD = m; }

	private:
		mathlib::VecX packState() const;
		void unpackState(const mathlib::VecX& s);

		Body* _body;
		std::unique_ptr<dynamics::SingleBodyDynamics> _dynamics;

		std::unique_ptr<integration::IntegrationService> _integrator;
		integration::eIntegrationMethod _curIntMethod{};
		std::unique_ptr<integration::DifferentiableIntegrator> _AD_integrator;
		integration::eAutoDiffIntegrationMethod _curIntMethod_AD{};

		// precomputed clamp lookup tables
		mutable std::vector<uint8_t> _clampVel;
		mutable std::vector<uint8_t> _clampAngVel;

		double _simTime = 0.0;
		double _mass = 1.0;
		mathlib::Vec3 _g{ 0.0, 0.0, 0.0 };
	};
}