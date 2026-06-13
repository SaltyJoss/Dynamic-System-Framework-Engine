// DSFE_CORE Body.h
#pragma once

#include "EngineCore.h"
#include <PhysLib.h>
#include "SingleBodySystems/Dynamics.h"
#include "Numerics/IntegrationService.h"

namespace single_body_system {
	class SingleBodySystem {
	public:
		SingleBodySystem() = default;
		~SingleBodySystem() = default;

		void step(double dt, double t);

		integration::eIntegrationMethod getIntegrationMethod() const { return _curIntMethod; }
		std::string getIntegratorName() const { return _integrator->IntegratorName(_curIntMethod); }
		void setStandardIntegrator(integration::eIntegrationMethod m) { _curIntMethod = m; }

		integration::eAutoDiffIntegrationMethod AD_IntegrationMethod() const { return _curIntMethod_AD; }
		std::string AD_integratorName() const { return _AD_integrator->IntegratorName(_curIntMethod_AD); }
		void setADIntegrator(integration::eAutoDiffIntegrationMethod m) { _curIntMethod_AD = m; }

	private:
		void packState(mathlib::VecX& x) const;
		void unpackState(const mathlib::VecX& x);

		Body* _body;
		std::unique_ptr<dynamics::SingleBodyDynamics> _dynamics;

		std::unique_ptr<integration::IntegrationService> _integrator;
		integration::eIntegrationMethod _curIntMethod{};
		std::unique_ptr<integration::DifferentiableIntegrator> _AD_integrator;
		integration::eAutoDiffIntegrationMethod _curIntMethod_AD{};

		double _mass = 1.0;
		mathlib::Vec3 _g{ 0.0, 0.0, 0.0 };
	};
}