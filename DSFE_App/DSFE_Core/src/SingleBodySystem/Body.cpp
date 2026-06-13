// DSFE_CORE Body.cpp
#include "pch.h"
#include "SingleBodySystem/Body.h"
#include "EngineLib/LogMacros.h"

namespace single_body_system {
	// Step the simulation forward by dt seconds at time t
	void SingleBodySystem::step(double dt, double t) {
		if (!_dynamics) {
			_dynamics = std::make_unique<dynamics::SingleBodyDynamics>();
		}
		mathlib::Vec3 externalForce{ 0.0, 0.0, 0.0 }; // Placeholder for external forces
		mathlib::Vec3 externalTorque{ 0.0, 0.0, 0.0 }; // Placeholder for external torques
		_dynamics->computeDynamics(*this, externalForce, externalTorque, dt);
	}
}