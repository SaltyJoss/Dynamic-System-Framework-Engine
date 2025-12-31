// ==================================================
//				File: PhysicsSystem.cpp
// ==================================================

#include "pch.h"
#include "Physics/PhysicsSystem.h"
#include "Scene/Object.h"
#include "Scene/Mesh.h"
#include "integrators/Integration.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/transform.hpp>

#include "EngineLib/LogMacros.h"

namespace physics {
	// Constructor
	PhysicsSystem::PhysicsSystem() {
		_ODE = std::make_unique<integration::ODE>();

		LOG_INFO("PhysicsSystem initialised.");
		D_INFO("Physics initialised");
	}


// --------------------------------------------------
//				  SIMULATION CONTROL
// --------------------------------------------------
	void PhysicsSystem::update(double dt, scene::Object* obj) {
		if (!obj) return;
		// Apply forces (uses mass)
		applyForces(dt, obj);

		// Translate object (uses velocity)
		updateTranslation(dt, obj);

		// Rotate object (uses angular velocity)
		updateRotation(dt, obj);

		// Handle floor collision
		//handleFloorCollision(dt, obj, 0.0f);
	}

// --------------------------------------------------
//				  PER-SYSTEM UPDATES
// --------------------------------------------------
	// Rotation update
	void PhysicsSystem::updateRotation(double dt, scene::Object* obj) {	// Euler angle vs Quaternion?? Make note for report, may try implelemtn
		if (!obj || !obj->getMesh()) return;

		auto& s = obj->state;

		// state vector: [theta_x, theta_y, theta_z, omega_x, omega_y, omega_z]
		VecX x(6);
		x(0) = s.theta.x();
		x(1) = s.theta.y();
		x(2) = s.theta.z();
		x(3) = s.angularVelocity.x();
		x(4) = s.angularVelocity.y();
		x(5) = s.angularVelocity.z();

		// define derivative function for RK2/RK4
		auto f = [&](double t, const VecX& state) -> VecX {
			VecX deriv(6);

			// unpacking state vector (theta = angle, omega = angular velocity)
			double theta_x = state(0);
			double theta_y = state(1);
			double theta_z = state(2);
			double omega_x = state(3);
			double omega_y = state(4);
			double omega_z = state(5);

			// derivative: dtheta/dt = omega
			deriv(0) = omega_x;
			deriv(1) = omega_y;
			deriv(2) = omega_z;

			// derivative: domega/dt = angular acceleration (damping is 0.0 by default!)
			deriv(3) = -(s.damping) * omega_x;
			deriv(4) = -(s.damping) * omega_y;
			deriv(5) = -(s.damping) * omega_z;

			return deriv;
		};

		// Euler integrate using your ODE helper
		VecX next = integrationMethod(x, 0.0, dt, f, method);

		// write back to state
		s.theta.x() = next(0);
		s.theta.y() = next(1);
		s.theta.z() = next(2);
		s.angularVelocity.x() = next(3);
		s.angularVelocity.y() = next(4);
		s.angularVelocity.z() = next(5);

		// WRAP ANGLES INTO [-pi, pi] OR [0, 2pi]
		auto wrapRad = [](double a) -> double {
			a = fmod(a, TWO_PI_d);
			if (a < 0.0) a += TWO_PI_d;
			return a;
		};

		s.theta.x() = wrapRad(s.theta.x());
		s.theta.y() = wrapRad(s.theta.y());
		s.theta.z() = wrapRad(s.theta.z());

		// sync to mesh (still radians!)
		obj->transform.rotation.x = static_cast<float>(s.theta.x());
		obj->transform.rotation.y = static_cast<float>(s.theta.y());
		obj->transform.rotation.z = static_cast<float>(s.theta.z());

		// update diagnostics if running
		if (_diagRunning && obj == _diagObject) {
			IntegratorDiagSample sample;
			sample.t = _t;
			sample.theta = s.theta;
			sample.omega = s.angularVelocity;
			_diagSamples.push_back(sample);
		}
	}

	// Translation update
	void PhysicsSystem::updateTranslation(double dt, scene::Object* obj) {
		if (!obj || !obj->getMesh()) return;

		auto& s = obj->state;

		// v * dt is displacement
		Vec3 dp = s.linearVelocity * dt;

		obj->transform.position += glm::vec3(dp.x(), dp.y(), dp.z());
	}

	// Force application
	void PhysicsSystem::applyForces(double dt, scene::Object* obj) {
		if (!obj || !obj->getMesh()) return;

		auto& s = obj->state;

		// acceleration = F / m
		Vec3 accel = s.forces / s.mass;

		// update velocity
		s.linearVelocity += accel * dt;

		// reset forces after applying
		s.forces.setZero();
	}

	// Torque application
	void PhysicsSystem::applyTorque(double dt, scene::Object* obj, const Vec3& torque) {
		if (!obj || !obj->getMesh()) return;

		auto& s = obj->state;

		// angular acceleration = torque / inertia
		Vec3 angAccel = s.inertia.inverse() * torque;
		// update angular velocity
		s.angularVelocity += angAccel * dt;
	}

	// Damping application
	void PhysicsSystem::applyDamping(double dt, scene::Object* obj, float dampingCoefficient) {
		if (!obj || !obj->getMesh()) return;

		auto& s = obj->state;
		s.linearVelocity *= dampingCoefficient;
		s.angularVelocity *= dampingCoefficient;
	}

	// Collision handling
	void PhysicsSystem::handleFloorCollision(double dt, scene::Object* obj, float floorY) {
		if (!obj || !obj->getMesh()) return;

		auto& s = obj->state;
		
		// simple floor collision at y = floorY
		if (obj->transform.position.y < floorY) {
			// Simple collision response: reset position and invert Y velocity
			obj->transform.position.y = floorY;
			s.linearVelocity.y() = -s.linearVelocity.y() * 0.5f; // lose some energy on bounce
		}
		
		// Dampen small bounces to zero
		if (obj->transform.position.y < floorY + 0.1f && s.linearVelocity.y() < 0.1f) {
			s.linearVelocity.y() = 0.0f; // stop small bounces
		}
	}

// --------------------------------------------------
//				   INTEGRATION (ODE)
// --------------------------------------------------
	VecX PhysicsSystem::integrationMethod(VecX& x, double t, double dt, std::function<VecX(double, const VecX &)> f, eIntegrationMethod method) {
		VecX dxdt = f(t, x); // compute derivative at current state (for Euler, but may revise euler function to do this inhouse, depends on efficiency honestly)
		if (method == eIntegrationMethod::Euler) {
			return _ODE->eulerStep(x, dxdt, dt);
		}
		else if (method == eIntegrationMethod::Midpoint) {
			return _ODE->midpointStep(x, t, dt, f);
		}
		else if (method == eIntegrationMethod::Heun) {
			return _ODE->heunStep(x, t, dt, f);
		}
		else if (method == eIntegrationMethod::Ralston) {
			return _ODE->ralstonStep(x, t, dt, f);
		}
		else if (method == eIntegrationMethod::RK4) {
			return _ODE->rk4Step(x, t, dt, f);
		}
		else {
			LOG_WARN("Unknown integration method: %s. Defaulting to Euler Method (simplest)", method);
			return _ODE->eulerStep(x, dxdt, dt);
		}

		if (!f) {
			// If no function provided, assume constant derivative (dxdt)
			D_WARN_ONCE("No derivative function provided for RK2/RK4 integration - Assuming constant derivative (Euler step)");
			return x + dxdt * dt;
		}
	}

// --------------------------------------------------
//				Integration Analysis
// --------------------------------------------------
	void PhysicsSystem::startDiagnostics(scene::Object* obj) {
		if (!obj) {
			_diagRunning = false;
			D_ERROR("Null diagnostic object");
			return;
		}

		if (_diagRunning) return; // already running
		_diagRunning = true;
		_diagObject = obj;
		_diagResult = IntegratorDiagResult();
	}

	void PhysicsSystem::stopDiagnostics() {
		if (!_diagRunning) return;
		_diagRunning = false;

		if (_diagSamples.empty()) {
			_diagResult = IntegratorDiagResult();
			D_WARN("Integrator diagnostics stopped");
			return;
		}

		// build scalar series: omega/time
		std::vector<double> omegaNorms;
		omegaNorms.reserve(_diagSamples.size());

		for (const auto& sample : _diagSamples) {
			omegaNorms.push_back(sample.omega.norm());
		}

		// MathLib computes stats
		integration::analysis analyser;
		integration::ErrorStats omegaStats = analyser.computeErrorStats(omegaNorms);

		// Fill result struct
		_diagResult.duration = _diagSamples.back().t;
		_diagResult.omegaNormStats = omegaStats;
		_diagResult.thetaNormStats = integration::ErrorStats(); // Not computed yet, just zeroed for now

		D_SUCCESS("Integrator diagnostics finished: \n\t\t\t Total Samples: %zu\n\t\t\t Min Error: %zu\n\t\t\t Max Error: %zu\n\t\t\t Mean Error: %zu\n\t\t\t RMS Error: %zu", 
			_diagSamples.size(), 
			_diagResult.omegaNormStats.minError,
			_diagResult.omegaNormStats.maxError,
			_diagResult.omegaNormStats.meanError,
			_diagResult.omegaNormStats.rmsError
		);
	}


} // namespace physics