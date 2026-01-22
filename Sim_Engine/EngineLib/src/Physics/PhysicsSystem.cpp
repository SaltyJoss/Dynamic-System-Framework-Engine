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
	PhysicsSystem::PhysicsSystem() 
		: _integrator(std::make_unique<integration::IntegrationService>()), _refSolver(std::make_unique<integration::ReferenceSolver>()), 
		_curIntMethod(integration::eIntegrationMethod::Euler) {
		if (!_integrator) { LOG_WARN("Physics got null IntegrationService*"); }
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
		//updateRotation(dt, obj);
		updateRotation(dt, obj);

		// Update Reference Integrator States
		updateRefRotation(dt, obj);

		// Handle floor collision
		//handleFloorCollision(dt, obj, 0.0f);
	}

// --------------------------------------------------
//				  PER-SYSTEM UPDATES
// --------------------------------------------------

	// Updates an objects rotation using quaternions instead of Euler angles
	void PhysicsSystem::updateRotation(double dt, scene::Object* obj) {
		if (!obj || !obj->getMesh()) return;

		auto& s = obj->state;
		
		// State vectory [qw,qx,qy,qz,wx,wy,wz]
		VecX x(7);
		x(0) = s.q.w(); x(1) = s.q.x(); x(2) = s.q.y(); x(3) = s.q.z(); // quaternion angles
		x(4) = s.angularVelocity.x(); x(5) = s.angularVelocity.y(); x(6) = s.angularVelocity.z(); // angular velocity

		// World-Frame OR Body-Frame Angular Velocity
		auto f = [&](double, const VecX& st) {
			VecX d(7);
			// dq = 1/2 * (0,w) * q -> world frame angular velocity
			Quat _q(st(0), st(1), st(2), st(3));
			Vec3 w(st(4), st(5), st(6));
			Quat dq = (_frame == FrameType::World) ? Quat(0, w.x(), w.y(), w.z()) * _q : _q * Quat(0, w.x(), w.y(), w.z());
			dq.coeffs() *= 0.5; // Eigen doesnt support scalar multiplication :(
			d(0) = dq.w(); d(1) = dq.x(); d(2) = dq.y(); d(3) = dq.z();
			d(4) = -s.damping * w.x(); d(5) = -s.damping * w.y(); d(6) = -s.damping * w.z();
			return d;
		};

		// Integrate to get next state
		VecX next = _integrator->stepODE(_curIntMethod, x, 0.0, dt, f);
		s.q = Quat(next(0), next(1), next(2), next(3)).normalized();
		s.angularVelocity = Vec3(next(4), next(5), next(6));
		obj->transform.rotQ = glm::quat((float)s.q.w(), (float)s.q.x(), (float)s.q.y(), (float)s.q.z());

		// update diagnostics if running
		if (_diagRunning && obj == _diagObject) {
			IntegratorDiagSample sample;
			sample.t = _t;
			sample.q_method = s.q;
			sample.omega = s.angularVelocity;
			_diagSamples.push_back(sample);
		}
	}

	void PhysicsSystem::updateRefRotation(double dt, scene::Object* obj) {
		if (!obj || !obj->getMesh()) return;
		auto& rt = gRefTracks[obj];

		// State vector [qw,qx,qy,qz,wx,wy,wz]
		if (!rt.init) {
			// Initialize reference track
			rt.x.resize(7);
			rt.x(0) = obj->state.q.w(); rt.x(1) = obj->state.q.x(); rt.x(2) = obj->state.q.y(); rt.x(3) = obj->state.q.z(); // quaternion angles
			rt.x(4) = obj->state.angularVelocity.x(); rt.x(5) = obj->state.angularVelocity.y(); rt.x(6) = obj->state.angularVelocity.z(); // angular velocity
			rt.t = _t;
			rt.dt = 1e-3; // initial step size
			rt.init = true;
		}

		const double t_next = _t + dt;

		// World-Frame Angular Velocity
		auto f = [&](double, const VecX& st) {
			VecX d(7);
			// dq = 1/2 * (0,w) * q -> world frame angular velocity
			Quat _q(st(0), st(1), st(2), st(3));
			Vec3 w(st(4), st(5), st(6));
			Quat dq = (_frame == FrameType::World) ? Quat(0, w.x(), w.y(), w.z()) * _q : _q * Quat(0, w.x(), w.y(), w.z());
			dq.coeffs() *= 0.5; // Eigen doesnt support scalar multiplication :(
			d(0) = dq.w(); d(1) = dq.x(); d(2) = dq.y(); d(3) = dq.z();
			d(4) = -(obj->state.damping * w.x()); d(5) = -(obj->state.damping * w.y()); d(6) = -(obj->state.damping * w.z());
			return d;
		};


		// Perform adaptive step to reach t_next
		while (rt.t < t_next) {
			double dt = std::min(rt.dt, t_next - rt.t);
			auto res = _refSolver->refStep(rt.x, rt.t, dt, f, 1e-6, 1e-9);
			rt.x = res.x_next;
			rt.t += res.dt_taken;
			rt.dt = res.dt_sug;
		}

		// update ref diagnostics if running
		if (_diagRunning) {
			integration::ReferenceSolver::RefIntegratorDiagSample samples;
			samples.t = rt.t;
			samples.q = Quat(rt.x(0), rt.x(1), rt.x(2), rt.x(3)); // quaternion angles
			samples.omega = Vec3(rt.x(4), rt.x(5), rt.x(6)); // angular velocities
			_refDiagSamples.push_back(samples);

			_t += dt; // advance global time
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
//				Integration Analysis
// --------------------------------------------------
	// Start diagnostics
	void PhysicsSystem::startDiagnostics(scene::Object* obj) {
		if (!obj) {
			_diagRunning = false;
			D_ERROR("Null diagnostic object");
			return;
		}

		if (_diagRunning) return; // already running
		
		_diagSamples.clear();
		_refDiagSamples.clear();
		_t = 0.0;
		
		_diagRunning = true;
		_diagObject = obj;
		_diagResult = IntegratorDiagResult();
		_refDiagResult = IntegratorDiagResult();
		gRefTracks[obj].init = false; // reset reference track for object
	}

	// Stop diagnostics and compute results using 
	void PhysicsSystem::stopDiagnostics() {
		if (!_diagRunning) return;
		_diagRunning = false;

		if (_refDiagSamples.empty()) {
			_refDiagResult = IntegratorDiagResult();
			D_WARN("Reference integrator diagnostics stopped");
			return;
		}

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
		// Reference Integrator Diagnostics
		std::vector<double> refOmegaNorms;
		refOmegaNorms.reserve(_refDiagSamples.size());
		for (const auto& sample : _refDiagSamples) {
			refOmegaNorms.push_back(sample.omega.norm());
		}

		integration::analysis analyser;
		
		// Fill result struct
		integration::ErrorStats omegaStats = analyser.computeErrorStats(omegaNorms);
		_diagResult.duration = _diagSamples.back().t;
		_diagResult.omegaNormStats = omegaStats; // Omega Stats are min, max, mean, rms of angular velocity norms over time
		_diagResult.thetaNormStats = integration::ErrorStats(); // Not computed yet, will use MSE and RMSE
		
		// Fill reference result struct
		integration::ErrorStats refOmegaStats = analyser.computeErrorStats(refOmegaNorms);
		_refDiagResult.duration = _refDiagSamples.back().t;
		_refDiagResult.omegaNormStats = refOmegaStats;
		_refDiagResult.thetaNormStats = integration::ErrorStats();


		// =============================

		// Log summary
		D_SUCCESS("Integrator diagnostics finished: \n ================================");

		D_INFO("Integrator diagnostics finished: \n\t\t\t Total Samples: %zu\n\t\t\t Min Error: %.6f\n\t\t\t Max Error: %.6f\n\t\t\t Mean Error: %.6f\n\t\t\t RMS Error: %.6f",
			_diagSamples.size(),
			_diagResult.omegaNormStats.minError,
			_diagResult.omegaNormStats.maxError,
			_diagResult.omegaNormStats.meanError,
			_diagResult.omegaNormStats.rmsError
		);

		D_INFO("Reference integrator diagnostics: \n\t\t\t Total Samples: %zu\n\t\t\t Min Error: %.6f\n\t\t\t Max Error: %.6f\n\t\t\t Mean Error: %.6f\n\t\t\t RMS Error: %.6f",
			_refDiagSamples.size(),
			_refDiagResult.omegaNormStats.minError,
			_refDiagResult.omegaNormStats.maxError,
			_refDiagResult.omegaNormStats.meanError,
			_refDiagResult.omegaNormStats.rmsError
		);

		// debugging output of omega norms references
		D_DEBUG("refOmegaNorms size=%zu front=%.6f back=%.6f",
			refOmegaNorms.size(),
			refOmegaNorms.front(),
			refOmegaNorms.back());

		D_DEBUG("Reference Integrator time taken: %f seconds", _refDiagResult.duration);
	}

} // namespace physics