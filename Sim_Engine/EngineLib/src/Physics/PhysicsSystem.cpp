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
		: _integrator(std::make_unique<integration::IntegrationService>()), _curIntMethod(integration::eIntegrationMethod::Euler) {
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

} // namespace physics