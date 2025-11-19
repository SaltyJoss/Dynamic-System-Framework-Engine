
#include "pch.h"
#include "Physics/PhysicsSystem.h"

#include "Scene/Mesh.h"
#include "integrators/Integration.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/transform.hpp>

#include "EngineLib/LogMacros.h"

namespace physics {
	PhysicsSystem::PhysicsSystem() {
		_ODE = std::make_unique<integration::ODE>();

		LOG_INFO("PhysicsSystem initialized.");
	}


// --------------------------------------------------
//				  SIMULATION CONTROL
// --------------------------------------------------
	void PhysicsSystem::update(double dt, elements::Object* obj) {
		if (!obj) return;

		// Apply forces (uses mass)
		applyForces(dt, obj);

		// Translate object (uses velocity)
		updateTranslation(dt, obj);

		// Rotate object (uses angular velocity)
		updateRotation(dt, obj);
	}

// --------------------------------------------------
//				  PER-SYSTEM UPDATES
// --------------------------------------------------

	// MOTION UPDATES
	void PhysicsSystem::updateRotation(double dt, elements::Object* obj) {	// Euler angle vs Quaternion?? Make note for report, may try implelemtn
		if (!obj || !obj->getMesh()) return;

		auto& s = obj->state;

		s.theta += s.angularVelocity.y() * dt;
		obj->getMesh()->_rotation.x = (float)s.theta;
		obj->getMesh()->_rotation.y = (float)s.theta;
		obj->getMesh()->_rotation.z = (float)s.theta;

		// Debug logging
		//_logTimer += dt;

		//if (_logTimer >= 0.2) { // print every 0.2 seconds (5 logs per sec)
		//	LOG_INFO("theta = %f", theta);
		//	_logTimer = 0.0;
		//}
	}

	void PhysicsSystem::updateTranslation(double dt, elements::Object* obj) {
		if (!obj || !obj->getMesh()) return;

		auto& s = obj->state;

		// v * dt is displacement
		Eigen::Vector3d dp = s.linearVelocity * dt;

		auto mesh = obj->getMesh();
		mesh->_position += glm::vec3(dp.x(), dp.y(), dp.z());
	}

	// FORCE APPLICATION
	void PhysicsSystem::applyForces(double dt, elements::Object* obj) {
		if (!obj || !obj->getMesh()) return;

		auto& s = obj->state;

		// acceleration = F / m
		Eigen::Vector3d accel = s.forces / s.mass;

		// update velocity
		s.linearVelocity += accel * dt;

		// reset forces after applying
		s.forces.setZero();
	}

	void PhysicsSystem::applyTorque(double dt, elements::Object* obj, const Eigen::Vector3d& torque) {
		if (!obj || !obj->getMesh()) return;

	}

	void PhysicsSystem::applyDamping(double dt, elements::Object* obj, float dampingFactor) {
		if (!obj || !obj->getMesh()) return;

	}

	// COLLISION AND BOUNDARIES
	void PhysicsSystem::handleFloorCollision(double dt, elements::Object* obj, float floorY) {
		if (!obj || !obj->getMesh()) return;

	}

// --------------------------------------------------
//				   INTEGRATION (ODE)
// --------------------------------------------------

	// Low-Level Integrators
	void PhysicsSystem::integrateEuler(Eigen::VectorXd& x, Eigen::VectorXd& dxdt, double dt) {
		x = _ODE->eulerStep(x, dxdt, dt);
	}
	
	void PhysicsSystem::integrateRK2(Eigen::VectorXd& x, Eigen::VectorXd& dxdt, double dt) {
		// RK2 integration implementation
		//x = _ODE->rk2Step(x, _t, dt, dxdt);
	}

	void PhysicsSystem::integrateRK4(Eigen::VectorXd& x, Eigen::VectorXd& dxdt, double dt) {
		// RK4 integration implementation
		//x = _ODE->rk4Step(x, _t, dt, dxdt);
	}


} // namespace physics