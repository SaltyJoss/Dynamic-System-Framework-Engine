
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
	void PhysicsSystem::update(double dt) {
		// THIS is going to be used for updating all objects in the scene
		// Currently no global updates needed
		// Not really needed yet, but keeping incase of future functionality
	}

// --------------------------------------------------
//				  PER-SYSTEM UPDATES
// --------------------------------------------------

	// MOTION UPDATES
	void PhysicsSystem::updateRotation(double dt, elements::Object* obj) {	// Euler angle vs Quaternion?? Make note for report, may try implelemtn
		if (!obj || !obj->getMesh()) return;

		auto& theta = obj->state.theta;
		auto& w = obj->state.angularVelocity;

		Eigen::VectorXd x(1);
		x(0) = theta;

		Eigen::VectorXd dxdt(1);
		dxdt(0) = w(1); // Rotation around y ---> (0) is x, (1) is y, (2) is z

		integrateEuler(x, dxdt, dt);
		theta = x(0);
		obj->getMesh()->_rotation.x = static_cast<float>(theta);
		obj->getMesh()->_rotation.y = static_cast<float>(theta);

		// Debug logging
		//_logTimer += dt;

		//if (_logTimer >= 0.2) { // print every 0.2 seconds (5 logs per sec)
		//	LOG_INFO("theta = %f", theta);
		//	_logTimer = 0.0;
		//}
	}

	void PhysicsSystem::updateTranslation(double dt, elements::Object* obj) {
		if (!obj || !obj->getMesh()) return;
	}

	// FORCE APPLICATION
	void PhysicsSystem::applyForces(double dt, elements::Object* obj) {
		if (!obj || !obj->getMesh()) return;

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