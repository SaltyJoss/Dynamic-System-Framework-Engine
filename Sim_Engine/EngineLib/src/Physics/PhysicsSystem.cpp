// ==================================================
//				File: PhysicsSystem.cpp
// ==================================================

#include "pch.h"
#include "Physics/PhysicsSystem.h"

#include "Scene/Object.h"
#include "Scene/Mesh.h"
#include "integrators/Integration.h"
#include "const_phys.h"

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
	// Rotation update
	void PhysicsSystem::updateRotation(double dt, elements::Object* obj) {	// Euler angle vs Quaternion?? Make note for report, may try implelemtn
		if (!obj || !obj->getMesh()) return;

		auto& s = obj->state;
		std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> f;

		// state vector = [theta_x, theta_y, theta_z]
		Eigen::VectorXd x(3);
		x(0) = s.theta.x();
		x(1) = s.theta.y();
		x(2) = s.theta.z();

		// derivative = angular velocity (rad/s)
		Eigen::VectorXd dxdt(3);
		dxdt(0) = s.angularVelocity.x();
		dxdt(1) = s.angularVelocity.y();
		dxdt(2) = s.angularVelocity.z();

		// define derivative function for RK2/RK4
		f = [&](double t, const Eigen::VectorXd& state) -> Eigen::VectorXd {
			Eigen::VectorXd deriv(3);
			deriv(0) = s.angularVelocity.x();
			deriv(1) = s.angularVelocity.y();
			deriv(2) = s.angularVelocity.z();
			return deriv;
		};

		// Euler integrate using your ODE helper
		Eigen::VectorXd next = integrationMethod(x, dxdt, 0.0, dt, f, method);

		// write back to state
		s.theta.x() = next(0);
		s.theta.y() = next(1);
		s.theta.z() = next(2);

		// WRAP ANGLES INTO [-pi, pi] OR [0, 2pi]
		auto wrapRad = [](double a) -> double {
			a = fmod(a, constants::PhysConstants::TWO_PI);
			if (a < 0.0) a += constants::PhysConstants::TWO_PI;
			return a;
			};

		s.theta.x() = wrapRad(s.theta.x());
		s.theta.y() = wrapRad(s.theta.y());
		s.theta.z() = wrapRad(s.theta.z());

		// sync to mesh (still radians!)
		obj->transform.rotation.x = static_cast<float>(s.theta.x());
		obj->transform.rotation.y = static_cast<float>(s.theta.y());
		obj->transform.rotation.z = static_cast<float>(s.theta.z());
	}

	// Translation update
	void PhysicsSystem::updateTranslation(double dt, elements::Object* obj) {
		if (!obj || !obj->getMesh()) return;

		auto& s = obj->state;

		// v * dt is displacement
		Eigen::Vector3d dp = s.linearVelocity * dt;

		obj->transform.position += glm::vec3(dp.x(), dp.y(), dp.z());
	}

	// Force application
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

	// Torque application
	void PhysicsSystem::applyTorque(double dt, elements::Object* obj, const Eigen::Vector3d& torque) {
		if (!obj || !obj->getMesh()) return;

	}

	// Damping application
	void PhysicsSystem::applyDamping(double dt, elements::Object* obj, float dampingFactor) {
		if (!obj || !obj->getMesh()) return;

	}

	// Collision handling
	void PhysicsSystem::handleFloorCollision(double dt, elements::Object* obj, float floorY) {
		if (!obj || !obj->getMesh()) return;

	}

// --------------------------------------------------
//				   INTEGRATION (ODE)
// --------------------------------------------------
	VectorXd PhysicsSystem::integrationMethod(Eigen::VectorXd& x, Eigen::VectorXd& dxdt, double t, double dt, std::function<Eigen::VectorXd(double, const Eigen::VectorXd &)> f, eIntegrationMethod method) {
		if (method == eIntegrationMethod::Euler) {
			return _ODE->eulerStep(x, dxdt, dt);
		}
		else if (method == eIntegrationMethod::RK2) {
			return _ODE->rk2Step(x, t, dt, f);
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


} // namespace physics