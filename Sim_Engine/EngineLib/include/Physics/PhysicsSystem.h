#pragma once

//=============================================
//            File: PhysicsSystem.h
//=============================================
// Class responsible for managing the physics simulation system.
//
// Summary:
// ============================================
//
// public:
// --------------------------------------------
// PhysicsSystem()
//      -> Constructor that initializes the physics system.
// void update(double dt, elements::Object* obj)
//      -> Updates the physics simulation for the given object over the time step dt.
// void updateRotation(double dt, elements::Object* obj)
//      -> Updates the rotation of the object based on its angular velocity.
// void updateTranslation(double dt, elements::Object* obj)
//      -> Updates the translation of the object based on its linear velocity.
// void applyForces(double dt, elements::Object* obj)
//      -> Applies forces to the object, updating its linear velocity.
// void applyTorque(double dt, elements::Object* obj, const Eigen::Vector3d& torque)
//      -> Applies torque to the object, updating its angular velocity.
// void applyDamping(double dt, elements::Object* obj, float dampingFactor = 0.98f)
//      -> Applies damping to the object's velocities to simulate energy loss.
// void handleFloorCollision(double dt, elements::Object* obj, float floorY = 0.0f)
//      -> Handles collision of the object with a floor at the specified Y position.
// void integrateEuler(Eigen::VectorXd& x, Eigen::VectorXd& dxdt, double dt)
//      -> Integrates the state vector x using the Euler method.
// void integrateRK2(Eigen::VectorXd& x, Eigen::VectorXd& dxdt, double dt)
//      -> Integrates the state vector x using the second-order Runge-Kutta method.
// void integrateRK4(Eigen::VectorXd& x, Eigen::VectorXd& dxdt, double dt)
//      -> Integrates the state vector x using the fourth-order Runge-Kutta method.
// void setGravity(const Eigen::Vector3d& gravity)
//      -> Sets the gravity vector for the physics simulation.
// Eigen::Vector3d getGravity()
//      -> Returns the current gravity vector.
// bool isGravityEnabled() const
//      -> Checks if gravity is enabled in the simulation.
// --------------------------------------------
//
// private:
// --------------------------------------------
// std::unique_ptr<integration::ODE> _ODE
//      -> Unique pointer to the ODE integrator.
// constants::MathConstants _const
//      -> Instance of mathematical constants.
// Eigen::Vector3d _gravity
//      -> Gravity vector for the simulation.
// double _dt
//      -> Time step for the simulation.
// double _h
//      -> Simulation parameter h.
// double _t
//      -> Simulation time variable.
// double _angularDamping
//      -> Coefficient for angular damping.
// double _linearDamping
//      -> Coefficient for linear damping.
// double _logTimer
//      -> Timer for logging purposes.
// --------------------------------------------
// 
// ============================================

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <integrators/numerical_integrators.h>
#include "const_math.h"

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "PhysicsState.h"
#include "Scene/Object.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;
using namespace integration;

namespace elements {
	class Mesh;
	class Object;
}

namespace constants {
	class PhysConstants;
}

namespace physics {

	class ENGINE_API PhysicsSystem {
	public:
		PhysicsSystem();
		
		// Simulation Control
		void update(double dt, elements::Object* obj);

		// System-Updates
		void updateRotation(double dt, elements::Object* obj);
		void updateTranslation(double dt, elements::Object* obj);

		void applyForces(double dt, elements::Object* obj);
		void applyTorque(double dt, elements::Object* obj, const Eigen::Vector3d& torque);
		void applyDamping(double dt, elements::Object* obj, float dampingCoefficient);

		void handleFloorCollision(double dt, elements::Object* obj, float floorY = 0.0f);

		// Integrator Methods
		enum class eIntegrationMethod {
			Euler = 0,		// First-Order Euler Method
			Midpoint = 1,	// Second-Order Runge-Kutta (Midpoint)
			Heun = 2,		// Second-Order Runge-Kutta (Heun)
			Ralston = 3,	// Second-Order Runge-Kutta (Ralston)
			RK4 = 4			// Fourth-Order Runge-Kutta 
		};
		
		VectorXd integrationMethod(Eigen::VectorXd& x, double t, double dt, std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> f, eIntegrationMethod method);

		eIntegrationMethod method = eIntegrationMethod::Euler;
		void setIntegrationMethod(eIntegrationMethod m) { method = m; }
		eIntegrationMethod getIntegrationMethod() const { return method; }

		// Config
		void setGravity(const Eigen::Vector3d& gravity) { _gravity = gravity; }
		Eigen::Vector3d getGravity() const { return _gravity; }

		// Misc
		bool isGravityEnabled() const { return _gravity != Eigen::Vector3d(0.0f, 0.0f, 0.0f); }
		
	private:
		std::unique_ptr<integration::ODE> _ODE;
		constants::MathConstants _const;
		Eigen::Vector3d _gravity = Eigen::Vector3d(0.0f, -9.81f, 0.0f);

		// Simulation parameters
		double _dt = 1.0f / 120.0f; // ~120 FPS
		double _h = 1.0f;
		double _t = 0.0f;
	};
} // namespace physics