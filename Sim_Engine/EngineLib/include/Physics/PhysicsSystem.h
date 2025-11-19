#pragma once
#include "EngineCore.h"
#include <MathLibAPI.h>
#include <integrators/numerical_integrators.h>

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
}

namespace physics {
	class ENGINE_API PhysicsSystem {
	public:
		PhysicsSystem();
		
		// Simulation Control
		void update(double dt);


		// System-Updates
		void updateRotation(double dt, elements::Object* obj);
		void updateTranslation(double dt, elements::Object* obj);

		void applyForces(double dt, elements::Object* obj);
		void applyTorque(double dt, elements::Object* obj, const Eigen::Vector3d& torque);
		void applyDamping(double dt, elements::Object* obj, float dampingFactor = 0.98f);

		void handleFloorCollision(double dt, elements::Object* obj, float floorY = 0.0f);

		// Low-Level Integrators
		void integrateEuler(Eigen::VectorXd& x, Eigen::VectorXd& dxdt, double dt);
		void integrateRK2(Eigen::VectorXd& x, Eigen::VectorXd& dxdt, double dt);
		void integrateRK4(Eigen::VectorXd& x, Eigen::VectorXd& dxdt, double dt);

		// Config
		void setGravity(const Eigen::Vector3d& gravity) { _gravity = gravity; }
		Eigen::Vector3d getGravity() const { return _gravity; }
			
		
	private:
		std::unique_ptr<integration::ODE> _ODE;

		Eigen::Vector3d _gravity = Eigen::Vector3d(0.0f, -9.81f, 0.0f);

		// Simulation parameters
		double _dt = 0.016f; // ~60 FPS
		double _h = 1.0f;
		double _t = 0.0f;

		// global coefficients
		double _angularDamping = 0.98f;
		double _linearDamping = 0.98f;

		// Debugging
		double _logTimer = 0.0;
	};
} // namespace physics