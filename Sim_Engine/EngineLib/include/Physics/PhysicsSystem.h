#pragma once

//=============================================
//            File: PhysicsSystem.h
//=============================================
// Class responsible for managing the physics simulation system.
//
// Summary:
// ============================================
// 
// structures & enumeratiors:
// --------------------------------------------
// struct IntegratorDiagSample
//      -> Structure to hold diagnostic samples for integrator analysis.
// struct IntegratorDiagResult
//      -> Structure to hold the results of integrator diagnostics.
// enum class eSimulationMode
//      -> Enumeration for simulation modes (Normal, IntegrationAnalysis).
// enum class eIntegrationMethod
//      -> Enumeration of available numerical integration methods (Euler, Midpoint, Heun, Ralston, RK4).
// --------------------------------------------
//
// public:
// --------------------------------------------
// PhysicsSystem()
//      -> Constructor that initializes the physics system.
// void update(double dt, scene::Object* obj)
//      -> Updates the physics simulation for the given object over the time step dt.
// void updateRotation(double dt, scene::Object* obj)
//      -> Updates the rotation of the object based on its angular velocity.
// void updateTranslation(double dt, scene::Object* obj)
//      -> Updates the translation of the object based on its linear velocity.
// void applyForces(double dt, scene::Object* obj)
//      -> Applies forces to the object, updating its linear velocity.
// void applyTorque(double dt, scene::Object* obj, const Vec3& torque)
//      -> Applies torque to the object, updating its angular velocity.
// void applyDamping(double dt, scene::Object* obj, float dampingFactor = 0.98f)
//      -> Applies damping to the object's velocities to simulate energy loss.
// void handleFloorCollision(double dt, scene::Object* obj, float floorY = 0.0f)
//      -> Handles collision of the object with a floor at the specified Y position.
// VectorXd integrationMethod(Eigen::VectorXd& x, double t, double dt, std::function<Eigen::VectorXd(double, const Eigen::VectorXd&)> f, eIntegrationMethod method)
//      -> Performs a single integration step using the specified eIntegrationMethod (Euler, Midpoint, Heun, Ralston, RK4).
// void setIntegrationMethod(eIntegrationMethod m)
//      -> Sets the current integration method.
// eIntegrationMethod getIntegrationMethod() const
//      -> Returns the current integration method.
// void setGravity(const Vec3& gravity)
//      -> Sets the gravity vector for the physics simulation.
// Vec3 getGravity()
//      -> Returns the current gravity vector
// void startDiagnostics(scene::Object* obj)
//      -> Starts the integration diagnostics for the specified object.
// void stopDiagnostics()
//      -> Stops the integration diagnostics.
// bool diagnosticsRunning() const
// 		-> Returns whether diagnostics are currently running.
// void setDiagnosticRunning(bool running)
// 		-> Sets the diagnostics running state.
// const IntegratorDiagResult& diagResult() const
//      -> Returns the results of the integration diagnostics.
// const std::vector<IntegratorDiagSample>& diagSamples() const
//      -> Returns the samples collected during integration diagnostics.
// --------------------------------------------
//
// private:
// --------------------------------------------
// std::unique_ptr<integration::ODE> _ODE
//      -> Unique pointer to the ODE integrator.
// constants::MathConstants _const
//      -> Instance of mathematical constants.
// Vec3 _gravity
//      -> Gravity vector for the simulation.
// eSimulationMode _simulationMode
//      -> Current simulation mode (Normal, IntegrationAnalysis).
// std::vector<integration::ErrorSample> _errorSamples
//      -> Vector of error samples for integration analysis.
// bool _diagRunning
//	    -> Indicates whether diagnostics are currently running.
// scene::Object* _diagObject
//      -> Pointer to the object being diagnosed.
// std::vector<IntegratorDiagSample> _diagSamples
//      -> Vector of samples collected during diagnostics.
// IntegratorDiagResult _diagResult
//      -> Results of the integration diagnostics.
// double _dt
//      -> Time step for the simulation.
// double _h
//      -> Simulation parameter h.
// double _t
//      -> Simulation time variable.
// double _logTimer
//      -> Timer for logging purposes.
// --------------------------------------------
// 
// ============================================
//			  GitHub: saltyjoss
// ============================================

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include <integrators/numerical_integrators.h>
#include <integrators/IntegrationAnalysis.h>

#include <memory>
#include <string>
#include <vector>

#include "PhysicsState.h"
#include "Scene/Object.h"
#include "Platform/Logger.h"

extern ENGINE_API Debug gLog;
using namespace integration;
using namespace constants;
using namespace mathlib;

namespace scene {
	class Mesh;
	class Object;
}

namespace constants {
	class PhysConstants;
}

namespace physics {
	struct ENGINE_API IntegratorDiagSample {
		double t = 0.0;              // simulation time
		Vec3 theta;       // angles (rad)
		Vec3 omega;       // angular velocity (rad/s)
	};

	struct ENGINE_API IntegratorDiagResult {
		double duration;
		Vec3 thetaMin, thetaMax, thetaMean, thetaRms;
		Vec3 omegaMin, omegaMax, omegaMean, omegaRms;

		integration::ErrorStats omegaNormStats;
		integration::ErrorStats thetaNormStats;
	};

	class ENGINE_API PhysicsSystem {
	public:
		PhysicsSystem();
		
		// Mode Selection
		enum class eSimulationMode {
			Normal = 0,				// standard physics simulation
			IntegrationAnalysis = 1 // run comparative tests of integrators
		};

		void setSimulationMode(eSimulationMode mode) { _simulationMode = mode; }
		eSimulationMode getSimulationMode() const { return _simulationMode; }

		// Simulation Control
		void update(double dt, scene::Object* obj);

		// System-Updates
		void updateRotation(double dt, scene::Object* obj);
		void updateTranslation(double dt, scene::Object* obj);

		void applyForces(double dt, scene::Object* obj);
		void applyTorque(double dt, scene::Object* obj, const Vec3& torque);
		void applyDamping(double dt, scene::Object* obj, float dampingCoefficient);

		void handleFloorCollision(double dt, scene::Object* obj, float floorY = 0.0f);

		// Integrator Methods
		enum class eIntegrationMethod {
			Euler = 0,		// First-Order Euler Method
			Midpoint = 1,	// Second-Order Runge-Kutta (Midpoint)
			Heun = 2,		// Second-Order Runge-Kutta (Heun)
			Ralston = 3,	// Second-Order Runge-Kutta (Ralston)
			RK4 = 4			// Fourth-Order Runge-Kutta 
		};
		
		VecX integrationMethod(VecX& x, double t, double dt, std::function<Eigen::VectorXd(double, const VecX&)> f, eIntegrationMethod method);

		eIntegrationMethod method = eIntegrationMethod::Euler;
		void setIntegrationMethod(eIntegrationMethod m) { method = m; }
		eIntegrationMethod getIntegrationMethod() const { return method; }

		// Config
		void setGravity(const Vec3& gravity) { _gravity = gravity; }
		Vec3 getGravity() const { return _gravity; }

		// Integration Analysis testing
		void startDiagnostics(scene::Object* obj);
		void stopDiagnostics();

		bool diagnosticsRunning() const { return _diagRunning; }
		void setDiagnosticRunning(bool running) { _diagRunning = running; }

		const IntegratorDiagResult& diagResult() const { return _diagResult; }
		const std::vector<IntegratorDiagSample>& diagSamples() const { return _diagSamples; }

	private:
		std::unique_ptr<integration::ODE> _ODE;
		Vec3 _gravity = Vec3(0.0f, -9.81f, 0.0f);

		// Integration analysis
		eSimulationMode _simulationMode = eSimulationMode::Normal;

		std::vector<integration::ErrorSample> _errorSamples;

		bool _diagRunning = false;
		scene::Object* _diagObject = nullptr;

		std::vector<IntegratorDiagSample> _diagSamples;
		IntegratorDiagResult _diagResult;

		// Simulation parameters
		double _dt = 1.0f / 120.0f; // ~120 FPS
		double _h = 1.0f;
		double _t = 0.0f;
	};
} // namespace physics