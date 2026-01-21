#pragma once

//=============================================
//            File: PhysicsSystem.h
//=============================================
// Class responsible for managing the physics simulation system.
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
// ============================================
//			  GitHub: saltyjoss
// ============================================

#include "EngineCore.h"
#include "Numerics/IntegrationService.h"
#include "Numerics/ReferenceSolver.h"

#include <memory>
#include <string>
#include <vector>

#include "PhysicsState.h"
#include "Scene/Object.h"
#include "Platform/Logger.h"

namespace scene {
	class ENGINE_API Mesh;
	class ENGINE_API Object;
}

namespace physics {
	struct ENGINE_API IntegratorDiagSample {
		double t;
		Quat q_method;	// integrator quaternion
		Quat q_ref;		// reference quaternion
		Vec3 omega;		// integrator angular velocity
		double alpha;	// angle error
	};

	struct ENGINE_API ErrorSample {
		double t = 0.0; // simulation time
		Vec3 x;			// integrator position
		Vec3 x_ref;		// reference position
		Vec3 error;		// error between integrator and reference
	};

	struct ENGINE_API RefTrack {
		VecX x;
		double t = 0.0;
		double dt = 1e-3; // adaptive step size suggestion found, more research may show me a better default?
		bool init = false;
	};

	enum class FrameType {
		World,
		Body
	};

	struct ENGINE_API IntegratorDiagResult {
		double duration = 0.0; // total simulation duration
		Vec3 thetaMin, thetaMax, thetaMean, thetaRms;
		Vec3 omegaMin, omegaMax, omegaMean, omegaRms;

		integration::ErrorStats omegaNormStats = {};
		integration::ErrorStats thetaNormStats = {};
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

		void setIntegrationMethod(integration::eIntegrationMethod method) { _integrator->setIntegrationMethod(method); }
		integration::eIntegrationMethod getIntegrationMethod() const { return _integrator->getIntegrationMethod(); }

		void setFrameType(FrameType type) { _frame = type;  }
		FrameType getFrameType() const { return _frame; }

		// Simulation Control
		void update(double dt, scene::Object* obj);

		// System-Updates
		void updateRotation(double dt, scene::Object* obj);
		void updateRefRotation(double dt, scene::Object* obj);

		void updateTranslation(double dt, scene::Object* obj);

		void applyForces(double dt, scene::Object* obj);
		void applyTorque(double dt, scene::Object* obj, const Vec3& torque);
		void applyDamping(double dt, scene::Object* obj, float dampingCoefficient);

		void handleFloorCollision(double dt, scene::Object* obj, float floorY = 0.0f);

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
		std::unique_ptr<integration::IntegrationService> _integrator;
		std::unique_ptr<integration::ReferenceSolver> _refSolver;
		integration::eIntegrationMethod _curIntMethod{};

		mathlib::Vec3 _gravity = mathlib::Vec3(0.0f, -9.81f, 0.0f);

		// Integration analysis
		eSimulationMode _simulationMode = eSimulationMode::Normal;
		FrameType _frame = FrameType::World;

		std::vector<integration::ErrorSample> _errorSamples;

		bool _diagRunning = false;
		bool _simRunning = false;

		scene::Object* _diagObject = nullptr;
		std::vector<IntegratorDiagSample> _diagSamples;
		IntegratorDiagResult _diagResult;

		scene::Object* _refDiagObject = nullptr;
		std::vector<integration::ReferenceSolver::RefIntegratorDiagSample> _refDiagSamples;
		IntegratorDiagResult _refDiagResult;

		// Track reference states for each object
		std::unordered_map<scene::Object*, RefTrack> gRefTracks;

		integration::eIntegrationMethod method = integration::eIntegrationMethod::Euler; // default method

		// Simulation parameters
		double _dt = 1.0f / 120.0f; // ~120 FPS
		double _h = 1.0f;
		double _t = 0.0f;
	};
} // namespace physics