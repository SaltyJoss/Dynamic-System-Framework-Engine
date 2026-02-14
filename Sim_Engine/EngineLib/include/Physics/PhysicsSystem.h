#pragma once
// File:   PhysicsSystem.h
// GitHub: SaltyJoss
#pragma warning(disable : 4251)
#include "EngineCore.h"
#include "Numerics/IntegrationService.h"

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

		void setIntegrationMethod(integration::eIntegrationMethod m) { _integrator->setIntegrationMethod(m); }
		integration::eIntegrationMethod getIntegrationMethod() const { return _integrator->getIntegrationMethod(); }

		void setFrameType(FrameType type) { _frame = type;  }
		FrameType getFrameType() const { return _frame; }

		// Simulation Control
		void update(double dt, scene::Object* obj);

		// System-Updates
		void updateRotation(double dt, scene::Object* obj);

		void updateTranslation(double dt, scene::Object* obj);

		void applyForces(double dt, scene::Object* obj);
		void applyTorque(double dt, scene::Object* obj, const Vec3& torque);
		void applyDamping(double dt, scene::Object* obj, float dampingCoefficient);

		void handleFloorCollision(double dt, scene::Object* obj, float floorY = 0.0f);

		// Config
		void setGravity(const Vec3& gravity) { _gravity = gravity; }
		Vec3 getGravity() const { return _gravity; }

	private:
		std::unique_ptr<integration::IntegrationService> _integrator;
		integration::eIntegrationMethod _curIntMethod{};

		mathlib::Vec3 _gravity = mathlib::Vec3(0.0f, -9.81f, 0.0f);

		// Integration analysis
		eSimulationMode _simulationMode = eSimulationMode::Normal;
		FrameType _frame = FrameType::World;

		bool _simRunning = false;

		integration::eIntegrationMethod method = integration::eIntegrationMethod::Euler; // default method

		// Simulation parameters
		double _h = 1.0f;
		double _t = 0.0f;
	};
} // namespace physics