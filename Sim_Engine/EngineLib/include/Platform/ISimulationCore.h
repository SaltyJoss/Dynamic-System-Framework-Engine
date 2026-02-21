#pragma once
// File:   ISimulationCore.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include <cstdint>
#include <string>

// Forward Declarations
namespace integration { enum class eIntegrationMethod; }
namespace physics { class ENGINE_API PhysicsSystem; }
namespace robots { class ENGINE_API RobotSystem; }
namespace control { class ENGINE_API TrajectoryManager; }
namespace scene { class ENGINE_API Object; enum class ObjectID : std::uint32_t; }
namespace diagnostics { class ENGINE_API TelemetryRecorder; }
namespace interpreter { class ENGINE_API IStoredProgram; }

namespace core {
    // Headless API for the Simulation Core
    // Aimed at allowing scripts and other systems to interact with the simulation without direct access to the full SimulationCore implementation
    struct ENGINE_API ISimulationCore {
        virtual ~ISimulationCore() = default;

        // Simulation control
        virtual void startSimulation() = 0;
        virtual void stopSimulation() = 0;
        virtual bool isSimRunning() const = 0;
        // Time stepping
        virtual void updatePhysics(double dt) = 0;
        virtual void setFixedDt(double dt) = 0;
        virtual double fixedDt() const = 0;
        virtual double simTime() const = 0;
        // Integrator
        virtual void setIntegrationMethod(integration::eIntegrationMethod method) = 0;
		virtual std::string integrationMethodName() const = 0;
        // Subsystems
        virtual physics::PhysicsSystem* physicsSystem() = 0;
        virtual robots::RobotSystem* robotSystem() = 0;
        virtual control::TrajectoryManager* trajectoryManager() = 0;
		// Robot management
        virtual bool hasRobot() const = 0;
        virtual void loadRobot(const std::string& name) = 0;
        virtual void clearRobot() = 0;
		// Scene objects management
        virtual std::vector<std::unique_ptr<scene::Object>>& getObjects() = 0;
        virtual void deleteObject(int index) = 0;
        virtual std::vector<scene::Object*> loadMeshReturn(const std::string& path) = 0;
        // Scene lookup (DSL access)
        virtual scene::Object* getObject() = 0;
        virtual scene::Object* getObjectByID(scene::ObjectID id) = 0;
		// Script execution
        virtual bool runScriptToCompletion(interpreter::IStoredProgram* program, integration::eIntegrationMethod method) = 0;
        // Telemetry access
        virtual diagnostics::TelemetryRecorder& telemetry() = 0;
		virtual size_t telemetrySampleCount() const = 0;
    };
}