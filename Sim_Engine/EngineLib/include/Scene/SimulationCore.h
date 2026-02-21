#pragma once
// File:   SimulationCore.h
// GitHub: SaltyJoss
#include "EngineCore.h"
#include "Platform/ISimulationCore.h"
#include <memory>
#include <string>
#include "Platform/SimulationState.h"
#include "Analysis/Telemetry.h"
#include "Analysis/MetricLogger.h"

#include "Platform/Logger.h"

// Forward Declarations
namespace integration { enum class eIntegrationMethod; }
namespace control	  { class ENGINE_API TrajectoryManager; }
namespace scene		  { class ENGINE_API Object; }
namespace physics	  { class ENGINE_API PhysicsSystem; }
namespace robots	  { class ENGINE_API RobotSystem; }
namespace interpreter { class ENGINE_API IStoredProgram; }

namespace core {
	// configurable defaults (not part of class to allow tuning without recompilation)
	constexpr double DEFAULT_INTERACTIVE_MINUTES = 60.0; // long runs for interactive mode
	constexpr double DEFAULT_SYNC_MINUTES = 10.0;        // short runs for synchronous mode
	constexpr size_t MAX_LOG_ENTRIES = 50'000'000;     // hard cap to avoid OutOfMemory crashes

	class ENGINE_API SimulationCore : public ISimulationCore {
	public:
		SimulationCore();

		// Simulation control
		void startSimulation() override;
		void stopSimulation() override;
		bool isSimRunning() const override { return _simRunning; }

		// Time stepping
		void updatePhysics(double dt) override;
		void setFixedDt(double dt) override;
		void setSimTime(double t) { _simTime = t; }
		double fixedDt() const override;
		double simTime() const override;

		// Integrator
		void setupSimulationIntegrator();
		void setIntegrationMethod(integration::eIntegrationMethod method) override;
		std::string integrationMethodName() const override;

		// Subsystems access
		physics::PhysicsSystem* physicsSystem() override;
		const physics::PhysicsSystem* physicsSystem() const;
		robots::RobotSystem* robotSystem() override;
		const robots::RobotSystem* robotSystem() const;
		control::TrajectoryManager* trajectoryManager() override;
		const control::TrajectoryManager* trajectoryManager() const;
		
		// Robot management
		bool hasRobot() const override;
		void loadRobot(const std::string& name) override;
		void clearRobot() override;

		// Scene objects management
		std::vector<std::unique_ptr<scene::Object>>& getObjects() override;
		void deleteObject(int index) override;
		std::vector<scene::Object*> loadMeshReturn(const std::string& path) override;
		// Object lookup
		scene::Object* getObject() override;
		scene::Object* getObjectByID(scene::ObjectID id) override;

		// Run a script to completion synchronously with a specific integrator
		bool runScriptToCompletion(interpreter::IStoredProgram* program, integration::eIntegrationMethod method) override;

		// Telemetry
		diagnostics::TelemetryRecorder& telemetry() override;
		const diagnostics::TelemetryRecorder& telemetry() const;
		size_t telemetrySampleCount() const override;

		// Setters for subsystems and scene objects
		void setPhysicsSystem(physics::PhysicsSystem* physics);
		void setRobotSystem(robots::RobotSystem* robot);
		void setTrajectoryManager(control::TrajectoryManager* traj);
		void setObjects(std::vector<std::unique_ptr<scene::Object>>* objects);
		void setJointLogBuffer(robots::JointLogBuffer* buffer);
		void setTrajRefBuffer(robots::TrajRefBuffer* buffer);

		// Helpers
		void tick(double frame_dt);
		void stepFixed(double frame_dt);

		// Export logged telemetry data to HDF5 files
		void exportLogsToHDF5();
		void exportRefsToHDF5();

		// Increment simulation time by dt (used in the simulation loop)
		void incrementSimTime(double dt) { _simTime += dt; }

		// Script Running State
		void setScriptRunning(bool running) { _scriptRunning = running; }
		const bool isScriptRunning() const { return _scriptRunning; }

		// Setter and getter for telemetry frequency (Hz)
		void setTelemetryHz(double hz) { _telHz = hz; }
		const double telemetryHz() const { return _telHz; }

		// Last script text (stored on run for comparison re-use)
		void setLastScriptText(const std::string& text) { _lastScriptText = text; }
		const std::string& lastScriptText() const { return _lastScriptText; }

		// Accesors for the active script program
		void setActiveProgram(interpreter::IStoredProgram* p);
		interpreter::IStoredProgram* activeProgram() const;

	private:
		// Simulation Timing
		double _dt = 1.0 / 180.0;		// [seconds], fixed timestep duration for physics updates
		double _telHz = 120.0;			// [Hz], controls how often telemetry updates during simulation runs
		double _accum = 0.0;			// Accumulator for fixed timestep
		double _simTime = 0.0;			// Current simulation time
		bool _simRunning = false;		// Whether the simulation loop is currently running
		bool _scriptRunning = false;	// Whether a script is currently running 

		// Core Systems
		robots::RobotSystem* _robot = nullptr;
		physics::PhysicsSystem* _physics = nullptr;
		control::TrajectoryManager* _traj = nullptr;
		std::vector<std::unique_ptr<scene::Object>>* _objects = nullptr;

		// Run mode
		eRunMode _runMode = eRunMode::Interactive;

		// Last script text for comparison re-use
		std::string _lastScriptText;

		// Active Script Program
		interpreter::IStoredProgram* _activeProgram = nullptr;

		// Telemetry
		diagnostics::TelemetryRecorder _telemetry; // Dynamic telemetry recorder
		robots::JointLogBuffer _jointLogBuffer;    // Buffer for logging joint data each step
		robots::TrajRefBuffer _trajRefBuffer;      // Buffer for logging trajectory reference data each step
		bool _telemetryBegun = false;
	};
}