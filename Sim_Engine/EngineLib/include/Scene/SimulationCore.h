#pragma once
// File:   SimulationCore.h
// GitHub: SaltyJoss
#include "EngineCore.h"
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

	class ENGINE_API SimulationCore {
	public:
		SimulationCore();

		// Physics
		void updatePhysics(double dt);
		void tick(double frame_dt);
		void stepFixed(double frame_dt);

		// Simulation Integrators
		void setupSimulationIntegrator();

		// Start or stop the simulation loop
		void startSimulation();
		void stopSimulation();
		const bool isSimRunning() const { return _simRunning; }

		// Export logged telemetry data to HDF5 files
		void exportLogsToHDF5();
		void exportRefsToHDF5();

		// Setter and getter for current simulation time (seconds)
		void setSimTime(double t) { _simTime = t; }
		const double simTime() const { return _simTime; }

		// Increment simulation time by dt (used in the simulation loop)
		void incrementSimTime(double dt) { _simTime += dt; }

		// Script Running State
		void setScriptRunning(bool running) { _scriptRunning = running; }
		const bool isScriptRunning() const { return _scriptRunning; }

		// Setter and getter for fixed timestep (seconds)
		void setFixedDt(double dt) { _dt = dt; }
		const double fixedDt() const { return _dt; }

		// Setter and getter for telemetry frequency (Hz)
		void setTelemetryHz(double hz) { _telHz = hz; }
		const double telemetryHz() const { return _telHz; }

		// Last script text (stored on run for comparison re-use)
		void setLastScriptText(const std::string& text) { _lastScriptText = text; }
		const std::string& lastScriptText() const { return _lastScriptText; }

		// Run a script to completion synchronously with a specific integrator
		bool runScriptToCompletion(interpreter::IStoredProgram* program, integration::eIntegrationMethod method);

		// Setter for the physics system
		void setPhysicsSystem(physics::PhysicsSystem* physics);

		// Accessor for the physics system (non-const and const versions)
		physics::PhysicsSystem* physicsSystem();
		const physics::PhysicsSystem* physicsSystem() const;

		// Setter and checker for Robot System
		void setRobotSystem(robots::RobotSystem* robot);
		bool hasRobot() const;

		// Accessor for the robot system (non-const and const versions)
		robots::RobotSystem* robotSystem();
		const robots::RobotSystem* robotSystem() const;

		// Setter for the trajectory manager
		void setTrajectoryManager(control::TrajectoryManager* traj);

		// Accessor for the trajectory manager (non-const and const versions)
		control::TrajectoryManager* trajectoryManager();
		const control::TrajectoryManager* trajectoryManager() const;

		// Setters the scene objects pointer (used for object lookup by scripts)
		void setObjects(std::vector<std::unique_ptr<scene::Object>>* objects);

		// Setters for the metric buffers
		void setJointLogBuffer(robots::JointLogBuffer* buffer);
		void setTrajRefBuffer(robots::TrajRefBuffer* buffer);

		// Accesors for the telemetry recorder (non-const and const versions)
		diagnostics::TelemetryRecorder& telemetry();
		const diagnostics::TelemetryRecorder& telemetry() const;

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

		// Run mode (interactive vs synchronous)
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