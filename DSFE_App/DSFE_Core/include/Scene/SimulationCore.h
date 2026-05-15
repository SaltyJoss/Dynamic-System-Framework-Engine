// DSFE_Core SimulationCore.h
#pragma once

#include "EngineCore.h"

#include <queue>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <memory>

#include "Platform/ISimulationCore.h"
#include "Platform/SimulationState.h"

#include <memory>
#include <string>

#include "Analysis/Telemetry.h"
#include "Platform/DataManager.h"

#include "Analysis/MetricLogger.h"
#include "Platform/Logger.h"

// Forward Declarations
namespace integration { enum class eIntegrationMethod; }
namespace control	  { class TrajectoryManager; }
namespace robots	  { class RobotSystem; }
namespace interpreter { class IStoredProgram; }

namespace core {
	// configurable defaults (not part of class to allow tuning without recompilation)
	constexpr double DEFAULT_INTERACTIVE_MINUTES = 60.0; // long runs for interactive mode
	constexpr double DEFAULT_SYNC_MINUTES = 10.0;        // short runs for synchronous mode
	constexpr size_t MAX_LOG_ENTRIES = 50'000'000;     // hard cap to avoid OutOfMemory crashes

	class DSFE_API SimulationCore : public ISimulationCore {
	public:
		SimulationCore();
		~SimulationCore();

		SimulationCore(robots::RobotSystem& robot, control::TrajectoryManager& traj);

		// Buffer queue for exporting sim outputs
		void startExportThread();
		void stopExportThread();
		void enqueueExportBuffer(std::unique_ptr<robots::JointLogBuffer> buf);

		// Simulation control
		void startSimulation() override;
		void stopSimulation() override;
		bool isSimRunning() const override { return _simRunning.load(); }

		// Time stepping
		void setFixedDt(double dt) override;
		void setSimTime(double t) { _simTime = t; }
		double fixedDt() const override;
		double simTime() const override;

		SimulationSnapshot snapshot() const override {
			std::lock_guard<std::mutex> lock(_stateMutex); // Ensure thread-safe access to snapshot data
			return SimulationSnapshot{
				.simTime = _simTime.load(),
				.simRunning = _simRunning.load(),
				.scriptRunning = _scriptRunning.load()
			};
		}

		// Integrator
		void setupSimulationIntegrator();
		void setIntegrationMethod(integration::eIntegrationMethod method) override;
		std::string integrationMethodName() const override;
		integration::eIntegrationMethod integrationMethod() const override;
		void setRunTag(const std::string& tag) override { _runTag = tag; }

		// Subsystems access
		robots::RobotSystem* robotSystem() override;
		const robots::RobotSystem* robotSystem() const;
		control::TrajectoryManager* trajectoryManager() override;
		const control::TrajectoryManager* trajectoryManager() const;
		
		// Robot management
		bool hasRobot() const override;
		void loadRobot(const std::string& name) override;
		void loadRobotInternal(const std::string& name); // Internal method that assumes ownership

		// Run a script to completion synchronously with a specific integrator
		bool runScriptToCompletion(interpreter::IStoredProgram* program, integration::eIntegrationMethod method) override;

		// Telemetry
		diagnostics::TelemetryRecorder& telemetry() override;
		const diagnostics::TelemetryRecorder& telemetry() const;
		size_t telemetrySampleCount() const override;

		// Setters for subsystems and scene objects
		void setRobotSystem(robots::RobotSystem* robot);
		void setTrajectoryManager(control::TrajectoryManager* traj);
		void setJointLogBuffer(robots::JointLogBuffer* buffer);
		void setTrajRefBuffer(robots::TrajRefBuffer* buffer);

		// Helpers
		void tick(double frame_dt);
		void stepFixed(double frame_dt);

		// Export logged telemetry data to HDF5 files
		void exportLogsToHDF5(const robots::JointLogBuffer& buf);
		void exportRefsToHDF5();

		// Increment simulation time by dt (used in the simulation loop)
		void incrementSimTime(double dt) {
			double newSimTime = _simTime.load() + dt;
			_simTime.store(newSimTime);
		}

		// Script Running State
		void setScriptRunning(bool running) { _scriptRunning.store(running); }
		const bool isScriptRunning() const { return _scriptRunning.load(); }

		// Setter and getter for telemetry frequency (Hz)
		void setTelemetryHz(double hz) { _telHz = hz; }
		const double telemetryHz() const { return _telHz; }

		// Last script text (stored on run for comparison re-use)
		void setLastScriptText(const std::string& text) { _lastScriptText = text; }
		const std::string& lastScriptText() const { return _lastScriptText; }

		// Accesors for the active script program
		void setActiveProgram(interpreter::IStoredProgram* p);
		interpreter::IStoredProgram* activeProgram() const;

		bool robotPresentationDirty() const { return _robotPresentationDirty; }
		void clearRobotPresentationDirty() { _robotPresentationDirty = false; }

	private:
		// Export thread management
		void exportThreadMain();

		std::thread _expThread;
		std::mutex _expMutex;
		std::condition_variable _expCondVar;
		std::queue<std::unique_ptr<robots::JointLogBuffer>> _expQ;
		std::atomic<bool> _expThreadRunning{ false };

		// Owning storage (used only in owning mode)
		// std::unique_ptr<std::vector<std::unique_ptr<scene::Object>>> _objectsOwned;
		std::unique_ptr<robots::RobotSystem> _robotOwned;
		std::unique_ptr<control::TrajectoryManager> _trajOwned;

		// Non-owning access (always used by logic)
		// std::vector<std::unique_ptr<scene::Object>>* _objects = nullptr;
		robots::RobotSystem* _robot = nullptr;
		control::TrajectoryManager* _traj = nullptr;

		mutable std::mutex _stateMutex;

		// Simulation Timing
		double _dt = 1.0 / 180.0;		// [seconds], fixed timestep duration for physics updates
		double _telHz = 120.0;			// [Hz], controls how often telemetry updates during simulation runs
		double _accum = 0.0;			// Accumulator for fixed timestep

		std::atomic<double> _simTime{ 0.0 };		// Current simulation time
		std::atomic<bool> _simRunning{ false };		// Whether the simulation loop is currently running
		std::atomic<bool> _scriptRunning{ false };	// Whether a script is currently running
		std::atomic<int> _exportsInFlight = 0;

		// Run mode
		eRunMode _runMode = eRunMode::Interactive;

		// Last script text for comparison re-use
		std::string _lastScriptText;
		std::string _runTag;

		// Active Script Program
		interpreter::IStoredProgram* _activeProgram = nullptr;
		bool _robotPresentationDirty = false;

		// Telemetry
		diagnostics::TelemetryRecorder _telemetry; // Dynamic telemetry recorder
		robots::JointLogBuffer _jointLogBuffer;    // Buffer for logging joint data each step
		robots::TrajRefBuffer _trajRefBuffer;      // Buffer for logging trajectory reference data each step
		bool _telemetryBegun = false;

		data::DataManager _data; // Data manager for handling telemetry data export and storage
	};
} // namespace core