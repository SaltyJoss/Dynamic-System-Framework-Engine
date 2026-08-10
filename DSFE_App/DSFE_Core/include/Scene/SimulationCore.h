/*
 * File: Scene/SimulationCore.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"

#include <queue>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <memory>

#include "Platform/ISimulationCore.h"
#include "Platform/SimulationState.h"
#include "Numerics/IntegratorState.h"
#include "physics/CollisionResolver.h"

#include "Analysis/Telemetry.h"
#include "Platform/DataManager.h"

#include "Analysis/MetricLogger.h"
#include "Platform/Logger.h"

// Forward Declarations
namespace control	  { class TrajectoryManager; }
namespace systems	  { class RigidBodySystem; }
namespace single_body_system { class SingleBodySystem; }
namespace dsl { class IStoredProgram; }

namespace core {
	// configurable defaults (not part of class to allow tuning without recompilation)
	inline constexpr double DEFAULT_INTERACTIVE_MINUTES = 60.0; // long runs for interactive mode
	inline constexpr double DEFAULT_SYNC_MINUTES = 10.0;        // short runs for synchronous mode
	inline constexpr size_t MAX_LOG_ENTRIES = 50'000'000;     // hard cap to avoid OutOfMemory crashes

	class DSFE_API SimulationCore : public ISimulationCore {
	public:
		SimulationCore();
		~SimulationCore();

		SimulationCore(systems::RigidBodySystem& sys, control::TrajectoryManager& traj);

		// Buffer queue for exporting sim outputs
		void startExportThread();
		void stopExportJointThread();
		void stopExportFreeBodyThread();
		void enqueueJointExportBuffer(std::unique_ptr<systems::JointLogBuffer> buf);
		void enqueueFreeBodyExportBuffer(std::unique_ptr<systems::FreeBodyLogBuffer> buf);
		void flushExports();

		// Simulation control
		void startSimulation() override;
		void stopSimulation() override;
		bool isSimRunning() const override { return _simRunning.load(); }

		// Time stepping
		void setFixedDt(double dt) override;
		double fixedDt() const override;
		void setSimTime(double t) override { _simTime = t; }
		double simTime() const override;

		SimulationSnapshot snapshot() const override {
			std::lock_guard<std::mutex> lock(_stateMutex); // Ensure thread-safe access to snapshot data
			return SimulationSnapshot{
				_simTime.load(),
				_simRunning.load(),
				_scriptRunning.load()
			};
		}

		// Integrator
		void setupSimulationIntegrator();
		void setIntegrationMethod(integration::eIntegrationMethod method) override;
		void setADIntegrationMethod(integration::eAutoDiffIntegrationMethod method) override;
		std::string integrationMethodName() const override;
		integration::eIntegrationMethod integrationMethod() const override;
		integration::eAutoDiffIntegrationMethod autoDiffIntegrationMethod() const override;
		void enableAutoDiff(bool enable) override;
		bool autoDiffEnabled() const override;

		// Physics and dynamics
		void setGravity(const mathlib::Vec3& g) override;
		mathlib::Vec3 gravity() const override;

		// Simulation run tag (used for logging and data management)
		void setRunTag(const std::string& tag) override { _runTag = tag; }

		// Accessors for the simulation state
		systems::RigidBodySystem& rigidBodySystem() override;
		single_body_system::SingleBodySystem& singleBodySystem() override;
		control::TrajectoryManager& trajectoryManager() override;
		
		// Body management
		bool hasSingleBody() const override;
		void loadSingleBody(const std::string& name) override;
		void loadSingleBodyInternal(const std::string& name); // Internal method that assumes ownership

		// RigidBody management
		bool hasRigidBody() const override;
		void loadRigidBody(const std::string& name) override;
		void loadRigidBodyInternal(const std::string& name); // Internal method that assumes ownership
		void resetRigidBody() override; // Reset the rigidBody system to its initial state, clearing any loaded rigidBody and resetting the simulation state
		// Body management for multiple rigid bodies
		std::size_t bodyCount() const override;
		systems::RigidBodySystem& body(int i) override;
		const systems::RigidBodySystem& body(int i) const override;
		int activeBodyIdx() const override;
		void setActiveBody(int i) override;
		void clearBodies() override;

		// Collision
		physlib::collision::Capsule makeCapsule(const systems::RigidBodyLink& link, const mathlib::Mat4& world_T) override;

		// Run a script to completion synchronously with a specific integrator
		bool runScriptToCompletion(dsl::IStoredProgram* program, integration::eIntegrationMethod method) override;

		// Telemetry
		diagnostics::TelemetryRecorder& telemetry() override;
		size_t telemetrySampleCount() const override;

		// Setters for subsystems and scene objects
		void setRigidBodySystem(systems::RigidBodySystem* sys);
		void setSingleBodySystem(single_body_system::SingleBodySystem* singleBody);
		void setTrajectoryManager(control::TrajectoryManager* traj);

		void setJointLogBuffer(systems::JointLogBuffer* buffer);
		void setFreeBodyLogBuffer(systems::FreeBodyLogBuffer* buffer);

		// Helpers
		void tick(double frame_dt) override;
		void stepFixed(double frame_dt);

		// Export logged telemetry data to HDF5 files
		void exportLogsToHDF5_j(const systems::JointLogBuffer& buf);
		void exportLogsToHDF5_fb(const systems::FreeBodyLogBuffer& buf);

		// Increment simulation time by dt (used in the simulation loop)
		void incrementSimTime(double dt) {
			double newSimTime = _simTime.load() + dt;
			_simTime.store(newSimTime);
		}

		// Script Running State
		void setScriptRunning(bool running) override { _scriptRunning.store(running); }
		bool isScriptRunning() const override { return _scriptRunning.load(); }

		// Setter and getter for telemetry frequency (Hz)
		void setTelemetryHz(double hz) override { _telHz = hz; }
		double telemetryHz() const override { return _telHz; }

		// Last script text (stored on run for comparison re-use)
		void setLastScriptText(const std::string& text) override { _lastScriptText = text; }
		std::string& lastScriptText() override { return _lastScriptText; }

		// Accesors for the active script program
		void setActiveProgram(dsl::IStoredProgram* p) override;
		dsl::IStoredProgram* activeProgram() const override;

		bool rigidBodyPresentationDirty() const override { return _rigidBodyPresentationDirty; }
		void clearRigidBodyPresentationDirty() override { _rigidBodyPresentationDirty = false; }

		void setManipulating(bool on) override;
		bool isManipulating() const override { return _manipulating.load(); }
		bool setLinkExternalForce(const std::string& link, const mathlib::Vec3& worldPoint, const mathlib::Vec3& worldForce) override;
		void clearExternalForces() override;
		const std::vector<mathlib::Mat4>& linkWorldTransforms() const override;
		std::vector<std::string> linkNames() const override;

	private:
		// Export thread management
		void exportJointThreadMain();
		void exportFreeBodyThreadMain();
		void scriptParallelisation(dsl::IStoredProgram* program);

		// Export thread for joint telemetry
		std::thread _expThread_j;
		std::mutex _expMutex_j; // Mutex for joint export queue
		std::condition_variable _expCondVar_j; // Condition variable for export thread synchronization (need one for each queue)
		std::queue<std::unique_ptr<systems::JointLogBuffer>> _expQ_j;
		std::atomic<bool> _expThreadRunning_j{ false };

		// Export thread for free body telemetry
		std::thread _expThread_fb;
		std::mutex _expMutex_fb; // Mutex for free body export queue#
		std::condition_variable _expCondVar_fb; // Condition variable for export thread synchronization (need one for each queue)
		std::queue<std::unique_ptr<systems::FreeBodyLogBuffer>> _expQ_fb;
		std::atomic<bool> _expThreadRunning_fb{ false };

		// Owning storage (used only in owning mode)
		std::unique_ptr<systems::RigidBodySystem> _rigidBodyOwned;
		std::unique_ptr<single_body_system::SingleBodySystem> _singleBodyOwned;
		std::unique_ptr<control::TrajectoryManager> _trajOwned;
		// Non-owning access (always used by logic)
		systems::RigidBodySystem* _rigidBody = nullptr;
		single_body_system::SingleBodySystem* _singleBody = nullptr;
		control::TrajectoryManager* _traj = nullptr;
		// Owned bodies (for managing multiple rigidbodies)
		std::vector<std::unique_ptr<systems::RigidBodySystem>> _bodiesOwned;
		int _activeBodyIdx = -1;

		// Collision Resolver
		physics::CollisionResolver _collisionResolver;

		mutable std::mutex _stateMutex;

		// Simulation Timing
		double _dt = 1.0 / 180.0;		// [seconds], fixed timestep duration for physics updates
		double _telHz = 120.0;			// [Hz], controls how often telemetry updates during simulation runs
		double _accum = 0.0;			// Accumulator for fixed timestep

		std::atomic<double> _simTime{ 0.0 };		// Current simulation time
		std::atomic<bool> _simRunning{ false };		// Whether the simulation loop is currently running
		std::atomic<bool> _scriptRunning{ false };	// Whether a script is currently running
		std::atomic<int> _exportsInFlight = 0;
		std::atomic<bool> _manipulating{ false }; 

		// Run mode
		eRunMode _runMode = eRunMode::Interactive;

		// Last script text for comparison re-use
		std::string _lastScriptText;
		std::string _runTag;

		// Active Script Program
		dsl::IStoredProgram* _activeProgram = nullptr;
		bool _rigidBodyPresentationDirty = false;
		bool _singleBodyPresentationDirty = false;

		// Telemetry
		diagnostics::TelemetryRecorder _telemetry; // Dynamic telemetry recorder
		systems::JointLogBuffer _jointLogBuffer;    // Buffer for logging joint data each step
		systems::FreeBodyLogBuffer _freeBodyLogBuffer; // Buffer for logging free body data each step
		bool _telemetryBegun = false;

		data::DataManager _data; // Data manager for handling telemetry data export and storage
	};
} // namespace core