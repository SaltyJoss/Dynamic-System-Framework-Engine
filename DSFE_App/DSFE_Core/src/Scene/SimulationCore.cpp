/*
 * File: Scene/SimulationCore.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"
#include "Scene/SimulationCore.h"

#include "Systems/RigidBodySystem.h"
#include "Systems/RigidBodyModel.h"
#include "Systems/TrajectoryManager.h"
#include "SingleBodySystem/Body.h"

#include "DSL/StoredProgram.h"
#include "DSL/Parser.h"

#include "Platform/Paths.h"
#include "EngineLib/LogMacros.h"

namespace core {
	// Owned constructed subsystems (default)
	SimulationCore::SimulationCore()
		: _trajOwned(std::make_unique<control::TrajectoryManager>()), _rigidBodyOwned(std::make_unique<systems::RigidBodySystem>()),
		_singleBodyOwned(std::make_unique<single_body_system::SingleBodySystem>())
	{
		_traj = _trajOwned.get();
		_rigidBody = _rigidBodyOwned.get();
		_singleBody = _singleBodyOwned.get();

		startExportThread();
	}
	// Destructor (logs destruction for debugging purposes)
	SimulationCore::~SimulationCore() {
		stopExportThread();
		std::cout << "CORE DESTROYED\n"; 
	}

	// Non-owning constructor (used when subsystems are managed externally, e.g. by the SimulationManager)
	SimulationCore::SimulationCore(systems::RigidBodySystem& rigidBody, control::TrajectoryManager& traj)
		: _rigidBody(&rigidBody), _traj(&traj), _singleBody(nullptr) {
		startExportThread();
	}

	// Simulation System
	void SimulationCore::setupSimulationIntegrator() {
		if (!_rigidBody && !_singleBody) { return; }
		integration::IntegrationService* intgr;
		integration::DifferentiableIntegrator* adIntgr;
		if (_rigidBody) {
			intgr = _rigidBody->getIntegrator();
			adIntgr = _rigidBody->getADIntegrator();
		}
		if (_singleBody) {
			intgr = _singleBody->getIntegrator();
			adIntgr = _singleBody->getADIntegrator();
		}
		intgr->resetAdaptiveState();
		intgr->setAdaptiveTolerances(1e-3, 1e-6);
		intgr->setMaxStep(_dt);
		adIntgr->runtimeState()->last_dt_taken = _dt;
		adIntgr->runtimeState()->last_dt_sug = _dt;
	}
	// Set the integration method for the simulation (also updates the rigidBody's integrator if it exists)
	void SimulationCore::setIntegrationMethod(integration::eIntegrationMethod method) {
		if (!_rigidBody && !_singleBody) { return; }
		if (_singleBody) { _singleBody->setStandardIntegrator(method); }
		if (_rigidBody) { _rigidBody->setStandardIntegrator(method); }
	}
	// Set the auto-diff integration method for the simulation (also updates the rigidBody's AD integrator if it exists)
	void SimulationCore::setADIntegrationMethod(integration::eAutoDiffIntegrationMethod method) {
		if (!_rigidBody && !_singleBody) { return; }
		if (_singleBody) { _singleBody->setADIntegrator(method); }
		if (_rigidBody) { _rigidBody->setADIntegrator(method); }
	}
	// Get the name of the current integration method (returns "no_rigidBody" if no rigidBody is loaded)
	std::string SimulationCore::integrationMethodName() const {
		std::string intName;
		if (_rigidBody) {
			const auto state = _rigidBody->runtimeIntegratorState();
			intName = (_rigidBody->autoDiffEnabled()) ? _rigidBody->AD_integratorName() : _rigidBody->getIntegratorName();
			return intName;
		}
		else if(_singleBody) {
			const auto state = _singleBody->runtimeIntegratorState();
			intName = (_singleBody->autoDiffEnabled()) ? _singleBody->AD_integratorName() : _singleBody->getIntegratorName();
			return intName;
		}
		else { 
			intName = "no_system";
			return intName; 
		}	
	} 
	// Get the current integration method
	integration::eIntegrationMethod SimulationCore::integrationMethod() const {
		if (_rigidBody) { return _rigidBody->getIntegrationMethod(); }
		if (_singleBody) { return _singleBody->getIntegrationMethod(); }
		return integration::eIntegrationMethod::RK4;
	}
	// Get the current auto-diff integration method
	integration::eAutoDiffIntegrationMethod SimulationCore::autoDiffIntegrationMethod() const {
		if (_rigidBody) { return _rigidBody->AD_IntegrationMethod(); }
		if (_singleBody) { return _singleBody->AD_IntegrationMethod(); }
		return integration::eAutoDiffIntegrationMethod::AD_ImplicitEuler; // default return value
	}
	void SimulationCore::enableAutoDiff(bool enable) {
		if (!_rigidBody) { return; }
		_rigidBody->enableAutoDiff(enable);
	}
	bool SimulationCore::autoDiffEnabled() const {
		if (!_rigidBody) { return false; }
		return _rigidBody->autoDiffEnabled();
	}

	// SimulationCore
	void SimulationCore::setGravity(const mathlib::Vec3& g) { _rigidBody->setGravityVec(g); }
	mathlib::Vec3 SimulationCore::gravity() const { return _rigidBody->getGravityVec(); }

	// Fixed timestep loop for physics and rigidBody updates, called from the main render loop with the frame delta time
	void SimulationCore::stepFixed(double frame_dt) {
		double simTime = _simTime.load();

		_accum += frame_dt; // accumulate frame time to step the simulation in fixed increments of _dt
		// Step the simulation forward in fixed increments of _dt until we catch up to the current frame time
		while (_accum >= _dt) {
			// Step the active script program if running and check for completion or faults
			if (_scriptRunning.load() && _activeProgram) {
				_activeProgram->step(_dt);
				const bool completed = _activeProgram->isCompleted();
				const bool stopped = _activeProgram->isStopped();
				const bool faulted = _activeProgram->isFaulted();
				if (completed) {
					D_SUCCESS("SCRIPT END: completed=%d (dt=%.6f s, simTime=%.3f s)", (int)completed, _dt, simTime);
					_scriptRunning.store(false);
					_activeProgram = nullptr;
					stopSimulation();
					D_RUNTIME("Program execution completed.");
				}
				else if (stopped || faulted) {
					D_FAIL("SCRIPT END: stopped=%d faulted=%d (dt=%.6f s, simTime=%.3f s)", (int)stopped, (int)faulted, _dt, simTime);
					_scriptRunning.store(false);
					_activeProgram = nullptr;
					stopSimulation();
					D_RUNTIME("Program execution completed.");
				}
			}
			else if (_scriptRunning.load() && !_activeProgram) {
				D_FAIL("SCRIPT END: _scriptRunning=1 but _activeProgram=nullptr");
				_scriptRunning.store(false);
			}

			const auto state = _rigidBody->runtimeIntegratorState();

			// Update physics and rigidBody system if sim is running
			if (_simRunning.load()) {
				simTime += _dt;
				// Update rigidBody trajectory inputs and step the rigidBody forward in time
				if (hasRigidBody()) {
					_rigidBody->updateTrajectoryInputs(*_traj, simTime);
					_rigidBody->step(_dt, simTime);
					// Telemetry update
					if (!_telemetryBegun) {
						_telemetry.beginRun(simTime, _telHz, 300.0);
						_telemetryBegun = true;
						D_INFO_ONCE("Telemtry Capture Started (dt=%.6f s, simTime=%.3f s)", (1 / _telHz), simTime);
					}
					_telemetry.update(simTime, *_rigidBody, _traj, diagnostics::eTelemetryLevel::FULL);
				}
				if (hasSingleBody()) { _singleBody->step(_dt, simTime); }
			}
			else if (_manipulating.load() && hasRigidBody()) {
				_rigidBody->step(_dt, simTime);
			}

			_accum -= _dt; // decrease accumulator by fixed timestep until we catch up to the current frame time
		}

		if (_simRunning.load()) {
			_simTime.store(simTime);
		}
		else {
			_simTime.store(0.0, std::memory_order_relaxed);
		}
	}

	// Start the simulation loop
	void SimulationCore::startSimulation() {
		if (_simRunning.load()) { return; }
		telemetry().clear();
		D_RUNTIME("starting simulation");
		LOG_INFO("Starting Simulation -> Debug Log");

		_simTime.store(0.0, std::memory_order_relaxed);
		_accum = 0.0;

		// Reset simulation system
		if (_rigidBody) {
			_rigidBody->resetRigidBody();
			_trajRefBuffer.clear();

			// Determine expected number of entries based on run mode and cap it to prevent OOM
			double expectedMinutes = (_runMode == eRunMode::Synchronous) ? DEFAULT_SYNC_MINUTES : DEFAULT_INTERACTIVE_MINUTES;

			// Convert minutes to steps
			size_t joints = _rigidBody->jointCount();
			double expectedSeconds = expectedMinutes * 60.0;
			size_t steps = static_cast<size_t>(expectedSeconds / _dt);

			// Guarding against overflow and capping
			uint64_t total64 = static_cast<uint64_t>(steps) * static_cast<uint64_t>(joints);
			size_t total = static_cast<size_t>(std::min<uint64_t>(total64, MAX_LOG_ENTRIES));

			// Internal double buf for high-rate joint log
			_rigidBody->useInternalLogBuffer(true);
			_rigidBody->reserveInternalLogBuffers(total);

			// Keeps external traj ref buffer for lower-rate traj ref (going to refactor this later)
			_trajRefBuffer.clear();
			_trajRefBuffer.reserve(std::max<size_t>(1024, total / (26 / 5))); // 26 to 5 entries, so reserving 1/(26/5) of total steps as a heuristic for ref buffer size
			_rigidBody->setRefBuffer(&_trajRefBuffer);
		}
		if (_singleBody) {
			_singleBody->resetBody();
		}

		std::string int_name;
		int_name = integrationMethodName();

		LOG_INFO("Integrator: %s (AD=%d), dt=%.6f s, simTime=%.3f s", int_name.c_str(), (int)autoDiffEnabled(), _dt, _simTime.load());

		_data.setParentFolder(paths::runs().string());

		// Ensure reference sim system have their integrators configured for the new run
		setupSimulationIntegrator();
		_data.setIntegratorName(integrationMethodName());
		_data.setRunTag(_runTag);

		_simRunning.store(true);
		_telemetryBegun = false;
		_data.setEnabled(true);
	}

	// Stop the simulation loop
	void SimulationCore::stopSimulation() {
		if (!_simRunning.load()) { return; }
		D_RUNTIME("stopping simulation");

		auto buf = _rigidBody->claimExportLogBuffer(); // Claim the export log buffer from the rigidBody
		if (buf) { enqueueExportBuffer(std::move(buf)); }
		flushExports();

		// Clear buffers to free memory and prepare for next run
		_data.setEnabled(false);
		_simRunning.store(false);
		_telemetryBegun = false;

		_simTime.store(0.0, std::memory_order_relaxed);
		_accum = 0.0;
	}

	// Exporst the logged joint data to HDF5 format using the custom macro for each log entry
	void SimulationCore::exportLogsToHDF5(const systems::JointLogBuffer& exportBuf) {
		auto t0 = std::chrono::steady_clock::now();

		const auto state = _rigidBody->runtimeIntegratorState();
		const std::string intName = (state && state->autoDiff) ? _rigidBody->AD_integratorName() : _rigidBody->getIntegratorName();
		const std::string rigidBodyName = _rigidBody->hasRigidBody() ? _rigidBody->rigidBodyName() : "no_rigidBody";
		const std::string header = rigidBodyName + "_sim_" + intName;

		// Check if there are any log entries to export
		const size_t N = exportBuf.size();
		if (N == 0) {
			D_RUNTIME("ExportLogs has no data to export (buffer size is 0)");
			return;
		}

		_data.captureJointBuffer(
			data::Stream::Simulation,
			header, exportBuf
		);
		
		// Log export duration
		auto dur = std::chrono::steady_clock::now() - t0;
		LOG_INFO("ExportLogs -> wrote %zu samples in %.3f s", N, std::chrono::duration<double>(dur).count());
	}

	// Exports the reference trajectory data to HDF5 format using the custom macro for each ref entry
	void SimulationCore::exportRefsToHDF5() {
		const std::string rigidBodyName = _rigidBody->hasRigidBody() ? _rigidBody->rigidBodyName() : "no_rigidBody";
		const std::string header = rigidBodyName + "_traj_ref";

		// Check if there are any log entries
		const size_t N = _trajRefBuffer.size();
		if (N == 0) return;

		// Simple validation of buffer sizes
		if (_trajRefBuffer.theta_ref.size() != N ||
			_trajRefBuffer.omega_ref.size() != N ||
			_trajRefBuffer.alpha_ref.size() != N ||
			_trajRefBuffer.sim_time.size()  != N ||
			_trajRefBuffer.joint_index.size() != N) {
			D_FAIL("exportRefsToHDF5: TrajRefBuffer size mismatch");
			return; // Return as reference data is useless if sizes don't match
		}

		// For each ref entry, create a field list and write to HDF5
		for (size_t i = 0; i < N; ++i) {
			// Create a list of fields for this log entry
			data::FieldList fields;
			// Sim Metadata
			fields.emplace_back("sim_time", (double)_trajRefBuffer.sim_time[i]);
			// Reference values
			fields.emplace_back("theta_ref", (double)_trajRefBuffer.theta_ref[i]);
			fields.emplace_back("omega_ref", (double)_trajRefBuffer.omega_ref[i]);
			fields.emplace_back("alpha_ref", (double)_trajRefBuffer.alpha_ref[i]);
			// Joint info
			fields.emplace_back("joint_index", (double)_trajRefBuffer.joint_index[i]);

			// Write this entry to HDF5
			_data.capture(data::Stream::Reference, header, fields);
		}
	}

	// --------------------------------------------------
	//		   SYNCHRONOUS SCRIPT EXECUTION
	// --------------------------------------------------

	// Run a script synchronously to completion, blocking the main thread. Returns true if completed successfully
	bool SimulationCore::runScriptToCompletion(dsl::IStoredProgram* program, integration::eIntegrationMethod method) {
		// Map method enum to string name, purely for logging purposes
		static const char* names[] = { "euler", "midpoint", "heun", "ralston", "rk4", "rk45", "implicit_euler", "implicit_midpoint", "glrk2", "glrk3" };
		const std::string methodName = names[static_cast<int>(method)];

		LOG_INFO("SimulationCore::runScriptToCompletion -> START method=%s dt=%.6f hasRigidBody=%d", methodName.c_str(), _dt, (int)hasRigidBody());

		// Reset rigidBody state
		_rigidBody->resetRigidBody();
		_traj->clearAll();

		// Clear reference buffer (external for now)
		_trajRefBuffer.clear();
		// Inject reference buffer only
		_rigidBody->setRefBuffer(&_trajRefBuffer);
		// Set integrator on both physics and rigidBody systems
		_rigidBody->setStandardIntegrator(method);

		scriptParallelisation(program);

		D_SUCCESS("Synchronous run completed: %s (%.1fs, %zu samples)", methodName.c_str(), _simTime.load(), _telemetry.ring.size());
		LOG_INFO("SimulationCore::runScriptToCompletion -> END method=%s result=%d simTime=%.6f samples=%zu", methodName.c_str(), (int)(_telemetry.ring.size() >= 2), _simTime.load(), _telemetry.ring.size());
		return (_telemetry.ring.size() >= 2);
	}

	void SimulationCore::scriptParallelisation(dsl::IStoredProgram* program) {
		// Reset simulation state
		_simTime.store(0.0, std::memory_order_relaxed);
		_simRunning.store(false);
		_telemetryBegun = false;
		_accum = 0.0;

		_activeProgram = program;
		_scriptRunning.store(true);

		// Set run mode to synchronous for the duration of this run
		_runMode = eRunMode::Synchronous;

		const double dt = _dt;
		const int maxSteps = static_cast<int>((24.0 * 3600.0) / dt); // safety to prevent infinite loops in faulty scripts (max 24 hours of sim time)

		// enable sim stepping and telemetry for synchronous run
		startSimulation();

		LOG_INFO("SimulationCore::runScriptToCompletion -> startSimulation called; simRunning=%d simTime=%.6f", (int)_simRunning, _simTime.load());

		// Run tight simulation loop until program completes
		double simTime = _simTime.load();

		// Main loop: step the program and simulation until completion
		for (int step = 0; step < maxSteps; ++step) {
			// Check program completion
			if (program->isCompleted() || program->isFaulted() || program->isStopped()) {
				LOG_INFO("SimulationCore::runScriptToCompletion -> program end detected at step=%d completed=%d faulted=%d stopped=%d", step, (int)program->isCompleted(), (int)program->isFaulted(), (int)program->isStopped());
				break;
			}

			// Step the program (DSL command execution)
			program->step(dt);

			// Step physics and rigidBody if sim is running
			if (_simRunning.load()) {
				simTime += dt;
				if (hasRigidBody()) {
					// Update Trajectory Inputs
					_rigidBody->updateTrajectoryInputs(*_traj, simTime);
					_rigidBody->step(dt, simTime);
					// Telemetry beginRun
					if (!_telemetryBegun) {
						_telemetry.beginRun(simTime, _telHz, 300.0);
						_telemetryBegun = true;
						D_INFO_ONCE("Telemtry Capture Started (dt=%.6f s, simTime=%.3f s)", (1 / _telHz), simTime);
					}
					// Telemetry update
					_telemetry.update(simTime, *_rigidBody, _traj, diagnostics::eTelemetryLevel::FULL);
				}
			}
		}

		if (_simRunning.load()) {
			_simTime.store(simTime);
		}
		else {
			_simTime.store(0.0);
		}

		// Clean up
		stopSimulation();

		LOG_INFO("SimulationCore::runScriptToCompletion -> stopSimulation called; simTime=%.6f telemetry_samples=%zu", _simTime.load(), _telemetry.ring.size());
		// Reset run mode to interactive (default)
		_runMode = eRunMode::Interactive;

		_activeProgram = nullptr;
		_scriptRunning.store(false);
		_simRunning.store(false);
		_telemetryBegun = false;
	}
	
	// Setter for fixed timestep duration
	void SimulationCore::setFixedDt(double dt) { _dt = dt; }
	// Getter for fixed timestep duration
	double SimulationCore::fixedDt() const { return _dt; }
	// Getter for current simulation time
	double SimulationCore::simTime() const {
		double t = _simTime.load();
		return t;
	}

	// Method to step the simulation with a fixed timestep
	void SimulationCore::tick(double frame_dt) { stepFixed(frame_dt); }

	// --- Setters and Getters for Systems and State ---

	// Accessor for the rigidBody system (non-const and const versions)
	systems::RigidBodySystem& SimulationCore::rigidBodySystem() { return *_rigidBody; }
	// Setter and checker for RigidBody System
	void SimulationCore::setRigidBodySystem(systems::RigidBodySystem* rigidBody) { _rigidBody = rigidBody; }
	bool SimulationCore::hasRigidBody() const { return _rigidBody && _rigidBody->hasRigidBody(); }
	// Loads a rigidBody into the rigidBody system by name
	void SimulationCore::loadRigidBody(const std::string& name) {
		loadRigidBodyInternal(name);
		_rigidBodyPresentationDirty = true;
	}
	// Internal method to load a rigidBody, assumes ownership of the rigidBody system
	void SimulationCore::loadRigidBodyInternal(const std::string& name) {
		if (!_rigidBody) { LOG_ERROR("Cannot load rigidBody: RigidBodySystem not set"); return; }
		_rigidBody->loadRigidBody(name);
	}
	// Sets an external force on a specific link of the rigidBody system at a given world point
	bool SimulationCore::setLinkExternalForce(const std::string& link, const mathlib::Vec3& worldPoint, const mathlib::Vec3& worldForce) {
		return _rigidBody->setLinkExtForce(link, worldPoint, worldForce);
	}
	// Accessor for the world transforms of the rigidBody links (const version)
	const std::vector<mathlib::Mat4>& SimulationCore::linkWorldTransforms() const { return _rigidBody->worldTransforms(); }
	// Accessor for the names of the rigidBody links (const version)
	std::vector<std::string> SimulationCore::linkNames() const { return _rigidBody->linkNames(); }
	// Clears all external forces applied to the rigidBody system
	void SimulationCore::clearExternalForces() { _rigidBody->clearExtForces(); }

	// Accessor for the single body system (non-const and const versions)
	single_body_system::SingleBodySystem& SimulationCore::singleBodySystem() { return *_singleBody; }
	// Setter and checker for Single Body System
	void SimulationCore::setSingleBodySystem(single_body_system::SingleBodySystem* singleBody) { _singleBody = singleBody; }
	bool SimulationCore::hasSingleBody() const { return _singleBody && _singleBody->hasBody(); }
	// Loads a single body into the single body system by name
	void SimulationCore::loadSingleBody(const std::string& name) {
		loadSingleBodyInternal(name);
		_singleBodyPresentationDirty = true;
	}
	// Internal method to load a single body, assumes ownership of the single body system
	void SimulationCore::loadSingleBodyInternal(const std::string& name) {
		if (!_singleBody) { LOG_ERROR("Cannot load single body: SingleBodySystem not set"); return; }
		_singleBody->loadBody(name);
	}

	// Setter for the trajectory manager
	void SimulationCore::setTrajectoryManager(control::TrajectoryManager* traj) { _traj = traj; }
	// Accessor for the trajectory manager (non-const and const versions)
	control::TrajectoryManager& SimulationCore::trajectoryManager() { return *_traj; }

	// Setters for the metric buffers
	void SimulationCore::setJointLogBuffer(systems::JointLogBuffer* buf) { _jointLogBuffer = *buf; }
	void SimulationCore::setTrajRefBuffer(systems::TrajRefBuffer* buf) { _trajRefBuffer = *buf; }
	
	// Accessor for the telemetry recorder (non-const and const versions)
	diagnostics::TelemetryRecorder& SimulationCore::telemetry() { return _telemetry; }
	// Get the current number of telemetry samples recorded
	size_t SimulationCore::telemetrySampleCount() const { return _telemetry.ring.size(); }

	// Setter and getter for the active script program
	void SimulationCore::setActiveProgram(dsl::IStoredProgram* p) { _activeProgram = p; }
	dsl::IStoredProgram* SimulationCore::activeProgram() const { return _activeProgram; }

	// Thread-based methods

	// Starts the export thread if it's not already running
	void SimulationCore::startExportThread() {
		if (_expThreadRunning.exchange(true)) { return; }
		_expThread = std::thread([this]() { 
			exportThreadMain();
		});
	}

	// Signals the export thread to stop and waits for it to finish
	void SimulationCore::stopExportThread() {
		if (!_expThreadRunning.exchange(false)) { return; }
		_expCondVar.notify_all();
		if (_expThread.joinable()) {
			_expThread.join();
		}
	}

	// Main loop for the export thread, waits for export buffers to be enqueued and processes them
	void SimulationCore::exportThreadMain() {
		while (true) {
			std::unique_ptr<systems::JointLogBuffer> buf;
			{
				std::unique_lock<std::mutex> lock(_expMutex);
				_expCondVar.wait(lock, [this]() {
					return !_expQ.empty() || !_expThreadRunning.load();
				});
				if (!_expThreadRunning.load() && _expQ.empty()) { break; }
				buf = std::move(_expQ.front());
				_expQ.pop();
			}

			if (buf) {
				try {
					exportLogsToHDF5(*buf);
				}
				catch (...) {
					LOG_ERROR("Export failed");
				}
				--_exportsInFlight;
			}
		}
	}

	void SimulationCore::enqueueExportBuffer(std::unique_ptr<systems::JointLogBuffer> buf) {
		{
			std::lock_guard<std::mutex> lock(_expMutex);
			++_exportsInFlight;
			_expQ.push(std::move(buf));
		}
		_expCondVar.notify_one();
	}

	void SimulationCore::flushExports() {
		while (_exportsInFlight.load(std::memory_order_acquire) > 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}

	void SimulationCore::setManipulating(bool on) {
		if (on == _manipulating.load()) { return; }
		if (on) {
			setupSimulationIntegrator();
			_accum = 0.0;
		}
		_manipulating.store(on);
		if (!on) { _rigidBody->clearExtForces(); }   // drop any residual drag force
	}
}