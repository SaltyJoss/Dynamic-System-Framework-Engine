#include "pch.h"
// File:   SimulationCore.cpp
// GitHub: SaltyJoss
#include "Scene/SimulationCore.h"
#include "Physics/PhysicsSystem.h"
#include "Robots/RobotSystem.h"
#include "Robots/RobotModel.h"
#include "Robots/TrajectoryManager.h"

#include "Assets/MeshLoader.h"

#include "Interpreter/StoredProgram.h"
#include "Interpreter/Parser.h"

#include "Platform/Paths.h"
#include "EngineLib/LogMacros.h"
#include "Platform/DataManager.h"

namespace core {
	// Constructor
	SimulationCore::SimulationCore() {}
	SimulationCore::~SimulationCore() {
		printf("CORE DESTROYED\n"); 
	}

	// Update physics for all objects in the scene using the physics system
	void SimulationCore::updatePhysics(double dt) {
		if (!_objects || !_physics) { return; }
		for (auto& obj : *_objects) {
			if (obj) { _physics->update(dt, obj.get()); }
		}
	}

	// Simulation System
	void SimulationCore::setupSimulationIntegrator() {
		if (!_robot) return;
		auto* intgr = _robot->getIntegrator();
		intgr->resetAdaptiveState();
		intgr->setAdaptiveTolerances(1e-3, 1e-6);
		intgr->setMaxStep(_dt);
	}
	// Set the integration method for the simulation (also updates the robot's integrator if it exists)
	void SimulationCore::setIntegrationMethod(integration::eIntegrationMethod method) {
		if (!_robot) { return; }
		_robot->getIntegrator()->setIntegrationMethod(method);
	}
	// Get the name of the current integration method (returns "no_robot" if no robot is loaded)
	std::string SimulationCore::integrationMethodName() const {
		if (!_robot) { return "no_robot"; }
		return _robot->getIntegratorName();
	} 
	// Get the current integration method
	integration::eIntegrationMethod SimulationCore::integrationMethod() const {
		if (!_robot) { return integration::eIntegrationMethod::RK4; }
		return _robot->getIntegrator()->getIntegrationMethod();
	}

	// Fixed timestep loop for physics and robot updates, called from the main render loop with the frame delta time
	void SimulationCore::stepFixed(double frame_dt) {
		_accum += frame_dt; // accumulate frame time to step the simulation in fixed increments of _dt
		// Step the simulation forward in fixed increments of _dt until we catch up to the current frame time
		while (_accum >= _dt) {
			// Step the active script program if running and check for completion or faults
			if (_scriptRunning && _activeProgram) {
				_activeProgram->step(_dt);
				const bool completed = _activeProgram->isCompleted();
				const bool stopped = _activeProgram->isStopped();
				const bool faulted = _activeProgram->isFaulted();
				if (completed) {
					D_SUCCESS("SCRIPT END: completed=%d (dt=%.6f s, simTime=%.3f s)", (int)completed, _dt, _simTime);
					_scriptRunning = false;
					_activeProgram = nullptr;
					stopSimulation();
					D_RUNTIME("Program execution completed.");
				}
				else if (stopped || faulted) {
					D_FAIL("SCRIPT END: stopped=%d faulted=%d (dt=%.6f s, simTime=%.3f s)", (int)stopped, (int)faulted, _dt, _simTime);
					_scriptRunning = false;
					_activeProgram = nullptr;
					stopSimulation();
					D_RUNTIME("Program execution completed.");
				}
			}
			else if (_scriptRunning && !_activeProgram) {
				D_FAIL("SCRIPT END: _scriptRunning=1 but _activeProgram=nullptr");
				_scriptRunning = false;
			}
			// Update physics and robot system if sim is running
			if (_simRunning) {
				_simTime += _dt;
				updatePhysics(_dt);
				// Update robot trajectory inputs and step the robot forward in time
				if (hasRobot()) {
					_robot->updateTrajectoryInputs(*_traj, _simTime);
					_robot->step(_dt, _simTime);
					// Telemetry update
					if (!_telemetryBegun) {
						_telemetry.beginRun(_simTime, _telHz, 300.0);
						_telemetryBegun = true;
						D_INFO_ONCE("Telemtry Capture Started (dt=%.6f s, simTime=%.3f s)", (1 / _telHz), _simTime);
					}
					_telemetry.update(_simTime, *_robot, _traj.get(), diagnostics::eTelemetryLevel::FULL);
				}
			}
			_accum -= _dt; // decrease accumulator by fixed timestep until we catch up to the current frame time
		}
	}

	// Start the simulation loop
	void SimulationCore::startSimulation() {
		if (_simRunning) return;
		telemetry().clear();
		D_RUNTIME("starting simulation");

		_simTime = 0.0;
		_accum = 0.0;

		// Reset simulation system
		if (_robot) {
			_robot->resetRobot();
			_trajRefBuffer.clear();

			// Determine expected number of entries based on run mode and cap it to prevent OOM
			double expectedMinutes = (_runMode == eRunMode::Synchronous) ? DEFAULT_SYNC_MINUTES : DEFAULT_INTERACTIVE_MINUTES;

			// Convert minutes to steps
			size_t joints = _robot->jointCount();
			double expectedSeconds = expectedMinutes * 60.0;
			size_t steps = static_cast<size_t>(expectedSeconds / _dt);

			// Guarding against overflow and capping
			uint64_t total64 = static_cast<uint64_t>(steps) * static_cast<uint64_t>(joints);
			size_t total = static_cast<size_t>(std::min<uint64_t>(total64, MAX_LOG_ENTRIES));

			// Internal double buf for high-rate joint log
			_robot->useInternalLogBuffer(true);
			_robot->reserveInternalLogBuffers(total);

			// Keeps external traj ref buffer for lower-rate traj ref (going to refactor this later)
			_trajRefBuffer.clear();
			_trajRefBuffer.reserve(std::max<size_t>(1024, total / (26 / 5))); // 26 to 5 entries, so reserving 1/(26/5) of total steps as a heuristic for ref buffer size
			_robot->setRefBuffer(&_trajRefBuffer);
		}

		_data.setParentFolder(paths::runs().string());

		// Ensure reference sim system have their integrators configured for the new run
		setupSimulationIntegrator();
		_data.setIntegratorName(integrationMethodName());
		_data.setRunTag(_runTag);

		_simRunning = true;
		_telemetryBegun = false;
		_data.setEnabled(true);
	}

	// Stop the simulation loop
	void SimulationCore::stopSimulation() {
		if (!_simRunning) { return; }
		D_RUNTIME("stopping simulation");

		// Export references
		if (_trajRefBuffer.size() > 0) {
			exportRefsToHDF5();
			_trajRefBuffer.clear();
		}

		// Claims export buffer and writes it for joint logs
		exportLogsToHDF5();

		// Clear buffers to free memory and prepare for next run
		_data.setEnabled(false);
		_simRunning = false;
		_telemetryBegun = false;
	}

	// Exporst the logged joint data to HDF5 format using the custom macro for each log entry
	void SimulationCore::exportLogsToHDF5() {
		auto t0 = std::chrono::steady_clock::now(); // start timer for export duration measurement
		// Construct a header for the HDF5 dataset based on the robot and integrator names
		const std::string intName = _robot->getIntegratorName();
		const std::string robotName = _robot->hasRobot() ? _robot->robotName() : "no_robot";
		const std::string header = robotName + "_sim_" + intName;

		// Claim the export log buffer from the robot (swap is internal!)
		robots::JointLogBuffer* exportBuf = _robot->claimExportLogBuffer();
		if (!exportBuf) { return; }

		// Validation check to ensure we have data to export
		std::string vmsg;
		if (!exportBuf->validate(&vmsg)) {
			D_FAIL("ExportLogs -> validation failed for export buffer: %s", vmsg.c_str());
			// Not returning, data is exported even if validation fails
		}

		// Check if there are any log entries to export
		const size_t N = exportBuf->size();
		if (N == 0) {
			D_RUNTIME("ExportLogs -> no data to export (buffer size is 0)"); // *REMNINDER* -> SHOULD I make a macro for ExportLogs?
			exportBuf->clear();
			return;
		}

		// For each log entry, create a field list and write to HDF5
		for (size_t i = 0; i < N; ++i) {
			data::FieldList fields;
			// Sim Metadata
			fields.emplace_back("sim_time",    (double)exportBuf->sim_time[i]);
			fields.emplace_back("dt_taken",    (double)exportBuf->dt_taken[i]);
			fields.emplace_back("dt_sug",      (double)exportBuf->dt_sug[i]);
			// States
			fields.emplace_back("theta",       (double)exportBuf->theta[i]);
			fields.emplace_back("omega",       (double)exportBuf->omega[i]);
			fields.emplace_back("alpha",       (double)exportBuf->alpha[i]);
			fields.emplace_back("err",         (double)exportBuf->err[i]);
			fields.emplace_back("err_d",       (double)exportBuf->err_d[i]);
			// Dynamics
			fields.emplace_back("I_eff",       (double)exportBuf->I_eff[i]);
			fields.emplace_back("tau",         (double)exportBuf->tau[i]);
			fields.emplace_back("tau_fb",      (double)exportBuf->tau_fb[i]);
			fields.emplace_back("tau_coriolis",(double)exportBuf->tau_coriolis[i]);
			fields.emplace_back("tau_gravity", (double)exportBuf->tau_gravity[i]);
			fields.emplace_back("tau_damping", (double)exportBuf->tau_damping[i]);
			fields.emplace_back("tau_friction",(double)exportBuf->tau_friction[i]);
			fields.emplace_back("tau_barrier", (double)exportBuf->tau_barrier[i]);
			fields.emplace_back("tau_sat",     (double)exportBuf->tau_sat[i]);
			// Energy, Work, & Power
			fields.emplace_back("KE",          (double)exportBuf->KE[i]);
			fields.emplace_back("PE",          (double)exportBuf->PE[i]);
			fields.emplace_back("E_total",     (double)exportBuf->E_total[i]);
			fields.emplace_back("W_actuator",  (double)exportBuf->W_actuator[i]);
			fields.emplace_back("P_damping",   (double)exportBuf->P_damping[i]);
			fields.emplace_back("P_friction",  (double)exportBuf->P_friction[i]);
			// Limit flags and info
			fields.emplace_back("clamp_theta", (double)exportBuf->clamp_theta[i]);
			fields.emplace_back("clamp_omega", (double)exportBuf->clamp_omega[i]);
			fields.emplace_back("sat_flag",    (double)exportBuf->sat_flag[i]);
			// Joint info
			fields.emplace_back("joint_index", (double)exportBuf->joint_index[i]);

			// Write entry to HDF5
			_data.capture(data::Stream::Simulation, header, fields);
		}
		// Log export duration
		auto dur = std::chrono::steady_clock::now() - t0;
		LOG_INFO("ExportLogs -> wrote %zu samples in %.3f s", N, std::chrono::duration<double>(dur).count());
		D_RUNTIME("ExportLogs -> wrote %zu samples in %.3f s", N, std::chrono::duration<double>(dur).count());

		// Clear exported buffer
		exportBuf->clear();
		D_RUNTIME("ExportLogs -> export buffer cleared");
		LOG_INFO("ExportLogs -> export buffer cleared");

		// Log success
		D_SUCCESS("ExportLogs -> export completed successfully");
		LOG_INFO("ExportLogs -> export completed successfully");

		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	// Exports the reference trajectory data to HDF5 format using the custom macro for each ref entry
	void SimulationCore::exportRefsToHDF5() {
		const std::string robotName = _robot->hasRobot() ? _robot->robotName() : "no_robot";
		const std::string header = robotName + "_traj_ref";

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
	bool SimulationCore::runScriptToCompletion(interpreter::IStoredProgram* program, integration::eIntegrationMethod method) {
		// Map method enum to string name, purely for logging purposes
		static const char* names[] = { "euler", "midpoint", "heun", "ralston", "rk4", "rk45" };
		const std::string methodName = names[static_cast<int>(method)];

		// Reset robot state
		_robot->resetRobot();
		_traj->clearAll();

		// Clear reference buffer (external for now)
		_trajRefBuffer.clear();
		// Inject reference buffer only
		_robot->setRefBuffer(&_trajRefBuffer);

		// Reset simulation state
		_simTime = 0.0;
		_simRunning = false;
		_telemetryBegun = false;
		_accum = 0.0;

		// Set integrator on both physics and robot systems
		_robot->setIntegrationMethod(method);
		_physics->setIntegrationMethod(method);

		_activeProgram = program;
		_scriptRunning = true;

		// Set run mode to synchronous for the duration of this run
		_runMode = eRunMode::Synchronous;

		// enable sim stepping and telemetry for synchronous run
		startSimulation();

		// Run tight simulation loop until program completes
		const double dt = _dt;
		const int maxSteps = static_cast<int>((24.0 * 3600.0) / dt); // safety to prevent infinite loops in faulty scripts (max 24 hours of sim time)

		// Main loop: step the program and simulation until completion
		for (int step = 0; step < maxSteps; ++step) {
			// Check program completion
			if (program->isCompleted() || program->isFaulted() || program->isStopped()) {
				break;
			}

			// Step the program (DSL command execution)
			program->step(dt);

			// Step physics and robot if sim is running
			if (_simRunning) {
				_simTime += dt;

				updatePhysics(dt);

				if (hasRobot()) {
					// Update Trajectory Inputs
					_robot->updateTrajectoryInputs(*_traj, _simTime);

					// Step robot system
					_robot->step(dt, _simTime);

					// Telemetry beginRun
					if (!_telemetryBegun) {
						_telemetry.beginRun(_simTime, _telHz, 300.0);
						_telemetryBegun = true;
						D_INFO_ONCE("Telemtry Capture Started (dt=%.6f s, simTime=%.3f s)", (1 / _telHz), _simTime);
					}

					// Telemetry update
					_telemetry.update(_simTime, *_robot, _traj.get(), diagnostics::eTelemetryLevel::FULL);
				}
			}
		}
		// Clean up
		stopSimulation();
		// Reset run mode to interactive (default)
		_runMode = eRunMode::Interactive;

		_activeProgram = nullptr;
		_scriptRunning = false;
		_simRunning = false;
		_telemetryBegun = false;

		D_SUCCESS("Synchronous run completed: %s (%.1fs, %zu samples)", methodName.c_str(), _simTime, _telemetry.ring.size());
		return (_telemetry.ring.size() >= 2);
	}
	
	// Setter for fixed timestep duration
	void SimulationCore::setFixedDt(double dt) { _dt = dt; }
	// Getter for fixed timestep duration
	double SimulationCore::fixedDt() const { return _dt; }
	// Getter for current simulation time
	double SimulationCore::simTime() const { return _simTime; }

	// Method to step the simulation with a fixed timestep
	void SimulationCore::tick(double frame_dt) { stepFixed(frame_dt); }

	// --- Setters and Getters for Systems and State ---

	// Setter for the physics system
	void SimulationCore::setPhysicsSystem(physics::PhysicsSystem* physics) { _physics.reset(physics); }

	// Accessor for the physics system (non-const and const versions)
	physics::PhysicsSystem* SimulationCore::physicsSystem() { return _physics.get(); }
	const physics::PhysicsSystem* SimulationCore::physicsSystem() const { return _physics.get(); }

	// Accessor for the robot system (non-const and const versions)
	robots::RobotSystem* SimulationCore::robotSystem() { return _robot.get(); }
	const robots::RobotSystem* SimulationCore::robotSystem() const { return _robot.get(); }

	// Setter and checker for Robot System
	void SimulationCore::setRobotSystem(robots::RobotSystem* robot) { _robot.reset(robot); }
	bool SimulationCore::hasRobot() const { return _robot && _robot->hasRobot(); }

	// Loads a robot into the robot system by name
	void SimulationCore::loadRobot(const std::string& name) {
		if (!_robot) { D_FAIL("Cannot load robot: RobotSystem not set"); return; }
		_robot->loadRobot(name);
	}
	// Clears the currently loaded robot from the robot system
	void SimulationCore::clearRobot() {
		if (!_robot) { D_FAIL("Cannot clear robot: RobotSystem not set"); return; }
		_robot->clearRobot();
	}

	// Getter for the scene objects reference (used for script object lookup)
	std::vector<std::unique_ptr<scene::Object>>& SimulationCore::getObjects() {
		if (!_objects) { throw std::runtime_error("Scene objects pointer not set in SimulationCore"); }
		return *_objects;
	}
	// Deletes an object from the scene by index, with bounds checking
	void SimulationCore::deleteObject(int index) {
		if (!_objects) { D_FAIL("Cannot delete object: Scene objects pointer not set"); return; }
		if (index < 0 || index >= static_cast<int>(_objects->size())) {
			D_FAIL("Cannot delete object: Index %d out of bounds (size=%zu)", index, _objects->size());
			return;
		}
		_objects->erase(_objects->begin() + index);
	}
	// Loads a mesh from the given path, adds it to the scene objects, and returns raw pointers to the new objects for script access
	std::vector<scene::Object*> SimulationCore::loadMeshReturn(const std::string& path) {
		assets::MeshLoader loader;
		auto meshes = loader.load(path);
		std::vector<scene::Object*> result;
		for (auto& m : meshes) {
			auto obj = std::make_unique<scene::Object>(m);
			auto raw = obj.get();
			raw->internal = true;
			_objects->push_back(std::move(obj));
			result.push_back(raw);
		}
		return result;
	}

	// Setter for the scene objects pointer (used for script object lookup)
	void SimulationCore::setObjects(std::vector<std::unique_ptr<scene::Object>>* objects) { _objects.reset(objects); }
	// Getter for object
	scene::Object* SimulationCore::getObject() {
		if (!_objects) { return nullptr; }
		for (auto& obj : *_objects) {
			if (obj) { return obj.get(); }
		}
		return nullptr;
	}
	// Getter for object by ID
	scene::Object* SimulationCore::getObjectByID(scene::ObjectID id) {
		if (!_objects) { return nullptr; }
		for (auto& obj : *_objects) {
			if (obj && obj->id == id) { return obj.get(); }
		}
		return nullptr;
	}

	// Setter for the trajectory manager
	void SimulationCore::setTrajectoryManager(control::TrajectoryManager* traj) { _traj.reset(traj); }

	// Accessor for the trajectory manager (non-const and const versions)
	control::TrajectoryManager* SimulationCore::trajectoryManager() { return _traj.get(); }
	const control::TrajectoryManager* SimulationCore::trajectoryManager() const { return _traj.get(); }

	// Setters for the metric buffers
	void SimulationCore::setJointLogBuffer(robots::JointLogBuffer* buf) { _jointLogBuffer = *buf; }
	void SimulationCore::setTrajRefBuffer(robots::TrajRefBuffer* buf) { _trajRefBuffer = *buf; }

	// Accessor for the telemetry recorder (non-const and const versions)
	diagnostics::TelemetryRecorder& SimulationCore::telemetry() { return _telemetry; }
	const diagnostics::TelemetryRecorder& SimulationCore::telemetry() const { return _telemetry; }

	// Get the current number of telemetry samples recorded
	size_t SimulationCore::telemetrySampleCount() const { return _telemetry.ring.size(); }

	// Setter and getter for the active script program
	void SimulationCore::setActiveProgram(interpreter::IStoredProgram* p) { _activeProgram = p; }
	interpreter::IStoredProgram* SimulationCore::activeProgram() const { return _activeProgram; }
}