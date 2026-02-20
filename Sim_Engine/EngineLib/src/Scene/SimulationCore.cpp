#include "pch.h"
// File:   SimulationCore.cpp
// GitHub: SaltyJoss
#include "Scene/SimulationCore.h"
#include "Physics/PhysicsSystem.h"
#include "Robots/RobotSystem.h"
#include "Robots/RobotModel.h"
#include "Robots/TrajectoryManager.h"

#include "Interpreter/StoredProgram.h"
#include "Interpreter/Parser.h"

#include "EngineLib/LogMacros.h"
#include "Platform/DataManager.h"

namespace core {
	// Constructor
	SimulationCore::SimulationCore() {
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

	// Fixed timestep loop for physics and robot updates, called from the main render loop with the frame delta time
	void SimulationCore::stepFixed(double frame_dt) {
		_accum += frame_dt;
		while (_accum >= _dt) {
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
					D_FAIL("SCRIPT END: stopped=%d faulted=%d (dt=%.6f s, simTime=%.3f s)",
						(int)stopped, (int)faulted, _dt, _simTime);

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

			if (_simRunning) {
				_simTime += _dt;

				updatePhysics(_dt);
				if (hasRobot()) {
					// Update Trajector Inputs
					_robot->updateTrajectoryInputs(*_traj, _simTime);
					// Step robot system
					_robot->step(_dt, _simTime);

					// Telemetry update
					if (!_telemetryBegun) {
						_telemetry.beginRun(_simTime, _telHz, 300.0);
						_telemetryBegun = true;
						D_INFO_ONCE("Telemtry Capture Started (dt=%.6f s, simTime=%.3f s)", (1 / _telHz), _simTime);
					}
					_telemetry.update(_simTime, *_robot, _traj, diagnostics::eTelemetryLevel::FULL);
				}
			}
			_accum -= _dt;
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

			// Clear and reserve telemetry buffers based on expected simulation length and robot DOF
			_trajRefBuffer.clear();
			_jointLogBuffer.clear();

			// Calculate the total number of entries needed for the buffers
			size_t steps = static_cast<size_t>(300.0 / _dt); // Assuming a max simulation length of 300 seconds
			size_t joints = _robot->jointCount();
			size_t total = steps * joints;

			// Reserve capacity to avoid reallocations during the run
			_trajRefBuffer.reserve(total);
			_jointLogBuffer.reserve(total);

			// IMPORTANT: inject buffer into robot
			_robot->setRefBuffer(&_trajRefBuffer);
			_robot->setLogBuffer(&_jointLogBuffer);
		}

		// Ensure reference sim system have their integrators configured for the new run
		setupSimulationIntegrator();
		SET_SIM_INTEGRATOR(_robot->getIntegratorName());

		_simRunning = true;
		_telemetryBegun = false;
		DATA_CAPTURE_ENABLE(true);
	}

	// Stop the simulation loop
	void SimulationCore::stopSimulation() {
		if (!_simRunning) return;
		D_RUNTIME("stopping simulation");

		D_RUNTIME("JointLogBuffer size = %zu", _jointLogBuffer.size());
		D_RUNTIME("TrajRefBuffer size = %zu", _trajRefBuffer.size());

		// Only export ref if we actually have samples
		if (_trajRefBuffer.size() > 0) {
			exportRefsToHDF5();
		}
		// Only export sim if we actually have samples
		if (_jointLogBuffer.size() > 0) {
			exportLogsToHDF5();
		}

		// Clear buffers to free memory and prepare for next run
		DATA_CAPTURE_ENABLE(false);
		_simRunning = false;
		_telemetryBegun = false;
	}

	// Exporst the logged joint data to HDF5 format using the custom macro for each log entry
	void SimulationCore::exportLogsToHDF5() {
		// Construct a header for the HDF5 dataset based on the robot and integrator names
		const std::string intName = _robot->getIntegratorName();
		const std::string robotName = _robot->hasRobot() ? _robot->robotName() : "no_robot";
		const std::string header = robotName + "_sim_" + intName;

		// Check if there are any log entries
		const size_t N = _jointLogBuffer.size();
		if (N == 0) return;

		// For each log entry, create a field list and write to HDF5
		for (size_t i = 0; i < N; ++i) {
			// Create a list of fields for this log entry
			data::FieldList fields;
			// Sim Metadata
			fields.emplace_back("sim_time",		(double)_jointLogBuffer.sim_time[i]);
			fields.emplace_back("dt_taken",		(double)_jointLogBuffer.dt_taken[i]);
			fields.emplace_back("dt_sug",		(double)_jointLogBuffer.dt_sug[i]);
			// States
			fields.emplace_back("theta",		(double)_jointLogBuffer.theta[i]);
			fields.emplace_back("omega",		(double)_jointLogBuffer.omega[i]);
			fields.emplace_back("alpha",		(double)_jointLogBuffer.alpha[i]);
			fields.emplace_back("err",			(double)_jointLogBuffer.err[i]);
			fields.emplace_back("err_d",		(double)_jointLogBuffer.err_d[i]);
			// Dynamics
			fields.emplace_back("I_eff",		(double)_jointLogBuffer.I_eff[i]);
			fields.emplace_back("tau",			(double)_jointLogBuffer.tau[i]);
			fields.emplace_back("tau_fb",		(double)_jointLogBuffer.tau_fb[i]);
			fields.emplace_back("tau_coriolis", (double)_jointLogBuffer.tau_coriolis[i]);
			fields.emplace_back("tau_gravity",	(double)_jointLogBuffer.tau_gravity[i]);
			fields.emplace_back("tau_damping",	(double)_jointLogBuffer.tau_damping[i]);
			fields.emplace_back("tau_friction", (double)_jointLogBuffer.tau_friction[i]);
			fields.emplace_back("tau_barrier",	(double)_jointLogBuffer.tau_barrier[i]);
			fields.emplace_back("tau_sat",		(double)_jointLogBuffer.tau_sat[i]);
			// Limit flags and info
			fields.emplace_back("clamp_theta",	(double)_jointLogBuffer.clamp_theta[i]);
			fields.emplace_back("clamp_omega",	(double)_jointLogBuffer.clamp_omega[i]);
			fields.emplace_back("sat_flag",		(double)_jointLogBuffer.sat_flag[i]);
			// Joint info
			fields.emplace_back("joint_index",	(double)_jointLogBuffer.joint_index[i]);

			// Write this entry to HDF5
			HDF5_SIM_DATA(header, fields);
		}
	}

	// Exports the reference trajectory data to HDF5 format using the custom macro for each ref entry
	void SimulationCore::exportRefsToHDF5() {
		const std::string robotName = _robot->hasRobot() ? _robot->robotName() : "no_robot";
		const std::string header = robotName + "_traj_ref";

		// Check if there are any log entries
		const size_t N = _trajRefBuffer.size();
		if (N == 0) return;

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
			HDF5_REF_DATA(header, fields);
		}

	}

	// --------------------------------------------------
	//		   SYNCHRONOUS SCRIPT EXECUTION
	// --------------------------------------------------

	// Run a script synchronously to completion, blocking the main thread. Returns true if completed successfully.
	// NOTE: this is a blocking call that runs a tight loop until the script finishes, so it should only be used for testing or non-interactive scenarios.
	// IMPORTANT: I want to make this REALLY clear:
	//		---> I have implemented this for short (<5 minute) test scripts where blocking is acceptable
	//		---> IT IS NOT intended for general use and WILL CAUSE THE UI TO FREEZE if used with long-running scripts
	bool SimulationCore::runScriptToCompletion(interpreter::IStoredProgram* program, integration::eIntegrationMethod method) {
		if (!hasRobot()) { return false; }

		// Map method enum to string name, purely for logging purposes
		static const char* names[] = { "euler", "midpoint", "heun", "ralston", "rk4", "rk45" };
		const std::string methodName = names[static_cast<int>(method)];

		// Reset robot state
		_robot->resetRobot();
		_traj->clearAll();

		// Clear telemetry and log buffers
		_trajRefBuffer.clear();
		_jointLogBuffer.clear();

		// Inject buffers BEFORE any stepping happens
		_robot->setRefBuffer(&_trajRefBuffer);
		_robot->setLogBuffer(&_jointLogBuffer);

		_simTime = 0.0;
		_simRunning = false;
		_telemetryBegun = false;
		_accum = 0.0;

		// Set integrator on both physics and robot systems
		_robot->setIntegrationMethod(method);
		_physics->setIntegrationMethod(method);

		_activeProgram = program;
		_scriptRunning = true;

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
					_telemetry.update(_simTime, *_robot, _traj, diagnostics::eTelemetryLevel::FULL);
				}
			}
		}
		// Clean up
		stopSimulation();

		_activeProgram = nullptr;
		_scriptRunning = false;
		_simRunning = false;
		_telemetryBegun = false;

		D_SUCCESS("Synchronous run completed: %s (%.1fs, %zu samples)",
			methodName.c_str(), _simTime, _telemetry.ring.size());

		return (_telemetry.ring.size() >= 2);
	}

	// Method to step the simulation with a fixed timestep
	void SimulationCore::tick(double frame_dt) { stepFixed(frame_dt); }

	// --- Setters and Getters for Systems and State ---

	// Setter for the physics system
	void SimulationCore::setPhysicsSystem(physics::PhysicsSystem* physics) { _physics = physics; }

	// Accessor for the physics system (non-const and const versions)
	physics::PhysicsSystem* SimulationCore::physicsSystem() { return _physics; }
	const physics::PhysicsSystem* SimulationCore::physicsSystem() const { return _physics; }

	// Setter and checker for Robot System
	void SimulationCore::setRobotSystem(robots::RobotSystem* robot) { _robot = robot; }
	bool SimulationCore::hasRobot() const { return _robot && _robot->hasRobot(); }

	// Accessor for the robot system (non-const and const versions)
	robots::RobotSystem* SimulationCore::robotSystem() { return _robot; }
	const robots::RobotSystem* SimulationCore::robotSystem() const { return _robot; }

	// Setter for the trajectory manager
	void SimulationCore::setTrajectoryManager(control::TrajectoryManager* traj) { _traj = traj; }

	// Accessor for the trajectory manager (non-const and const versions)
	control::TrajectoryManager* SimulationCore::trajectoryManager() { return _traj; }
	const control::TrajectoryManager* SimulationCore::trajectoryManager() const { return _traj; }

	// Setter for the scene objects pointer (used for script object lookup)
	void SimulationCore::setObjects(std::vector<std::unique_ptr<scene::Object>>* objects) { _objects = objects; }

	// Setters for the metric buffers
	void SimulationCore::setJointLogBuffer(robots::JointLogBuffer* buf) { _jointLogBuffer = *buf; }
	void SimulationCore::setTrajRefBuffer(robots::TrajRefBuffer* buf) { _trajRefBuffer = *buf; }

	// Accessor for the telemetry recorder (non-const and const versions)
	diagnostics::TelemetryRecorder& SimulationCore::telemetry() { return _telemetry; }
	const diagnostics::TelemetryRecorder& SimulationCore::telemetry() const { return _telemetry; }

	// Setter and getter for the active script program
	void SimulationCore::setActiveProgram(interpreter::IStoredProgram* p) { _activeProgram = p; }
	interpreter::IStoredProgram* SimulationCore::activeProgram() const { return _activeProgram; }
}