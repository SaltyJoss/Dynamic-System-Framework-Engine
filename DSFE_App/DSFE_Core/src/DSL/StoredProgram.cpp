/*
 * File: DSL/StoredProgram.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "DSL/StoredProgram.h"

#include "Platform/ISimulationCore.h"
#include "Systems/RigidBodySystem.h"

#include "EngineLib/LogMacros.h"

namespace dsl {
	StoredProgram::StoredProgram(core::ISimulationCore* core)
		: _currentLineNumber(0), _core(core), PC(0), _cntx(core) {
	}

	StoredProgram::~StoredProgram() { clear(); }

	// Add a command to the program
	void StoredProgram::add(std::unique_ptr<commands::ICommand> cmd) {
		if (cmd == nullptr) {
			throw std::invalid_argument("Attempted to add null command to StoredProgram.");
		}

		cmd->setContext(_cntx.motion());
		cmd->setProgram(this);
		_commands.push_back(std::move(cmd));
	}

	// Add a command to the program
	void StoredProgram::add(commands::ICommand* cmd) {
		if (cmd == nullptr) {
			throw std::invalid_argument("Attempted to add null command to StoredProgram.");
		}
		cmd->setContext(_cntx.motion());
		cmd->setProgram(this);
		_commands.emplace_back(cmd);
	}

	// Reset program counters
	void StoredProgram::reset() {
		_currentLineNumber = 0;
		PC = 0;
	}

	// Clear all stored instructions
	void StoredProgram::clear() {
		for (auto& cmd : _commands) { if (cmd) { cmd->setProgram(nullptr); } } // Clear program reference from commands
		_commands.clear();
		_currentLineNumber = 0;
		PC = 0;
		_state = ProgramState::Stopped;
		_stopRequested = false;
	}

	// Start program execution
	void StoredProgram::start() {
		_state = ProgramState::Running;
		_stopRequested = false;
	}

	// Start simulation
	void StoredProgram::startSim() {
		if (_state != ProgramState::Running) { start(); }
		if (_core && !_core->isSimRunning()) { _core->startSimulation(); }
	}

	// Stop program execution
	void StoredProgram::stop() {
		_state = ProgramState::Stopped;
		_stopRequested = true;

		stopSim(); // Also stop simulation, if running
	}
	
	// Stop simulation
	void StoredProgram::stopSim() {
		if (_core->isSimRunning()) { _core->stopSimulation(); }
		if (_core->hasRigidBody()) { _cntx.motion().RigidBody().stopAll(); }
	}

	// Pause program execution
	void StoredProgram::pause() {
		if (!_core) { return; }
		_state = ProgramState::Paused;
		if (_core->hasRigidBody()) { _cntx.motion().RigidBody().stopAll(); }
	}

	// Wait for simulation to run for dt seconds
	void StoredProgram::waitSim(double dt) {
		if (!_core) { return; }
		if (!_core->isSimRunning()) { _core->startSimulation(); } // I do not like this line

		double elapsed = 0.0;
		const double stepDt = _core ? _core->fixedDt() : static_cast<double>(1.0 / 180.0);
		while (elapsed < dt) {
			elapsed += stepDt;
		}
		if (_core->isSimRunning()) { _core->stopSimulation(); }
	}

	// Get current program status
	ProgramStatus StoredProgram::status() const { return ProgramStatus{}; }
	// Bool for tracking if the program has reached the end
	bool StoredProgram::atEnd() const { return PC >= static_cast<int>(_commands.size()); }
	// Bool for tracking if there are commands left to execute
	bool StoredProgram::commandsLeft() const { return PC >= 0 && PC < static_cast<int>(_commands.size()); }

	// Step through the program by dt seconds
	void StoredProgram::step(double dt) {
		// State checks
		if (_state == ProgramState::Paused) { return; }
		if (_state == ProgramState::Stopped || _state == ProgramState::Completed || _state == ProgramState::Faulted) { return; }
		if (_state != ProgramState::Running) { start(); }
		if (_stopRequested) { stop(); return; }
		// Command checks
		if (_commands.empty()) { _state = ProgramState::Faulted; return; }
		if (!commandsLeft()) { _state = ProgramState::Completed; return; _core->stopSimulation(); }

		// Get current command
		auto& cmd = _commands[PC];
		cmd->setContext(_cntx.motion());

		if (!cmd->hasStarted()) { cmd->execute(); }

		// Check current result
		CmdResult r0 = cmd->currentResult();
		if (r0.state == CmdState::Failed) { _state = ProgramState::Faulted; return; }
		if (r0.state == CmdState::Executed) {
			++PC;
			if (!commandsLeft()) _state = ProgramState::Completed;
			return;
		}

		// Update command
		CmdResult r = cmd->update(_cntx.motion(), dt);

		// Check result
		if (r.state == CmdState::Failed) { _state = ProgramState::Faulted; return; }
		if (r.state == CmdState::Executed) {
			PC++; // not ++PC because we may want to re-execute the same command
			if (!commandsLeft()) { _state = ProgramState::Completed; return; }
		}
	}

	// Update the command state
	CmdResult StoredProgram::updateState() { return CmdResult{}; }

	// Set Integrator Method
	void StoredProgram::setIntegratorMethod(IntegratorMethod method) {
		_integratorMethod = method;
		if (_core) {
			if (_core->hasRigidBody()) {
				auto& rs = _core->rigidBodySystem();
				std::vector<IntegratorMethod> adMethods = { IntegratorMethod::AD_ImplicitEuler, IntegratorMethod::AD_ImplicitMidpoint, IntegratorMethod::AD_GLRK2, IntegratorMethod::AD_GLRK3 };
				if (std::find(adMethods.begin(), adMethods.end(), method) != adMethods.end()) {
					_core->setADIntegrationMethod(static_cast<integration::eAutoDiffIntegrationMethod>(method));
				}
				else {
					_core->setIntegrationMethod(static_cast<integration::eIntegrationMethod>(method));
				}
			}
		}
	}

	// Get Integrator Method
	IntegratorMethod StoredProgram::getIntegratorMethod() const { return _integratorMethod; }

	// Set Omega
	void StoredProgram::setAngularVel(mathlib::Vec3 wv, utils::AngularUnits units) {
		_cntx.motion().setAngularUnits(units);
	}

	void StoredProgram::setVelocity(mathlib::Vec3 wv, mathlib::Vec3 lv) {
		/* Not added yet, may remove this function if not needed */
	}

	// Set Fixed Dt
	void StoredProgram::setFixedDt(double dt) { _dt = dt; if (_core) { _core->setFixedDt(dt); } }
	// Get Fixed Dt
	double StoredProgram::getFixedDt() const {
		if (_core) { return _core->fixedDt(); }
		return _dt;
	}

	// Set Gravity
	void StoredProgram::setGravity(double gravity) { 
		_gravity.z() = gravity;
		if (_core) {
			if (_core->hasRigidBody()) { auto& rs = _core->rigidBodySystem(); rs.setGravity(gravity); }
		}
	}
	// Get Gravity
	double StoredProgram::getGravity() const {
		if (_core) {
			if (_core->hasRigidBody()) { const auto& rs = _core->rigidBodySystem(); return rs.getGravity(); }
		}
		return _gravity.z();
	}
	// Set Gravity Vector
	void StoredProgram::setGravityVec(mathlib::Vec3 g) {
		if (_core) {
			if (_core->hasRigidBody()) { auto& rs = _core->rigidBodySystem(); rs.setGravityVec(g); }
		}
	}
	// Get Gravity Vector
	mathlib::Vec3 StoredProgram::getGravityVec() const {
		if (_core) {
			if (_core->hasRigidBody()) { const auto& rs = _core->rigidBodySystem(); return rs.getGravityVec(); }
		}
		return _gravity;
	}
} // namespace dsl