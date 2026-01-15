#include "pch.h"
#include "Interpreter/StoredProgram.h"

#include "EngineLib/LogMacros.h"

namespace interpreter {
	StoredProgram::StoredProgram(gui::simManager* sim) : _currentLineNumber(0), PC(0), _sim(sim), _cntx(sim, sim ? sim->getObject() : nullptr) {
		_commands = std::vector<commands::ICommand*>();
	}

	void StoredProgram::add(commands::ICommand* cmd) {
		if (cmd == nullptr) {
			D_FAIL("Attempted to add null command to StoredProgram.");
			throw std::invalid_argument("Attempted to add null command to StoredProgram.");
		}

		cmd->setContext(_cntx);
		cmd->setProgram(this);
		_commands.push_back(cmd);
	}
	
	void StoredProgram::load(ProgramData program) {
		// Not implemented yet
	}

	// Reset the program to its initial state
	void StoredProgram::reset() {
		_currentLineNumber = 0;
		PC = 0;
	}

	// Clear all stored instructions
	void StoredProgram::clear() {
		_currentLineNumber = 0;
		PC = 0;
		_commands.clear();
	}

	void StoredProgram::start() {
		_state = ProgramState::Running;
		_stopRequested = false;
	}

	void StoredProgram::stop() {
		_state = ProgramState::Stopped;
		_stopRequested = true;

		scene::Object* obj = _sim ? _sim->getObject() : nullptr;
		if (obj) {
			utils::AxisMask all{ true,true,true };
			_cntx.stopRotation(obj, all);
			_cntx.stopTranslation(obj, all);
		}
	}

	void StoredProgram::pause() {
		_state = ProgramState::Paused;

		scene::Object* obj = _sim ? _sim->getObject() : nullptr;
		if (obj) {
			utils::AxisMask all{ true,true,true };
			_cntx.stopRotation(obj, all);
			_cntx.stopTranslation(obj, all);
		}
	}

	ProgramStatus StoredProgram::status() const {
		return ProgramStatus{};
	}

	bool StoredProgram::atEnd() const {
		return PC >= static_cast<int>(_commands.size());
	}

	bool StoredProgram::commandsLeft() const {
		return PC >= 0 && PC < static_cast<int>(_commands.size());
	}

	void StoredProgram::step(double dt) {
		if (_state != ProgramState::Running) { return; }
		if (_stopRequested) { _state = ProgramState::Stopped;  return; }
		if (!commandsLeft()) { _state = ProgramState::Completed; return; }
	
		_cntx.setDefaultObject(_sim ? _sim->getObject() : nullptr);

		auto& cmd = _commands[PC];
		cmd->setContext(_cntx);
		if (!cmd->hasStarted()) {
			cmd->execute();
		}

		auto r = cmd->update(_cntx, dt);

		// <-- add this
		if (_sim && _sim->hasRobot()) {
			_sim->updateRobotKinematics(glm::mat4(1.0f)); 
		}

		if (r.state == CmdState::Executed || r.state == CmdState::Failed) {
			PC++;
			if (!commandsLeft()) {
				_state = ProgramState::Stopped;
			}
		}
	}

	CmdResult StoredProgram::updateState() {
		return CmdResult{};
	}

	void StoredProgram::setIntegratorMethod(IntegratorMethod method) {
		_integratorMethod = method;
		if (_sim) {
			auto& physics = _sim->getPhysicsSystem();
			physics.setIntegrationMethod(static_cast<physics::PhysicsSystem::eIntegrationMethod>(method));
		}
	}
	IntegratorMethod StoredProgram::getIntegratorMethod() const {
		return _integratorMethod;
	}
} // namespace interpreter