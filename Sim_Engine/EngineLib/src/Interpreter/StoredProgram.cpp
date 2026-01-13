#include "pch.h"
#include "Interpreter/StoredProgram.h"

#include "EngineLib/LogMacros.h"

namespace interpreter {
	StoredProgram::StoredProgram(gui::simManager* sim) : _currentLineNumber(0), PC(0), _sim(sim) {
		_commands = std::vector<commands::ICommand*>();
	}

	void StoredProgram::add(commands::ICommand* cmd) {
		if (cmd == nullptr) {
			D_FAIL("Attempted to add null command to StoredProgram.");
			throw std::invalid_argument("Attempted to add null command to StoredProgram.");
		}

		_commands.push_back(cmd);
	}
	
	void StoredProgram::load(ProgramData program) {
		// Not implemented yet
	}

	// Reset the program to its initial state
	void StoredProgram::reset() {
		_currentLineNumber = 0;
		PC = 0;
		_commands.clear();
	}

	// Clear all stored instructions
	void StoredProgram::clear() {
		_currentLineNumber = 0;
		PC = 0;
		_commands.clear();
	}

	void StoredProgram::start() {
		ProgramState state = ProgramState::Running;
	}

	void StoredProgram::stop() {
		ProgramState state = ProgramState::Stopped;
	}

	void StoredProgram::pause() {
		ProgramState state = ProgramState::Paused;
	}

	void StoredProgram::step(double dt) {
		// Not implemented yet ~ REQUIRED for STEP, and RUN commands
	}

	ProgramStatus StoredProgram::status() const {
		return ProgramStatus{};
	}

	bool StoredProgram::atEnd() const {
		return PC >= _commands.size();
	}

	bool StoredProgram::commandsLeft() const {
		return PC >= 0 && PC < _commands.size();
	}

	void StoredProgram::run() {
		while (commandsLeft()) {
			auto& cmd = _commands[PC];

			int oldPC = PC;

			scene::Object* sel = _sim ? _sim->getObject() : nullptr;
			commands::CommandContextMotion cntx(_sim, sel);
			cmd->setContext(cntx);
			cmd->execute();

			double dt = 1.0 / 60.0;
			for (int i = 0; i < 2000; ++i) {
				auto r = cmd->update(cntx, dt);
				if (r.state != CmdState::Executing) break;
			}

			if (PC == oldPC) {
				PC++;
			}
		}
	}

	CmdResult StoredProgram::updateState() {
		return CmdResult{};
	}
} // namespace interpreter