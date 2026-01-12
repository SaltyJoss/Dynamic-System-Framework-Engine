#include "pch.h"
#include "Interpreter/StoredProgram.h"
#include "Interpreter/CommandContextMotion.h"

#include "EngineLib/LogMacros.h"

namespace interpreter {
	void StoredProgram::load(ProgramData program) {
		// Not implemented yet
	}

	void StoredProgram::reset() {
		// Not implemented yet
	}

	void StoredProgram::clear() {
		// Not implemented yet
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
} // namespace interpreter