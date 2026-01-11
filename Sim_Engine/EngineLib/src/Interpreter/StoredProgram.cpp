#include "pch.h"
#include "Interpreter/StoredProgram.h"

namespace interpreter {
	StoredProgram::StoredProgram(CommandFactory& factory, CommandContextMotion& cntx) {
	}

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
		// Not implemented yet
	}

	ProgramStatus StoredProgram::status() const {
		return ProgramStatus{};
	}
} // namespace interpreter