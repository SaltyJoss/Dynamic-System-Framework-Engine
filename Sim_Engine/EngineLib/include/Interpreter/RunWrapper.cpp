#include "pch.h"
#include "Interpreter/RunWrapper.h"

#include "EngineLib/LogMacros.h"

namespace interpreter {
	RunWrapper::RunWrapper(Parser* parser, IStoredProgram* program) : _parser(parser), _program(program) {
		if (_parser == nullptr) {
			D_FAIL("RunWrapper initialised with null Parser pointer.");
			throw std::invalid_argument("RunWrapper initialised with null Parser pointer.");
		}
		if (_program == nullptr) {
			D_FAIL("RunWrapper initialised with null IStoredProgram pointer.");
			throw std::invalid_argument("RunWrapper initialised with null IStoredProgram pointer.");
		}
	}
	void RunWrapper::runProgram(const std::string& code) {
		if (!_program || !_parser) {
			D_FAIL("RunWrapper has null Parser pointer.");
			throw std::runtime_error("RunWrapper has null Parser pointer.");
		}

		_program->clear();
		_parser->parse(code);
		_program->start();
	}
} // namespace interpreter