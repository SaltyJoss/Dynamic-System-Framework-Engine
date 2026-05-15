#include "pch.h"
// File:   RunWrapper.cpp
// GitHub: SaltyJoss
#include "Interpreter/RunWrapper.h"

#include "Platform/Logger.h"
#include "EngineLib/LogMacros.h"

extern DSFE_API Debug gLog;

namespace interpreter {
	// Constructor
	RunWrapper::RunWrapper(Parser* parser, IStoredProgram* program) : _parser(parser), _program(program) {
		if (_parser == nullptr) {
			LOG_ERROR("RunWrapper initialised with null Parser pointer.");
			throw std::invalid_argument("RunWrapper initialised with null Parser pointer.");
		}
		if (_program == nullptr) {
			LOG_ERROR("RunWrapper initialised with null IStoredProgram pointer.");
			throw std::invalid_argument("RunWrapper initialised with null IStoredProgram pointer.");
		}
	}

	// Parse and store the program from a code string
	void RunWrapper::runProgram(const std::string& code) {
		if (!_program || !_parser) {
			LOG_ERROR("RunWrapper has null Parser pointer.");
			throw std::runtime_error("RunWrapper has null Parser pointer.");
		}
		// Clear any existing program and parse new code
		_program->clear();
		_parser->parse(code);
		_program->start();
	}
} // namespace interpreter