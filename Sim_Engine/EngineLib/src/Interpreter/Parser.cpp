#include "pch.h"
#include "Interpreter/Parser.h"
#include "Interpreter/IStoredProgram.h"

namespace interpreter {
	// Constructor
	Parser::Parser(std::string& code) : rawCode(code) { // NOTE: need to look at how I want to call this? Maybe a wrapper is best??, StoredProgram has run, so aim to use that for actual running. Might be better to have a storedprogram argument - but for now keeping like this
		_program = std::make_unique<IStoredProgram>();
	}

	void Parser::buildProgram() {
		// Implementation for building the program from rawCode, not doing yet
	}

	void Parser::buildGenericCommand() {
		// Implementation for building a generic command, not doing yet
	}

} // namespace interpreter