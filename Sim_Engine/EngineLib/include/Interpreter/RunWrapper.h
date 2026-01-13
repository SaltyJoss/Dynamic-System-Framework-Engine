#pragma once

#include "EngineCore.h"
#include "Parser.h"
#include "IStoredProgram.h"

namespace interpreter {
	// Class that wraps the parsing and storing of a program
	class ENGINE_API RunWrapper {
	public:
		RunWrapper(Parser* parser, IStoredProgram* program);
		// Parse and store the program from a code string
		void runProgram(const std::string& code);
	private:
		Parser* _parser;
		IStoredProgram* _program;
	};
} // namespace interpreter