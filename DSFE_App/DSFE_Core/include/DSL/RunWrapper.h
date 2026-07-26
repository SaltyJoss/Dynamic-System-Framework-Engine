/*
 * File: DSL/RunWrapper.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include "DSL/Parser.h"
#include "DSL/IStoredProgram.h"

namespace dsl {
	// Class that wraps the parsing and storing of a program
	class DSFE_API RunWrapper {
	public:
		RunWrapper(Parser* parser, IStoredProgram* program);
		// Parse and store the program from a code string
		void runProgram(const std::string& code);
	private:
		Parser* _parser;
		IStoredProgram* _program;
	};
} // namespace interpreter