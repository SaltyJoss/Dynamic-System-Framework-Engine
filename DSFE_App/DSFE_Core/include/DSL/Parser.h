/*
 * File: DSL/Parser.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include "DSL/ProgramData.h"
#include "DSL/IStoredProgram.h"
#include "DSL/Token.h"
#include <memory>
#include <string>
#include <vector>

#include "Platform/Logger.h"

namespace dsl {
	// Class representing a parsed command
	class DSFE_API Parser {
	public:
		Parser(IStoredProgram* program);
		void parse(std::string code);

	private:
		IStoredProgram* _program = nullptr;
		program_data::ProgramData _programData;
		Command _currentCmd;

		std::vector<Token> _tokens;
		std::vector<std::string> lines;
		size_t pos = 0;

		// --- Helper Functions ---
		
		// Helpers for parsing
		static bool requiresIdentifier(std::string_view cmdName);
		static bool matchIdentifier(const std::string& s);

		// Tokenise and classify code into commands
		void tokeniseAndClassifyCode(const std::string& code);

		// --- Command and Program Builders ---

		void buildProgram();
		void buildCommand(Command& cmd);
	};
} // namespace interpreter