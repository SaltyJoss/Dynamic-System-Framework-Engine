#pragma once

#include "EngineCore.h"
#include "ProgramData.h"
#include "IStoredProgram.h"
#include "Token.h"
#include <memory>
#include <string>
#include <vector>

#include "Platform/Logger.h"

namespace interpreter {
	// Class representing a parsed command
	class ENGINE_API Parser {
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