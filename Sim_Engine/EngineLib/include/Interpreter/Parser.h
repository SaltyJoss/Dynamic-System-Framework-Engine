#pragma once

#include "EngineCore.h"
#include "ProgramData.h"
#include "IStoredProgram.h"
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
		void tokenAndClassifyLine(const std::string& line, Command& outCmd);
		void buildProgram();
		void buildCommand(Command& cmd);

		void motionCommandHandler(const Command& cmd);

		static bool isBlankOrComment(std::string_view line);

		std::vector<std::string> split(const std::string& s, const std::vector<std::string>& delimiters);

		IStoredProgram* _program = nullptr;

		interpreter::Command& _currentCmd;
		interpreter::ProgramData& _programData;
		std::vector<std::string>& lines;
	};
} // namespace interpreter