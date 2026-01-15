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
		IStoredProgram* _program = nullptr;
		Command _currentCmd;
		ProgramData _programData;

		std::vector<std::string> lines;

		void tokeniseAndClassifyCode(const std::string& code);
		void buildProgram();
		void buildCommand(Command& cmd);

		void motionCommandHandler(const Command& cmd);

		static bool isBlankOrComment(std::string_view line);
		static bool hasCommentInline(const std::string_view s);
	};
} // namespace interpreter