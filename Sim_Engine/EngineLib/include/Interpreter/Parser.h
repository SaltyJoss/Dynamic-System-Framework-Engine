#pragma once

#include "EngineCore.h"
#include "ProgramData.h"
#include <memory>
#include <string>
#include <vector>

namespace interpreter {
	// Class representing a parsed command
	class ENGINE_API Parser {
	public:
		ProgramData parse(std::string code) const;
		ProgramData parseFile(const std::string& filename) const;

	private:
		void buildCommand(const Instruction& inst);
		void buildProgram(const ProgramData& program);

		void lineHandler(const Instruction& inst);
		void motionCommandHandler(const Instruction& inst);

		static bool isBlankOrComment(std::string_view line);
	};
} // namespace interpreter