#pragma once

#include "EngineCore.h"
#include <string>
#include <vector>

namespace interpreter {
	// Struct representing source location
	struct ENGINE_API SrcLocation {
		std::string filename; // Name of the source file
		int line = 0;             // Line number in the source file
		int column = 0;           // Column number in the source file
	};

	// Struct representing a single instruction
	struct ENGINE_API Instruction {
		std::string name;
		std::vector<std::string> args;
		SrcLocation loc;
	};

	// Struct representing program data
	struct ENGINE_API ProgramData {
		std::vector<Instruction> instructions; // Vector storing the instructions
	};

	// Methods for ProgramData
	bool empty();
	size_t size();
	const Instruction& at(size_t index);
} // namespace interpreter