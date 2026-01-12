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
	struct ENGINE_API Command {
		std::string rawLine;				// The original line of code
		std::string cmdName;				// The command name
		std::string identifier;				// The command identifier
		std::string tokens;	// Vector of tokens/arguments
		int lineNumber = 0;					// Line number in the source code
	};

	// Struct representing program data
	struct ENGINE_API ProgramData {
		std::vector<Command> cmd; // Vector storing the instructions
	};

	// Methods for ProgramData
	bool empty();
	size_t size();
	const Command& at(size_t index);
} // namespace interpreter