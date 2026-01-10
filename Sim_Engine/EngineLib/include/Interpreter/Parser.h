#pragma once

#include "EngineCore.h"
#include "IStoredProgram.h"
#include <memory>
#include <string>
#include <vector>

namespace interpreter {
	// Class representing a parsed command.
	class ENGINE_API Parser {
	public:
		// Constructor
		Parser();

		// Get the command name
		std::string getCommandName() const { return commandName; }
		// Get the command parameters
		std::vector<std::string> getParameters() const { return parameters; }
	private:
		std::unique_ptr<IStoredProgram> storedProgram; // Associated stored program
		
		std::string rawCode;					// Raw code input
		std::vector<std::string> lines;			// Lines of code
		std::vector<std::string> rawTokens;		// Raw tokens from parsing
		std::vector<std::string> tokens;		// Processed tokens

		std::string commandName;				// Name of the command
		std::vector<std::string> parameters;	// Parameters of the command
	};
} // namespace interpreter