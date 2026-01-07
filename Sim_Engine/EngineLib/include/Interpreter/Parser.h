#pragma once

#include "EngineCore.h"
#include "IStoredProgram.h"
#include <string>
#include <vector>

namespace interpreter {
	// Class representing a parsed command.
	class ENGINE_API ParsedCommand {
	public:
		// Constructor
		ParsedCommand(const std::string& name, const std::vector<std::string>& params)
			: commandName(name), parameters(params) {}
		// Get the command name
		std::string getCommandName() const { return commandName; }
		// Get the command parameters
		std::vector<std::string> getParameters() const { return parameters; }
	private:

		std::string commandName;               // Name of the command
		std::vector<std::string> parameters;   // Parameters of the command
	};
} // namespace interpreter