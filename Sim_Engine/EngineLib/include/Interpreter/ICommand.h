#pragma once

#include "EngineCore.h"
#include "IStoredProgram.h"

namespace interpreter {
	class ENGINE_API ICommand {
		// StoredProgram Instance
		IStoredProgram program;

		// check parameters
		virtual bool checkParams(std::vector<std::string> params) = 0;
		// Set up the command
		virtual void set() = 0;
		// Execute the command
		virtual void execute() = 0;
	};
	
} // namespace interpreter