#pragma once

#include "EngineCore.h"
#include "IStoredProgram.h"
#include <memory>

namespace interpreter {
	class ENGINE_API ICommand {
		// StoredProgram Instance
		std::unique_ptr<IStoredProgram> program;

		// check parameters
		virtual bool checkParams(std::vector<std::string> params) = 0;
		// Set up the command
		virtual void set() = 0;
		// Execute the command
		virtual void execute() = 0;
	};
	
} // namespace interpreter