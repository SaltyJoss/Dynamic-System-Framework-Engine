#pragma once

#include "EngineCore.h"
#include "IStoredProgram.h"
#include "CommandContextMotion.h"
#include <memory>
#include <string>

namespace commands {
	// ICommand interface
	class ENGINE_API ICommand {
		// Check parameters
		virtual void validateParameters(const std::vector<std::string>& params) const = 0;
		// Setup command with parameters
		virtual bool set(const std::vector<std::string>& params) = 0;
		// Execute command
		virtual void execute() = 0;

		// Update command
		virtual CmdResult update(CommandContextMotion& cntx, double dt) = 0;

		// Mark the command as failed with a message
		virtual void markFailed(const std::string& message);
		// Mark the command as completed
		virtual void markCompleted();
		// Check if the command has started
		virtual bool hasStarted() const;
	};
	
} // namespace interpreter