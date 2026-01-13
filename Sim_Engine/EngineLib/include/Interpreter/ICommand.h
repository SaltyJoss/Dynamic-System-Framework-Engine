#pragma once

#include "EngineCore.h"
#include "IStoredProgram.h"
#include <memory>
#include <string>

namespace commands {
	class CommandContextMotion;
	class UIContext;
}

namespace commands {
	// ICommand interface
	class ENGINE_API ICommand {
	public:
		// Virtual destructor
		virtual ~ICommand() = default;

		// Context setters
		virtual void setContext(CommandContextMotion& cntx) = 0;
		virtual void setContext(UIContext& cntx) = 0;

		// Update command
		virtual interpreter::CmdResult update(CommandContextMotion& cntx, double dt) = 0;
		// Execute command
		virtual void execute() = 0;

		// Mark the command as failed with a message
		virtual void markFailed(const std::string& message) = 0;
		// Mark the command as completed
		virtual void markCompleted() = 0;
		// Check if the command has started
		virtual bool hasStarted() const = 0;
	};
	
} // namespace interpreter