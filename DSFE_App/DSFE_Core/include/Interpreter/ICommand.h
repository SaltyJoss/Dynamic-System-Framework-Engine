// DSFE_Core ICommand.h
#pragma once

#include "EngineCore.h"
#include "IStoredProgram.h"
#include <memory>
#include <string>

namespace commands {
	// Forward declaration of ICommand for use in IStoredProgram
	class CommandContextMotion;
	class UIContext;

	// ICommand interface
	class DSFE_API ICommand {
	public:
		// Virtual destructor
		virtual ~ICommand() = default;

		// Context setters
		virtual void setContext(CommandContextMotion& cntx) = 0;
		virtual void setContext(UIContext& cntx) = 0;

		// Update command
		virtual program_data::CmdResult update(CommandContextMotion& cntx, double dt) = 0;

		// Get current result
		virtual program_data::CmdResult currentResult() const = 0;

		// Execute command
		virtual void execute() = 0;

		virtual interpreter::IStoredProgram* getProgram() const = 0;
		virtual void setProgram(interpreter::IStoredProgram* program) = 0;

		// Mark the command as failed with a message
		virtual void markFailed(const std::string& message) = 0;
		// Mark the command as completed
		virtual void markCompleted() = 0;
		// Check if the command has started
		virtual bool hasStarted() const = 0;
	};
	
} // namespace interpreter