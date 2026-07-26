/*
 * File: DSL/ICommand.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include "DSL/IStoredProgram.h"
#include <memory>
#include <string>

namespace commands {
	// Forward declaration of ICommand for use in IStoredProgram
	class CommandContext;

	// ICommand interface
	class DSFE_API ICommand {
	public:
		// Virtual destructor
		virtual ~ICommand() = default;

		// Context setters
		virtual void setContext(CommandContext& cntx) = 0;

		// Update command
		virtual program_data::CmdResult update(CommandContext& cntx, double dt) = 0;

		// Get current result
		virtual program_data::CmdResult currentResult() const = 0;

		// Execute command
		virtual void execute() = 0;

		virtual dsl::IStoredProgram* getProgram() const = 0;
		virtual void setProgram(dsl::IStoredProgram* program) = 0;

		// Mark the command as failed with a message
		virtual void markFailed(const std::string& message) = 0;
		// Mark the command as completed
		virtual void markCompleted() = 0;
		// Check if the command has started
		virtual bool hasStarted() const = 0;
	};
	
} // namespace interpreter