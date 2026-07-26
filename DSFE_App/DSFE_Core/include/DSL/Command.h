/*
 * File: DSL/Command.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once
#include "EngineCore.h"
#include "DSL/ICommand.h"
#include "DSL/MainContext.h"

namespace commands {
	// Class representing a generic command
	class DSFE_API Command : public ICommand {
	public:
		// Set the command context
		void setContext(CommandContext& cntx) override { _cntx = &cntx; }
		
		program_data::CmdResult update(CommandContext& cntx, double dt) override;

		void execute() override;

		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;

		dsl::IStoredProgram* getProgram() const override { return _program; }
		void setProgram(dsl::IStoredProgram* program) override { _program = program; }

	protected:
		dsl::IStoredProgram* _program = nullptr;
		CommandContext* _cntx = nullptr;
	};
} // namespace commands
