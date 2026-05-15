// DSFE_Core Command.h
#pragma once
#pragma warning(disable : 4100)

#include "EngineCore.h"

#include "ICommand.h"
#include "MainContext.h"

using namespace interpreter;

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

		interpreter::IStoredProgram* getProgram() const override { return _program; }
		void setProgram(interpreter::IStoredProgram* program) override { _program = program; }

	protected:
		IStoredProgram* _program = nullptr;
		CommandContext* _cntx = nullptr;
	};
} // namespace commands
