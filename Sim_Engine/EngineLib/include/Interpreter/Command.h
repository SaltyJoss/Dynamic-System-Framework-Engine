#pragma once

#include "EngineCore.h"
#include "ICommand.h"
#include "CommandContextMotion.h"

using namespace interpreter;

namespace commands {
	// Class representing a generic command
	class ENGINE_API Command : public ICommand {
	public:
		// Set the command context
		void setContext(CommandContextMotion& cntx) override { _cntx = &cntx; }

		CmdResult update(CommandContextMotion& cntx, double dt) override;

		// Execute command
		void execute() override; // No base implementation

		// Mark the command as failed with a message
		void markFailed(const std::string& message) override;
		// Mark the command as completed
		void markCompleted() override;
		// Check if the command has started
		bool hasStarted() const override;

	protected:
		CommandContextMotion* _cntx = nullptr;
	};
} // namespace commands
