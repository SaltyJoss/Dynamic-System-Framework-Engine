#pragma once

#include "EngineCore.h"
#include "ICommand.h"
#include "CommandContextMotion.h"

namespace commands {
	// Class representing a generic command
	class ENGINE_API Command : public ICommand {
	public:
		// Get the command name
		std::string_view name() const override;

		// Start the command
		void start(CommandContextMotion& cntx) override;
		// Update the command
		CmdResult update(CommandContextMotion& cntx, double dt) override;
		// Stop the command
		void stop(CommandContextMotion& cntx) override;
	protected:
		// Mark the command as failed with a message
		void markFailed(const std::string& message);
		// Mark the command as completed
		void markCompleted();
		// Check if the command has started
		bool hasStarted() const;
	};
} // namespace commands
