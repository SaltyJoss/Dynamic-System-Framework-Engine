#pragma once

#include "EngineCore.h"
#include "ICommand.h"
#include "CommandContext.h"

namespace commands {
	// Class representing a generic command
	class ENGINE_API Command : public ICommand {
	public:
		// Get the command name
		std::string_view name() const override;

		// Start the command
		void start(CommandContext& cntx) override;
		// Update the command
		CmdResult update(CommandContext& cntx, double dt) override;
		// Stop the command
		void stop(CommandContext& cntx) override;
	protected:
		void markFailed(const std::string& message);
		void markCompleted();
		bool started() const;
	};
} // namespace commands
