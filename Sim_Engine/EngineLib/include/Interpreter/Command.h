#pragma once

#include "EngineCore.h"
#include "ICommand.h"
#include "CommandContextMotion.h"

using namespace interpreter;

namespace commands {
	// Class representing a generic command
	class ENGINE_API Command : public ICommand {
	public:
		// Check parameters
		void validateParameters(const std::vector<std::string>& params) const override {
			if (params.empty()) {
				return false; // No parameters provided
			}
			return true; // Default implementation=
		}
		// Setup command with parameters
		bool set(const std::vector<std::string>& params) override {
			validateParameters(params);
			return true; // Default implementation
		}
		// Execute command
		void execute() override; // No base implementation

		CmdResult update(CommandContextMotion& cntx, double dt) override;

	protected:
		// Mark the command as failed with a message
		void markFailed(const std::string& message);
		// Mark the command as completed
		void markCompleted();
		// Check if the command has started
		bool hasStarted() const;

	};
} // namespace commands
