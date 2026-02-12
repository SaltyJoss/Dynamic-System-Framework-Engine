#include "pch.h"
// File:   Command.cpp
#include "Interpreter/Command.h"

namespace commands {
	// Execute command
	void Command::execute() {
		// No base implementation
	}
	CmdResult Command::update(CommandContextMotion& cntx, double dt) {
		return CmdResult{ CmdState::NotStarted, {}, "" };
	}
	// Mark the command as failed with a message
	void Command::markFailed(const std::string& message) {
		// Base implementation (if any) can go here
	}
	// Mark the command as completed
	void Command::markCompleted() {
		// Base implementation (if any) can go here
	}
	// Check if the command has started
	bool Command::hasStarted() const {
		return false; // Base implementation (if any) can go here
	}
} // namespace commands