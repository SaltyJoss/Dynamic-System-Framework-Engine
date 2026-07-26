/*
 * File: DSL/Command.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "DSL/Command.h"

namespace commands {
	// Execute command
	void Command::execute() {}
	// Update command with time step dt
	CmdResult Command::update(CommandContext& cntx, double dt) { return CmdResult{ CmdState::NotStarted, {}, "" }; }
	// Mark the command as failed with a message
	void Command::markFailed(const std::string& message) {}
	// Mark the command as completed
	void Command::markCompleted() {}
	// Check if the command has started
	bool Command::hasStarted() const { return false; }
} // namespace commands