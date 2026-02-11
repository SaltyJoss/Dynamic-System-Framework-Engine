#pragma once
// File:    RegisterCommand.h
// GitHub:  SaltyJoss
#include "CommandFactory.h"

namespace commands {
	// Free function to register all commands
	void RegisterAllCommands(CommandFactory& factory);
} // namespace commands