/*
 * File: DSL/RegisterCommand.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "DSL/CommandFactory.h"

namespace commands {
	// Free function to register all commands
	void RegisterAllCommands(CommandFactory& factory);
} // namespace commands