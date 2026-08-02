/*
 * File: DSL/RegisterCommand.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "DSL/RegisterCommand.h"

// Motion commands
#include "DSL/Commands/TrajSetCmd.h"
#include "DSL/Commands/TrajClearCmd.h"
#include "DSL/Commands/SetVelocityCmd.h"

// Primary function commands
#include "DSL/Commands/StartCmd.h"
#include "DSL/Commands/StopCmd.h"
#include "DSL/Commands/WaitCmd.h"
#include "DSL/Commands/LoadCmd.h"
#include "DSL/Commands/SetCmd.h"

namespace commands {
	// Register all commands with the factory
	void RegisterAllCommands(CommandFactory& factory) {
		// Motion commands
		factory.registerCommand("trajset",			&commands::CreateTrajSetCmd);		// trajSet command
		factory.registerCommand("trajclear",		&commands::CreateTrajClearCmd);		// trajClear command
		factory.registerCommand("setvelocity", 		&commands::CreateSetVelocityCmd);	// setVelocity command
		// Primary Function commands
		factory.registerCommand("start",			&commands::CreateStartCmd);	 // start command
		factory.registerCommand("stop",				&commands::CreateStopCmd);	 // stop command
		factory.registerCommand("wait",				&commands::CreateWaitCmd);	 // pause command
		factory.registerCommand("load",				&commands::CreateLoadCmd);	 // load command
		factory.registerCommand("set",				&commands::CreateSetCmd);	 // set command
		// New commands later
	}
} // namespace commands