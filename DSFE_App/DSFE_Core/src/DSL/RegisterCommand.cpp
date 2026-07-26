/*
 * File: DSL/RegisterCommand.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "DSL/RegisterCommand.h"

// Motion commands
#include "DSL/Commands/SpinCmd.h"
#include "DSL/Commands/RotateToCmd.h"
#include "DSL/Commands/RotateByCmd.h"
#include "DSL/Commands/RotateJointToCmd.h"
#include "DSL/Commands/RotateJointByCmd.h"
#include "DSL/Commands/TrajSetCmd.h"
#include "DSL/Commands/TrajClearCmd.h"
#include "DSL/Commands/SetOmegaCmd.h"

// Primary function commands
#include "DSL/Commands/StartCmd.h"
#include "DSL/Commands/StopCmd.h"
#include "DSL/Commands/WaitCmd.h"
#include "DSL/Commands/SelectCmd.h"
#include "DSL/Commands/LoadCmd.h"
#include "DSL/Commands/SetCmd.h"

namespace commands {
	// Register all commands with the factory
	void RegisterAllCommands(CommandFactory& factory) {
		// Motion commands
		factory.registerCommand("spin",				&commands::CreateSpinCmd);			// spin command
		factory.registerCommand("rotateto",			&commands::CreateRotateToCmd);		// rotate command
		factory.registerCommand("rotateby",			&commands::CreateRotateByCmd);		// rotate command
		factory.registerCommand("rotatejointto",	&commands::CreateRotateJointToCmd);	// rotateJoint command
		factory.registerCommand("rotatejointby",	&commands::CreateRotateJointByCmd);	// rotateJoint command
		factory.registerCommand("trajset",			&commands::CreateTrajSetCmd);		// trajSet command
		factory.registerCommand("trajclear",		&commands::CreateTrajClearCmd);		// trajClear command
		factory.registerCommand("setomega",			&commands::CreateSetOmegaCmd);		// setOmega command
		// Primary Function commands
		factory.registerCommand("start",	&commands::CreateStartCmd);	 // start command
		factory.registerCommand("stop",		&commands::CreateStopCmd);	 // stop command
		factory.registerCommand("wait",		&commands::CreateWaitCmd);	 // pause command
		factory.registerCommand("select",	&commands::CreateSelectCmd); // select command
		factory.registerCommand("load",		&commands::CreateLoadCmd);	 // load command
		factory.registerCommand("set",		&commands::CreateSetCmd);	 // set command
		// New commands later
	}
} // namespace commands