// DSFE_Core RegisterCommand.cpp
#include "pch.h"

#include "Interpreter/RegisterCommand.h"

// Motion commands
#include "Interpreter/Commands/SpinCmd.h"
#include "Interpreter/Commands/RotateToCmd.h"
#include "Interpreter/Commands/RotateByCmd.h"
#include "Interpreter/Commands/RotateJointToCmd.h"
#include "Interpreter/Commands/RotateJointByCmd.h"
#include "Interpreter/Commands/TrajSetCmd.h"
#include "Interpreter/Commands/TrajClearCmd.h"
#include "Interpreter/Commands/SetOmegaCmd.h"

// Primary function commands
#include "Interpreter/Commands/StartCmd.h"
#include "Interpreter/Commands/StopCmd.h"
#include "Interpreter/Commands/WaitCmd.h"
#include "Interpreter/Commands/SelectCmd.h"
#include "Interpreter/Commands/LoadCmd.h"
#include "Interpreter/Commands/SetCmd.h"

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