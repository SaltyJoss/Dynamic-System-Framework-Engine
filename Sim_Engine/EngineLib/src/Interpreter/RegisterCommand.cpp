#include "pch.h"
#include "Interpreter/RegisterCommand.h"
#include "Interpreter/Commands/SpinCmd.h"
#include "Interpreter/Commands/RotateToCmd.h"
#include "Interpreter/Commands/RotateByCmd.h"
#include "Interpreter/Commands/RotateJointToCmd.h"
#include "Interpreter/Commands/RotateJointByCmd.h"
#include "Interpreter/Commands/LoadCmd.h"
#include "Interpreter/Commands/SetCmd.h"

namespace commands {
	void RegisterAllCommands(CommandFactory& factory) {
		// Motion commands
		factory.registerCommand("spin",				&commands::CreateSpinCmd);			// spin command
		factory.registerCommand("rotateto",			&commands::CreateRotateToCmd);		// rotate command
		factory.registerCommand("rotateby",			&commands::CreateRotateByCmd);		// rotate command
		factory.registerCommand("rotatejointto",	&commands::CreateRotateJointToCmd);	// rotateJoint command
		factory.registerCommand("rotatejointby",	&commands::CreateRotateJointByCmd);	// rotateJoint command
		// Setup commands
		factory.registerCommand("load",	&commands::CreateLoadCmd);		// load command
		factory.registerCommand("set",	&commands::CreateSetCmd);		// set command
		// New commands later
	}
} // namespace commands