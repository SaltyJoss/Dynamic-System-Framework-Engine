#include "pch.h"
#include "Interpreter/RegisterCommand.h"
#include "Interpreter/Commands/RotateCmd.h"
#include "Interpreter/Commands/LoadCmd.h"
#include "Interpreter/Commands/SetCmd.h"
#include "Interpreter/Commands/ColourCmd.h"

namespace commands {
	void RegisterAllCommands(CommandFactory& factory) {
		factory.registerCommand("rotate", &commands::CreateRotateCmd);	// rotate command
		factory.registerCommand("load", &commands::CreateLoadCmd);		// load command
		factory.registerCommand("set", &commands::CreateSetCmd);		// set command
		factory.registerCommand("colour", &commands::CreateColourCmd);	// colour command
		// New commands later
	}
} // namespace commands