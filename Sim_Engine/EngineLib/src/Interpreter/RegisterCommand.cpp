#include "pch.h"
#include "Interpreter/RegisterCommand.h"
#include "Interpreter/Commands/RotateCmd.h"

namespace commands {
	void RegisterAllCommands(CommandFactory& factory) {
		factory.registerCommand("ROTATE", &CreateRotateCmd); // Register ROTATE command
	}
} // namespace language