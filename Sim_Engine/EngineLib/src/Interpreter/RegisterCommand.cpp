#include "pch.h"
#include "Interpreter/RegisterCommand.h"
#include "Interpreter/Commands/RotateCmd.h"

namespace commands {
	void RegisterAllCommands(CommandFactory& factory) {
		factory.registerCommand("ROTATE", [](const std::string id, const std::vector<std::string>& args) -> ICommand* { return CreateRotateCmd(id, args).release(); });
		// New commands later
	}
} // namespace language