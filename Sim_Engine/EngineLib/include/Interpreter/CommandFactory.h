#pragma once

#include "EngineCore.h"
#include <memory>
#include <string>
#include <unordered_map>
#include <functional>

class ICommand;
struct ParserArgs;

namespace commands {
	// Type alias for command creator function
	using Creator = std::unique_ptr<ICommand>(*)(const ParserArgs&);

	// CommandFactory class for registering and creating commands
	class ENGINE_API CommandFactory {
	public:
		// Get the singleton instance of CommandFactory
		static CommandFactory& Instance();

		// Public API
		bool registerCommand(const std::string& name, Creator creator);
		// Create a command by name
		std::unique_ptr<ICommand> create(const std::string& name, const ParserArgs& args) const;
		// Check if a command is registered
		bool hasCommand(const std::string& name) const;
		// Register all available commands
		void RegisterAllCommands(CommandFactory& factory); // Registers all available commands

		// Delete copy constructor and assignment operator to prevent copies
		CommandFactory(const CommandFactory&) = delete;
		CommandFactory& operator=(const CommandFactory&) = delete;

	private:
		//singleton instance
		CommandFactory() = default;

		std::unordered_map<std::string, Creator> _registry;
	};
} // namespace commands