#pragma once
// File:    CommandFactory.h
// GitHub:  SaltyJoss
#include "EngineCore.h"
#include "ICommand.h"
#include <unordered_map>
#include <functional>

namespace commands {
	// Type alias for command creator function
	using Creator = std::unique_ptr<commands::ICommand>(*)(const std::string&, const std::vector<std::string>&);

	// CommandFactory class for registering and creating commands
	class ENGINE_API CommandFactory {
	public:
		// Get the singleton instance of CommandFactory
		static CommandFactory& Instance();

		// Public API
		bool registerCommand(const std::string name, Creator creator);
		// Create a command by name
		ICommand* create(const std::string_view& name, const std::string& id, const std::vector<std::string>& args) const;
		// Check if a command is registered
		bool hasCommand(const std::string_view& name) const;
		// Get a list of registered command names
		std::vector<std::string> commandNames() const;

		// Delete copy constructor and assignment operator to prevent copies
		CommandFactory(const CommandFactory&) = delete;
		CommandFactory& operator=(const CommandFactory&) = delete;

	private:
		//singleton instance
		CommandFactory() = default;

		std::unordered_map<std::string, Creator> _registry;
	};
} // namespace commands