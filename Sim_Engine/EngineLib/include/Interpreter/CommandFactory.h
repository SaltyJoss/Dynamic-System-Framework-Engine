#pragma once

#include "EngineCore.h"
#include <memory>
#include <string>
#include <unordered_map>
#include <functional>

class ICommand;
struct ParserArgs;

namespace language {
	class ENGINE_API CommandFactory {
	public:
		// Get the singleton instance of CommandFactory
		static CommandFactory& Instancse();

		// Public API
		using Creator = std::unique_ptr<ICommand>(*)(const ParserArgs&);
		bool registerCommand(const std::string& name, Creator creator);
		std::unique_ptr<ICommand> create(const std::string& name, const ParserArgs& args) const;
		bool hasCommand(const std::string& name) const;

		void RegisterAllCommands(CommandFactory& factory); // Registers all available commands

		// Delete copy constructor and assignment operator to prevent copies
		CommandFactory(const CommandFactory&) = delete;
		CommandFactory& operator=(const CommandFactory&) = delete;

	private:
		//singleton instance
		CommandFactory() = default;

		std::unordered_map<std::string, Creator> _registry;
	};
}