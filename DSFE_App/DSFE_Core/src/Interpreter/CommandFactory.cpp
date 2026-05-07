// DSFE_Core CommandFactory.cpp
#include "pch.h"

#include "Interpreter/CommandFactory.h"

namespace commands {
	CommandFactory& CommandFactory::Instance() {
		static CommandFactory instance;
		return instance;
	}

	bool CommandFactory::registerCommand(const std::string name, Creator creator) {
		auto result = _registry.emplace(name, creator);
		return result.second; // returns true if insertion took place
	}

	ICommand* CommandFactory::create(const std::string_view& name, const std::string& id, const std::vector<std::string>& args) const {
		// new logic to find and create command
		auto it = _registry.find(std::string(name));
		if (it != _registry.end()) {
			Creator creator = it->second;
			std::unique_ptr<ICommand> cmdPtr = creator(id, args);
			return cmdPtr.release(); // transfer ownership to caller
		}
		return nullptr; // Command not found
	}

	bool CommandFactory::hasCommand(const std::string_view& name) const {
		return _registry.find(std::string(name)) != _registry.end();
	}
} // namespace commands