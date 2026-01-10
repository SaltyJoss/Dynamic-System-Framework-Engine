#include "pch.h"
#include "Interpreter/CommandFactory.h"

namespace language {
	CommandFactory& CommandFactory::Instance() {
		static CommandFactory instance;
		return instance;
	}

	bool CommandFactory::registerCommand(const std::string& name, Creator creator) {
		auto result = _registry.emplace(name, creator);
		return result.second; // returns true if insertion took place
	}

	std::unique_ptr<ICommand> CommandFactory::create(const std::string& name, const ParserArgs& args) const {
		auto it = _registry.find(name);
		if (it != _registry.end()) {
			return (it->second)(args);
		}
		return nullptr; // or throw an exception if preferred
	}

	bool CommandFactory::hasCommand(const std::string& name) const {
		return _registry.find(name) != _registry.end();
	}
} // namespace language