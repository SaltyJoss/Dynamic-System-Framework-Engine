#pragma once

#include "EngineCore.h"
#include <string>
#include <unordered_map>

namespace language {
	class ENGINE_API CommandFactory {
	public:
		// Get the singleton instance of CommandFactory
		static CommandFactory* Instance() {
			if (!_instance) {
				_instance = new CommandFactory();
			}
			return _instance;
		}
		// Delete copy constructor and assignment operator to prevent copies
		CommandFactory(const CommandFactory&) = delete;
		CommandFactory& operator=(const CommandFactory&) = delete;

	private:
		//singleton instance
		static CommandFactory* _instance;

		std::unordered_map<std::string, d> _commandMap;
	};
}