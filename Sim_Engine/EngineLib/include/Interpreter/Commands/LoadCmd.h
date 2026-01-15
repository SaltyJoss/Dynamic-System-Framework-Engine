#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include "Interpreter/Command.h"
#include "Interpreter/UIContext.h"

namespace commands {
	// Enum for load target type
	enum class LoadTargetType {
		Object,
		Robot,
		Texture
	};

	// Struct for load target
	struct LoadTarget {
		LoadTargetType type = LoadTargetType::Object;
		std::string path;
	};

	// Class representing the LOAD command
	class ENGINE_API LoadCmd final : public Command {
	public:
		// Constructor
		LoadCmd(const std::string& id, const std::vector<std::string>& tokens);

		// Get the command name
		std::string_view getName() const { return "LOAD"; }

		CmdResult getResult() const { return _result; }
		void setResult(const CmdResult& result) { _result = result; }

		// Execute the command
		void execute() override;

	private:
		LoadTarget _target{};
		CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		// Mark the command as failed with a message
		void markFailed(const std::string& message) override;
		// Mark the command as completed
		void markCompleted() override;
		// Check if the command has started
		bool hasStarted() const override;
	};

	// --- Free Function to Create LoadCmd ---
	std::unique_ptr<ICommand> CreateLoadCmd(const std::string& id, const std::vector<std::string>& tokens);
}
