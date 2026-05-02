#pragma once
// File:    LoadCmd.h
// GitHub:  SaltyJoss
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
	struct DSFE_API LoadTarget {
		LoadTargetType type = LoadTargetType::Object;
		std::string path;
	};

	// Class representing the LOAD command
	class DSFE_API LoadCmd final : public Command {
	public:
		// Constructor
		LoadCmd(const std::string& id, const std::vector<std::string>& tokens);

		// Get the command name
		std::string_view getName() const { return "load"; }

		program_data::CmdResult getResult() const { return _result; }
		void setResult(const program_data::CmdResult& result) { _result = result; }

		// Get current result
		program_data::CmdResult currentResult() const override { return getResult(); }

		// Execute the command
		void execute() override;

	private:
		LoadTarget _target{};
		program_data::CmdResult _result = { CmdState::NotStarted, {}, "" };
		std::string _path;

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
} // namespace commands
