/*
 * File: DSL/Commands/LoadCmd.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include <core/Types.h>

#include "DSL/Command.h"
#include "DSL/CommandContext.h"

namespace commands {
	// Enum for load target type
	enum class LoadTargetType {
		RigidBody
	};

	// Struct for load target
	struct DSFE_API LoadTarget {
		LoadTargetType type = LoadTargetType::RigidBody;
		std::string path;
	};

	// Class representing the LOAD command
	class DSFE_API LoadCmd final : public Command {
	public:
		// Constructor
		LoadCmd(const std::string& id, const std::vector<std::string>& tokens);

		// Get the command name
		std::string_view getName() const { return "load"; }

		CmdResult getResult() const { return _result; }
		void setResult(const CmdResult& result) { _result = result; }

		// Get current result
		CmdResult currentResult() const override { return getResult(); }

		// Execute the command
		void execute() override;

	private:
		LoadTarget _target{};
		CmdResult _result = { CmdState::NotStarted, {}, "" };
		std::string _path;

	protected:
		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;
	};

	// --- Free Function to Create LoadCmd ---
	std::unique_ptr<ICommand> CreateLoadCmd(const std::string& id, const std::vector<std::string>& tokens);
} // namespace commands
