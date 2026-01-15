#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include "Interpreter/Command.h"
#include <memory>
#include <string>
#include <vector>

#include "Platform/Logger.h"

namespace commands {

	enum class SetTargetType {
		IntegratorMethod,
		FunctionDefinitio
	};

	struct SetTarget {
		SetTargetType type = SetTargetType::IntegratorMethod;
		IntegratorMethod method = IntegratorMethod::Euler; // Default method
	};

	// Class representing the SET command
	class ENGINE_API SetCmd final : public Command {
	public:
		// Constructor
		SetCmd(const std::string& id, const std::vector<std::string>& tokens);
		// Get the command name
		std::string_view getName() const { return "SET"; }

		// Getters and Setters for Result
		CmdResult getResult() const { return _result; }
		void setResult(const CmdResult& result) { _result = result; }

		// Execute the command
		void execute() override;

	private:
		SetTarget _target{};

		std::string _id;
		std::vector<std::string> _tokens;
		CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		// Mark the command as failed with a message
		void markFailed(const std::string& message) override;
		// Mark the command as completed
		void markCompleted() override;
		// Check if the command has started
		bool hasStarted() const override;
	};

	// --- Free Function to Create SetCmd ---
	std::unique_ptr<ICommand> CreateSetCmd(const std::string& id, const std::vector<std::string>& tokens);
} // namespace commands