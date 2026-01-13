#pragma once

#include "EngineCore.h"
#include <MathLibAPI.h>
#include <core/Types.h>
#include "Interpreter/Command.h"
#include "Interpreter/CommandContextMotion.h"

namespace commands {
	// Enum for translation target type
	enum class TranslateTargetType {
		AxisMask,
		objID
	};

	// struct for translation target
	struct TranslateTarget {
		AxisMask axisMask;
		std::string objID = "";
	};

	// Class representing the TRANSLATE command
	class ENGINE_API TranslateCmd final : public Command {
	public:
		// Constructor
		TranslateCmd(TranslateTarget target, double velocity);

		// Get the command name
		std::string_view getName() const { return "TRANSLATE"; }

		// Set the command context
		void setContext(CommandContextMotion& cntx) override { _cntx = &cntx; }
		
		// Getters and Setters for Result
		CmdResult getResult() const { return _result; }
		void setResult(const CmdResult& result) { _result = result; }

		// Execute the command
		void execute() override;
		// Update the command
		CmdResult update(CommandContextMotion& cntx, double dt) override;

	private: 
		TranslateTarget _target{};
		double _velocity = 0.0; // Translation velocity (units/s)
		bool  _started = false;  // Flag to indicate if translation has started

		double _currentDistance = 0.0; // Current translated distance
		double _totalTranslated = 0.0; // Total translated distance

		CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		// Mark the command as failed with a message
		void markFailed(const std::string& message) override;
		// Mark the command as completed
		void markCompleted() override;
		// Check if the command has started
		bool hasStarted() const override;
	};

	// --- Free Function to Create TranslateCmd ---
	std::unique_ptr<ICommand> CreateTranslateCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands
