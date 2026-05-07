// DSFE_Core SetCmd.h
#pragma once

#include "EngineCore.h"

#include <MathLibAPI.h>
#include <core/Types.h>

#include "Interpreter/Command.h"
#include "Interpreter/UIContext.h"

#include <memory>
#include <string>
#include <vector>

#include "Platform/Logger.h"

namespace commands {

	enum class SetTargetType {
		IntegratorMethod,
		Omega,
		FixedDt,
		Gravity,
		Colour
	};

	struct DSFE_API SetTarget {
		SetTargetType type = SetTargetType::IntegratorMethod;
		IntegratorMethod method = IntegratorMethod::RK4; // Default method
		mathlib::Vec3 omega{ 0.0, 0.0, 0.0 };
		double fixedDt = 0.0;
		double gravity = 0.0;
		Colour colour{ BlockColour::Red, mathlib::Vec3{ 1.0, 0.0, 0.0 } };
	};

	// Class representing the SET command
	class DSFE_API SetCmd final : public Command {
	public:
		// Constructor
		SetCmd(const std::string& id, const std::string& tokens);
		// Get the command name
		std::string_view getName() const { return "set"; }
		// Set the command context
		void setContext(UIContext& cntx) { _uiCntx = &cntx; }

		// Getters and Setters for Result
		program_data::CmdResult getResult() const { return _result; }
		void setResult(const program_data::CmdResult& result) { _result = result; }

		// Get current result
		program_data::CmdResult currentResult() const override { return getResult(); }

		// Get current method
		IntegratorMethod getCurrentMethod() const { return _method; }

		// Colour Setters
		void setColour(const mathlib::Vec3& rgb);
		void setColour(const std::string& hex);

		// Execute the command
		void execute() override;

	private:
		SetTarget _target{};

		std::string _id;
		std::string _tokens;

		IntegratorMethod _method = IntegratorMethod::RK4;		// Default - Euler
		mathlib::Vec3 _colRGB = mathlib::Vec3{ 1.0, 0.0, 0.0 };	// Default - RED
		Colour _col{ BlockColour::Red };

		UIContext* _uiCntx = nullptr;

		program_data::CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;
	};

	// --- Free Function to Create SetCmd ---
	std::unique_ptr<ICommand> CreateSetCmd(const std::string& id, const std::vector<std::string>& tokens);
} // namespace commands