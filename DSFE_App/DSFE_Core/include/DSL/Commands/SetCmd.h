/*
 * File: DSL/Commands/SetCmd.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include <core/Types.h>
#include <memory>
#include <string>
#include <vector>

#include "DSL/Command.h"
#include "DSL/CommandContext.h"

#include "Platform/Logger.h"

namespace commands {

	enum class SetTargetType {
		IntegratorMethod,
		Omega,
		FixedDt,
		Gravity
	};

	struct DSFE_API SetTarget {
		SetTargetType type = SetTargetType::IntegratorMethod;
		IntegratorMethod method = IntegratorMethod::RK4; // Default method
		mathlib::Vec3 omega{ 0.0, 0.0, 0.0 };
		double fixedDt = 0.0;
		double gravity = 0.0;
	};

	// Class representing the SET command
	class DSFE_API SetCmd final : public Command {
	public:
		SetCmd(const std::string& id, const std::string& tokens);
		std::string_view getName() const { return "set"; }
		void setContext(CommandContext& cntx) { _cntx = &cntx; }

		// Getters and Setters for Result
		CmdResult getResult() const { return _result; }
		void setResult(const CmdResult& result) { _result = result; }
		CmdResult currentResult() const override { return getResult(); }
		// Get current method
		IntegratorMethod getCurrentMethod() const { return _method; }

		// Execute the command
		void execute() override;

	private:
		SetTarget _target{};
		std::string _id;
		std::string _tokens;
		IntegratorMethod _method = IntegratorMethod::RK4;
		CommandContext* _cntx = nullptr;
		CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;
	};

	// --- Free Function to Create SetCmd ---
	std::unique_ptr<ICommand> CreateSetCmd(const std::string& id, const std::vector<std::string>& tokens);
} // namespace commands