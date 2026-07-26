/*
 * File: DSL/Commands/RotateByCmd.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include <core/Types.h>

#include "DSL/Command.h"
#include "DSL/CommandContext.h"

namespace commands {

	// Class representing the ROTATE command
	class DSFE_API RotateByCmd final : public Command {
	public:
		// Constructor
		RotateByCmd(utils::AxisMask axis, double maxOmegaDeg, double deltaDeg);

		std::string_view getName() const { return "rotateBy"; }
		void setContext(CommandContext& cntx) override { _cntx = &cntx; }
		program_data::CmdResult getResult() const { return _result; }
		void setResult(const CmdResult& result) { _result = result; }
		CmdResult currentResult() const override { return getResult(); }

	private:
		void execute() override;
		CmdResult update(CommandContext& cntx, double dt) override;

		utils::AxisMask _axes{};
		double _deltaDeg;
		double _omegaDeg;
		double _totalRotated = 0.0;
		bool _started = false;

		CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;
	};

	// Free function to create a RotateCmd
	std::unique_ptr<ICommand> CreateRotateByCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands