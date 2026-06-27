// DSFE_Core SpinCmd.h
#pragma once

#include "EngineCore.h"
#include <core/Types.h>

#include "Interpreter/Command.h"
#include "Interpreter/CommandContext.h"

namespace commands {

	// Class representing the ROTATE command
	class DSFE_API SpinCmd final : public Command {
	public:
		// Constructor
		SpinCmd(utils::AxisMask axes, double omegaDeg, double duration);

		std::string_view getName() const { return "SpinCmd"; }
		void setContext(CommandContext& cntx) override { _cntx = &cntx; }
		program_data::CmdResult getResult() const { return _result; }
		void setResult(const program_data::CmdResult& result) { _result = result; }
		program_data::CmdResult currentResult() const override { return getResult(); }

	private:
		void execute() override;
		program_data::CmdResult update(CommandContext& cntx, double dt) override;

		utils::AxisMask _axes;
		double _omegaDeg = 0.0;
		double _duration = 0.0;
		double _remainingTime = 0.0;
		bool _started = false;

		CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		void markFailed(const std::string& message);
		void markCompleted();
		bool hasStarted() const;
	};

	// Free function to create a RotateCmd
	std::unique_ptr<ICommand> CreateSpinCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands