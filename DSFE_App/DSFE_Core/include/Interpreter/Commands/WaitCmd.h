// DSFE_Core WaitCmd.h
#pragma once

#include "EngineCore.h"
#include <memory>
#include <string>
#include <vector>

#include "Interpreter/SimFwd.h"
#include "Interpreter/Command.h"
#include "Interpreter/CommandContext.h"

namespace commands {
	class DSFE_API WaitCmd final : public Command {
	public:
		// Constructor
		WaitCmd(double t);

		std::string_view getName() const { return "stop"; }
		void setContext(CommandContext& cntx) { _cntx = &cntx; }

		program_data::CmdResult getResult() const { return _result; }
		void setResult(const program_data::CmdResult& result) { _result = result; }
		program_data::CmdResult currentResult() const override { return getResult(); }

	private:
		void execute() override;
		program_data::CmdResult update(CommandContext& cntx, double dt) override;

		utils::AxisMask _axes{};

		double _remainingTime = 0.0;
		bool _started = false;

		program_data::CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;
	};

	std::unique_ptr<ICommand> CreateWaitCmd(const std::string& id, const std::vector<std::string>& args);
}