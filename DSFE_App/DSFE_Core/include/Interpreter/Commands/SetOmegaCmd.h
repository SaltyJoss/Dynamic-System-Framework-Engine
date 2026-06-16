// DSFE_Core SetOmegaCmd.h
#pragma once

#include "EngineCore.h"


#include <core/Types.h>

#include "Interpreter/SimFwd.h"
#include "Interpreter/Command.h"
#include "Interpreter/CommandContext.h"

namespace commands {
	class SetOmegaCmd final : public Command {
	public:
		SetOmegaCmd(std::string link, const double omega);
		~SetOmegaCmd() override = default;

		std::string_view getName() const { return "setomega"; }
		void setContext(CommandContext& cntx) override { _cntx = &cntx; }
		program_data::CmdResult getResult() const { return _result; }
		void setResult(const program_data::CmdResult& result) { _result = result; }
		program_data::CmdResult currentResult() const override { return getResult(); }

	private:
		program_data::CmdResult update(CommandContext& cntx, double dt) override;
		void execute() override;

		std::string _link;
		double _omega;

		bool _started = false;

		program_data::CmdResult _result{ CmdState::NotStarted, {}, "" };

	protected:
		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;
	};

	std::unique_ptr<ICommand> CreateSetOmegaCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands