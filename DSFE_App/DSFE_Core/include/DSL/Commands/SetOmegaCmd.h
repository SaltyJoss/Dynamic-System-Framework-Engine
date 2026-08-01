/*
 * File: DSL/Commands/SetOmegaCmd.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include <core/Types.h>

#include "DSL/SimFwd.h"
#include "DSL/Command.h"
#include "DSL/CommandContext.h"

namespace commands {
	class SetOmegaCmd final : public Command {
	public:
		SetOmegaCmd(std::string link, const double omega);
		~SetOmegaCmd() override = default;

		std::string_view getName() const { return "setomega"; }
		void setContext(CommandContext& cntx) override { _cntx = &cntx; }
		CmdResult getResult() const { return _result; }
		void setResult(const CmdResult& result) { _result = result; }
		CmdResult currentResult() const override { return getResult(); }

	private:
		CmdResult update(CommandContext& cntx, double dt) override;
		void execute() override;

		std::string _link;
		double _omega;
		bool _started = false;
		CmdResult _result{ CmdState::NotStarted, {}, "" };

	protected:
		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;
	};

	std::unique_ptr<ICommand> CreateSetOmegaCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands