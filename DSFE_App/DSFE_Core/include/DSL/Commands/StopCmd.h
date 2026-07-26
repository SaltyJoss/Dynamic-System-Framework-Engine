/*
 * File: DSL/Commands/StopCmd.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include <memory>
#include <string>
#include <vector>

#include "DSL/SimFwd.h"
#include "DSL/Command.h"
#include "DSL/CommandContext.h"

namespace commands {
	class DSFE_API StopCmd final : public Command {
	public:
		// Constructor
		StopCmd();

		std::string_view getName() const { return "stop"; }
		void setContext(CommandContext& cntx) { _cntx = &cntx; }
		
		CmdResult getResult() const { return _result; }
		void setResult(const CmdResult& result) { _result = result; }
		CmdResult currentResult() const override { return getResult(); }

	private:
		void execute() override;

		bool _started = false;
		CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;
	};

	std::unique_ptr<ICommand> CreateStopCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands