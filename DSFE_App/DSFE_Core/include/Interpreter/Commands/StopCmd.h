// DSFE_Core StopCmd.h
#pragma once

#include "EngineCore.h"

#include "Interpreter/SimFwd.h"
#include "Interpreter/Command.h"
#include "Interpreter/CommandContextMotion.h"

#include <memory>
#include <string>
#include <vector>

namespace commands {
	class DSFE_API StopCmd final : public Command {
	public:
		// Constructor
		StopCmd();

		std::string_view getName() const { return "stop"; }
		void setContext(CommandContextMotion& cntx) { _mtnCntx = &cntx; }
		
		program_data::CmdResult getResult() const { return _result; }
		void setResult(const program_data::CmdResult& result) { _result = result; }
		program_data::CmdResult currentResult() const override { return getResult(); }

	private:
		void execute() override;


		CommandContextMotion* _mtnCntx = nullptr;
		bool _started = false;

		program_data::CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;
	};

	std::unique_ptr<ICommand> CreateStopCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands