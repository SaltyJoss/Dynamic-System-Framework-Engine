// DSFE_Core SelectCmd.h
#pragma once

#include "EngineCore.h"

#include "Interpreter/SimFwd.h"
#include "Interpreter/Command.h"

#include <memory>
#include <string>
#include <vector>

namespace commands {
	class DSFE_API SelectCmd final : public Command {
	public:
		// Constructor
		SelectCmd();

		std::string_view getName() const { return "select"; }
		void setContext(CommandContext& cntx) { _cntx = &cntx; }

		program_data::CmdResult getResult() const { return _result; }
		void setResult(const program_data::CmdResult& result) { _result = result; }
		program_data::CmdResult currentResult() const override { return getResult(); }

	private:
		void execute() override;

		CommandContext* _cntx = nullptr;

		program_data::CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;
	};

	std::unique_ptr<ICommand> CreateSelectCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands