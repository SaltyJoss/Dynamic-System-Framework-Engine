#pragma once

#include "EngineCore.h"
#include "Interpreter/SimFwd.h"
#include "Interpreter/Command.h"
#include "Interpreter/UIContext.h"
#include <memory>
#include <string>
#include <vector>

namespace commands {
	class ENGINE_API StartCmd final : public Command {
	public:
		// Constructor
		StartCmd();

		std::string_view getName() const { return "stop"; }
		void setContext(UIContext& cntx) { _uiCntx = &cntx; }

		program_data::CmdResult getResult() const { return _result; }
		void setResult(const program_data::CmdResult& result) { _result = result; }
		program_data::CmdResult currentResult() const override { return getResult(); }

	private:
		void execute() override;

		UIContext* _uiCntx = nullptr;
		bool _started = false;

		program_data::CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		// Mark the command as failed with a message
		void markFailed(const std::string& message) override;
		// Mark the command as completed
		void markCompleted() override;
		// Check if the command has started
		bool hasStarted() const override;
	};

	std::unique_ptr<ICommand> CreateStartCmd(const std::string& id, const std::vector<std::string>& args);
}