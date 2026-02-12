#pragma once
// File:    SelectCmd.h
// GitHub:  SaltyJoss
#include "EngineCore.h"
#include "Interpreter/SimFwd.h"
#include "Interpreter/Command.h"
#include "Interpreter/UIContext.h"
#include <memory>
#include <string>
#include <vector>

namespace commands {
	class ENGINE_API SelectCmd final : public Command {
	public:
		// Constructor
		SelectCmd(scene::ObjectID obj);

		std::string_view getName() const { return "stop"; }
		void setContext(UIContext& cntx) { _uiCntx = &cntx; }

		program_data::CmdResult getResult() const { return _result; }
		void setResult(const program_data::CmdResult& result) { _result = result; }
		program_data::CmdResult currentResult() const override { return getResult(); }

		void listObjID();

	private:
		void execute() override;

		scene::ObjectID _objID{};
		UIContext* _uiCntx = nullptr;

		program_data::CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		// Mark the command as failed with a message
		void markFailed(const std::string& message) override;
		// Mark the command as completed
		void markCompleted() override;
		// Check if the command has started
		bool hasStarted() const override;
	};

	std::unique_ptr<ICommand> CreateSelectCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands