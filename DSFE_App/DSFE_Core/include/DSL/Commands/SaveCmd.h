/*
 * File: DSL/Commands/SaveCmd.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once

#include "EngineCore.h"
#include <memory>
#include <string>
#include <vector>

#include "DSL/SimFwd.h"
#include "DSL/Command.h"

namespace commands {
	// Types of saves that can be performed by the SaveCmd
	enum class eSaveType {
		SimData,
		Plots
	};

	struct ENGINE_API SaveCmdArgs {
		eSaveType type;
		std::string filename;
		bool isIntegratorName;
	};

	class ENGINE_API SaveCmd : public Command {
	public:
		// Constructor
		SaveCmd(const std::string& id, const std::vector<std::string>& tokens);

		std::string_view getName() const { return "save"; }
		void setContext(UIContext& cntx) { _cntx = &cntx; }

		CmdResult getResult() const { return _result; }
		void setResult(const CmdResult& result) { _result = result; }
		CmdResult currentResult() const override { return getResult(); }

	private:
		void execute() override;

		CommandContext* _cntx = nullptr;
		SaveCmdArgs _target;
		std::string _filename = "";   // Filename to save to
		bool _integratorName  = false; // Whether to include integrator name in the filename
		bool _started = false;

		CmdResult _result = { CmdState::NotStarted, {}, "" };

	protected:
		void markFailed(const std::string& message) override;
		void markCompleted() override;
		bool hasStarted() const override;
	};
	
	std::unique_ptr<ICommand> CreateSaveCmd(const std::string& id, const std::vector<std::string>& args);
} // namespace commands