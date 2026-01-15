#include "pch.h"
#include "Interpreter/Commands/ColourCmd.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;

namespace commands {
	ColourCmd::ColourCmd(const std::string& id, const std::vector<std::string>& tokens)
		: _id(id), _tokens(tokens) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	// --- ColourCmd Method Implementations ---
	void ColourCmd::markFailed(const std::string& message) {
		setResult({ CmdState::Failed, {}, message });
		// Implementation to mark the command as failed
	}
	void ColourCmd::markCompleted() {
		// Implementation to mark the command as completed
	}
	bool ColourCmd::hasStarted() const {
		return getResult().state != CmdState::NotStarted;
	}
	void ColourCmd::execute() {
		if (_cntxUI == nullptr) {
			std::string errMsg = "UI context is not set for colour() command";
			markFailed(errMsg);
			D_FAIL(errMsg.c_str());
			return;
		}
	}

	std::unique_ptr<ICommand> CreateColourCmd(const std::string& id, const std::vector<std::string>& tokens) {
		return std::make_unique<ColourCmd>(id, tokens);
	}
} // namespace commands