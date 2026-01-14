#include "pch.h"
#include "Interpreter/Commands/ColourCmd.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;

namespace commands {
	// --- ColourCmd Method Implementations ---
	void SetCmd::markFailed(const std::string& message) {
		setResult({ CmdState::Failed, {}, message });
		// Implementation to mark the command as failed
	}
	void SetCmd::markCompleted() {
		// Implementation to mark the command as completed
	}
	bool SetCmd::hasStarted() const {
		return getResult().state != CmdState::NotStarted;
	}
	void SetCmd::execute() {
		if (_cntxUI == nullptr) {
			markFailed("UI context is not set.");
			return;
		}
	}

	std::unique_ptr<ICommand> CreateColourCmd(const std::string& parameter, const std::string& value) {
		return std::make_unique<SetCmd>(parameter, value);
	}
} // namespace commands