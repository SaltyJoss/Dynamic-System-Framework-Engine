// DSFE_Core StopCmd.cpp
#include "pch.h"

#include "Interpreter/Commands/StopCmd.h"

#include "EngineLib/LogMacros.h"

namespace commands {
	// --- StopCmd Mark Methods ---
	void StopCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void StopCmd::markCompleted() { setResult({ CmdState::Executed, {}, "stopSim() ran successfully" }); }
	bool StopCmd::hasStarted() const { return getResult().state != CmdState::NotStarted; }

	// Constructor
	StopCmd::StopCmd() : _started(nullptr) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	// Execute the command
	void StopCmd::execute() {
		if (!getProgram()) {
			std::string errMsg = "stopSim() command has no program context.";
			markFailed(errMsg);
			D_FAIL("%s", errMsg.c_str());
			return;
		}
		getProgram()->stopSim();
		markCompleted();
		D_SUCCESS("stopSim() command executed: Simulation stopped.");
	}

	// Factory function to create a StopCmd from arguments
	std::unique_ptr<ICommand> CreateStopCmd(const std::string& id, const std::vector<std::string>& args) {
		if (id.size() != 0 || !args.empty()) { D_FAIL("stopSim() does not take any arguments."); }
		return std::make_unique<StopCmd>();
	}
}