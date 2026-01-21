#include "pch.h"
#include "Interpreter/Commands/StartCmd.h"

#include "EngineLib/LogMacros.h"

namespace commands {

	// --- StartCmd Mark Methods ---
	void StartCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void StartCmd::markCompleted() { setResult({ CmdState::Executed, {}, "startSim() ran successfully" }); }
	bool StartCmd::hasStarted() const { return getResult().state != CmdState::NotStarted; }

	// --- StartCmd Constructor ---
	StartCmd::StartCmd() : _started(false) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	// --- StartCmd Method Implementations ---
	void StartCmd::execute() {
		if (!getProgram()) {
			std::string errMsg = "startSim() command has no program context.";
			markFailed(errMsg);
			D_FAIL("%s", errMsg.c_str());
			return;
		}
		getProgram()->startSim();
		markCompleted();
		D_SUCCESS("startSim() command executed: Simulation started.");
	}

	std::unique_ptr<ICommand> CreateStartCmd(const std::string& id, const std::vector<std::string>& args) {
		if (id.size() != 0 || !args.empty()) { D_FAIL("startSim() does not take any arguments."); }
		return std::make_unique<StartCmd>();
	}
}