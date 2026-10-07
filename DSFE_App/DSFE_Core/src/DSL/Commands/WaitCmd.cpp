/*
 * File: DSL/WaitCmd.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "DSL/Commands/WaitCmd.h"

#include "EngineLib/LogMacros.h"

namespace commands {
	// --- WaitCmd Mark Methods ---
	void WaitCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void WaitCmd::markCompleted() { setResult({ CmdState::Executed, {}, "wait() ran successfully" }); }
	bool WaitCmd::hasStarted() const { return _started; }
	
	// Constructor
	WaitCmd::WaitCmd(double duration) : _remainingTime(duration), _started(false) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	// Update the command
	CmdResult WaitCmd::update(CommandContext& cntx, double dt) {
		if (!_started) { markFailed("wait() not started."); return CmdResult{ CmdState::Failed, {}, "wait() not started." }; }
		_remainingTime -= dt;
		if (_remainingTime <= 0.0) { markCompleted(); return CmdResult{ CmdState::Executed, {}, "" }; }
		return CmdResult{ CmdState::Executing, {}, "" };
	}

	// Execute the command
	void WaitCmd::execute() {
		_started = true;
		setResult({ CmdState::Executing, {}, "wait() started" });
	}

	// Factory function to create a WaitCmd from arguments
	std::unique_ptr<ICommand> CreateWaitCmd(const std::string& id, const std::vector<std::string>& args) {
		if (args.size() != 1) { D_FAIL("wait(<duration>) expects exactly 1 argument."); return nullptr; }
		double duration = utils::parseDouble(args[0]);
		return std::make_unique<WaitCmd>(duration);
	}
}