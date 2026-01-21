#include "pch.h"
#include "Interpreter/Commands/WaitCmd.h"

#include "EngineLib/LogMacros.h"

namespace commands {
	// --- WaitCmd Mark Methods ---
	void WaitCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void WaitCmd::markCompleted() { setResult({ CmdState::Executed, {}, "wait() ran successfully" }); }
	bool WaitCmd::hasStarted() const { return _started; }
	
	// --- WaitCmd Constructor ---
	WaitCmd::WaitCmd(double duration) : _remainingTime(duration), _started(false) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	program_data::CmdResult WaitCmd::update(CommandContextMotion& cntx, double dt) {
		if (!_started) { markFailed("wait() not started."); return CmdResult{ CmdState::Failed, {}, "wait() not started." }; }
		_remainingTime -= dt;
		if (_remainingTime <= 0.0) { markCompleted(); return CmdResult{ CmdState::Executed, {}, "" }; }
		return CmdResult{ CmdState::Executing, {}, "" };
	}

	void WaitCmd::execute() {
		_started = true;
		setResult({ CmdState::Executing, {}, "wait() started" });
	}

	std::unique_ptr<ICommand> CreateWaitCmd(const std::string& id, const std::vector<std::string>& args) {
		if (args.size() != 1) { D_FAIL("wait(<duration>) expects exactly 1 argument."); return nullptr; }
		double duration = utils::parseDouble(args[0]);
		return std::make_unique<WaitCmd>(duration);
	}
}