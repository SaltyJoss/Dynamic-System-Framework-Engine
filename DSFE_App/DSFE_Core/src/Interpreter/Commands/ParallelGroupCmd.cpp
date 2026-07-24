// DSFE_Core ParallelGroupCmd.cpp
#include "pch.h"

#include "Interpreter/Commands/ParallelGroupCmd.h"
#include "Robots/RobotSystem.h"

#include <EngineLib/LogMacros.h>

namespace commands {
	// Constructor
	ParallelGroupCmd::ParallelGroupCmd( Policy policy, std::vector<std::unique_ptr<ICommand>> cmds, double timeout)
		: _policy(policy), _cmds(std::move(cmds)), _timeoutSec(timeout) {
	}

	// Update method
	CmdResult ParallelGroupCmd::update(CommandContext& cntx, double dt) {
		if (_cmds.empty()) { _result = { CmdState::Executed, {}, "parallel: empty group" }; return _result; }
		// Start the group if not already started
		if (!_started) {
			_started = true;
			_elapsed = 0.0;
			_result = { CmdState::Executing, {}, "" };
		}

		_elapsed += dt;

		int executed = 0;
		int failed = 0;
		int running = 0;

		for (auto& cmd : _cmds) {
			if (!cmd) { continue; }
			if (!cmd->hasStarted()) { cmd->execute(); }
			CmdResult r = cmd->update(cntx, dt);

			if (r.state == CmdState::Executed) { executed++; }
			else if (r.state == CmdState::Failed) { failed++; }
			else { running++; }
		}

		// Timeout handling 
		if (_timeoutSec > 0.0 && _elapsed >= _timeoutSec) {
			D_WARN("parallel timed out after %.3fs", _elapsed);

			// “soft finish”
			cntx.Robot().stopAll();

			// treats timeout as Executed - ill keep for now, may explore different timeout policies later
			_result = { CmdState::Executed, {}, "parallel: timeout (soft-finish)" };
			return _result;
		}

		if (_policy == Policy::All) { // Policy::All
			if (failed > 0) { _result = { CmdState::Failed, {}, "parallel: child failed" }; return _result; }
			if (executed == (int)_cmds.size()) { _result = { CmdState::Executed, {}, "" }; return _result; }
			_result = { CmdState::Executing, {}, "" };
			return _result;
		}
		else { // Policy::Any
			if (executed > 0) { _result = { CmdState::Executed, {}, "" }; return _result; }
			if (failed == (int)_cmds.size()) { _result = { CmdState::Failed, {}, "parallel: all children failed" }; return _result; }
			_result = { CmdState::Executing, {}, "" }; 
			return _result;
		}
	}

	// Execute method
	void ParallelGroupCmd::execute() {
		_started = false;
		_elapsed = 0.0;
		_result = { CmdState::Executing, {}, "" };
	}

} // namespace commands