#include "pch.h"
#include "Interpreter/Commands/ParallelGroupCmd.h"
#include "Robots/RobotSystem.h"

#include <EngineLib/LogMacros.h>

namespace commands {
	ParallelGroupCmd::ParallelGroupCmd( Policy policy, std::vector<std::unique_ptr<ICommand>> cmds, double timeout)
		: _policy(policy), _cmds(std::move(cmds)), _timeoutSec(timeout) {
	}

	CmdResult ParallelGroupCmd::update(CommandContextMotion& cntx, double dt) {
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

			CmdResult r = cmd->update(cntx, dt);

			switch (r.state) {
				case CmdState::Executed: executed++; break;
				case CmdState::Failed: failed++; break;
				case CmdState::Executing:
				default: running++; break;
			}
		}

		// Check for timeout
		if (_timeoutSec > 0.0 && _elapsed >= _timeoutSec) {
			D_WARN("parallel timed out after %.3fs (executed=%d failed=%d running=%d)", _elapsed, executed, failed, running);
			if (cntx.Robot()) { cntx.Robot()->stopAll(); }
			_result = { CmdState::Failed, {}, "ParallelGroupCmd: Timeout reached (soft-finish)" };
			return _result;
		}
		
		// Evaluate based on policy
		if (_policy == Policy::Any) { // Policy::Any
			if (failed > 0) { _result = { CmdState::Failed, {}, "ParallelGroupCmd: At least one command failed" }; return _result; } 
			else if (running == 0) { _result = { CmdState::Executed, {}, "" }; return _result; }
			_result = { CmdState::Executing, {}, "" };
			return _result;
		} else { // Policy::All
			if (executed > 0) { _result = { CmdState::Executed, {}, "" }; return _result; } 
			else if (running == 0 && failed == (int)_cmds.size()) { _result = { CmdState::Failed, {}, "ParallelGroupCmd: At least one command failed" }; return _result; }
			_result = { CmdState::Executing, {}, "" };
			return _result;
		}
	}

} // namespace commands