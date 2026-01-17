#include "pch.h"
#include "Interpreter/Commands/RotateJointToCmd.h"
#include "Interpreter/Utils.h"

using namespace utils;
using namespace mathlib;

namespace commands {
	// --- RotateTo Mark Methods ---
	void RotateJointToCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void RotateJointToCmd::markCompleted() { setResult({ CmdState::Executed, {}, "rotateJointTo() ran successfully" }); }
	bool RotateJointToCmd::hasStarted() const { return _started; }

	RotateJointToCmd::RotateJointToCmd(const std::string& linkName, double maxOmegaDegPerSec, double angleDeg)
		: _link(std::move(linkName)), _maxOmegaDeg(maxOmegaDegPerSec), _angleDeg(angleDeg), _started(false) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	program_data::CmdResult RotateJointToCmd::update(CommandContextMotion& cntx, double dt) {
		if (!_started) {
			markFailed("rotateJointTo() not started.");
			return CmdResult{ CmdState::Failed, {}, "rotateJointTo() not started." };
		}
		auto result = cntx.updateJointRotateTo(dt);
		if (!result.ok) {
			markFailed(result.message);
			D_FAIL("Failed to update rotateJointTo -> %s", result.message.c_str());
			return CmdResult{ CmdState::Failed, {}, result.message };
		}

		markCompleted();
		return CmdResult{ CmdState::Executing, {}, "" };
	}

	void RotateJointToCmd::execute() {
		_started = true;
		setResult({ CmdState::Executing, {}, "rotateJointTo() started" });
		auto result = _cntxMtn->beginJointRotateTo(_link, _angleDeg, _maxOmegaDeg);
		if (!result.ok) {
			markFailed(result.message);
			D_FAIL("Failed to start rotateJointTo -> %s", result.message.c_str());
			return;
		}
	}

	std::unique_ptr<ICommand> CreateRotateJointToCmd(const std::string& id, const std::vector<std::string>& args) {
		// rotateJointTo(<linkName>, <omegaDeg>, <angleDeg>)
		if (args.size() != 3) {
			D_FAIL("rotateJointBy expects 2 args: <omegaDeg>, <angleDeg>, got %zu.", args.size());
			return nullptr;
		}

		const std::string& linkName = id;

		auto omegaOpt = parseDouble(args[1]);
		auto angleOpt = parseDouble(args[2]);
		if (!omegaOpt || !angleOpt) {
			D_FAIL("rotateJointTo command requires numeric omega and angle.");
			return nullptr;
		}

		return std::make_unique<RotateJointToCmd>(linkName, *omegaOpt, *angleOpt);
	}
} // namespace commands