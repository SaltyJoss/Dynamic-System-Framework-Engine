#include "pch.h"
#include "Interpreter/Commands/RotateJointToCmd.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;
using namespace mathlib;

namespace commands {
	// --- RotateTo Mark Methods ---
	void RotateJointToCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void RotateJointToCmd::markCompleted() { setResult({ CmdState::Executed, {}, "rotateJointTo() ran successfully" }); }
	bool RotateJointToCmd::hasStarted() const { return _started; }

	RotateJointToCmd::RotateJointToCmd(std::string linkName, double maxOmegaDegPerSec, double angleDeg)
		: _link(std::move(linkName)), _maxOmegaDeg(maxOmegaDegPerSec), _angleDeg(angleDeg), _started(false) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	program_data::CmdResult RotateJointToCmd::update(CommandContextMotion& cntx, double dt) {
		if (!_started){
			_started = true;

			auto start = cntx.beginJointRotateTo(_link, _maxOmegaDeg, _angleDeg);
			if (!start.ok) {
				markFailed(start.message);
				return { CmdState::Failed, {}, start.message };
			}

			D_INFO("Starting rotateJointTo() on link '%s' to angle %.2f deg at max omega %.2f deg/s", _link.c_str(), _angleDeg, _maxOmegaDeg);
		}

		auto result = cntx.updateJointRotateTo(dt);
		if (!result.ok) {
			markFailed(result.message);
			D_FAIL("Failed to update rotateJointTo -> %s", result.message.c_str());
			return CmdResult{ CmdState::Failed, {}, result.message };
		}

		if (result.done) {
			markCompleted();
			return CmdResult{ CmdState::Executed, {}, "rotateJointTo() completed successfully" };
		}

		return CmdResult{ CmdState::Executing, {}, "" };
	}

	void RotateJointToCmd::execute() {
		_started = true;
		setResult({ CmdState::Executing, {}, "rotateJointTo() started" });
	}

	std::unique_ptr<ICommand> CreateRotateJointToCmd(const std::string& id, const std::vector<std::string>& args) {
		// rotateJointTo(<linkName>, <omegaDeg>, <angleDeg>)
		if (args.size() != 2) {
			D_FAIL("rotateJointBy expects 2 args: <omegaDeg>, <angleDeg>, got %zu.", args.size());
			return nullptr;
		}

		const std::string& linkName = id;

		auto omegaOpt = parseDouble(args[0]);
		auto angleOpt = parseDouble(args[1]);
		if (!omegaOpt || !angleOpt) {
			D_FAIL("rotateJointTo command requires numeric omega and angle.");
			return nullptr;
		}

		return std::make_unique<RotateJointToCmd>(linkName, omegaOpt, angleOpt);
	}
} // namespace commands