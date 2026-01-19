#include "pch.h"
#include "Interpreter/Commands/RotateJointByCmd.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;
using namespace mathlib;

namespace commands {
	// --- RotateTo Mark Methods ---
	void RotateJointByCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void RotateJointByCmd::markCompleted() { setResult({ CmdState::Executed, {}, "rotateJointBy() ran successfully" }); }
	bool RotateJointByCmd::hasStarted() const { return _started; }

	RotateJointByCmd::RotateJointByCmd(std::string linkName, double omegaDegPerSec, double deltaDeg) 
		: _link(std::move(linkName)), _omegaDeg(omegaDegPerSec), _deltaDeg(deltaDeg), _totalRotated(0.0), _started(false) {
		_result = { CmdState::NotStarted, {}, "" };
	}


	program_data::CmdResult RotateJointByCmd::update(CommandContextMotion& cntx, double dt) {
		if (!_started) {
			markFailed("rotateJointBy() not started.");
			return CmdResult{ CmdState::Failed, {}, "rotateJointBy() not started." };
		}

		const double stepDeg = _omegaDeg * dt;

		auto result = cntx.rotationJointDelta(_link, stepDeg, std::abs(_omegaDeg));
		if (!result.ok) {
			markFailed(result.message);
			D_FAIL("Failed to rotate link %s -> %s", _link.c_str(), result.message.c_str());
			return CmdResult{ CmdState::Failed, {}, result.message };
		}

		_totalRotated += stepDeg;
		if (std::abs(_totalRotated) >= std::abs(_deltaDeg)) {
			markCompleted();
			D_SUCCESS("Completed rotation of %.2f degrees.", _deltaDeg);
			return CmdResult{ CmdState::Executed, {}, "" };
		}

		return { CmdState::Executing, {}, "" };
	}

	void RotateJointByCmd::execute() {
		_started = true;
		_totalRotated = 0.0;
		setResult({ CmdState::Executing, {}, "rotateJointBy() started" });
	}

	std::unique_ptr<ICommand> CreateRotateJointByCmd(const std::string& id, const std::vector<std::string>& args) {
		// rotateJointBy(<linkName>, <omegaDeg>, <deltaDeg>)
		if (args.size() != 2) {
			D_FAIL("rotateJointBy expects 2 args: <omegaDeg>, <deltaDeg>, got %zu.", args.size());
			return nullptr;
		}

		const std::string linkName = id;

		auto omegaOpt = parseDouble(args[0]);
		auto deltaOpt = parseDouble(args[1]);
		if (!omegaOpt || !deltaOpt) {
			D_FAIL("rotateJointBy requires numeric omega and delta.");
			return nullptr;
		}

		return std::make_unique<RotateJointByCmd>(linkName, *omegaOpt, *deltaOpt);
	}
} // namespace commands