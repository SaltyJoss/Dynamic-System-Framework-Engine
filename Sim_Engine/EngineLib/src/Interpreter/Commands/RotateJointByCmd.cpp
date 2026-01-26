#include "pch.h"
#include "Interpreter/Commands/RotateJointByCmd.h"
#include "Scene/SimulationManager.h"
#include "Robots/RobotSystem.h"
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
			_started = true;

			auto r1 = cntx.setJointMaxOmegaRad(_link, degToRad(_omegaDeg));
			if (!r1.ok) { markFailed(r1.message); SIM_FAIL("Failed to set max omega for link '%s' -> %s", _link.c_str(), r1.message.c_str()); 
			return CmdResult{ CmdState::Failed, {}, r1.message }; }

			auto r2 = cntx.setJointTargetDeltaRad(_link, degToRad(_deltaDeg));
			if (!r2.ok) { markFailed(r2.message); SIM_FAIL("Failed to set target delta for link '%s' -> %s", _link.c_str(), r2.message.c_str()); return CmdResult{ CmdState::Failed, {}, r2.message }; }

			SIM_RUNTIME("Starting rotateJointTo() on link '%s' to delta %.2f deg at max omega %.2f deg/s", _link.c_str(), _deltaDeg, _omegaDeg);
			return CmdResult{ CmdState::Executing, {}, "rotateJointTo() started" };
		}

		auto* robot = cntx.Robot();
		if (!robot) {
			markFailed("No robot loaded."); SIM_FAIL("rotateJointTo() failed: no robot loaded.");
			return CmdResult{ CmdState::Failed, {}, "No robot loaded." };
		}

		if (cntx.Robot()->isJointAtTargetDeg(_link, 0.25f)) {
			markCompleted(); SIM_SUCCESS("Completed rotateJointBy() on link '%s' by delta %.2f deg", _link.c_str(), _deltaDeg);
			return CmdResult{ CmdState::Executed, {}, "" };
		}

		return { CmdState::Executing, {}, "" };
	}

	void RotateJointByCmd::execute() {
		_totalRotated = 0.0;
		setResult({ CmdState::Executing, {}, "rotateJointBy() started" });
	}

	std::unique_ptr<ICommand> CreateRotateJointByCmd(const std::string& id, const std::vector<std::string>& args) {
		// rotateJointBy(<linkName>, <omegaDeg>, <deltaDeg>)
		if (args.size() != 2) {
			SIM_FAIL("rotateJointBy expects 2 args: <omegaDeg>, <deltaDeg>, got %zu.", args.size());
			return nullptr;
		}

		const std::string linkName = id;

		auto omegaOpt = parseDouble(args[0]);
		auto deltaOpt = parseDouble(args[1]);
		if (!omegaOpt || !deltaOpt) {
			SIM_FAIL("rotateJointBy requires numeric omega and delta.");
			return nullptr;
		}

		return std::make_unique<RotateJointByCmd>(linkName, omegaOpt, deltaOpt);
	}
} // namespace commands