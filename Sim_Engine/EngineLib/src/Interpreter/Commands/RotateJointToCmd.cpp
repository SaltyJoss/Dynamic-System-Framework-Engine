#include "pch.h"
#include "Interpreter/Commands/RotateJointToCmd.h"
#include "Scene/SimulationManager.h"
#include "Robots/RobotSystem.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;
using namespace constants;
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

			auto r1 = cntx.setJointMaxOmegaRad(_link, _maxOmegaDeg * (constants::PI / 180.0));
			if (!r1.ok) { markFailed(r1.message); D_FAIL("Failed to set max omega for link '%s' -> %s", _link.c_str(), r1.message.c_str()); return CmdResult{ CmdState::Failed, {}, r1.message }; }

			auto r2 = cntx.setJointTargetRad(_link, _angleDeg * (constants::PI / 180.0));
			if (!r2.ok) { markFailed(r2.message); D_FAIL("Failed to set target angle for link '%s' -> %s", _link.c_str(), r2.message.c_str()); return CmdResult{ CmdState::Failed, {}, r2.message }; }

			D_INFO("Starting rotateJointTo() on link '%s' to angle %.2f deg at max omega %.2f deg/s", _link.c_str(), _angleDeg, _maxOmegaDeg);
			return CmdResult{ CmdState::Executing, {}, "rotateJointTo() started" };
		}

		auto* robot = cntx.Robot();
		if (!robot) {
			markFailed("No robot loaded."); D_FAIL("rotateJointTo() failed: no robot loaded.");
			return CmdResult{ CmdState::Failed, {}, "No robot loaded." };
		}

		constexpr float tolDeg = 0.25f;	
		if (robot->isJointAtTargetDeg(_link, tolDeg)) {
			markCompleted(); D_SUCCESS("Completed rotateJointTo() on link '%s' to angle %.2f deg", _link.c_str(), _angleDeg);
			return CmdResult{ CmdState::Executed, {}, "" };
		}

		return CmdResult{ CmdState::Executing, {}, "" };
	}

	void RotateJointToCmd::execute() {
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