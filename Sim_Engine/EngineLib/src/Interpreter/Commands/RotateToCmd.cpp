#include "pch.h"
// File:   RotateToCmd.cpp
// GitHub: SaltyJoss
#include "Interpreter/Commands/RotateToCmd.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;
using namespace mathlib;

namespace commands {
	// --- RotateTo Mark Methods ---
	void RotateToCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void RotateToCmd::markCompleted() { setResult({ CmdState::Executed, {}, "rotateTo() ran successfully" }); }
	bool RotateToCmd::hasStarted() const { return _started; }

	RotateToCmd::RotateToCmd(scene::ObjectID objID, utils::AxisMask axes, double maxOmegaDegPerSec, double angleDeg)
		: _objID(objID), _axes(axes), _maxOmegaDeg(maxOmegaDegPerSec), _angleDeg(angleDeg), _started(false) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	// Update command state
	CmdResult RotateToCmd::update(CommandContextMotion& cntx, double dt) {
		if (!_started) {
			markFailed("rotateTo() not started.");
			return CmdResult{ CmdState::Failed, {}, "rotateTo() not started." };
		}

		auto result = cntx.updateRigidRotateTo(dt);
		if (!result.ok) {
			markFailed(result.message);
			SIM_FAIL("Failed to update rotateTo -> %s", result.message.c_str());
			return CmdResult{ CmdState::Failed, {}, result.message };
		}

		scene::Object* obj = cntx.resolveObject(_objID);
		if (!obj) return { CmdState::Failed, {}, "rotateTo: object disappeared." };

		if (result.done) {
			cntx.stopRotation(obj, _axes.any() ? _axes : utils::AxisMask{ false,false,true });
			markCompleted();
			return { CmdState::Executed, {}, "" };
		}

		return CmdResult{ CmdState::Executing, {}, "" };
	}

	// Execute the command
	void RotateToCmd::execute() {
		_started = true;
		setResult({ CmdState::Executing, {}, "rotateBy() started" });

		mathlib::Vec3 axis(0.0, 0.0, 0.0);
		if (_axes.x) axis.x() = 1.0;
		else if (_axes.y) axis.y() = 1.0;
		else axis.z() = 1.0;

		scene::Object* obj = _cntxMtn->resolveObject(_objID);
		if (!obj) { markFailed("rotateTo: invalid object."); return; }

		auto result = _cntxMtn->beginRigidRotateTo(obj, axis, _maxOmegaDeg, _angleDeg);
		if (!result.ok) {
			markFailed(result.message);
			SIM_FAIL("Failed to start rotateTo -> %s", result.message.c_str());
			return;
		}
	}

	// Factory function to create RotateToCmd from command arguments
	std::unique_ptr<ICommand> CreateRotateToCmd(const std::string& id, const std::vector<std::string>& args) {
		// rotateTo(<objID>, <axes>, <omegaDeg>, <angleDeg>)
		if (args.size() != 3) {
			SIM_FAIL("rotateTo command expects 3 args: <axes>, <omegaDeg>, <angleDeg>, got %zu.", args.size());
			return nullptr;
		}

		scene::ObjectID objID{};
		if (!tryParseObjID(id, objID)) {
			SIM_FAIL("rotateTo command requires a valid object ID as the first argument.");
			return nullptr;
		}

		AxisMask axes = utils::parseAxisMask(args[0]);
		if (!axes.any()) { axes.z = true; }

		auto omegaOpt = parseDouble(args[1]);
		auto angleOpt = parseDouble(args[2]);
		if (!omegaOpt || !angleOpt) {
			SIM_FAIL("rotateTo command requires numeric omega and angle.");
			return nullptr;
		}

		return std::make_unique<RotateToCmd>(objID, axes, omegaOpt, angleOpt);
	}
} // namespace commands