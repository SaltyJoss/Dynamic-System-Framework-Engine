// DSFE_Core RotateToCmd.cpp
#include "pch.h"

#include "Interpreter/Commands/RotateByCmd.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;
using namespace mathlib;

namespace commands {
	// --- RotateBy Mark Methods ---
	void RotateByCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void RotateByCmd::markCompleted() { setResult({ CmdState::Executed, {}, "rotateBy() ran successfully" }); }
	bool RotateByCmd::hasStarted() const { return _started; }

	// Constructor
	RotateByCmd::RotateByCmd(utils::AxisMask axes, double omegaDegPerSec, double deltaDeg)
		: _axes(axes), _omegaDeg(omegaDegPerSec), _deltaDeg(deltaDeg), _totalRotated(0.0), _started(false) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	// Update the command
	CmdResult RotateByCmd::update(CommandContext& cntx, double dt) {
		if (!_started) {
			markFailed("rotateBy() not started.");
			return CmdResult{ CmdState::Failed, {}, "rotateBy() not started." };
		}

		//// Resolve target object (must be called after start() to ensure default object is set)
		//scene::Object* obj = cntx.resolveDefaultObject();
		//if (!obj) {
		//	markFailed("rotateBy(<objID>,...) target but no current object selected.");
		//	SIM_FAIL("rotateBy(<objID>,...) target but no current object selected.");
		//	return CmdResult{ CmdState::Failed, {}, "No current object selected." };
		//}

		AxisMask mask = _axes;
		if (!mask.any()) { mask.z = true; }

		const double stepDeg = _omegaDeg * dt;

		//// If the next step would overshoot the target angle, clamp it to the remaining angle
		//auto result = cntx.rotateObject(obj, mask, _omegaDeg, dt);
		//if (!result.ok) {
		//	markFailed(result.message);
		//	SIM_FAIL("Failed to rotate object -> %s", result.message.c_str());
		//	return CmdResult{ CmdState::Failed, {}, result.message };
		//}

		// Update total rotated angle
		_totalRotated += stepDeg;
		//if (std::abs(_totalRotated) >= std::abs(_deltaDeg)) {
		//	if (obj) { cntx.stopRotation(obj, mask); }

		//	markCompleted();
		//	SIM_SUCCESS("Completed rotation of %.2f degrees.", _deltaDeg);
		//	return CmdResult{ CmdState::Executed, {}, "" };
		//}

		SIM_RUNTIME("Total rotated: %.2f / %.2f degrees.", _totalRotated, _deltaDeg);

		return CmdResult{ CmdState::Executing, {}, "" };
	}

	// Execute the command
	void RotateByCmd::execute() {
		_started = true;
		_totalRotated = 0.0;

		setResult({ CmdState::Executing, {}, "rotateBy() started" });
	}

	// Factory function to create RotateByCmd from command arguments
	std::unique_ptr<ICommand> CreateRotateByCmd(const std::string& id, const std::vector<std::string>& args) {
		// rotateBy(<objID>, <axes>, <omegaDeg>, <deltaDeg>)
		if (args.size() != 3) {
			SIM_FAIL("rotateBy command expects 3 args: <axes>, <omegaDeg>, <deltaDeg>, got %zu.", args.size());
			return nullptr;
		}

		//scene::ObjectID objID{};
		//if(!tryParseObjID(id, objID)) {
		//	SIM_FAIL("rotateBy command requires a valid object ID as the first argument.");
		//	return nullptr;
		//}

		AxisMask axes = utils::parseAxisMask(args[0]);
		if (!axes.any()) { axes.z = true; }

		auto omegaOpt = parseDouble(args[1]);
		auto deltaOpt = parseDouble(args[2]);
		if (!omegaOpt || !deltaOpt) {
			SIM_FAIL("rotateBy command requires numeric omega and delta.");
			return nullptr;
		}

		return std::make_unique<RotateByCmd>(axes, omegaOpt, deltaOpt);
	}
} // namespace commands