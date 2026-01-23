#include "pch.h"
#include "Interpreter/Commands/SpinCmd.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;
using namespace mathlib;

namespace commands {
	// --- SpinCmd Mark Methods ---
	void SpinCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void SpinCmd::markCompleted() { setResult({ CmdState::Executed, {}, "spin() ran successfully" }); }
	bool SpinCmd::hasStarted() const { return _started; }

	// Constructor
	SpinCmd::SpinCmd(scene::ObjectID obj, utils::AxisMask axes, double omegaDeg, double duration)
		: _obj(obj), _axes(axes), _omegaDeg(omegaDeg), _remainingTime(duration), _started(false) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	program_data::CmdResult SpinCmd::update(CommandContextMotion& cntx, double dt) {
		if (!_started) {
			markFailed("spin() not started.");
			return CmdResult{ CmdState::Failed, {}, "spin() not started." };
		}

		scene::Object* obj = _cntxMtn->resolveDefaultObject();
		if (!obj) {
			markFailed("spin(<objID>,...) target but no current object selected.");
			D_FAIL("spin(<objID>,...) target but no current object selected.");
			return CmdResult{ CmdState::Failed, {}, "No current object selected." };
		}

		AxisMask mask = _axes;
		if (!mask.any()) { mask.z = true; }
		auto result = cntx.rotateObject(obj, mask, _omegaDeg, dt);
		if (!result.ok) {
			markFailed(result.message);
			D_FAIL("Failed to spin object -> %s", result.message.c_str());
			return CmdResult{ CmdState::Failed, {}, result.message };
		}

		_remainingTime -= dt;
		if (_remainingTime <= 0.0) {
			cntx.stopRotation(obj, mask);
			markCompleted();
			D_SUCCESS("Completed spin command.");
			return CmdResult{ CmdState::Executed, {}, "" };
		}

		D_RUNTIME("Remaining spin time: %.2f seconds.", _remainingTime);
		return CmdResult{ CmdState::Executing, {}, "" };
	}

	void SpinCmd::execute() {
		_started = true;
		setResult({ CmdState::Executing, {}, "spin() started" });
	}

	std::unique_ptr<ICommand> CreateSpinCmd(const std::string& id, const std::vector<std::string>& args) {
		// spin(<objID>, <axes>, <omegaDeg>, <duration>)
		if (args.size() != 3) {
			D_FAIL("spin command expects 3 args: <axes>, <omegaDeg>, <duration>, got %zu.", args.size());
			return nullptr;
		}

		scene::ObjectID objID{};
		if (!tryParseObjID(id, objID)) {
			D_FAIL("spin command requires a valid object ID as the first argument.");
			return nullptr;
		}

		AxisMask axes = utils::parseAxisMask(args[0]);
		if (!axes.any()) { axes.z = true; }

		auto omegaOpt = parseDouble(args[1]);
		auto durOpt = parseDouble(args[2]);
		if (!omegaOpt || !durOpt) {
			D_FAIL("spin command requires numeric values for omegaDeg and duration.");
			return nullptr;
		}

		return std::make_unique<SpinCmd>(objID, axes, omegaOpt, durOpt);
	}
} // namespace commands