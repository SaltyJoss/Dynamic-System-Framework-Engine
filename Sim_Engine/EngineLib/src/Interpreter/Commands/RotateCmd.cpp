#include "pch.h"
#include "Interpreter/Commands/RotateCmd.h"
#include "Interpreter/Utils.h"

using namespace utils;
using namespace mathlib;

namespace commands {
	// Helper function to parse double from string_view
	static std::optional<double> parseDouble(const std::string_view s) {
		double out = 0.0;
		auto first = s.data();
		auto last = s.data() + s.size();

		auto res = std::from_chars(first, last, out); // Format: COMMAND <identifier>/<axis> <first>, ...<args_n>..., <last> "# Description"
		if (res.ec != std::errc{} || res.ptr != last) { return std::nullopt; }
		return out;
	}

	// Helper function to parse AxisMask from string
	static AxisMask parseAxisMask(const std::string_view axesStr) {
		AxisMask mask{};
		for (char c : axesStr) {
			switch (c) {
			case 'X': case 'x': mask.x = true; break;
			case 'Y': case 'y': mask.y = true; break;
			case 'Z': case 'z': mask.z = true; break;
			default:
				break;
			}
		}
		return mask;
	}

	// Helper function to parse RotateTarget from string
	// Formats: "AXIS:XYZ" or "Link:linkName"
	static std::optional<RotateTarget> parseRotateTarget(const std::string& arg) {
		if (startsWith(arg, "link")) { return RotateTarget{ RotateTargetType::linkName, {}, arg }; } // for this we ignore axis mask right now, assume boundary conditions
		if (startsWith(arg, "obj")) { return RotateTarget{ RotateTargetType::ObjID, {}, "" }; } // may consider adding something like "obj.x", "obj.y", "obj.z", or "obj.xyz" later ~ could be better than "{x,y,z}"
		if (!arg.empty() && arg.front() == '{' && arg.back() == '}') {
			// Formats: "{x,y,z}" or "{xyz}" or "{x y z}", etc
			std::string axesStr = arg.substr(1, arg.size() - 2); // remove braces
			AxisMask mask = parseAxisMask(axesStr);

			if (!mask.any()) {
				D_WARN("No valid axes found in rotate target: %s. Defaulting to Z axis.", arg.c_str());
				return std::nullopt;
			}
			return RotateTarget{ RotateTargetType::AxisMask, mask, "" };
		}
		// Try parsing as axis mask directly: "xyz", "xy", "x", etc
		else {
			AxisMask mask = parseAxisMask(arg);
			if (mask.any()) {
				D_DEBUG("Parsed rotate target axis mask: X:%d Y:%d Z:%d", mask.x ? 1 : 0, mask.y ? 1 : 0, mask.z ? 1 : 0);
				return RotateTarget{ RotateTargetType::AxisMask, mask, "" };
			}
		}

		return std::nullopt;
	}

	// --- RotateCmd Method Implementations ---

	void RotateCmd::markFailed(const std::string& message) {
		setResult({ CmdState::Failed, {}, message });
		// Implementation to mark the command as failed
	}

	void RotateCmd::markCompleted() {
		setResult({ CmdState::Executed, {}, "rotate() ran successfully" });
		// Implementation to mark the command as completed
	}

	bool RotateCmd::hasStarted() const {
		return _started;
	}

	// Constructor
	RotateCmd::RotateCmd(RotateTarget target, double omega, double startDeg, double endDeg)
		: _target(target), _omega(omega), _angleDeg(endDeg - startDeg), _started(false), _targetObjID(1) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	// Update the command
	CmdResult RotateCmd::update(CommandContextMotion& cntx, double dt) {
		if (!_started) {
			markFailed("rotate() not started.");
			return CmdResult{ CmdState::Failed, {}, "rotate() not started." };
		}

		double rotationThisStepDeg = _omega * dt;
		D_INFO("step: dt=%.4f omega=%.3f deg/s -> dtheta=%.4f deg", dt, _omega, rotationThisStepDeg);
		if (_target.type == RotateTargetType::AxisMask) {
			auto result = cntx.rotateAxes(_target.axisMask, _omega, dt);
			if (!result.ok) {
				markFailed(result.message);
				D_FAIL("Failed to rotate axes: %s", result.message.c_str());
				return CmdResult{ CmdState::Failed, {}, result.message };
			}

			D_DEBUG("Rotated axes (X:%d Y:%d Z:%d) by %.2f degrees this step.", 
				_target.axisMask.x ? 1 : 0, 
				_target.axisMask.y ? 1 : 0, 
				_target.axisMask.z ? 1 : 0, 
				rotationThisStepDeg);
		}
		else if (_target.type == RotateTargetType::ObjID) {
			auto* obj = cntx.getObject(_targetObjID);
			if (!obj) {
				markFailed("rotate(<objID>,...) target but no current object selected.");
				D_FAIL("rotate(<objID>,...) target but no current object selected.");
				return CmdResult{ CmdState::Failed, {}, "No current object selected." };
			}

			AxisMask mask = _target.axisMask;
			if (!mask.x && !mask.y && !mask.z) mask.z = true;
			auto result = cntx.rotateObject(obj, mask, _omega, dt);

			if (!result.ok) {
				markFailed(result.message);
				D_FAIL("Failed to rotate object: %s", result.message.c_str());
				return CmdResult{ CmdState::Failed, {}, result.message };
			}

			D_DEBUG("Rotated by % .2f degrees.", rotationThisStepDeg);
		} else if (_target.type == RotateTargetType::linkName) {
			auto result = cntx.rotationJointDelta(_target.linkName, rotationThisStepDeg, std::abs(_omega));
			if (!result.ok) {
				markFailed(result.message);
				D_FAIL("Failed to rotate joint %s: %s", _target.linkName.c_str(), result.message.c_str());
				return CmdResult{ CmdState::Failed, {}, result.message };
			}

			D_DEBUG("Rotated joint %s by %.2f degrees this step.", 
				_target.linkName.c_str(), 
				rotationThisStepDeg);
		}
		
		_totalRotated += rotationThisStepDeg;
		if (std::abs(_totalRotated) >= std::abs(_angleDeg)) {
			AxisMask mask = _target.axisMask;
			if (!mask.any()) mask.z = true;

			if (_target.type == RotateTargetType::ObjID) {
				cntx.stopRotation(cntx.getDefaultObject(), mask);
			}
			if (_target.type == RotateTargetType::AxisMask) {
				auto* obj = cntx.getDefaultObject();
				if (obj) cntx.stopRotation(obj, mask);
			}

			markCompleted();
			D_SUCCESS("Completed rotation of %.2f degrees.", _angleDeg);
			return CmdResult{ CmdState::Executed, {}, "" };
		}

		D_RUNTIME("Total rotated: %.2f / %.2f degrees.", _totalRotated, _angleDeg);

		return CmdResult{ CmdState::Executing, {}, "" };
	}

	void RotateCmd::execute() {
		_started = true;
		_result = { CmdState::Executing, {}, "rotate() started" };

		_cntxMtn->rotateAxes(_target.axisMask, _omega, 0.0); // Initial call with dt=0 to set up rotation

		D_INFO("RotateCmd execution started with omega: %.2f deg/s, angle: %.2f degrees.", _omega, _angleDeg);
	}

	// --- Free Function to Create RotateCmd ---

	std::unique_ptr<ICommand> CreateRotateCmd(const std::string& id, const std::vector<std::string>& args) {
		// args: [omega, startDeg, endDeg?]
		if (args.size() < 2) {
			D_FAIL("Rotate Command requires: <omega>,<startDeg>,<endDeg>");
			return nullptr;
		}

		auto targetOpt = parseRotateTarget(id);
		if (!targetOpt.has_value()) {
			D_FAIL("Invalid ROTATE target: %s (expected <objID> OR <{x,y,z}> OR <\"name\">)", id.c_str());
			return nullptr;
		}
		RotateTarget target = *targetOpt;

		auto omegaOpt = parseDouble(args[0]);
		if (!omegaOpt.has_value()) {
			D_FAIL("Invalid rotate() omega argument: %s", args[0].c_str());
			return nullptr;
		}

		auto startDegOpt = parseDouble(args[1]);
		if (!startDegOpt.has_value()) {
			D_FAIL("Invalid rotate() start angle argument: %s", args[1].c_str());
			return nullptr;
		}

		double omega = *omegaOpt;
		double startDeg = *startDegOpt;

		double endDeg = startDeg; // default: no rotation span unless endDeg provided
		if (args.size() >= 3) {
			auto endDegOpt = parseDouble(args[2]);
			if (!endDegOpt.has_value()) {
				D_FAIL("Invalid rotate() end angle argument: %s", args[2].c_str());
				return nullptr;
			}
			endDeg = *endDegOpt;
		}

		D_INFO("Creating RotateCmd targetType=%d omega=%.2f startDeg=%.2f endDeg=%.2f", static_cast<int>(target.type), omega, startDeg, endDeg);
		return std::make_unique<RotateCmd>(target, omega, startDeg, endDeg);
	}
} // namespace commands