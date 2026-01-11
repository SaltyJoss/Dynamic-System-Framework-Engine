#include "pch.h"
#include "Interpreter/Commands/RotateCmd.h"

namespace commands {
	// Helper function to parse double from string_view
	static std::optional<double> parseDouble(const std::string_view s) {
		double out = 0.0;
		auto first = s.data();
		auto last = s.data() + s.size();

		auto res = std::from_chars(first, last, out);
		if (res.ec != std::errc{} || res.ptr != last) { return std::nullopt; }
		return out;
	}

	// Helper function to check if a string starts with a prefixomega_
	static bool startsWith(const std::string& str, const std::string& prefix) {
		return str.size() >= prefix.size() && str.substr(0, prefix.size()) == prefix;
	}

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
	// Expected formats: "AXIS:XYZ" or "JOINT:joint_name"
	static std::optional<RotateTarget> parseRotateTarget(const std::string& arg) {
		if (startsWith(arg, "AXIS:")) {
			std::string axesStr = arg.substr(5); // Extract substring after "AXIS:"
			AxisMask mask = parseAxisMask(axesStr);
			return RotateTarget{ RotateTargetType::AxisMask, mask, "" };
		}
		else if (startsWith(arg, "JOINT:")) {
			std::string linkName = arg.substr(6); // Extract substring after "JOINT:"
			return RotateTarget{ RotateTargetType::linkName, {}, linkName };
		}
		return std::nullopt;
	}

	// Constructor
	RotateCmd::RotateCmd(RotateTarget target, double omega, double startDeg, double endDeg)
		: _target(target), _omega(omega), _angleDeg(endDeg - startDeg), _started(false) {}

	// Command name
	std::string_view RotateCmd::name() const {
		return "ROTATE";
	}

	// Start the command
	void RotateCmd::start(CommandContextMotion& cntx) {
		_started = true;
	}

	// Update the command
	CmdResult RotateCmd::update(CommandContextMotion& cntx, double dt) {
		if (!_started) {
			markFailed("RotateCmd not started.");
			return CmdResult{ CmdState::Failed, {}, "RotateCmd not started." };
		}
		double rotationThisStep = _omega * dt;
		if (_target.type == RotateTargetType::AxisMask) {
			auto result = cntx.rotateAxes(_target.axisMask, _omega, dt);
			if (!result.ok) {
				markFailed(result.message);
				return CmdResult{ CmdState::Failed, {}, result.message };
			}
		}
		else if (_target.type == RotateTargetType::linkName) {
			auto result = cntx.rotateJoint(_target.linkName, rotationThisStep, std::abs(_omega));
			if (!result.ok) {
				markFailed(result.message);
				return CmdResult{ CmdState::Failed, {}, result.message };
			}
		}
		
		_totalRotated += rotationThisStep;
		if (std::abs(_totalRotated) >= std::abs(_angleDeg)) {
			markCompleted();
			return CmdResult{ CmdState::Completed, {}, "" };
		}

		return CmdResult{ CmdState::Running, {}, "" };
	}

	// Stop the command
	void RotateCmd::stop(CommandContextMotion& cntx) {
		_started = false;
	}

	std::unique_ptr<ICommand> CreateRotateCmd(const std::vector<std::string>& args) {
		if (args.size() < 3) {
			D_FAIL("ROTATE command requires at least 3 arguments.");
			return nullptr;
		}
		auto targetOpt = parseRotateTarget(args[0]);
		if (!targetOpt.has_value()) {
			D_FAIL("Invalid ROTATE target argument: %s", args[0].c_str());
			return nullptr;
		}
		RotateTarget target = targetOpt.value();
		auto omegaOpt = parseDouble(args[1]);
		if (!omegaOpt.has_value()) {
			D_FAIL("Invalid ROTATE omega argument: %s", args[1].c_str());
			return nullptr;
		}
		double omega = omegaOpt.value();
		auto startDegOpt = parseDouble(args[2]);
		if (!startDegOpt.has_value()) {
			D_FAIL("Invalid ROTATE start angle argument: %s", args[2].c_str());
			return nullptr;
		}
		double startDeg = startDegOpt.value();
		double endDeg = 0.0;
		if (args.size() >= 4) {
			auto endDegOpt = parseDouble(args[3]);
			if (!endDegOpt.has_value()) {
				D_FAIL("Invalid ROTATE end angle argument: %s", args[3].c_str());
				return nullptr;
			}
			endDeg = endDegOpt.value();
		}
		return std::make_unique<RotateCmd>(target, omega, startDeg, endDeg);
	}
} // namespace commands