#include "pch.h"
// File:   SetOmegaCmd.cpp
// GitHub: SaltyJoss
#include "Interpreter/Commands/SetOmegaCmd.h"
#include "Robots/RobotSystem.h"

#include "Interpreter/Utils.h"
#include "EngineLib/LogMacros.h"

using namespace utils;

// Syntax is setOmega(<linkName>, <omegaValue>)

namespace commands {
	// --- Markers ---
	void SetOmegaCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void SetOmegaCmd::markCompleted() { setResult({ CmdState::Executed, {}, "trajSet ran successfully" }); }
	bool SetOmegaCmd::hasStarted() const { return _started; }

	// --- SetOmegaCmd Implementation ---

	// Constructor
	SetOmegaCmd::SetOmegaCmd(std::string link, const double omega)
		: _link(std::move(link)), _omega(omega) {
		_result = { CmdState::NotStarted, {}, "" };
	}

	// Updates setOmega command
	program_data::CmdResult SetOmegaCmd::update(CommandContextMotion& cntx, double dt) {
		auto* robot = cntx.Robot();
		if (!robot) {
			markFailed("setOmega: no RobotSystem in simulation.");
			D_ERROR("setOmega: no RobotSystem in simulation.");
			return { CmdState::Failed, {}, "setOmega failed: no RobotSystem in simulation." };
		}
		if (!robot->hasLinkName(_link)) {
			markFailed("setOmega: link '" + _link + "' not found in robot.");
			return { CmdState::Failed, {}, "setOmega failed: link '" + _link + "' not found in robot." };
		}

		double omegaRad = degToRad(_omega);

		if (!robot->injectJointOmegaRad(_link, omegaRad)) {
			markFailed("setOmega: failed to set joint omega for link='" + _link + "'.");
			return { CmdState::Failed, {}, "setOmega failed to set joint omega for link='" + _link + "'." };
		}
		markCompleted();
		D_SUCCESS("setOmega command executed: Link '%s' omega set to %.6f rad/s.", _link.c_str(), omegaRad);
		LOG_INFO("setOmega command executed: Link '%s' omega set to %.6f rad/s.", _link.c_str(), omegaRad);
		return { CmdState::Executed, {}, "setOmega executed successfully." };
	}

	// Execute command
	void SetOmegaCmd::execute() {
		setResult({ CmdState::Executing, {}, "setOmega started" });
	}

	// --- Factory ---

	// Factory function to create SetOmegaCmd from arguments
	std::unique_ptr<ICommand> CreateSetOmegaCmd(const std::string& id, const std::vector<std::string>& args) {
		if (id.empty()) {
			D_FAIL("setOmega: missing identifier (link name).");
			return nullptr;
		}
		if (args.size() != 1) {
			D_FAIL("setOmega expects exactly 1 argument: <omegaValue>.");
			return nullptr;
		}
		const std::string link = id;
		if (!utils::isDouble(args[0])) {
			D_FAIL("setOmega: omega value '%s' is not a valid double.", args[0].c_str());
			return nullptr;
		}
		const double omega = utils::parseDouble(args[0]);
		return std::make_unique<SetOmegaCmd>(link, omega);
	}
}