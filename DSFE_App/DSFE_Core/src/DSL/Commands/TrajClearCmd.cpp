/*
 * File: DSL/TrajClearCmd.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "DSL/Commands/TrajClearCmd.h"
#include "Systems/TrajectoryManager.h"
#include "Systems/RigidBodySystem.h"
#include "Scene/SimulationCore.h"
#include "DSL/Utils.h"

#include "EngineLib/LogMacros.h"

namespace commands {
	// --- Markers ---
    void TrajClearCmd::markFailed(const std::string& message) {
        _result = { CmdState::Failed, {}, message };
        _done = true;
    }
    void TrajClearCmd::markCompleted() {
        _result = { CmdState::Executed, {}, "trajClear() completed" };
        _done = true;
    }
	bool TrajClearCmd::hasStarted() const { return _started; }

	// --- TrajClearCmd Implementation ---

	// Update method for TrajClearCmd
    CmdResult TrajClearCmd::update(CommandContext& cntx, double /*dt*/) {
        if (_done) return { CmdState::Executed, {}, "" };

        core::ISimulationCore* core = cntx.Core();
        if (!core) return { CmdState::Failed, {}, "trajClear: no sim in context." };

		auto& trajMgr = core->trajectoryManager();
		trajMgr.clearAll(); // clear all trajectories

        // Zero qd/qdd refs for all joints so nothing lingers
        auto& body = cntx.RigidBody();
        body.tryZeroJointRefDerivatives(); // make sure this exists as a NO-ARG method (see step 4)

        SIM_SUCCESS("trajClear(): cleared all trajectories and zeroed ref derivatives");
        _done = true;
        return { CmdState::Executed, {}, "" };
    }

	// Execute method for TrajClearCmd
    void TrajClearCmd::execute() {
        _done = false;
        _started = true;
        _result = { CmdState::Executing, {}, "trajClear() started" };
    }

	// Factory function to create TrajClearCmd
    std::unique_ptr<ICommand> CreateTrajClearCmd(const std::string& id, const std::vector<std::string>& args) {
        // trajClear() takes no args
        if (!args.empty()) {
            SIM_FAIL("trajClear: expects no arguments, got %zu.", args.size());
            return nullptr;
        }
		return std::make_unique<TrajClearCmd>();
    }

} // namespace commands