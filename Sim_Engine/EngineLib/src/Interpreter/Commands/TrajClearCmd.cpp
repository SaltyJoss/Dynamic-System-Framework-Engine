#include "pch.h"
// File:   TrajClearCmd.cpp
// GitHub: SaltyJoss
#include "Interpreter/Commands/TrajClearCmd.h"
#include "Robots/TrajectoryManager.h"
#include "Robots/RobotSystem.h"
#include "Scene/SimulationManager.h"

#include "EngineLib/LogMacros.h"
#include "Interpreter/Utils.h"

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
    program_data::CmdResult TrajClearCmd::update(CommandContextMotion& cntx, double /*dt*/) {
        if (_done) return { CmdState::Executed, {}, "" };

        gui::SimManager* sim = cntx.Sim();
        if (!sim) return { CmdState::Failed, {}, "trajClear: no sim in context." };

        // Clear all trajectories
		sim->traj().clearAll();

        // Zero qd/qdd refs for all joints so nothing lingers
        auto* robot = cntx.Robot();
        if (robot) {
            robot->tryZeroJointRefDerivatives(); // make sure this exists as a NO-ARG method (see step 4)
        }

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