/*
 * File: DSL/Commands/SetVelocityCmd.cpp
 * Created by: Joss Salton, 26-07-2026 
 */
#include "pch.h"

#include "DSL/Commands/SetVelocityCmd.h"
#include "Scene/SimulationCore.h"
#include "Systems/RigidBodySystem.h"

#include "DSL/Utils.h"
#include "EngineLib/LogMacros.h"

namespace commands {
    	// --- Markers ---
	void SetVelocityCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void SetVelocityCmd::markCompleted() { setResult({ CmdState::Executed, {}, "setVelocity ran successfully" }); }
	bool SetVelocityCmd::hasStarted() const { return _started; }

    // Constructor
    SetVelocityCmd::SetVelocityCmd(std::string link, double wx, double wy, double wz, double vx, double vy, double vz)
        : _link(std::move(link)), _wx(wx), _wy(wy), _wz(wz), _vx(vx), _vy(vy), _vz(vz) {
        _result = { CmdState::NotStarted, {}, "" };
    }

    CmdResult SetVelocityCmd::update(CommandContext& cntx, double dt) {
        auto* core = cntx.Core();
        if (!core) {
            SIM_FAIL("setVelocity: SimulationCore is null.");
            markFailed("setVelocity: SimulationCore is null.");
            return { CmdState::Failed, {}, "setVelocity failed" };
        }

        auto& body = cntx.RigidBody();
        mathlib::VecX v = mathlib::VecX::Zero(6);
        v << _wx, _wy, _wz, _vx, _vy, _vz; // [Angular Velocity (rad/s), Linear Velocity (m/s)]

        if (!body.trySetFreeVelocity(_link, v)) {
            SIM_FAIL("setVelocity: '%s' is not a free body.", _link.c_str());
			markFailed("setVelocity: target is not a free body.");
			return { CmdState::Failed, {}, "setVelocity failed" };
        }
        markCompleted();
        return { CmdState::Executed, {}, "setVelocity executed" };
    }

    // Execute command
    void SetVelocityCmd::execute() {
        setResult({ CmdState::Executing, {}, "setVelocity started" });
    }

    // --- Factory ---
    std::unique_ptr<ICommand> CreateSetVelocityCmd(const std::string& id, const std::vector<std::string>& args) {
        if (id.empty()) {
            D_FAIL("setVelocity: missing identifier (link).");
            return nullptr;
        }
        if (args.size() != 6) {
            D_FAIL("setVelocity expects 6 args: wx, wy, wz, vx, vy, vz.");
            return nullptr;
        }
        const std::string link = id;
        double wx = utils::parseDouble(args[0]);
        double wy = utils::parseDouble(args[1]);
        double wz = utils::parseDouble(args[2]);
        double vx = utils::parseDouble(args[3]);
        double vy = utils::parseDouble(args[4]);
        double vz = utils::parseDouble(args[5]);

        return std::make_unique<SetVelocityCmd>(link, wx, wy, wz, vx, vy, vz);
    }
} // namespace commands