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
		auto* robot = cntx.Robot();
		// Defensive dt - my research shows I need to avoid giant dt spikes causing weird timing/logic.
        if (dt < 0.0) dt = 0.0;
        if (dt > 0.05) dt = 0.05; // 50ms

        // --- One-time setup ---
        if (!_started) {
            _started = true;
            _elapsed = 0.0;
            _settleT = 0.0;
            _noProgressT = 0.0;
            _bestAbsErr = std::numeric_limits<double>::infinity();

            // Convert inputs
            _targetRad = degToRad(_angleDeg);
            _maxOmegaRad = degToRad(std::abs(_maxOmegaDeg)); // treat negative omega as magnitude

            // Sanity clamp on max omega (avoid division by zero & "infinite timeout")
            const double minOmegaRad = degToRad(0.5); // 0.5 deg/s minimum meaningful speed
            if (_maxOmegaRad < minOmegaRad) {
                markFailed("rotateJointTo: maxOmega too small.");
                SIM_FAIL("rotateJointTo(): maxOmega too small (%.3f deg/s) on '%s'", _maxOmegaDeg, _link.c_str());
                return { CmdState::Failed, {}, "rotateJointTo: maxOmega too small." };
            }

            // Compute an informed timeout
            float theta0 = 0.0f;
            if (!robot->tryGetJointAngleRad(_link, theta0)) {
                markFailed("rotateJointTo: joint not found (angle).");
                SIM_FAIL("rotateJointTo: joint not found (angle) for '%s'", _link.c_str());
                return { CmdState::Failed, {}, "rotateJointTo: joint not found (angle)." };
            }

			// Set max omega
            auto r1 = cntx.setJointMaxOmegaRad(_link, _maxOmegaRad);
            if (!r1.ok) { markFailed(r1.message); SIM_FAIL("Failed to set max omega for '%s' -> %s", _link.c_str(), r1.message.c_str()); return { CmdState::Failed, {}, r1.message }; }

			// Set target
            auto r2 = cntx.setJointTargetRad(_link, _targetRad);
            if (!r2.ok) { markFailed(r2.message); SIM_FAIL("Failed to set target for '%s' -> %s", _link.c_str(), r2.message.c_str()); return { CmdState::Failed, {}, r2.message }; }

			// Estimate minimum time to reach target at max speed
            const double delta = std::abs(_targetRad - (double)theta0);
            const double Tmin = delta / _maxOmegaRad;

			// Set timeout based on Tmin
            _timeoutSec = std::clamp(3.0 * Tmin + 0.5, 2.0, 60.0);

            SIM_ROTATE("rotateJointTo start: link='%s' target=%.2f deg start=%.2f deg maxOmega=%.2f deg/s Tmin=%.2fs timeout=%.2fs",
                _link.c_str(), _angleDeg, radToDeg(theta0), _maxOmegaDeg, Tmin, _timeoutSec);

            return { CmdState::Executing, {}, "rotateJointTo started" };
        }

        // --- Progress / completion ---
        _elapsed += dt;

        const double tolPosRad = degToRad(0.25);    // 0.25 deg
        const double tolOmegaRad = degToRad(0.20);  // 0.20 deg/s
		const double settleSec = 0.10;              // must be stable for 100ms - i need to tune this more

        float theta = 0.0f;
        float omega = 0.0f;

        const bool gotTheta = robot->tryGetJointAngleRad(_link, theta);
        const bool gotOmega = robot->tryGetJointOmegaRad(_link, omega);

        if (!gotTheta) {
            markFailed("rotateJointTo: joint not found (angle).");
            SIM_FAIL("rotateJointTo: joint not found (angle) for '%s'", _link.c_str());
            return { CmdState::Failed, {}, "rotateJointTo: joint not found (angle)." };
        }
        if (!gotOmega) { omega = 0.0f; }

        const double errRad = _targetRad - (double)theta;
        const double absErr = std::abs(errRad);
        const double absOm = std::abs((double)omega);

        if (absErr + 1e-9 < _bestAbsErr) { _bestAbsErr = absErr; _noProgressT = 0.0; }
        else { _noProgressT += dt; }

        const bool posOK = absErr <= tolPosRad;
        const bool omegaOK = absOm <= tolOmegaRad;

        if (posOK && omegaOK) {
            _settleT += dt;
            if (_settleT >= settleSec) {
                markCompleted();
                SIM_SUCCESS("rotateJointTo done: '%s' err=%.6f rad (%.3f deg) omega=%.6f rad/s t=%.3fs", _link.c_str(), errRad, errRad * 180.0 / PI, (double)omega, _elapsed);
                return { CmdState::Executed, {}, "" };
            }
        }
        else { _settleT = 0.0; }

        // Stuck watchdog: error not improving for too long
        if (_noProgressT >= 3.0) {
            markFailed("rotateJointTo stuck (no progress).");
            SIM_FAIL("rotateJointTo stuck: '%s' err=%.6f rad omega=%.6f rad/s t=%.3fs bestErr=%.6f", _link.c_str(), errRad, (double)omega, _elapsed, _bestAbsErr);
            return { CmdState::Failed, {}, "rotateJointTo stuck (no progress)." };
        }

        // Timeout
        if (_elapsed >= _timeoutSec) {
            markFailed("rotateJointTo timed out.");
            SIM_FAIL("rotateJointTo timeout: '%s' err=%.6f rad (%.3f deg) omega=%.6f rad/s t=%.3fs timeout=%.2fs", _link.c_str(), errRad, errRad * 180.0 / PI, (double)omega, _elapsed, _timeoutSec);
            return { CmdState::Executed, {}, "rotateJointTo timed out." };
        }

        return { CmdState::Executing, {}, "" };
    }

	void RotateJointToCmd::execute() { setResult({ CmdState::Executing, {}, "rotateJointTo() started" }); }

	std::unique_ptr<ICommand> CreateRotateJointToCmd(const std::string& id, const std::vector<std::string>& args) {
		// rotateJointTo(<linkName>, <omegaDeg>, <angleDeg>)
		if (args.size() != 2) {
			SIM_FAIL("rotateJointBy expects 2 args: <omegaDeg>, <angleDeg>, got %zu.", args.size());
			return nullptr;
		}

		const std::string& linkName = id;

		auto omegaOpt = parseDouble(args[0]);
		auto angleOpt = parseDouble(args[1]);
		if (!omegaOpt || !angleOpt) {
			SIM_FAIL("rotateJointTo command requires numeric omega and angle.");
			return nullptr;
		}

		return std::make_unique<RotateJointToCmd>(linkName, omegaOpt, angleOpt);
	}
} // namespace commands