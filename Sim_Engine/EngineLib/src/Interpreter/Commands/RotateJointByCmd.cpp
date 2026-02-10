#include "pch.h"
#include "Interpreter/Commands/RotateJointByCmd.h"
#include "Scene/SimulationManager.h"
#include "Robots/RobotSystem.h"
#include "Interpreter/Utils.h"

#include "EngineLib/LogMacros.h"

using namespace utils;
using namespace mathlib;

namespace commands {
	// --- RotateTo Mark Methods ---
	void RotateJointByCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void RotateJointByCmd::markCompleted() { setResult({ CmdState::Executed, {}, "rotateJointBy ran successfully" }); }
	bool RotateJointByCmd::hasStarted() const { return _started; }

	RotateJointByCmd::RotateJointByCmd(std::string linkName, double omegaDegPerSec, double deltaDeg) 
		: _link(std::move(linkName)), _omegaDeg(omegaDegPerSec), _deltaDeg(deltaDeg), _totalRotated(0.0), _started(false) {
		_result = { CmdState::NotStarted, {}, "" };
	}


	program_data::CmdResult RotateJointByCmd::update(CommandContextMotion& cntx, double dt) {
		auto* robot = cntx.Robot();
		// Defensive dt - my research shows I need to avoid giant dt spikes causing weird timing/logic.
		double maxDt = 1.0 / 60.0; // 1/60s, 60Hz, or 16.67ms
		if (dt < 0.0) dt = 0.0;
		if (dt > maxDt) dt = maxDt;
		
		if (!_started) {
			D_DEBUG("dt clamped to %.6f s", dt);
			_started = true;
			_elapsed = 0.0;
			_settleT = 0.0;
			_noProgressT = 0.0;
			_bestAbsErr = std::numeric_limits<double>::infinity();

			_deltaRad = degToRad(_deltaDeg);
			_maxOmegaRad = degToRad(std::abs(_omegaDeg)); // treat negative omega as magnitude

			const double minOmegaRad = degToRad(0.5); // 0.5 deg/s minimum meaningful speed
			if (_maxOmegaRad < minOmegaRad) {
				markFailed("rotateJointBy: maxOmega too small."); 
				D_FAIL("rotateJointBy(): maxOmega too small (%.3f deg/s) on '%s'", _omegaDeg, _link.c_str());
				return CmdResult{ CmdState::Failed, {}, "rotateJointBy: maxOmega too small." };
			}

			// Get starting angle
			double theta0 = 0.0f;
			if (!robot->tryGetJointAngleRad(_link, theta0)) {
				markFailed("rotateJointBy: joint not found (angle)."); 
				D_FAIL("rotateJointBy: joint not found (angle) for '%s'", _link.c_str());
				return CmdResult{ CmdState::Failed, {}, "rotateJointBy: joint not found (angle)." };
			}

			_thetaStartRad = (double)theta0;
			_targetRad = _thetaStartRad + _deltaRad;

			auto r1 = cntx.setJointMaxOmegaRad(_link, _maxOmegaRad);
			if (!r1.ok) { 
				markFailed(r1.message); 
				D_FAIL("Failed to set max omega for link '%s' -> %s", _link.c_str(), r1.message.c_str()); 
				return CmdResult{ CmdState::Failed, {}, r1.message }; 
			}

			auto r2 = cntx.setJointTargetDeltaRad(_link, _deltaRad);
			if (!r2.ok) { 
				markFailed(r2.message); 
				D_FAIL("Failed to set target delta for link '%s' -> %s", _link.c_str(), r2.message.c_str()); 
				return CmdResult{ CmdState::Failed, {}, r2.message }; 
			}

			const double delta = std::abs(_targetRad - _thetaStartRad);
			const double Tmin = delta / _maxOmegaRad;

			_timeoutSec = std::clamp(3.0 * Tmin + 5.0, 10.0, 60.0); // robust timeout estimate min = 10s, max = 60s

			SIM_ROTATE("rotateJointBy start: link='%s' delta=%.2f deg start=%.2f deg target=%.2f deg maxOmega=%.2f deg/s Tmin=%.2fs timeout=%.2fs",
				_link.c_str(), _deltaDeg, radToDeg(_thetaStartRad), radToDeg(_targetRad), _omegaDeg, Tmin, _timeoutSec); 

			return CmdResult{ CmdState::Executing, {}, "rotateJointTo() started" };
		}

		_elapsed += dt;

		const double tolPosRad = degToRad(0.25);	// 0.25 deg
		const double tolOmegaRad = degToRad(0.5);	// 0.20 deg/s
		const double settleSec = 0.15;				// must be stable for 100ms - i need to tune this more

		double theta = 0.0f;
		double omega = 0.0f;

		const bool gotTheta = robot->tryGetJointAngleRad(_link, theta);
		const bool gotOmega = robot->tryGetJointOmegaRad(_link, omega);

		if (!gotTheta) {
			markFailed("rotateJointBy: joint not found (angle).");
			D_FAIL("rotateJointBy: joint not found (angle) for '%s'", _link.c_str());
			return { CmdState::Failed, {}, "rotateJointBy: joint not found (angle)." };
		}

		if (!gotOmega) { omega = 0.0f; }

		const double errRad = _targetRad - (double)theta;
		const double absErr = std::abs(errRad);
		const double absOm = std::abs((double)omega);

		if (absErr + 1e-9 < _bestAbsErr) {
			_bestAbsErr = absErr;
			_noProgressT = 0.0;
		} else { 
			_noProgressT += dt; 
		}

		const bool posOK = robot->isJointAtTargetRad(_link, tolPosRad); // consider at target if within position tolerance
		const bool omegaOK = (absOm <= tolOmegaRad); // consider stopped if omega is small enough

		if (posOK && omegaOK) {
			_settleT += dt;
			if (_settleT >= settleSec) {
				markCompleted();
				SIM_SUCCESS("rotateJointBy done: '%s' err=%.6f rad (%.3f deg) omega=%.6f rad/s t=%.3fs", _link.c_str(), errRad, radToDeg(errRad), (double)omega, _elapsed);
				return { CmdState::Executed, {}, "rotateJointBy() completed successfully" };
			}
		} else { 
			_settleT = 0.0; 
		}

		if (_noProgressT >= 2.0) {
			markFailed("rotateJointBy: no progress made towards target.");
			D_FAIL("rotateJointBy stuck: '%s' err=%.6f rad omega=%.6f rad/s t=%.3fs bestErr=%.6f", 
				_link.c_str(), errRad, (double)omega, _elapsed, _bestAbsErr);
			return { CmdState::Failed, {}, "rotateJointBy: no progress made towards target." };
		}

		if (_elapsed >= _timeoutSec) {
			markFailed("rotateJointBy: timeout reached.");
			SIM_FAIL("rotateJointBy timeout: '%s' err=%.6f rad (%.3f deg) omega=%.6f rad/s t=%.3fs timeout=%.2fs",
				_link.c_str(), errRad, radToDeg(errRad), (double)omega, _elapsed, _timeoutSec);
			return { CmdState::Executed, {}, "rotateJointBy: timeout reached." };
		}

		return { CmdState::Executing, {}, "" };
	}

	void RotateJointByCmd::execute() {
		_totalRotated = 0.0;
		setResult({ CmdState::Executing, {}, "rotateJointBy started" });
	}

	std::unique_ptr<ICommand> CreateRotateJointByCmd(const std::string& id, const std::vector<std::string>& args) {
		// rotateJointBy(<linkName>, <omegaDeg>, <deltaDeg>)
		if (args.size() != 2) {
			SIM_FAIL("rotateJointBy expects 2 args: <omegaDeg>, <deltaDeg>, got %zu.", args.size());
			return nullptr;
		}

		const std::string linkName = id;

		auto omegaOpt = parseDouble(args[0]);
		auto deltaOpt = parseDouble(args[1]);
		if (!omegaOpt || !deltaOpt) {
			SIM_FAIL("rotateJointBy requires numeric omega and delta.");
			return nullptr;
		}

		return std::make_unique<RotateJointByCmd>(linkName, omegaOpt, deltaOpt);
	}
} // namespace commands