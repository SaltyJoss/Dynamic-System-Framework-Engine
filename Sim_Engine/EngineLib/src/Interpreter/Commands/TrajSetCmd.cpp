#include "pch.h"

#include "Interpreter/Commands/TrajSetCmd.h"
#include "Interpreter/CommandContextMotion.h"
#include "Scene/SimulationManager.h"
#include "Robots/RobotSystem.h"
#include "Robots/TrajectoryManager.h"

#include "Control/TrapezoidTrajectory.h"
#include "Control/SinusoidalTrajectory.h"
#include "Control/MultisineTrajectory.h"

#include <algorithm>
#include <cctype>
#include <string>

#include "Interpreter/Utils.h"
#include "EngineLib/LogMacros.h"

using namespace utils;

namespace commands {

	// --- Utility Functions ---

	// Trims whitespace from both ends of a string and returns a copy
    static inline std::string trimCopy(const std::string& s) {
        size_t a = 0;
        while (a < s.size() && std::isspace((unsigned char)s[a])) ++a;
        size_t b = s.size();
        while (b > a && std::isspace((unsigned char)s[b - 1])) --b;
        return s.substr(a, b - a);
    }

	// Converts string to uppercase copy
    std::string TrajSetCmd::upperCopy(std::string s) {
        std::transform(s.begin(), s.end(), s.begin(),
            [](unsigned char c) { return (unsigned char)std::toupper(c); });
        return s;
    }

    
	// --- Markers ---
	void TrajSetCmd::markFailed(const std::string& message) { setResult({ CmdState::Failed, {}, message }); }
	void TrajSetCmd::markCompleted() { setResult({ CmdState::Executed, {}, "trajSet ran successfully" }); }
	bool TrajSetCmd::hasStarted() const { return _started; }

	// --- TrajSetCmd Implementation ---

	// Constructor
    TrajSetCmd::TrajSetCmd(std::string link, std::string type, std::vector<double> params)
        : _link(std::move(link)), _type(std::move(type)), _params(std::move(params)) {
        _result = { CmdState::NotStarted, {}, "" };
    }

	// Updates trajSet command
	program_data::CmdResult TrajSetCmd::update(CommandContextMotion& cntx, double dt) {
		auto* sim = cntx.Sim();
		if (!sim) {
			markFailed("trajSet: no SimulationManager in context.");
			return { CmdState::Failed, {}, "trajSet failed" };
		}

		auto* robot = cntx.Robot();
		if (!robot) {
			markFailed("trajSet: no robot loaded.");
			return { CmdState::Failed, {}, "trajSet failed" };
		}

		// Validate joint exists and get current angle as q0.
		double q0 = 0.0f;
		if (!robot->tryGetJointAngleRad(_link, q0)) {
			SIM_FAIL("trajSet: joint not found '%s'", _link.c_str());
			markFailed("trajSet: joint not found.");
			return { CmdState::Failed, {}, "trajSet failed" };
		}

		const double t0 = sim->getSimTime();
		const std::string typeU = upperCopy(trimCopy(_type));

		// Get hardware max omega
		double wMax_hw = 0.0;
		if (!robot->tryGetJointOmegaMaxRad(_link, wMax_hw)) {
			D_WARN("trajSet: failed to get joint max omega for link='%s'", _link.c_str());
			wMax_hw = std::numeric_limits<double>::infinity();
		}

		// ===== TRAPEZOID =====
		// trajSet(link, TRAP, q1, vmax, amax)
		if (typeU == "TRAP" || typeU == "TRAPEZOID") {
			if (_params.size() != 3) {
				SIM_FAIL("trajSet TRAP expects 3 params: q1, vmax, amax (got %zu)", _params.size());
				markFailed("trajSet(TRAP): expects 3 params (q1, vmax, amax).");
				return { CmdState::Failed, {}, "trajSet failed" };
			}

			const double q1 = degToRad(_params[0]);
			const double vmax = degToRad(_params[1]);
			const double amax = degToRad(_params[2]);

			auto traj = std::make_unique<control::TrapezoidTrajectory>(t0, q0, q1, vmax, amax);
			sim->traj().set(_link, std::move(traj));
	
			double wMax_est = std::abs(vmax);
			wMax_est = std::min(wMax_est, (double)wMax_hw);
			if (!robot->trySetJointOmegaRefMaxRad(_link, wMax_est)) {
				D_WARN("trajSet(TRAP): failed to set joint omega ref max for link='%s'", _link.c_str());
			}

			SIM_SUCCESS("trajSet: TRAP link='%s' q0=%.6f q1=%.6f vmax=%.6f amax=%.6f", _link.c_str(), q0, q1, vmax, amax);

			_done = true;
			markCompleted();
			return { CmdState::Executed, {}, "trajSet TRAP executed" };
		}

		// ===== SINE =====
		// trajSet(link, SINE, durationSec, centerDeg, amp, freqHz, phaseRad?)
		if (typeU == "SINE" || typeU == "SIN") {
			if (!(_params.size() == 4 || _params.size() == 5)) {
				SIM_FAIL("trajSet SINE expects 4 or 5 params: durationSec, centerDeg, amp, freqHz [,phaseRad] (got %zu)", _params.size());
				markFailed("trajSet(SINE): expects durationSec, centerDeg, amp, freqHz [,phaseRad].");
				return { CmdState::Failed, {}, "trajSet failed" };
			}

			const double dur = _params[0]; // seconds
			const double centre = degToRad(_params[1]); // radians
			const double amp = degToRad(_params[2]);	// radians
			const double fHz = _params[3]; // Hz

			// Duration check
			if (dur <= 0.0) {
				markFailed("trajSet(SINE): duration must be positive.");
				return { CmdState::Failed, {}, "trajSet failed" };
			}
			// Frequency check
			if (fHz <= 0.0) {
				markFailed("trajSet(SINE): frequency must be positive.");
				return { CmdState::Failed, {}, "trajSet failed" };
			}
			// Amplitude check
			if (!(amp > 0.0)) { 
				markFailed("trajSet(SINE): amplitude must be > 0."); 
				return { CmdState::Failed, {}, "trajSet failed" }; 
			}

			// Phase
			double phi = (_params.size() == 5) ? degToRad(_params[4]) : 0.0; // radians

			auto traj = std::make_unique<control::SinusoidalTrajectory>(t0, t0 + dur, centre, amp, fHz, phi);
			sim->traj().set(_link, std::move(traj));

			double wMax_est = TWO_PI_d * fHz * amp;
			wMax_est = std::min(wMax_est, wMax_hw);
			if (!robot->trySetJointOmegaRefMaxRad(_link, wMax_est)) {
				D_WARN("trajSet(SINE): failed to set joint omega ref max for link='%s'", _link.c_str());
			}

			SIM_SUCCESS("trajSet: SINE link='%s' dur=%.6fs centre=%.6f amp=%.6f f=%.6fHz phi=%.6f", _link.c_str(), dur, centre, amp, fHz, phi);

			markCompleted();
			return { CmdState::Executed, {}, "trajSet SINE executed" };
		}

		// ===== MULTISINE =====
		// trajSet(link, MSINE, durationSec, centerDeg, amp1, f1, ph1, amp2, f2, ph2, ...)
		if (typeU == "MSINE" || typeU == "MULTISINE") {
			if (_params.size() < 5) {
				markFailed("trajSet(MSINE): expects duration, centre(deg), and then (amp,f,phase) triples.");
				return { CmdState::Failed, {}, "trajSet(MSINE): expects duration, centre(deg), and then (amp,f,phase) triples." };
			}

			const double dur = _params[0]; // seconds
			if (dur <= 0.0) {
				markFailed("trajSet(MSINE): duration must be positive.");
				return { CmdState::Failed, {}, "trajSet(MSINE): duration must be positive." };
			}

			const double centre = degToRad(_params[1]);

			const size_t rest = _params.size() - 2;
			if (rest % 3 != 0) {
				markFailed("trajSet(MSINE): params after duration must be triples (amp,f,phase).");
				return { CmdState::Failed, {}, "trajSet(MSINE): params after duration must be triples (amp,f,phase)." };
			}

			std::vector<control::SineComponent> comps;
			comps.reserve(rest / 3);

			for (size_t k = 2; k + 2 < _params.size(); k += 3) {
				control::SineComponent comp;
				comp.amp = degToRad(_params[k + 0]);	  // radians
				comp.freqHz = _params[k + 1];			  // Hz
				comp.phaseRad = degToRad(_params[k + 2]); // radians

				if (comp.freqHz <= 0.0) {
					markFailed("trajSet(MSINE): all frequencies must be positive.");
					return { CmdState::Failed, {}, "trajSet(MSINE): all frequencies must be positive." };
				}
				comps.push_back(comp);
			}

			const size_t nComps = comps.size();

			double wMax_est = 0.0;
			for (const auto& c : comps) {
				wMax_est += TWO_PI_d * c.freqHz * std::abs(c.amp);
			}

			// Clamps estimated max omega to hardware limit
			wMax_est = std::min(wMax_est, wMax_hw);

			// Set joint omega ref max
			if (!robot->trySetJointOmegaRefMaxRad(_link, wMax_est)) {
				D_WARN("trajSet(MSINE): failed to set joint omega ref max for link='%s'", _link.c_str());

			}

			auto traj = std::make_unique<control::MultisineTrajectory>(t0, t0 + dur, centre, std::move(comps));
			sim->traj().set(_link, std::move(traj));

			SIM_SUCCESS("trajSet: MSINE link='%s' dur=%.6fs centre=%.6f nComps=%zu", _link.c_str(), centre, dur, nComps);

			_done = true;
			markCompleted();
			return { CmdState::Executed, {}, "trajSet MULTISINE executed" };
		}

		SIM_FAIL("trajSet: unknown type '%s'", _type.c_str());
		markFailed("trajSet: unknown type (use TRAP/SINE/MSINE).");
		return { CmdState::Failed, {}, "trajSet failed" };
	}

	// Execute command
    void TrajSetCmd::execute() {
        setResult({ CmdState::Executing, {}, "trajSet started" });
    }

	// --- Factory ---

	// Factory function to create TrajSetCmd
    std::unique_ptr<ICommand> CreateTrajSetCmd(const std::string& id, const std::vector<std::string>& args) {
        if (id.empty()) {
            D_FAIL("trajSet: missing identifier (link).");
            return nullptr;
        }
        if (args.size() < 1) {
            D_FAIL("trajSet expects at least 1 arg: <type>.");
            return nullptr;
        }

        const std::string link = id;
        const std::string type = trimCopy(args[0]);

        std::vector<double> params;
		params.reserve(args.size() - 1 ? (args.size() - 1) : 0);

        for (size_t i = 1; i < args.size(); ++i) {
            if (!utils::isDouble(args[i])) {
                SIM_FAIL("trajSet: param %zu ('%s') is not a valid double.", i, args[i].c_str());
                return nullptr;
            }
            params.push_back(utils::parseDouble(args[i]));
        }
        return std::make_unique<TrajSetCmd>(link, type, std::move(params));
    }

} // namespace commands


// Syntax:
// trajSet(<linkName>, <type>, <params...>)
// 
// Types and params:
// trajSet(link, TRAP, center(deg), vmax(deg/s), amax(deg/s²))
// trajSet(link, SINE, center(deg), amp(deg), freq(Hz), duration(s) [, phase(deg)])
// trajSet(link, MSINE, duration(s), amp1(deg), f1(Hz), ph1(deg), amp2, f2, ph2, ...)