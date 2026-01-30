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

    static inline std::string trimCopy(const std::string& s) {
        size_t a = 0;
        while (a < s.size() && std::isspace((unsigned char)s[a])) ++a;
        size_t b = s.size();
        while (b > a && std::isspace((unsigned char)s[b - 1])) --b;
        return s.substr(a, b - a);
    }

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

    TrajSetCmd::TrajSetCmd(std::string link, std::string type, std::vector<double> params)
        : _link(std::move(link)), _type(std::move(type)), _params(std::move(params)) {
        _result = { CmdState::NotStarted, {}, "" };
    }

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
		float q0f = 0.0f;
		if (!robot->tryGetJointAngleRad(_link, q0f)) {
			SIM_FAIL("trajSet: joint not found '%s'", _link.c_str());
			markFailed("trajSet: joint not found.");
			return { CmdState::Failed, {}, "trajSet failed" };
		}

		const double q0 = (double)q0f;
		const double t0 = sim->getSimTime();
		const std::string typeU = upperCopy(trimCopy(_type));

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

			cntx.setJointMaxOmegaRad(_link, std::abs(vmax));

			SIM_SUCCESS("trajSet: TRAP link='%s' q0=%.6f q1=%.6f vmax=%.6f amax=%.6f",
				_link.c_str(), q0, q1, vmax, amax);

			LOG_INFO("trajSet: TRAP link=%s t0=%.6f q0=%.9f q1=%.9f vmax=%.9f amax=%.9f",
				_link.c_str(), t0, q0, q1, vmax, amax);

			_done = true;
			markCompleted();
			return { CmdState::Executed, {}, "trajSet TRAP executed" };
		}

		// ===== SINE =====
		// trajSet(link, SINE, durationSec, centerDeg, amp, freqHz, phaseRad?)
		if (typeU == "SINE" || typeU == "SIN") {
			if (!(_params.size() == 4 || _params.size() == 5)) {
				SIM_FAIL("trajSet SINE expects 4 or 5 params: centerDeg, amp, freqHz, durationSec [,phaseRad] (got %zu)", _params.size());
				markFailed("trajSet(SINE): expects centerDeg, amp, freqHz, durationSec [,phaseRad].");
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

			SIM_SUCCESS("trajSet: SINE link='%s' dur=%.6fs centre=%.6f amp=%.6f f=%.6fHz phi=%.6f",
				_link.c_str(), dur, centre, amp, fHz, phi);

			LOG_INFO("trajSet: SINE link=%s t0=%.6f q0=%.9f centre=%.9f amp=%.9f fHz=%.6f dur=%.6f phi=%.9f phaseMode=%s",
				_link.c_str(), t0, q0, centre, amp, fHz, dur, phi,
				(_params.size() == 5 ? "USER" : "AUTO"));

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

			auto traj = std::make_unique<control::MultisineTrajectory>(t0, t0 + dur, centre, std::move(comps));
			sim->traj().set(_link, std::move(traj));

			SIM_SUCCESS("trajSet: MSINE link='%s' dur=%.6fs centre=%.6f nComps=%zu",
				_link.c_str(), centre, dur, nComps);

			LOG_INFO("trajSet: MSINE link=%s t0=%.6f q0=%.9f centre=%.9f dur=%.6f nComps=%zu", 
				_link.c_str(), t0, q0, centre, dur, nComps);

			_done = true;
			markCompleted();
			return { CmdState::Executed, {}, "trajSet MULTISINE executed" };
		}

		SIM_FAIL("trajSet: unknown type '%s'", _type.c_str());
		markFailed("trajSet: unknown type (use TRAP/SINE/MSINE).");
		return { CmdState::Failed, {}, "trajSet failed" };
	}


    void TrajSetCmd::execute() {
        setResult({ CmdState::Executing, {}, "trajSet started" });
    }

	// --- Factory ---

    std::unique_ptr<ICommand> CreateTrajSetCmd(const std::string& id, const std::vector<std::string>& args) {
        // DSL format: trajSet(<link>, <type>, <params...>)
        // Parser convention (like rotateJointTo): id == <link>, args == [type, param1, param2, ...]
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

// Types and params:
// trajSet(link, TRAP, q1(deg), vmax(deg/s), amax(deg/s²))
// trajSet(link, SINE, centerDeg(deg), amp(deg), freq(Hz), duration(s) [, phase(deg)])
// trajSet(link, MSINE, duration(s), amp1(deg), f1(Hz), ph1(deg), amp2, f2, ph2, ...)