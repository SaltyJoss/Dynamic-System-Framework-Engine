#pragma once

#include "MathLibAPI.h"
#include "core/constants.h"
#include "IJointTrajectory.h"

using namespace mathlib;
using namespace constants;

namespace control {
	// Structure to define a sine wave component
	struct SineComponent {
		double amp = 0.0;      // rad
		double freqHz = 0.0;   // Hz
		double phaseRad = 0.0; // rad
	};

	class MultisineTrajectory : public IJointTrajectory {
	public:
		MultisineTrajectory(double t0, double tf, double q0, const std::vector<SineComponent> comps)
			: _t0(t0), _tf(tf), _q0(q0), _comps(std::move(comps)) {
		}

		// Get the trajectory state at time t
		TrajState eval(double t) const override {
			if (t <= _t0) { return { _q0, 0.0, 0.0 }; } // before start time

			// after end time
			const double tt = (t >= _tf) ? _tf : t; // clamp to end time
			const double tau = tt - _t0;			// time since start

			double q = _q0, qd = 0.0, qdd = 0.0;
			// Sum contributions from all sine components
			for (const auto& c : _comps) {
				const double w = 2.0 * PI * c.freqHz;				// angular frequency
				const double s = std::sin(w * tau + c.phaseRad);	// sine term
				const double coss = std::cos(w * tau + c.phaseRad); // cosine term
				q += c.amp * s;		   // position
				qd += c.amp * w * coss;  // velocity
				qdd += -c.amp * w * w * s; // acceleration
			}
			return { q, qd, qdd };
		}

		// Get the time span of the trajectory
		TrajTimeSpan span() const override { return { _t0, _tf }; }

	private:
		double _t0{ 0 }, _tf{ 0 }, _q0{ 0 }; // start time, end time, position offset
		std::vector<SineComponent> _comps;     // amplitudes of the sine components
	};
}