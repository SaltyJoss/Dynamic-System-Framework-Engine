#pragma once

#include "MathLibAPI.h"
#include "core/constants.h"
#include "IJointTrajectory.h"

#include <vector> 
#include <utility>

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
		MultisineTrajectory(double t0, double tf, double q0, std::vector<SineComponent> comps)
			: _t0(t0), _tf(tf), _q0(q0), _comps(std::move(comps)) {}

		// Get the trajectory state at time t
		TrajState eval(double t) const override {
			if (t <= _t0) { return { _q0, 0.0, 0.0 }; } // before start time

			// after end time
			const double tt = std::min(t, _tf); // clamp to end time
			const double tau = tt - _t0;		// time since start
			const double blendTime = 0.5;		// seconds

			// Blend in with a smooth polynomial ramp
			double ramp = 1.0, rampd = 0.0, rampdd = 0.0;
			if (tau < blendTime) {
				const double u = tau / blendTime; // [0, 1]
				// smooth cubic ramp
				ramp =
					3.0 * u * u -
					2.0 * u * u * u;
				// derivative of ramp
				rampd =
					(6.0 * u - 6.0 * u * u) /
					blendTime;
				// second derivative of ramp
				rampdd = 
					(6.0 - 12.0 * u) /
					(blendTime * blendTime);
			}
			else if (tau > (_tf - _t0 - blendTime)) {
				const double u = (_tf - _t0 - tau) / blendTime; // [0, 1]
				// smooth cubic ramp down
				ramp =
					3.0 * u * u -
					2.0 * u * u * u;
				// derivative of ramp
				rampd =
					-(6.0 * u - 6.0 * u * u) /
					blendTime;
				// second derivative of ramp
				rampdd =
					(-6.0+ 12.0 * u) /
					(blendTime * blendTime);
			}

			// Sum contributions from all sine components
			double q = _q0, qd = 0.0, qdd = 0.0;
			for (const auto& c : _comps) {
				const double w = 2.0 * PI_d * c.freqHz;  // angular frequency
				const double arg = w * tau + c.phaseRad;
				const double sins = std::sin(arg);		 // sine term
				const double coss = std::cos(arg);		 // cosine term
				// raw sinsuoidal
				const double qs = c.amp * sins;			   // position
				const double qds = c.amp * w * coss;	   // velocity
				const double qdds = -c.amp * w * w * sins; // acceleration
				// blended sinusoidal
				q += ramp * qs;
				qd += ramp * qds + rampd * qs;
				qdd += ramp * qdds + 2.0 * rampd * qds + rampdd * qs;
			}
			return { q, qd, qdd };
		}

		// Get the time span of the trajectory
		TrajTimeSpan span() const override { return { _t0, _tf }; }

	private:
		double _t0{ 0 }, _tf{ 0 }, _q0{ 0 }; // start time, end time, position offset
		std::vector<SineComponent> _comps;   // amplitudes of the sine components
	};
}