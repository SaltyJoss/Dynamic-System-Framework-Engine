#pragma once

#include <core/MathLib.h>
#include "IJointTrajectory.h"

#include <vector> 
#include <utility>

using namespace mathlib;
using namespace constants;

namespace control {
	// Structure to define a sine wave component
	template<typename Scalar>
	struct SineComponent {
		Scalar amp = Scalar(0);      // rad
		Scalar freqHz = Scalar(0);   // Hz
		Scalar phaseRad = Scalar(0); // rad
	};

	template<typename Scalar>
	class MultisineTrajectory : public IJointTrajectory {
	public:
		// Constructor
		MultisineTrajectory(Scalar t0, Scalar tf, Scalar q0, std::vector<SineComponent<Scalar>> comps)
			: _t0(t0), _tf(tf), _q0(q0), _comps(std::move(comps)) {}

		// Get the trajectory state at time t
		inline TrajState<Scalar> eval(double t) const {
			if (t <= _t0) { return { _q0, Scalar(0), Scalar(0) }; } // before start time

			// after end time
			const Scalar tt = min(t, _tf); // clamp to end time
			const Scalar tau = tt - _t0;		// time since start
			const Scalar blendTime = Scalar(0.5);		// seconds

			// Blend in with a smooth polynomial ramp
			Scalar ramp = Scalar(1), rampd = Scalar(0), rampdd = Scalar(0);
			if (tau < blendTime) {
				const Scalar u = tau / blendTime; // [0, 1]
				// smooth cubic ramp
				ramp = Scalar(3) * u * u - Scalar(2) * u * u * u;
				// derivative of ramp
				rampd = (Scalar(6) * u - Scalar(6) * u * u) / blendTime;
				// second derivative of ramp
				rampdd =  (Scalar(6) - Scalar(12) * u) / (blendTime * blendTime);
			}
			else if (tau > (_tf - _t0 - blendTime)) {
				const double u = (_tf - _t0 - tau) / blendTime; // [0, 1]
				// smooth cubic ramp down
				ramp = Scalar(3) * u * u - Scalar(2) * u * u * u;
				// derivative of ramp
				rampd = -(Scalar(6) * u - Scalar(6) * u * u) / blendTime;
				// second derivative of ramp
				rampdd = (-Scalar(6) + Scalar(12) * u) / (blendTime * blendTime);
			}

			// Sum contributions from all sine components
			Scalar q = _q0, qd = Scalar(0), qdd = Scalar(0);
			for (const auto& c : _comps) {
				const Scalar w = Scalar(2) * PI_d * c.freqHz;  // angular frequency
				const Scalar arg = w * tau + c.phaseRad;
				const Scalar sins = sin(arg);		 // sine term
				const Scalar coss = cos(arg);		 // cosine term
				// raw sinsuoidal
				const Scalar qs = c.amp * sins;			   // position
				const Scalar qds = c.amp * w * coss;	   // velocity
				const Scalar qdds = -c.amp * w * w * sins; // acceleration
				// blended sinusoidal
				q += ramp * qs;
				qd += ramp * qds + rampd * qs;
				qdd += ramp * qdds + Scalar(2) * rampd * qds + rampdd * qs;
			}
			return { q, qd, qdd };
		}

		// Get the time span of the trajectory
		TrajTimeSpan<double> span() const override { return { _t0, _tf }; }

	private:
		Scalar _t0{ Scalar(0) }, _tf{ Scalar(0) }, _q0{ Scalar(0) }; // start time, end time, position offset
		std::vector<SineComponent<Scalar>> _comps;   // amplitudes of the sine components
	};
}