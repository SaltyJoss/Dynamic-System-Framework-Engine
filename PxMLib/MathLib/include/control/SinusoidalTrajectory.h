#pragma once

#include <core/MathLib.h>
#include "IJointTrajectory.h"

using namespace mathlib;
using namespace constants;

namespace control {
	template<typename Scalar>
	class SinusoidalTrajectory : public IJointTrajectory {
	public:
		SinusoidalTrajectory(
			Scalar t0, Scalar tf, Scalar q0,
			Scalar amp, Scalar freqHz, Scalar phaseRad = Scalar(0)
		) : _t0(t0), _tf(tf), _q0(q0), _A(amp), _f(freqHz), _phi(phaseRad) {}

		// Evaluate the trajectory state at time t (Scale)
		inline TrajState<Scalar> eval(Scalar t) const {
			if (t <= _t0) { return { _q0, Scalar(0), Scalar(0) }; } // before start time
			// after end time
			if (t >= _tf) {
				const Scalar tau = _tf - _t0;
				const Scalar w = Scalar(2) * PI_d * _f;
				const Scalar s = sin(w * tau + _phi); // sine term
				const Scalar c = cos(w * tau + _phi); // cosine term
				
				const Scalar q = _q0 + _A * s;		// position
				const Scalar qd = _A * w * c;		// velocity
				const Scalar qdd = -_A * w * w * s; // acceleration

				return { q, qd, qdd };
			}

			const Scalar tau = t - _t0;
			const Scalar w = Scalar(2) * PI * _f;
			const Scalar s = sin(w * tau + _phi); // sine term
			const Scalar c = cos(w * tau + _phi); // cosine term

			const Scalar q = _q0 + _A * s;		// position
			const Scalar qd = _A * w * c;		// velocity
			const Scalar qdd = -_A * w * w * s; // acceleration
			return { q, qd, qdd };
		}

		// Get the time span of the trajectory
		TrajTimeSpan<double> span() const override { return { _t0, _tf }; }
	private:
		Scalar _t0{ Scalar(0) }, _tf{ Scalar(0) };					  // start and end times
		Scalar _q0{ Scalar(0) }, _A{ Scalar(0) }, _f{ Scalar(0) }, _phi{ Scalar(0) }; // position offset, amplitude, frequency (Hz), phase (rad)

	};
} // namespace control