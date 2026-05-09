#pragma once

#include "MathLibAPI.h"
#include "core/constants.h"
#include "IJointTrajectory.h"

using namespace mathlib;
using namespace constants;

namespace control {
	class SinusoidalTrajectory : public IJointTrajectory {
	public:
		SinusoidalTrajectory(double t0, double tf, double centre, double amp, double freqHz, double phaseRad = 0.0)
			: _t0(t0), _tf(tf), _centre(centre), _A(amp), _f(freqHz), _phi(phaseRad) {}

		TrajState eval(double t) const override {
			if (t <= _t0) { return { _centre, 0.0, 0.0 }; } // before start time

			// after end time
			if (t >= _tf) {
				const double tau = _tf - _t0;
				const double w = 2.0 * PI_d * _f;
				const double sin = std::sin(w * tau + _phi); // sine term
				const double cos = std::cos(w * tau + _phi); // cosine term
				
				const double q = _centre + _A * sin;		// position
				const double qd = _A * w * cos;		// velocity
				const double qdd = -_A * w * w * sin; // acceleration

				return { q, qd, qdd };
			}

			const double tau = t - _t0;
			const double w = 2.0 * PI_d * _f;
			const double sin = std::sin(w * tau + _phi); // sine term
			const double cos = std::cos(w * tau + _phi); // cosine term

			const double q = _centre + _A * sin;	  // position
			const double qd = _A * w * cos;		  // velocity
			const double qdd = -_A * w * w * sin; // acceleration
			return { q, qd, qdd };
		}

		TrajTimeSpan span() const override { return { _t0, _tf }; }
	private:
		double _t0{ 0 }, _tf{ 0 }; // start and end times
		double _centre{ 0 }, _A{ 0 }, _f{ 0 }, _phi{ 0 }; // position offset, amplitude, frequency (Hz), phase (rad)

	};
} // namespace control