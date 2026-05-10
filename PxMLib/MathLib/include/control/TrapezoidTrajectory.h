#pragma once

#include "MathLibAPI.h"
#include "IJointTrajectory.h"

using namespace mathlib;

namespace control {
	class TrapezoidTrajectory : public IJointTrajectory {
	public:
		TrapezoidTrajectory(double t0, double q0, double q1, double vMax, double aMax)
			: _t0(t0), _q0(q0), _q1(q1), _vMax(std::abs(vMax)), _aMax(std::abs(aMax)) {
			const double dq = _q1 - _q0;   // total displacement
			_sgn = (dq >= 0) ? 1.0 : -1.0; // direction sign
			_D = std::abs(dq);			   // total distance

			if (_D <= 0.0 || _vMax <= 0.0 || _aMax <= 0.0) {
				_ta = _tc = _td = 0.0;
				_tf = _t0;
				_triangular = true;
				return;
			}

			const double ta_full = _vMax / _aMax;		  // time to reach max velocity
			const double D_min = (_vMax * _vMax) / _aMax; // distance during accel and decel
		
			if (_D >= D_min) {
				_triangular = false; // trapezoidal profile
				_ta = ta_full;
				_td = ta_full;
				_tc = (_D - D_min) / _vMax;
			}
			else {
				_triangular = true; // triangular profile
				_ta = std::sqrt(_D / _aMax);
				_td = _ta;
				_tc = 0.0;
				_vp = _aMax * _ta; // peak velocity
			}

			_t1 = _t0 + _ta; // end of acceleration phase
			_t2 = _t1 + _tc; // end of cruise phase
			_t3 = _t2 + _td; // end of deceleration phase
			_tf = _t3;		 // final time
		}

		TrajState eval(double t) const override {
			if (t <= _t0) { return { _q0, 0.0, 0.0 }; }
			if (t >= _tf) { return { _q1, 0.0, 0.0 }; }

			const double tau = t - _t0;							// time since start
			const double a = _aMax * _sgn;						// acceleration
			const double vp = _triangular ? _vp : _vMax * _sgn; // peak velocity

			// Phase 1: Acceleration [_t0, _t1)
			if (t < _t1) {
				const double q = _q0 + 0.5 * a * tau * tau;
				const double qd = a * tau;
				const double qdd = a;
				return { q, qd, qdd };
			}

			// Precompute distance
			const double qa = _q0 + 0.5 * a * _ta * _ta; // position at end of accel phase

			// Phase 2: Cruise [_t1, _t2)
			if (t < _t2) {
				const double q = qa + vp * (tau - _ta);
				const double qd = vp;
				const double qdd = 0.0;
				return { q, qd, qdd };
			}

			// Phase 3: Deceleration [_t2, _t3]
			{
				const double td = tau - _ta - _tc;
				const double q = qa + vp * _tc + vp * td - 0.5 * a * td * td;
				const double qd = vp - a * td;
				const double qdd = -a;
				return { q, qd, qdd };
			}
		}
			
		TrajTimeSpan span() const override { return { _t0, _tf }; }

	private:
		double _t0{ 0 }, _tf{ 0 }; // start and end times
		double _q0{ 0 }, _q1{ 0 }; // start and end positions
		double _sgn{ 1 };		   // direction sign

		double _vMax{ 0 }, _aMax{ 0 };		 // max velocity and acceleration
		double _ta{ 0 }, _tc{ 0 }, _td{ 0 }; // accel, cruise, decel times
		double _qa{ 0 }, _qc{ 0 }, _qd{ 0 }; // positions at phase transitions

		double _D{ 0 };			// total distance
		bool _triangular{ false };	// flag for triangular profile

		// key times
		double _t1{ 0 }, _t2{ 0 }, _t3{ 0 }; // phase transition times

		double _vp{ 0 }; // peak velocity for triangular profile
	};
} // namespace control