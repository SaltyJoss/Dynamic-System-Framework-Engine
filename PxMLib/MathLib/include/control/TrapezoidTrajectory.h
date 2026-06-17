#pragma once

#include <core/MathLib.h>
#include "IJointTrajectory.h"

using namespace mathlib;

namespace control {
	template<typename Scalar>
	class TrapezoidTrajectory : public IJointTrajectory {
	public:
		TrapezoidTrajectory(Scalar t0, Scalar q0, Scalar q1, Scalar vMax, Scalar aMax)
			: _t0(t0), _q0(q0), _q1(q1), _vMax(mathlib::abs(vMax)), _aMax(mathlib::abs(aMax)) {
			const Scalar dq = _q1 - _q0; // total displacement
			_sgn = mathlib::sgn(dq);				 // direction signum
			_D = abs(dq);		 // total distance

			if (_D <= Scalar(0) || _vMax <= Scalar(0) || _aMax <= Scalar(0)) {
				_ta = _tc = _td = Scalar(0);
				_tf = _t0;
				_triangular = true;
				return;
			}

			const Scalar ta_full = _vMax / _aMax;		  // time to reach max velocity
			const Scalar D_min = (_vMax * _vMax) / _aMax; // distance during accel and decel
		
			if (_D >= D_min) {
				_triangular = false; // trapezoidal profile
				_ta = ta_full;
				_td = ta_full;
				_tc = (_D - D_min) / _vMax;
			}
			else {
				_triangular = true; // triangular profile
				_ta = sqrt(_D / _aMax);
				_td = _ta;
				_tc = Scalar(0);
				_vp = _aMax * _ta; // peak velocity
			}

			_t1 = _t0 + _ta; // end of acceleration phase
			_t2 = _t1 + _tc; // end of cruise phase
			_t3 = _t2 + _td; // end of deceleration phase
			_tf = _t3;		 // final time
		}

		inline TrajState<Scalar> eval(Scalar t) const {
			if (t <= _t0) { return { _q0, Scalar(0), Scalar(0) }; }
			if (t >= _tf) { return { _q1, Scalar(0), Scalar(0) }; }

			const Scalar tau = t - _t0;							// time since start
			const Scalar a = _aMax * _sgn;						// acceleration
			const Scalar vp = _triangular ? _vp : _vMax * _sgn; // peak velocity

			// Phase 1: Acceleration [_t0, _t1)
			if (t < _t1) {
				const Scalar q = _q0 + Scalar(0.5) * a * tau * tau;
				const Scalar qd = a * tau;
				const Scalar qdd = a;
				return { q, qd, qdd };
			}

			// Precompute distance
			const Scalar qa = _q0 + Scalar(0.5) * a * _ta * _ta; // position at end of accel phase

			// Phase 2: Cruise [_t1, _t2)
			if (t < _t2) {
				const Scalar q = qa + vp * (tau - _ta);
				const Scalar qd = vp;
				const Scalar qdd = Scalar(0);
				return { q, qd, qdd };
			}

			// Phase 3: Deceleration [_t2, _t3]
			{
				const Scalar td = tau - _ta - _tc;
				const Scalar q = qa + vp * _tc + vp * td - Scalar(0.5) * a * td * td;
				const Scalar qd = vp - a * td;
				const Scalar qdd = -a;
				return { q, qd, qdd };
			}
		}
		
		// Get the time span of the trajectory
		TrajTimeSpan<double> span() const override { return { _t0, _tf }; }

	private:
		Scalar _t0{ Scalar(0) }, _tf{ Scalar(0) }; // start and end times
		Scalar _q0{ Scalar(0) }, _q1{ Scalar(0) }; // start and end positions
		Scalar _sgn{ Scalar(1) };		   // direction sign

		Scalar _vMax{ Scalar(0) }, _aMax{ Scalar(0) };		 // max velocity and acceleration
		Scalar _ta{ Scalar(0) }, _tc{ Scalar(0) }, _td{ Scalar(0) }; // accel, cruise, decel times
		Scalar _qa{ Scalar(0) }, _qc{ Scalar(0) }, _qd{ Scalar(0) }; // positions at phase transitions

		Scalar _D{ Scalar(0) };	// total distance
		bool _triangular{ false };	// flag for triangular profile

		// key times
		Scalar _t1{ Scalar(0) }, _t2{ Scalar(0) }, _t3{ Scalar(0) }; // phase transition times

		Scalar _vp{ Scalar(0) }; // peak velocity for triangular profile
	};
} // namespace control