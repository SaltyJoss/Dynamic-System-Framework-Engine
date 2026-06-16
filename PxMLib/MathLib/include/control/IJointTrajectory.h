// PxMLib/MathLib IJointTrajectory.h
#pragma once
#include <core/MathLib.h>
#include "TrajectoryTypes.h"

using namespace mathlib;

namespace control {
	// Joint Trajectory Interface
	class IJointTrajectory {
	public:
		virtual ~IJointTrajectory() = default;

		// Get the desired trajectory state at time t (double overload)
		virtual TrajState<double> eval(double t) const = 0;
		// Get the time span of the trajectory
		virtual TrajTimeSpan<double> span() const = 0;
		// Check if the trajectory is finished at time t
		bool finished(double t) const {
			const auto s = span();
			return t >= s.tf;
		}
	};
} // namespace control