#pragma once

#include "MathLibAPI.h"
#include "TrajectoryTypes.h"

using namespace mathlib;

namespace control {
	// Joint Trajectory Interface
	class MATHLIB_API IJointTrajectory {
	public:
		virtual ~IJointTrajectory() = default;

		// Get the desired trajectory state at time t
		virtual TrajState eval(double t) const = 0;
		// Get the time span of the trajectory
		virtual TrajTimeSpan span() const = 0;
		// Check if the trajectory is finished at time t
		bool finished(double t) const { return t >= span().tf; }
	};
} // namespace control