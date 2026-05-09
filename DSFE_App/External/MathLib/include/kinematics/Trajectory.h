#pragma once

#include "MathLibAPI.h"
#include "core/Types.h"

using namespace mathlib;

namespace kinematics {
	struct MATHLIB_API JointTrajectoryPoint {
		double t;   // Time at this trajectory point
		VecX q;     // Joint positions
		VecX qdot;	// Joint velocities
		VecX qddot; // Joint accelerations
	};

	class MATHLIB_API JointTrajectory {
	public:
		/// <summary>
		/// Constructor for JointTrajectory
		/// </summary>
		JointTrajectory(const std::vector<JointTrajectoryPoint>& points);
		
		/// <summary>
		/// The trajectory points that define the joint trajectory
		/// </summary>
		std::vector<JointTrajectoryPoint> trajectoryPoints;

		/// <summary>
		/// Evaluate the trajectory at time t to get joint positions
		/// </summary>
		/// <param name="t">Time at which to evaluate the trajectory</param>
		/// <returns>Joint positions at time t</returns>
		VecX position(double t) const;

		/// <summary>
		/// Evaluate the trajectory at time t to get joint velocities
		/// </summary>
		/// <param name="t">Time at which to evaluate the trajectory</param>
		/// <returns>Joint velocities at time t</returns>
		VecX velocity(double t) const;

		/// <summary>
		/// Evaluate the trajectory at time t to get joint accelerations
		/// </summary>
		/// <param name="t">Time at which to evaluate the trajectory</param>
		/// <returns>Joint accelerations at time t</returns>
		VecX acceleration(double t) const;

		/// <summary>
		/// Get the total duration of the trajectory
		/// </summary>
		/// <param name="q0">Initial joint positions</param>
		/// <param name="qT">Final joint positions</param>
		/// <param name="T">Total duration of the trajectory</param>
		/// <returns>JointTrajectory object</returns>
		JointTrajectory makeCubicTrajectory(const VecX& q0, const VecX& qT, double T);
	};
}