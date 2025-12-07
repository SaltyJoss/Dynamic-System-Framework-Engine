#include "pch.h"
#include "kinematics/Trajectory.h"

namespace kinematics {
	/// <inheritdoc/>
	JointTrajectory::JointTrajectory(const std::vector<JointTrajectoryPoint>& points) : trajectoryPoints(points) {}

	/// <inheritdoc/>
	VecX JointTrajectory::position(double t) const {
		// Find the appropriate segment for interpolation
		if (t <= trajectoryPoints.front().t) {
			return trajectoryPoints.front().q;
		}

		if (t >= trajectoryPoints.back().t) {
			return trajectoryPoints.back().q;
		}

		for (size_t i = 0; i < trajectoryPoints.size() - 1; ++i) {
			if (t >= trajectoryPoints[i].t && t <= trajectoryPoints[i + 1].t) {
				double t0 = trajectoryPoints[i].t;
				double t1 = trajectoryPoints[i + 1].t;
				VecX q0 = trajectoryPoints[i].q;
				VecX q1 = trajectoryPoints[i + 1].q;
				double tau = (t - t0) / (t1 - t0);
				return (1 - tau) * q0 + tau * q1; // Linear interpolation
			}
		}
		return VecX(); // Should not reach here
	}

	/// <inheritdoc/>
	VecX JointTrajectory::velocity(double t) const {
		// Find the appropriate segment for interpolation
		if (t <= trajectoryPoints.front().t) {
			return trajectoryPoints.front().qdot;
		}

		if (t >= trajectoryPoints.back().t) {
			return trajectoryPoints.back().qdot;
		}

		for (size_t i = 0; i < trajectoryPoints.size() - 1; ++i) {
			if (t >= trajectoryPoints[i].t && t <= trajectoryPoints[i + 1].t) {
				double t0 = trajectoryPoints[i].t;
				double t1 = trajectoryPoints[i + 1].t;
				VecX qdot0 = trajectoryPoints[i].qdot;
				VecX qdot1 = trajectoryPoints[i + 1].qdot;
				double tau = (t - t0) / (t1 - t0);
				return (1 - tau) * qdot0 + tau * qdot1; // Linear interpolation
			}
		}
		return VecX(); // Should not reach here
	}

	/// <inheritdoc/>
	VecX JointTrajectory::acceleration(double t) const {
		// Find the appropriate segment for interpolation
		if (t <= trajectoryPoints.front().t) {
			return trajectoryPoints.front().qddot;
		}

		if (t >= trajectoryPoints.back().t) {
			return trajectoryPoints.back().qddot;
		}

		for (size_t i = 0; i < trajectoryPoints.size() - 1; ++i) {
			if (t >= trajectoryPoints[i].t && t <= trajectoryPoints[i + 1].t) {
				double t0 = trajectoryPoints[i].t;
				double t1 = trajectoryPoints[i + 1].t;
				VecX qddot0 = trajectoryPoints[i].qddot;
				VecX qddot1 = trajectoryPoints[i + 1].qddot;
				double tau = (t - t0) / (t1 - t0);
				return (1 - tau) * qddot0 + tau * qddot1; // Linear interpolation
			}
		}
		return VecX(); // Should not reach here
	}

	/// <inheritdoc/>
	JointTrajectory JointTrajectory::makeCubicTrajectory(const VecX& q0, const VecX& qT, double T) {
		std::vector<JointTrajectoryPoint> points;
		JointTrajectoryPoint startPoint;

		startPoint.t = 0.0;
		startPoint.q = q0;
		startPoint.qdot = VecX::Zero(q0.size());
		startPoint.qddot = VecX::Zero(q0.size());

		JointTrajectoryPoint endPoint;

		endPoint.t = T;
		endPoint.q = qT;
		endPoint.qdot = VecX::Zero(qT.size());
		endPoint.qddot = VecX::Zero(qT.size());

		points.push_back(startPoint);
		points.push_back(endPoint);

		return JointTrajectory(points);
	}
}