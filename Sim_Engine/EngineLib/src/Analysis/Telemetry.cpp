#include "pch.h"
// File:   Telemetry.cpp
// GitHub: SaltyJoss
#include "Analysis/Telemetry.h"
#include "Robots/RobotSystem.h"
#include "Robots/TrajectoryManager.h"

namespace diagnostics {
	// Record telemetry data at time t
	void TelemetryRecorder::record(double t, const robots::RobotSystem& robotSys, const control::TrajectoryManager* trajOpt, eTelemetryLevel /*level*/) {
		const int n = (int)robotSys.getRobot().joints().size();

		// Begin write
		TelemetrySample& s = ring.beginWrite();
		s.timeSec = t;

		// Resize joint vector (if needed)
		const auto& joints = robotSys.getRobot().joints();
		s.j.resize(joints.size());

		// Accumulators for error statistics
		double sum_e2	= 0.0;	// sum of squared errors
		double max_abs_e = 0.0f; // max absolute error
		int worstJ		= -1;	// index of worst joint
		int clampSum	= 0;	// sum of clamping events

		// Collect telemetry for each joint
		for (int i = 0; i < n; ++i) {
			const auto& j = robotSys.getRobot().joints()[i];

			JointTelemetry jt;

			// Joint data
			jt.eta		   = j.eta;
			jt.thetaRad	   = j.thetaRad;
			jt.omegaRad_s  = j.omegaRad_s;
			jt.torqueNm	   = j.torque;
			jt.damping	   = j.dynamics.damping;
			jt.friction	   = j.dynamics.friction;
			jt.effort = (j.limits.maxEffort > 0.0f) ? (j.torque / j.limits.maxEffort) : 0.0f;

			// Reference data
			jt.thetaRefRad	  = j.thetaRefRad;
			jt.omegaRefRad_s  = j.omegaRefRad_s;
			jt.alphaRefRad_s2 = j.alphaRefRad_s2;

			// Clamping flags
			jt.clampTheta = (j.thetaRad <= j.limits.minAngle) || (j.thetaRad >= j.limits.maxAngle);
			jt.clampOmega = (j.omegaRad_s <= 0.0f) || (j.omegaRad_s >= j.limits.maxOmegaRad_s);
			clampSum += (int)jt.clampTheta + (int)jt.clampOmega;

			// Trajectory data
			if (trajOpt) {
				control::TrajState ts{};
				if (trajOpt->tryEval(std::string(j.child), t, ts)) {
					jt.traj_q   = ts.q;
					jt.traj_qd  = ts.qd;
					jt.traj_qdd = ts.qdd;
					jt.traj_active = true;
				}
			}

			// Joint error
			const double e = jt.thetaRefRad - jt.thetaRad;
			sum_e2 += e * e;
			
			// Max absolute error and worst joint
			const double abs_e = std::abs(e);
			if (abs_e > max_abs_e) { max_abs_e = abs_e; worstJ = i; }

			s.j[i] = jt; // store joint telemetry
		}

		// Error statistics
		s.err_rms = (n > 0) ? std::sqrt(sum_e2 / (double)n) : 0.0f; // RMS error
		s.err_max = max_abs_e;	// max error
		s.worst_joint = worstJ; // index of worst joint
		s.clamp_sum = clampSum; // total clamping events

		// Finalize write
		ring.endWrite();
	}
		
} // namespace diagnostics