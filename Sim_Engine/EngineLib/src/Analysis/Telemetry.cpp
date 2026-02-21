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
		double sum_e2	  = 0.0;	// sum of squared errors
		double max_abs_e  = 0.0; // max absolute error
		int clampThetaSum = 0;	// sum of angle clamping events
		int clampOmegaSum = 0;	// sum of velocity clamping events
		int clampSum	  = 0;	// sum of clamping events
		int worstJ		  = -1;	// index of worst joint

		// Collect telemetry for each joint
		for (int i = 0; i < n; ++i) {
			const auto& j = robotSys.getRobot().joints()[i];

			JointTelemetry jt;

			// Joint data
			jt.q   = j.q;
			jt.qd  = j.qd;
			jt.eta = j.eta;

			// Reference data
			jt.q_ref = j.q_ref;
			jt.qd_ref = j.qd_ref;
			jt.qdd_ref = j.qdd_ref;

			// Dynamics Data
			jt.torqueNm = j.torque;
			jt.damping	= j.dynamics.damping;
			jt.friction = j.dynamics.friction;
			jt.effort	= (j.limits.maxEffort > 0.0f) ? (j.torque / j.limits.maxEffort) : 0.0f;

			// Clamping flags
			jt.clampTheta = (j.q <= j.limits.minAngle) || (j.q >= j.limits.maxAngle);
			jt.clampOmega = (std::abs(j.qd) >= j.limits.maxqd);

			// Accumulate clamping events
			clampThetaSum += (int)jt.clampTheta;
			clampOmegaSum += (int)jt.clampOmega;
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
			const double e = jt.q_ref - jt.q;
			sum_e2 += e * e;
			
			// Max absolute error and worst joint
			const double abs_e = std::abs(e);
			if (abs_e > max_abs_e) { max_abs_e = abs_e; worstJ = i; }

			s.j[i] = jt; // store joint telemetry
		}

		// Error statistics
		s.err_rms	  = (n > 0) ? std::sqrt(sum_e2 / (double)n) : 0.0f; // RMS error
		s.err_max	  = max_abs_e;	   // max error
		s.clamp_theta = clampThetaSum; // total angle clamping events
		s.clamp_omega = clampOmegaSum; // total velocity clamping events
		s.clamp_sum	  = clampSum;	   // total clamping events
		s.worst_joint = worstJ;		   // index of worst joint

		// Finalize write
		ring.endWrite();
	}
		
} // namespace diagnostics