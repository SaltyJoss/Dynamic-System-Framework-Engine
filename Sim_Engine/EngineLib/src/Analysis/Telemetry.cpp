#include "pch.h"

#include "Analysis/Telemetry.h"
#include "Robots/RobotSystem.h"
#include "Robots/TrajectoryManager.h"

namespace diagnostics {
	void TelemetryRecorder::record(double t, const robots::RobotSystem& robotSys, const control::TrajectoryManager* trajOpt, eTelemetryLevel level) {
		const int n = (int)robotSys.getRobot().joints().size();

		// Begin write
		TelemetrySample s = ring.beginWrite();
		s.timeSec = t;

		// Resize joint vector (if needed)
		if (s.j.size() != (size_t)n) { s.j.resize((size_t)n); }

		// Accumulators for error statistics
		double sum_e2	= 0.0;	// sum of squared errors
		float max_abs_e = 0.0f; // max absolute error
		int worstJ		= -1;	// index of worst joint
		int clampSum	= 0;	// sum of clamping events

		for (int i = 0; i < n; ++i) {
			const auto& joint = robotSys.getRobot().joints()[i];

			JointTelemetry jt;

			// Joint data
			jt.thetaRad	   = joint.thetaRad;
			jt.omegaRad_s  = joint.omegaRad_s;
			jt.alphaRad_s2 = joint.alphaRefRad_s2; // approximate
			jt.torqueNm	   = joint.torque;
			jt.damping	   = joint.dynamics.damping;
			jt.friction	   = joint.dynamics.friction;
			jt.effort = (joint.limits.maxEffort > 0.0f) ? (joint.torque / joint.limits.maxEffort) : 0.0f;
			jt.I_eff	   = robotSys.computeJointMetrics(joint, robotSys.getRobot().links()[i + 1],
				(double)joint.thetaRad,    (double)joint.omegaRad_s, 
				(double)joint.thetaRefRad, (double)joint.omegaRefRad_s,
				(double)joint.alphaRefRad_s2).I_eff;

			// Reference data
			jt.thetaRefRad	  = joint.thetaRefRad;
			jt.omegaRefRad_s  = joint.omegaRefRad_s;
			jt.alphaRefRad_s2 = joint.alphaRefRad_s2;

			// Clamping flags
			if (jt.clampTheta) { ++clampSum; }
			if (jt.clampOmega) { ++clampSum; }

			// Trajectory data
			if (trajOpt) {
				control::TrajState ts{};
				if (trajOpt->tryEval(std::string(joint.child), t, ts)) {
					jt.traj_q   = (float)ts.q;
					jt.traj_qd  = (float)ts.qd;
					jt.traj_qdd = (float)ts.qdd;
					jt.traj_active = true;
				}
			}

			// Joint error
			const float e = jt.thetaRefRad - jt.thetaRad;
			sum_e2 += pow((double)e, 2.0);
			
			// Max absolute error and worst joint
			const float abs_e = std::abs(e);
			if (abs_e > max_abs_e) { max_abs_e = abs_e; worstJ = i; }

			s.j[i] = jt; // store joint telemetry
		}

		// Error statistics
		s.err_rms = (n > 0) ? (float)std::sqrt(sum_e2 / (double)n) : 0.0f; // RMS error
		s.err_max = max_abs_e;	// max error
		s.worst_joint = worstJ; // index of worst joint
		s.clamp_sum = clampSum; // total clamping events

		// Finalize write
		ring.endWrite();
	}
		
} // namespace diagnostics