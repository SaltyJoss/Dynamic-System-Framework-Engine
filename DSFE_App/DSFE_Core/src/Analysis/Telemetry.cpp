/*
 * File: Analysis/Telemetry.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"

#include "Analysis/Telemetry.h"
#include "Systems/RigidBodySystem.h"
#include "Systems/TrajectoryManager.h"

namespace diagnostics {
	// Convert a FreeBodyLogEntry to FreeBodyTelemetry
	static FreeBodyTelemetry toTelemetry(const systems::FreeBodyLogBuffer::FreeBodyLogEntry& e) {
		FreeBodyTelemetry t;
		t.pos_x=e.pos_x; t.pos_y=e.pos_y; t.pos_z=e.pos_z;
		t.quat_w=e.quat_w; t.quat_x=e.quat_x; t.quat_y=e.quat_y; t.quat_z=e.quat_z;
		t.linVel_x=e.linVel_x; t.linVel_y=e.linVel_y; t.linVel_z=e.linVel_z;
		t.angVel_x=e.angVel_x; t.angVel_y=e.angVel_y; t.angVel_z=e.angVel_z;
		t.linAcc_x=e.linAcc_x; t.linAcc_y=e.linAcc_y; t.linAcc_z=e.linAcc_z;
		t.angAcc_x=e.angAcc_x; t.angAcc_y=e.angAcc_y; t.angAcc_z=e.angAcc_z;
		t.F_net_x=e.F_net_x; t.F_net_y=e.F_net_y; t.F_net_z=e.F_net_z;
		t.tau_net_x=e.tau_net_x; t.tau_net_y=e.tau_net_y; t.tau_net_z=e.tau_net_z;
		t.KE=e.KE; t.PE=e.PE; t.E_total=e.E_total;
		t.linMom_x=e.linMom_x; t.linMom_y=e.linMom_y; t.linMom_z=e.linMom_z;
		t.angMom_x=e.angMom_x; t.angMom_y=e.angMom_y; t.angMom_z=e.angMom_z;
		t.sleepState=(e.sleep_state>0.5);
		t.mass=e.mass; t.inv_mass=(e.mass>0?1.0/e.mass:0.0);
		t.Ixx=e.Ixx; t.Iyy=e.Iyy; t.Izz=e.Izz;
		t.inv_Ixx=(e.Ixx>0?1.0/e.Ixx:0.0); t.inv_Iyy=(e.Iyy>0?1.0/e.Iyy:0.0); t.inv_Izz=(e.Izz>0?1.0/e.Izz:0.0);
		t.body_index=e.body_index;
		return t;
	}

	// Record telemetry data at time t
	void TelemetryRecorder::record(double t, const systems::RigidBodySystem& sys, const control::TrajectoryManager* trajOpt, eTelemetryLevel /*level*/) {
		const int n = (int)sys.getRigidBody().joints().size();

		// Begin write
		TelemetrySample& s = ring.beginWrite();
		s.timeSec = t;

		// Resize joint vector (if needed)
		const auto& joints = sys.getRigidBody().joints();
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
			const auto& j = sys.getRigidBody().joints()[i];

			JointTelemetry jt;

			// Joint data
			jt.q   = j.q;
			jt.qd  = j.qd;

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
			jt.clampOmega = (mathlib::abs<double>(j.qd) >= j.limits.maxqd);

			// Accumulate clamping events
			clampThetaSum += (int)jt.clampTheta;
			clampOmegaSum += (int)jt.clampOmega;
			clampSum += (int)jt.clampTheta + (int)jt.clampOmega;

			// Trajectory data
			if (trajOpt) {
				control::TrajState<double> ts{};
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

		if (sys.hasFreeJoint()) {
			s.fb.clear();
			systems::FreeBodyLogBuffer::FreeBodyLogEntry e{};
			int bodyIdx = 0;
			while (sys.getRigidBody().latestFreeBodyEntry(e, bodyIdx)) {
				s.fb.push_back(toTelemetry(e));
				++bodyIdx;
			}
		}

		// Error statistics
		s.err_rms	  = (n > 0) ? mathlib::sqrt<double>(sum_e2 / (double)n) : 0.0f; // RMS error
		s.err_max	  = max_abs_e;	   // max error
		s.clamp_theta = clampThetaSum; // total angle clamping events
		s.clamp_omega = clampOmegaSum; // total velocity clamping events
		s.clamp_sum	  = clampSum;	   // total clamping events
		s.worst_joint = worstJ;		   // index of worst joint

		// Finalize write
		ring.endWrite();
	}
		
} // namespace diagnostics