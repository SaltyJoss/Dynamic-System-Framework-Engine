/*
 * File: Analysis/MetricLogger.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once
#include "EngineCore.h"
#include <vector>
#include <string>

namespace systems {
	// Struct for logging joint data each step (for later analysis)
    struct JointLogBuffer {
		// Sim Metadata
        std::vector<double> sim_time;
        std::vector<double> dt_taken;
        std::vector<double> dt_sug;
        // States
        std::vector<double> theta;
        std::vector<double> omega;
        std::vector<double> alpha;
        std::vector<double> err;
        std::vector<double> err_d;
        // Dynamics
        std::vector<double> I_eff;
        std::vector<double> tau;
        std::vector<double> tau_fb;
        std::vector<double> tau_ff;
        std::vector<double> tau_coriolis;
        std::vector<double> tau_gravity;
        std::vector<double> tau_damping;
        std::vector<double> tau_friction;
        std::vector<double> tau_barrier;
        std::vector<double> tau_sat;
        // Energy, Work, & Power
		std::vector<double> KE;
		std::vector<double> PE;
		std::vector<double> E_total;
        std::vector<double> W_actuator;
		std::vector<double> P_damping;
		std::vector<double> P_friction;
		// Limit flags and info
        std::vector<double> clamp_theta;
        std::vector<double> clamp_omega;
        std::vector<double> sat_flag;
        // Joint Index 
        std::vector<int> joint_index;

		// Clear all logged data
        void clear() {
            sim_time.clear();
            dt_taken.clear();
            dt_sug.clear();
            theta.clear();
            omega.clear();
            alpha.clear();
            err.clear();
            err_d.clear();
            I_eff.clear();
            tau.clear();
            tau_fb.clear();
            tau_ff.clear();
            tau_coriolis.clear();
            tau_gravity.clear();
            tau_damping.clear();
            tau_friction.clear();
            tau_barrier.clear();
            tau_sat.clear();
			KE.clear();
			PE.clear();
			E_total.clear();
            W_actuator.clear();
			P_damping.clear();
			P_friction.clear();
            clamp_theta.clear();
            clamp_omega.clear();
            sat_flag.clear();
            joint_index.clear();
        }

		// Reserve space for a certain number of samples
        void reserve(size_t n) {
            sim_time.reserve(n);
            dt_taken.reserve(n);
            dt_sug.reserve(n);
            theta.reserve(n);
            omega.reserve(n);
            alpha.reserve(n);
            err.reserve(n);
            err_d.reserve(n);
            I_eff.reserve(n);
            tau.reserve(n);
            tau_fb.reserve(n);
            tau_ff.reserve(n);
            tau_coriolis.reserve(n);
            tau_gravity.reserve(n);
            tau_damping.reserve(n);
            tau_friction.reserve(n);
            tau_barrier.reserve(n);
            tau_sat.reserve(n);
			KE.reserve(n);
			PE.reserve(n);
			E_total.reserve(n);
            W_actuator.reserve(n);
			P_damping.reserve(n);
			P_friction.reserve(n);
            clamp_theta.reserve(n);
            clamp_omega.reserve(n);
            sat_flag.reserve(n);
            joint_index.reserve(n);
        }

		// Get the number of logged samples (assuming all vectors are the same size)
        size_t size() const { 
            return theta.size();
        }

		// Struct representing a single log entry for a joint at a specific time step
        struct DSFE_API JointLogEntry {
            double sim_time;
            double dt_taken;
            double dt_sug;
            double theta, omega, alpha, err, err_d;
            double I_eff;
            double tau, tau_fb, tau_ff;
            double tau_coriolis, tau_gravity, tau_damping, tau_friction;
            double tau_barrier, tau_sat;
            double KE, PE, E_total, W_actuator, P_damping, P_friction;
            double clamp_theta, clamp_omega, sat_flag;
            int joint_index;
        };

		// Push a new log entry into the buffer
        void push_entry(const JointLogEntry& e) {
            sim_time.push_back(e.sim_time);
            dt_sug.push_back(e.dt_sug);
            dt_taken.push_back(e.dt_taken);
            theta.push_back(e.theta);
            omega.push_back(e.omega);
            alpha.push_back(e.alpha);
            err.push_back(e.err);
            err_d.push_back(e.err_d);
            I_eff.push_back(e.I_eff);
            tau.push_back(e.tau);
            tau_fb.push_back(e.tau_fb);
			tau_ff.push_back(e.tau_ff);
            tau_coriolis.push_back(e.tau_coriolis);
            tau_gravity.push_back(e.tau_gravity);
            tau_damping.push_back(e.tau_damping);
            tau_friction.push_back(e.tau_friction);
            tau_barrier.push_back(e.tau_barrier);
            tau_sat.push_back(e.tau_sat);
            KE.push_back(e.KE);
            PE.push_back(e.PE);
            E_total.push_back(e.E_total);
            W_actuator.push_back(e.W_actuator);
            P_damping.push_back(e.P_damping);
            P_friction.push_back(e.P_friction);
            clamp_theta.push_back(e.clamp_theta);
            clamp_omega.push_back(e.clamp_omega);
            sat_flag.push_back(e.sat_flag);
            joint_index.push_back(e.joint_index);
        }

		// Validate that all vectors have the same size
        bool validate(std::string* outMsg) const {
			size_t n = theta.size(); // ref size
			// Lambda to check size of each vector against n
            auto checkSize = [&](const auto& v, const char* name) -> bool {
                if (v.size() != n) {
                    if (outMsg) { *outMsg = "Size mismatch for " + std::string(name) + ": expected " + std::to_string(n) + ", got " + std::to_string(v.size()); }
                    return false;
                }
                return true;
            };
			// Check that all vectors have the same size
			if (!checkSize(sim_time, "sim_time")) return false;
			if (!checkSize(dt_taken, "dt_taken")) return false;
			if (!checkSize(dt_sug, "dt_sug")) return false;
			if (!checkSize(theta, "theta")) return false;
			if (!checkSize(omega, "omega")) return false;
			if (!checkSize(alpha, "alpha")) return false;
			if (!checkSize(err, "err")) return false;
			if (!checkSize(err_d, "err_d")) return false;
			if (!checkSize(I_eff, "I_eff")) return false;
			if (!checkSize(tau, "tau")) return false;
			if (!checkSize(tau_fb, "tau_fb")) return false;
			if (!checkSize(tau_ff, "tau_ff")) return false;
			if (!checkSize(tau_coriolis, "tau_coriolis")) return false;
			if (!checkSize(tau_gravity, "tau_gravity")) return false;
			if (!checkSize(tau_damping, "tau_damping")) return false;
			if (!checkSize(tau_friction, "tau_friction")) return false;
			if (!checkSize(tau_barrier, "tau_barrier")) return false;
			if (!checkSize(tau_sat, "tau_sat")) return false;
			if (!checkSize(KE, "KE")) return false;
			if (!checkSize(PE, "PE")) return false;
			if (!checkSize(E_total, "E_total")) return false;
			if (!checkSize(W_actuator, "W_actuator")) return false;
			if (!checkSize(P_damping, "P_damping")) return false;
			if (!checkSize(P_friction, "P_friction")) return false;
			if (!checkSize(clamp_theta, "clamp_theta")) return false;
			if (!checkSize(clamp_omega, "clamp_omega")) return false;
			if (!checkSize(sat_flag, "sat_flag")) return false;
			if (!checkSize(joint_index, "joint_index")) return false;
			return true; // all sizes match
        }

        void swap(JointLogBuffer& other) noexcept {
            sim_time.swap(other.sim_time);
            dt_taken.swap(other.dt_taken);
            dt_sug.swap(other.dt_sug);
            theta.swap(other.theta);
            omega.swap(other.omega);
            alpha.swap(other.alpha);
            err.swap(other.err);
            err_d.swap(other.err_d);
            I_eff.swap(other.I_eff);
            tau.swap(other.tau);
            tau_fb.swap(other.tau_fb);
			tau_ff.swap(other.tau_ff);
            tau_coriolis.swap(other.tau_coriolis);
            tau_gravity.swap(other.tau_gravity);
            tau_damping.swap(other.tau_damping);
            tau_friction.swap(other.tau_friction);
            tau_barrier.swap(other.tau_barrier);
            tau_sat.swap(other.tau_sat);
            KE.swap(other.KE);
            PE.swap(other.PE);
            E_total.swap(other.E_total);
            W_actuator.swap(other.W_actuator);
            P_damping.swap(other.P_damping);
            P_friction.swap(other.P_friction);
            clamp_theta.swap(other.clamp_theta);
            clamp_omega.swap(other.clamp_omega);
            sat_flag.swap(other.sat_flag);
            joint_index.swap(other.joint_index);
        }
    };

	// Struct for logging reference trajectory data
    struct TrajRefBuffer {
		// Sim Metadata
        std::vector<double> sim_time;
		// Reference states
        std::vector<double> theta_ref;
		std::vector<double> omega_ref;
		std::vector<double> alpha_ref;
		// Joint Index
        std::vector<int> joint_index;

		// Clear all logged data
        void clear() {
            // Sim Metadata
			sim_time.clear();
			// Reference states
			theta_ref.clear();
			omega_ref.clear();
			alpha_ref.clear();
			// Joint Index
            joint_index.clear();
        }

		// Reserve space for a certain number of samples
        void reserve(size_t n) {
            // Sim Metadata reserve
            sim_time.reserve(n);
			// Reference states reserve
			theta_ref.reserve(n);
			omega_ref.reserve(n);
			alpha_ref.reserve(n);
			// Joint index reserve
			joint_index.reserve(n);
        }

		// Get the number of logged samples (assuming all vectors are the same size)
        size_t size() const {
            return theta_ref.size();
		}
    };

    // Analysis/MetricLogger.h — add alongside JointLogBuffer, TrajRefBuffer
	// Struct for logging free-body (6-DOF floating base) data each step
	struct FreeBodyLogBuffer {
		// Sim Metadata
		std::vector<double> sim_time;
		std::vector<double> dt_taken;
		std::vector<double> dt_sug;
		// State — position (3), orientation quat (4), linear vel (3), angular vel (3)
		std::vector<double> pos_x, pos_y, pos_z;
		std::vector<double> quat_w, quat_x, quat_y, quat_z;
		std::vector<double> linVel_x, linVel_y, linVel_z;
		std::vector<double> angVel_x, angVel_y, angVel_z;
		// Time derivatives — linear/angular accel (3 each), net force/torque (3 each)
		std::vector<double> linAcc_x, linAcc_y, linAcc_z;
		std::vector<double> angAcc_x, angAcc_y, angAcc_z;
		std::vector<double> F_net_x, F_net_y, F_net_z;
		std::vector<double> tau_net_x, tau_net_y, tau_net_z;
		// Energy & momenta — KE, PE scalars; linear/angular momentum (3 each)
		std::vector<double> KE;
		std::vector<double> PE;
		std::vector<double> E_total;
		std::vector<double> linMom_x, linMom_y, linMom_z;
		std::vector<double> angMom_x, angMom_y, angMom_z;
		// Mass & inertia — scalar mass
		std::vector<double> mass;
		std::vector<double> Ixx, Iyy, Izz; // Inertia tensor diagonal elements (3x3 matrix flattened, using the principal axes)
		// Sleep + body index (for many free bodies / particles)
		std::vector<double> sleep_state;
		std::vector<int> body_index;

		void clear() {
			sim_time.clear(); dt_taken.clear(); dt_sug.clear();
			pos_x.clear(); pos_y.clear(); pos_z.clear();
			quat_w.clear(); quat_x.clear(); quat_y.clear(); quat_z.clear();
			linVel_x.clear(); linVel_y.clear(); linVel_z.clear();
			angVel_x.clear(); angVel_y.clear(); angVel_z.clear();
			linAcc_x.clear(); linAcc_y.clear(); linAcc_z.clear();
			angAcc_x.clear(); angAcc_y.clear(); angAcc_z.clear();
			F_net_x.clear(); F_net_y.clear(); F_net_z.clear();
			tau_net_x.clear(); tau_net_y.clear(); tau_net_z.clear();
			KE.clear(); PE.clear(); E_total.clear();
			linMom_x.clear(); linMom_y.clear(); linMom_z.clear();
			angMom_x.clear(); angMom_y.clear(); angMom_z.clear();
			mass.clear(); Ixx.clear(); Iyy.clear(); Izz.clear();
			sleep_state.clear(); body_index.clear();
		}

		void reserve(size_t n) {
			sim_time.reserve(n); dt_taken.reserve(n); dt_sug.reserve(n);
			pos_x.reserve(n); pos_y.reserve(n); pos_z.reserve(n);
			quat_w.reserve(n); quat_x.reserve(n); quat_y.reserve(n); quat_z.reserve(n);
			linVel_x.reserve(n); linVel_y.reserve(n); linVel_z.reserve(n);
			angVel_x.reserve(n); angVel_y.reserve(n); angVel_z.reserve(n);
			linAcc_x.reserve(n); linAcc_y.reserve(n); linAcc_z.reserve(n);
			angAcc_x.reserve(n); angAcc_y.reserve(n); angAcc_z.reserve(n);
			F_net_x.reserve(n); F_net_y.reserve(n); F_net_z.reserve(n);
			tau_net_x.reserve(n); tau_net_y.reserve(n); tau_net_z.reserve(n);
			KE.reserve(n); PE.reserve(n); E_total.reserve(n);
			linMom_x.reserve(n); linMom_y.reserve(n); linMom_z.reserve(n);
			angMom_x.reserve(n); angMom_y.reserve(n); angMom_z.reserve(n);
			mass.reserve(n); Ixx.reserve(n); Iyy.reserve(n); Izz.reserve(n);
			sleep_state.reserve(n); body_index.reserve(n);
		}

		size_t size() const { return pos_x.size(); }

		// Single-sample entry — one free body at one time step
		struct DSFE_API FreeBodyLogEntry {
			double sim_time, dt_taken, dt_sug;
			double pos_x, pos_y, pos_z;
			double quat_w, quat_x, quat_y, quat_z;
			double linVel_x, linVel_y, linVel_z;
			double angVel_x, angVel_y, angVel_z;
			double linAcc_x, linAcc_y, linAcc_z;
			double angAcc_x, angAcc_y, angAcc_z;
			double F_net_x, F_net_y, F_net_z;
			double tau_net_x, tau_net_y, tau_net_z;
			double KE, PE, E_total;
			double linMom_x, linMom_y, linMom_z;
			double angMom_x, angMom_y, angMom_z;
			double mass, Ixx, Iyy, Izz;
			double sleep_state;
			int body_index;
		};

		void push_entry(const FreeBodyLogEntry& e) {
			sim_time.push_back(e.sim_time); dt_taken.push_back(e.dt_taken); dt_sug.push_back(e.dt_sug);
			pos_x.push_back(e.pos_x); pos_y.push_back(e.pos_y); pos_z.push_back(e.pos_z);
			quat_w.push_back(e.quat_w); quat_x.push_back(e.quat_x); quat_y.push_back(e.quat_y); quat_z.push_back(e.quat_z);
			linVel_x.push_back(e.linVel_x); linVel_y.push_back(e.linVel_y); linVel_z.push_back(e.linVel_z);
			angVel_x.push_back(e.angVel_x); angVel_y.push_back(e.angVel_y); angVel_z.push_back(e.angVel_z);
			linAcc_x.push_back(e.linAcc_x); linAcc_y.push_back(e.linAcc_y); linAcc_z.push_back(e.linAcc_z);
			angAcc_x.push_back(e.angAcc_x); angAcc_y.push_back(e.angAcc_y); angAcc_z.push_back(e.angAcc_z);
			F_net_x.push_back(e.F_net_x); F_net_y.push_back(e.F_net_y); F_net_z.push_back(e.F_net_z);
			tau_net_x.push_back(e.tau_net_x); tau_net_y.push_back(e.tau_net_y); tau_net_z.push_back(e.tau_net_z);
			KE.push_back(e.KE); PE.push_back(e.PE); E_total.push_back(e.E_total);
			linMom_x.push_back(e.linMom_x); linMom_y.push_back(e.linMom_y); linMom_z.push_back(e.linMom_z);
			angMom_x.push_back(e.angMom_x); angMom_y.push_back(e.angMom_y); angMom_z.push_back(e.angMom_z);
			mass.push_back(e.mass); Ixx.push_back(e.Ixx); Iyy.push_back(e.Iyy); Izz.push_back(e.Izz);
			sleep_state.push_back(e.sleep_state); body_index.push_back(e.body_index);
		}

        bool validate(std::string* outMsg) const {
            size_t n = pos_x.size(); // ref size
            auto checkSize = [&](const auto& v, const char* name) -> bool {
                if (v.size() != n) {
                    if (outMsg) { *outMsg = "Size mismatch for " + std::string(name) + ": expected " + std::to_string(n) + ", got " + std::to_string(v.size()); }
                    return false;
                }
                return true;
            };
            if (!checkSize(sim_time, "sim_time")) return false;
            if (!checkSize(dt_taken, "dt_taken")) return false;
            if (!checkSize(dt_sug, "dt_sug")) return false;
            if (!checkSize(pos_x, "pos_x")) return false;
            if (!checkSize(pos_y, "pos_y")) return false;
            if (!checkSize(pos_z, "pos_z")) return false;
            if (!checkSize(quat_w, "quat_w")) return false; 
            if (!checkSize(quat_x, "quat_x")) return false;
            if (!checkSize(quat_y, "quat_y")) return false;
            if (!checkSize(quat_z, "quat_z")) return false;
            if (!checkSize(linVel_x, "linVel_x")) return false;
            if (!checkSize(linVel_y, "linVel_y")) return false;
            if (!checkSize(linVel_z, "linVel_z")) return false;
            if (!checkSize(angVel_x, "angVel_x")) return false;
            if (!checkSize(angVel_y, "angVel_y")) return false;
            if (!checkSize(angVel_z, "angVel_z")) return false;
            if (!checkSize(linAcc_x, "linAcc_x")) return false;
            if (!checkSize(linAcc_y, "linAcc_y")) return false;
            if (!checkSize(linAcc_z, "linAcc_z")) return false;
            if (!checkSize(angAcc_x, "angAcc_x")) return false;
            if (!checkSize(angAcc_y, "angAcc_y")) return false;
            if (!checkSize(angAcc_z, "angAcc_z")) return false;
            if (!checkSize(F_net_x, "F_net_x")) return false;
            if (!checkSize(F_net_y, "F_net_y")) return false;
            if (!checkSize(F_net_z, "F_net_z")) return false;
            if (!checkSize(tau_net_x, "tau_net_x")) return false;
            if (!checkSize(tau_net_y, "tau_net_y")) return false;
            if (!checkSize(tau_net_z, "tau_net_z")) return false;
            if (!checkSize(KE, "KE")) return false;
            if (!checkSize(PE, "PE")) return false;
            if (!checkSize(E_total, "E_total")) return false;
            if (!checkSize(linMom_x, "linMom_x")) return false;
            if (!checkSize(linMom_y, "linMom_y")) return false;
            if (!checkSize(linMom_z, "linMom_z")) return false; 
            if (!checkSize(angMom_x, "angMom_x")) return false;
            if (!checkSize(angMom_y, "angMom_y")) return false; 
            if (!checkSize(angMom_z, "angMom_z")) return false;
            if (!checkSize(mass, "mass")) return false;
            if (!checkSize(Ixx, "Ixx")) return false;
            if (!checkSize(Iyy, "Iyy")) return false;
            if (!checkSize(Izz, "Izz")) return false;
            if (!checkSize(sleep_state, "sleep_state")) return false;
            if (!checkSize(body_index, "body_index")) return false;
            return true; // all sizes match
        }

        void swap(FreeBodyLogBuffer& other) noexcept {
            sim_time.swap(other.sim_time); dt_taken.swap(other.dt_taken); dt_sug.swap(other.dt_sug);
            pos_x.swap(other.pos_x); pos_y.swap(other.pos_y); pos_z.swap(other.pos_z);
            quat_w.swap(other.quat_w); quat_x.swap(other.quat_x); quat_y.swap(other.quat_y); quat_z.swap(other.quat_z);
            linVel_x.swap(other.linVel_x); linVel_y.swap(other.linVel_y); linVel_z.swap(other.linVel_z);
            angVel_x.swap(other.angVel_x); angVel_y.swap(other.angVel_y); angVel_z.swap(other.angVel_z);
            linAcc_x.swap(other.linAcc_x); linAcc_y.swap(other.linAcc_y); linAcc_z.swap(other.linAcc_z);
            angAcc_x.swap(other.angAcc_x); angAcc_y.swap(other.angAcc_y); angAcc_z.swap(other.angAcc_z);
            F_net_x.swap(other.F_net_x); F_net_y.swap(other.F_net_y); F_net_z.swap(other.F_net_z);
            tau_net_x.swap(other.tau_net_x); tau_net_y.swap(other.tau_net_y); tau_net_z.swap(other.tau_net_z);
            KE.swap(other.KE); PE.swap(other.PE); E_total.swap(other.E_total);
            linMom_x.swap(other.linMom_x); linMom_y.swap(other.linMom_y); linMom_z.swap(other.linMom_z);
            angMom_x.swap(other.angMom_x); angMom_y.swap(other.angMom_y); angMom_z.swap(other.angMom_z);
            mass.swap(other.mass); Ixx.swap(other.Ixx); Iyy.swap(other.Iyy); Izz.swap(other.Izz);
            sleep_state.swap(other.sleep_state); body_index.swap(other.body_index);
        }
	};
} // namespace robots