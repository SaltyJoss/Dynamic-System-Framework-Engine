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
} // namespace robots