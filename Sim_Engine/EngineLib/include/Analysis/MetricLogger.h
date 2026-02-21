#pragma once
#include "EngineCore.h"
#include <vector>
#include <string>

namespace robots {
	// Struct for logging joint data each step (for later analysis)
    struct ENGINE_API JointLogBuffer {
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

        struct ENGINE_API JointLogEntry {
            double sim_time;
            double dt_taken;
            double dt_sug;
            double theta, omega, alpha, err, err_d;
            double I_eff, tau, tau_fb, tau_coriolis, tau_gravity, tau_damping, tau_friction, tau_barrier, tau_sat;
            double KE, PE, E_total, W_actuator, P_damping, P_friction;
            double clamp_theta, clamp_omega, sat_flag;
            int joint_index;
        };

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
    };

	// Struct for logging reference trajectory data
    struct ENGINE_API TrajRefBuffer {
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