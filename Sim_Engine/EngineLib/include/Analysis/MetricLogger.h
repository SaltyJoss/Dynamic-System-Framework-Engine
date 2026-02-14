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
		// Limit flags and info
        std::vector<double> clamp_theta;
        std::vector<double> clamp_omega;
        std::vector<double> sat_flag;
        // Joint Index 
        std::vector<int> joint_index;

		// Clear all logged data
        void clear() {
			// Sim Metadata
            sim_time.clear();
            dt_taken.clear();
            dt_sug.clear();
			// States
            theta.clear();
            omega.clear();
            alpha.clear();
            err.clear();
            err_d.clear();
			// Dynamics
            I_eff.clear();
            tau.clear();
            tau_fb.clear();
            tau_coriolis.clear();
            tau_gravity.clear();
            tau_damping.clear();
            tau_friction.clear();
            tau_barrier.clear();
            tau_sat.clear();
			// Limit flags and info
            clamp_theta.clear();
            clamp_omega.clear();
            sat_flag.clear();
            // Joint Index
            joint_index.clear();
        }

		// Reserve space for a certain number of samples
        void reserve(size_t n) {
			// Sim Metadata reserve
            sim_time.reserve(n);
            dt_taken.reserve(n);
            dt_sug.reserve(n);
			// States reserve
            theta.reserve(n);
            omega.reserve(n);
            alpha.reserve(n);
            err.reserve(n);
            err_d.reserve(n);
			// Dynamics reserve
            I_eff.reserve(n);
            tau.reserve(n);
            tau_fb.reserve(n);
            tau_coriolis.reserve(n);
            tau_gravity.reserve(n);
            tau_damping.reserve(n);
            tau_friction.reserve(n);
            tau_barrier.reserve(n);
            tau_sat.reserve(n);
			// Limit flags and info reserve
            clamp_theta.reserve(n);
            clamp_omega.reserve(n);
            sat_flag.reserve(n);
			// Joint index reserve
            joint_index.reserve(n);
        }

		// Get the number of logged samples (assuming all vectors are the same size)
        size_t size() const { 
            return theta.size();
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