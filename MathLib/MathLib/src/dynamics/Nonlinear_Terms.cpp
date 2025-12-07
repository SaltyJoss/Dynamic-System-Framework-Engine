#include "pch.h"
#include "dynamics/Nonlinear_Terms.h"

namespace dynamics {
	/// <inheritdoc/>
	VecX Nonlinear_Terms::coriolisCentrifugal(const std::vector<DH_Param>& dh_p, const std::vector<LinkInertia>& inertias, const VecX& q, const VecX& q_dot) {
		size_t n = dh_p.size();
		VecX C = VecX::Zero(n);
		// Placeholder implementation: In a real scenario, this would involve complex calculations
		// based on the robot's kinematics and dynamics.
		for (size_t i = 0; i < n; ++i) {
			C(i) = 0.1 * q_dot(i); // Simplified linear relation
		}
		return C; // Return the computed nonlinear terms
	}

	/// <inheritdoc/>
	VecX Nonlinear_Terms::gravityTorque(const std::vector<DH_Param>& dh_p, const std::vector<LinkInertia>& inertias, const VecX& q, const Vec3& gravity) {
		size_t n = dh_p.size();
		VecX G = VecX::Zero(n);
		// Placeholder implementation: In a real scenario, this would involve complex calculations
		// based on the robot's kinematics and dynamics.
		for (size_t i = 0; i < n; ++i) {
			G(i) = inertias[i].mass * gravity(2) * 0.5; // Simplified gravity torque
		}
		return G; // Return the computed gravity torque vector
	}
}