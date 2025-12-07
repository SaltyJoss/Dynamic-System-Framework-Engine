#include "pch.h"
#include "dynamics/StateSpace.h"

namespace dynamics {
	/// <inheritdoc/>
	VecX StateSpace::dynamicRHS(const std::vector<DH_Param>& dh_p, const std::vector<LinkInertia>& inertias, const VecX& q, const VecX& q_dot, const VecX& tau, const Vec3& gravity) {
		size_t n = dh_p.size();
		VecX rhs = VecX::Zero(n);

		// Placeholder implementation: When actually used I will define the would involve complex calculations based on the robot's kinematics and dynamics.
		// Not fully sure how to do that yet.
		// For demonstration, this just computes a simple relation

		for (size_t i = 0; i < n; ++i) {
			rhs(i) = tau(i) - 0.1 * q_dot(i) - inertias[i].mass * gravity(2) * 0.5; // Simplified dynamics
		}
		return rhs; // Return the computed dynamic right-hand side
	}
}