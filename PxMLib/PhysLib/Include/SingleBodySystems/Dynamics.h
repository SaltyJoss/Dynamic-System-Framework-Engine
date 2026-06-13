// PxM/PhysLib SingleBodySystems/Dynamics.h
#pragma once
#include "SingleBodySystem.h"

namespace single_body_system::dynamics {
	class SingleBodyDynamics {
	public:
		void computeDynamics(Body& body, mathlib::Vec3& externalForce, mathlib::Vec3& externalTorque, double dt) {
			// Compute linear acceleration
			body.state.xdd = (externalForce / body.inertia.mass).eval();
			// Compute angular acceleration
			mathlib::Mat3 inertiaInv = body.inertia.inertiaTensor.inverse();
			body.state.wd = inertiaInv * (externalTorque - body.state.w.cross(body.inertia.inertiaTensor * body.state.w));
			// Update linear velocity and position
			body.state.xd += body.state.xdd * dt;
			body.state.x += body.state.xd * dt;
			// Update angular velocity and orientation (using quaternion integration)
			body.state.w += body.state.wd * dt;
			mathlib::Quat dq = mathlib::Quat(0.0, body.state.w * dt);
			body.state.q = (body.state.q.coeffs() + (0.5 * dq.coeffs() * body.state.q.coeffs())).normalized();
		}

		void derivatives(const Body& body, mathlib::Vec3& externalForce, mathlib::Vec3& externalTorque, mathlib::Mat3& dxdot_dx, mathlib::Mat3& dxdot_dxdot) {
			// Compute the Jacobian of the dynamics with respect to state variables
			dxdot_dx.setZero();
			dxdot_dxdot.setZero();
			// Linear acceleration derivatives
			dxdot_dx.block<3, 3>(0, 0) = mathlib::Mat3::Zero(); // dx/dx = 0
			dxdot_dxdot.block<3, 3>(0, 3) = mathlib::Mat3::Identity() / body.inertia.mass; // dx/dxdot = 1/mass
			// Angular acceleration derivatives
			mathlib::Mat3 inertiaInv = body.inertia.inertiaTensor.inverse();
			dxdot_dx.block<3, 3>(3, 6) = -inertiaInv * skewSymmetric(body.state.w) * body.inertia.inertiaTensor; // dw/dq
			dxdot_dxdot.block<3, 3>(3, 9) = inertiaInv; // dw/dwd = I^-1
		}
	};
}