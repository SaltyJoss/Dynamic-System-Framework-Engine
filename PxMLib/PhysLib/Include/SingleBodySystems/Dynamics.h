// PxM/PhysLib SingleBodySystems/Dynamics.h
#pragma once
#include "SingleBodySystem.h"

namespace single_body_system::dynamics {
	class SingleBodyDynamics {
	public:
		void buildInertiaTensor(Body& body) {
			// Compute the inertia tensor based on the body's mass and center of mass
			double m = body.inertia.mass;
			mathlib::Vec3 com = body.inertia.com_xyz;
			// Inertia tensor for a point mass at the center of mass
			mathlib::Mat3 I = mathlib::Mat3::Zero(); // Local inertia tensor
			I <<
				(1.0 / 12.0) * m * (com.y() * com.y() + com.z() * com.z()), 0, 0,
				0, (1.0 / 12.0)* m * (com.x() * com.x() + com.z() * com.z()), 0,
				0, 0, (1.0 / 12.0)* m * (com.x() * com.x() + com.y() * com.y());
			body.inertia.inertiaTensor = I; // Assign the computed inertia tensor to the body
		}

		void buildParticleInertia(Body& body) {
			// For a particle, in the simple case the body is treated as a point mass, so the inertia tensor is zero
			body.inertia.inertiaTensor = mathlib::Mat3::Zero();
		}

		mathlib::VecX derivatives(Body& body, const mathlib::VecX& x, mathlib::Vec3& F_ext, mathlib::Vec3& tau_ext, double dt) {
			mathlib::VecX dxdt(13);

			mathlib::Vec3 p = x.block<3, 1>(0, 0);
			mathlib::Vec3 pd = x.block<3, 1>(3, 0);
			mathlib::Vec4 v = x.block<4, 1>(6, 0);
			mathlib::Quat q_coeffs(v);
			mathlib::Vec3 w = x.block<3, 1>(10, 0);

			if (F_ext == mathlib::Vec3::Zero()) { body.state.pdd = F_ext; }
			else { body.state.pdd = (F_ext / body.inertia.mass).eval(); }

			// Update linear velocity and position
			body.state.pd = body.state.pdd * dt;
			body.state.p = body.state.pd * dt;

			if (tau_ext == mathlib::Vec3::Zero()) {
				body.state.wd = tau_ext;
				body.state.w = tau_ext;
			}
			else {
				mathlib::Mat3 inertiaInv = body.inertia.inertiaTensor.inverse();
				body.state.wd = inertiaInv * (tau_ext - body.state.w.cross(body.inertia.inertiaTensor * body.state.w));
				body.state.w = body.state.wd * dt;
			}

			// Update orientation quaternion
			if (q_coeffs.norm() > 1e-12) {
				body.state.q = mathlib::Quat(q_coeffs).normalized();
			}
			else {
				body.state.q.coeffs() = mathlib::Quat(1.0, 0.0, 0.0, 0.0).coeffs();
			}

			double theta = body.state.w.norm() * dt;
			if (theta > 1e-12) {
				mathlib::Quat dq(Eigen::AngleAxis(theta, body.state.w.normalized()));
				body.state.q = (body.state.q * dq).normalized();
			}

			dxdt.block<3, 1>(0, 0) = body.state.pd; // dp/dt = pd
			dxdt.block<3, 1>(3, 0) = body.state.pdd; // dpd/dt = pdd
			dxdt.block<4, 1>(6, 0) = body.state.q.coeffs(); // dq/dt = q (quaternion)
			dxdt.block<3, 1>(10, 0) = body.state.w; // dw/dt = w

			return dxdt;
		}

		void jacobian(Body& body, const mathlib::VecX& x, mathlib::MatX& J_out) {
			J_out.setZero(13, 13);
			// Partial derivatives for position and velocity
			J_out.block<3, 3>(0, 3) = mathlib::Mat3::Identity(); // dp/dt = pd
			J_out.block<3, 3>(3, 10) = mathlib::Mat3::Identity(); // dpd/dt = pdd
			// Partial derivatives for orientation (quaternion)
			J_out.block<4, 4>(6, 6) = mathlib::Mat4::Identity(); // dq/dt = q
			// Partial derivatives for angular velocity
			mathlib::Mat3 inertiaInv = body.inertia.inertiaTensor.inverse();
			J_out.block<3, 3>(10, 10) = inertiaInv; // dw/dt = inertiaInv * (tau_ext - w x (I * w))
		}
	};
}