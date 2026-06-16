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

		mathlib::VecX derivatives(Body& body, const mathlib::VecX& x, mathlib::Vec3& F_ext, mathlib::Vec3& tau_ext) {
			mathlib::VecX dxdt(13);
			dxdt.setZero();

			mathlib::Vec3 p = x.block<3, 1>(0, 0);
			mathlib::Vec3 pd = x.block<3, 1>(3, 0);
			mathlib::Vec4 qv = x.block<4, 1>(6, 0);
			mathlib::Quat q(qv);
			mathlib::Vec3 w = x.block<3, 1>(10, 0);

			const double m = body.inertia.mass;

			mathlib::Vec3 pdd;
			if (m <= 1e-12) { pdd = mathlib::Vec3(0.0, 0.0, 0.0); }
			else { pdd = F_ext / m; }

			mathlib::Vec3 wd = mathlib::Vec3::Zero();
			mathlib::Mat3 I = body.inertia.inertiaTensor;
			mathlib::Mat3 I_inv = I.inverse();

			if (I.determinant() > 1e-12) {
				wd = I_inv * (tau_ext - w.cross(I * w));
			}

			dxdt.block<3, 1>(0, 0) = pd; // dp/dt = v
			dxdt.block<3, 1>(3, 0) = pdd; // dpd/dt = a

			mathlib::Quat dq;
			mathlib::Vec3 omega = w;

			mathlib::Quat omega_quat(0.0, omega.x(), omega.y(), omega.z());
			dq.coeffs() = 0.5 * (omega_quat * q).coeffs();

			dxdt.block<4, 1>(6, 0) = dq.coeffs(); // dq/dt = 0.5 * q * w
			dxdt.block<3, 1>(10, 0) = wd;

			return dxdt;
		}

		void jacobian(Body& body, const mathlib::VecX& x, mathlib::MatX& J_out) {
			auto p = x.block<3, 1>(0, 0); // Position
			auto pd = x.block<3, 1>(3, 0); // Velocity
			auto qv = x.block<4, 1>(6, 0); // Quaternion
			auto w = x.block<3, 1>(10, 0); // Angular velocity
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