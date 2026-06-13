// PxM/PhysLib SingleBodySystems/SingleBodySystem.h
#pragma once

#include <core/MathLib.h>
#include <numbers>

using namespace constants;

namespace single_body_system {
	// Inertia struct representing the mass and inertia tensor of a rigid body
	struct BodyInertia {
		double mass = 0.0;
		mathlib::Vec3 com_xyz{ 0.0, 0.0, 0.0 }; // Center of mass position in the body frame
		mathlib::Mat3 inertiaTensor; // 3x3 inertia tensor matrix
	};

	struct DynamicsState {
		mathlib::Vec3 x;   // Position of the body in 3D space
		mathlib::Vec3 xd;  // Velocity of the body in 3D space
		mathlib::Vec3 xdd; // Acceleration of the body in 3D space
		mathlib::Quat q;   // Orientation of the body (quaternion)
		mathlib::Vec3 w;   // Angular velocity of the body
		mathlib::Vec3 wd;  // Angular acceleration of the body
	};

	struct BodyLimits {
		mathlib::Vec3 xdMin{ 0.0 };			   // Minimum velocity limits (xd, yd, zd)
		mathlib::Vec3 xdMax{ constants::c_0 }; // Maximum velocity limits (xd, yd, zd)
		mathlib::Vec3 xddMin{ 0.0 }; // Minimum acceleration limits (xdd, ydd, zdd)
		mathlib::Vec3 xddMax;		 // Maximum acceleration limits (xdd, ydd, zdd)
	};

	struct SingleBodySystem {
		std::string name = "UnnamedBody";
		float scale = 1.0f;

		BodyInertia inertia;
		DynamicsState state;
		BodyLimits limits;
	};
}