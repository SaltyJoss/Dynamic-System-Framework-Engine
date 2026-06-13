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
		mathlib::Vec3 p{ 0.0 };   // Position of the body in 3D space (meters)
		mathlib::Vec3 pd{ 0.0 };  // Velocity of the body in 3D space (meters/second)
		mathlib::Vec3 pdd{ 0.0 }; // Acceleration of the body in 3D space (meters/second^2)
		mathlib::Quat q{ 1.0, 0.0, 0.0, 0.0 }; // Orientation of the body (quaternion)
		mathlib::Vec3 w{ 0.0 };   // Angular velocity of the body (rads/second)
		mathlib::Vec3 wd{ 0.0 };  // Angular acceleration of the body (rads/second^2)
	};

	struct BodyLimits {
		mathlib::Vec3 pdMin{ 0.0 };			   // Minimum velocity limits (xd, yd, zd)
		mathlib::Vec3 pdMax{ constants::c_0 }; // Maximum velocity limits (xd, yd, zd)
		mathlib::Vec3 pddMin{ 0.0 }; // Minimum acceleration limits (xdd, ydd, zdd)
		mathlib::Vec3 pddMax;		 // Maximum acceleration limits (xdd, ydd, zdd)
		mathlib::Vec3 wMin{ 0.0 };			   // Minimum angular velocity limits (wx, wy, wz)
		mathlib::Vec3 wMax{ constants::c_0 }; // Maximum angular velocity limits (wx, wy, wz)
	};

	struct Body {
		std::string name = "UnnamedBody";
		float scale = 1.0f;

		BodyInertia inertia;
		DynamicsState state;
		BodyLimits limits;
	};
}