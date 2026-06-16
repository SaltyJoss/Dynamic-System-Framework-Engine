// PxM/PhysLib SingleBodySystems/SingleBodySystem.h
#pragma once

#include <core/MathLib.h>
#include <numbers>

using namespace constants;

namespace single_body_system {
	// Inertia struct representing the mass and inertia tensor of a rigid body
	struct BodyInertia {
		double mass = 1.0;
		mathlib::Vec3 com_xyz = mathlib::Vec3::Zero();; // Center of mass position in the body frame
		mathlib::Mat3 inertiaTensor = mathlib::Mat3::Zero();; // Inertia tensor in the body frame
	};

	struct DynamicsState {
		mathlib::Vec3 p = mathlib::Vec3::Zero();	   // Position of the body in 3D space (meters)
		mathlib::Vec3 pd = mathlib::Vec3::Zero();	   // Velocity of the body in 3D space (meters/second)
		mathlib::Vec3 pdd = mathlib::Vec3::Zero();	   // Acceleration of the body in 3D space (meters/second^2)
		mathlib::Quat q = mathlib::Quat::Identity(); // Orientation of the body as a quaternion
		mathlib::Vec3 w = mathlib::Vec3::Zero();	   // Angular velocity of the body (rads/second)
		mathlib::Vec3 wd = mathlib::Vec3::Zero();	   // Angular acceleration of the body (rads/second^2)
	};

	struct BodyLimits {
		mathlib::Vec3 pdMin = mathlib::Vec3::Zero();  // Minimum velocity limits (xd, yd, zd)
		mathlib::Vec3 pdMax = mathlib::Vec3(c_0, c_0, c_0); // Maximum velocity limits (xd, yd, zd)
		mathlib::Vec3 pddMin = mathlib::Vec3::Zero(); // Minimum acceleration limits (xdd, ydd, zdd)
		mathlib::Vec3 pddMax = mathlib::Vec3::Zero(); // Maximum acceleration limits (xdd, ydd, zdd)
		mathlib::Vec3 wMin = mathlib::Vec3::Zero();   // Minimum angular velocity limits (wx, wy, wz)
		mathlib::Vec3 wMax = mathlib::Vec3(c_0, c_0, c_0); // Maximum angular velocity limits (wx, wy, wz)
	};

	struct Body {
		std::string name = "UnnamedBody";
		float scale = 1.0f;

		BodyInertia inertia;
		DynamicsState state;
		BodyLimits limits;
	};
}