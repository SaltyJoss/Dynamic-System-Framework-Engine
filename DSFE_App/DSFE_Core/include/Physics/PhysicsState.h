/*
 * File: Physics/PhysicsState.h
 * Created by: Joss Salton, 26-07-2026
 */
#pragma once
#pragma warning(disable : 4251)

#include "EngineCore.h"
#include <core/Types.h>
#include <core/constants.h>

namespace physics {
	// public struct for physical states
	struct PhysicsState {
		mathlib::Quat q = mathlib::Quat::Identity();			 // default to no rotation
		mathlib::Vec3 linearVelocity = mathlib::Vec3::Zero();  // default to no movement
		mathlib::Vec3 angularVelocity = mathlib::Vec3::Zero(); // default to no rotation

		double gravity = 0.0; // default to zero-g
		double mass = 1.0;	  // default to 1kg
		double damping = 0.0; // default to no damping

		mathlib::Mat3 inertia = mathlib::Mat3::Identity(); // default to identity inertia tensor
		mathlib::Vec3 forces = mathlib::Vec3::Zero();		 // default to no forces
		mathlib::Vec3 position = mathlib::Vec3::Zero();	 // default to origin
		mathlib::Vec3 torques = mathlib::Vec3::Zero();	 // default to no torques
	};
} // namespace physics