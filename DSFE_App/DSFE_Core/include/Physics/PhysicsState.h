#pragma once
// File:   PhysicsState.h
// GitHub: SaltyJoss
#pragma warning(disable : 4251)
#include "EngineCore.h"
#include <core/constants.h>
#include "PhysicsSystem.h"

namespace physics {
	// public struct for physical states
	struct DSFE_API PhysicsState {
		Quat q = Quat::Identity();			 // default to no rotation
		Vec3 linearVelocity = Vec3::Zero();  // default to no movement
		Vec3 angularVelocity = Vec3::Zero(); // default to no rotation

		double gravity = 0.0; // default to zero-g
		double mass = 1.0;	  // default to 1kg
		double damping = 0.0; // default to no damping

		Mat3 inertia = Mat3::Identity(); // default to identity inertia tensor
		Vec3 forces = Vec3::Zero();		 // default to no forces
		Vec3 position = Vec3::Zero();	 // default to origin
		Vec3 torques = Vec3::Zero();	 // default to no torques
	};
} // namespace physics