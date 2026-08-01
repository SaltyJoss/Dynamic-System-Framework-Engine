/*
 * File: Physics/RigidBodyDynamics.cpp
 * Created by: Joss Salton, 26-07-2026
 */
#include "pch.h"
#include "Physics/RigidBodyDynamics.h"

namespace physics {
	// Constructor
	RigidBodyDynamics::RigidBodyDynamics() 
		: _kinematics(std::make_unique<RigidBodyKinematics>()) {
	}
} // namespace systems