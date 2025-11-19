#pragma once

#include "EngineCore.h"
#include "PhysicsSystem.h"

namespace physics {
	// public struct for physical states
	struct ENGINE_API PhysicsState {
		double theta;
		Eigen::Vector3d linearVelocity;
		Eigen::Vector3d angularVelocity;
	};
}